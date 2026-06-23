#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import ctypes as C
from ctypes import byref, c_uint, c_int, c_char_p, c_void_p, c_long, c_ushort

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


# ============================================================
# EPOS 接続設定
# ============================================================

LIB_PATH       = "/usr/local/lib/libEposCmd.so"
DEVICE_NAME    = b"EPOS2"
PROTOCOL_STACK = b"MAXON SERIAL V2"
INTERFACE_NAME = b"USB"
PORT_NAME      = b"USB0"
NODE_ID        = 1


# ============================================================
# 制御パラメータ
# ============================================================

TIMER_HZ = 50.0

# モータ回転方向の補正
# 動作方向が逆の場合は -1.0 にしてください。
MOTOR_DIR_SIGN = 1.0

RPM_SPEED = 10000.0
CLOSE_RPM_SPEED = 10000.0
MAX_RPM = 10000.0

# 停止後のモータ音対策
AUTO_DISABLE_WHEN_STOPPED = True
AUTO_DISABLE_AFTER_STOP_SEC = 0.8

# PID / PD制御
# 動作速度を上げるため、Kpを少し上げ、Dブレーキを少し弱める。
PID_KP = 16.0
PID_KI = 0.0
PID_KD = 0.20
PID_I_LIMIT = 300.0

# 戻り方向・引く方向の速度差をなくす
FAST_MIN_RPM = 10000.0
CLOSE_FAST_MIN_RPM = 10000.0
NEAR_MIN_RPM = 2500.0
CLOSE_NEAR_MIN_RPM = 2500.0

SLOW_ZONE_ERROR = 120.0

RPM_FILTER_ALPHA = 1.0
RPM_SLEW_RATE = 120000.0
RPM_COMMAND_EPS = 10.0

# ============================================================
# 角度・スライダ対応
# ============================================================

# /hand_index_angle の実測値に合わせて調整してください。
# 人差し指が開いているときの角度、閉じているときの角度。
ANGLE_OPEN = 5.0
ANGLE_CLOSE = 60.0

# 制御入力
INDEX_ANGLE_TOPIC = "/hand_index_angle"
FALLBACK_ANGLE_TOPIC = "/hand_plane_angle"
USE_FALLBACK_PLANE_ANGLE = True

SLIDER_TOPIC = "/slider_smooth"

# 可変抵抗の実測範囲
SLIDER_MIN = 0.0
SLIDER_MAX = 560.0

# 開き側・閉じ側のスライダ目標
SLIDER_TARGET_AT_OPEN = 560.0
SLIDER_TARGET_AT_CLOSE = SLIDER_MIN

# 停止判定
SLIDER_STOP_BAND = 55.0
SLIDER_MOVE_BAND = 80.0

ANGLE_STOP_BAND = 3.0
ANGLE_MOVE_BAND = 5.0

HOLD_AFTER_STOP_SEC = 0.25
SLIDER_JUMP_REJECT = 150.0
SAFETY_TIMEOUT_SEC = 15.0

DEBUG_LOG_SEC = 0.2
MISSING_INPUT_LOG_SEC = 1.0

# 入力が一定時間途切れたら停止
INPUT_TIMEOUT_SEC = 0.7

# 端到達判定
# 以前の OPEN_SLIDER_STOP_VALUE=680 は SLIDER_MAX=660 より大きいため、
# 開き端到達判定が成立しません。ここでは目標値からの余裕で判定します。
OPEN_SLIDER_END_MARGIN = 30.0
CLOSE_SLIDER_END_MARGIN = 30.0

OPEN_ANGLE_STOP_MARGIN = 8.0
CLOSE_ANGLE_STOP_MARGIN = 8.0


# ============================================================
# EPOS wrapper
# ============================================================

class EPOS:
    def __init__(self):
        self.lib = C.cdll.LoadLibrary(LIB_PATH)
        self.key = None

        self.lib.VCS_OpenDevice.restype = c_void_p
        self.lib.VCS_OpenDevice.argtypes = [
            c_char_p, c_char_p, c_char_p, c_char_p, C.POINTER(c_uint)
        ]

        self.lib.VCS_CloseDevice.argtypes = [
            c_void_p, C.POINTER(c_uint)
        ]

        self.lib.VCS_GetProtocolStackSettings.argtypes = [
            c_void_p, C.POINTER(c_uint), C.POINTER(c_uint), C.POINTER(c_uint)
        ]

        self.lib.VCS_SetProtocolStackSettings.argtypes = [
            c_void_p, c_uint, c_uint, C.POINTER(c_uint)
        ]

        self.lib.VCS_GetErrorInfo.argtypes = [
            c_uint, c_char_p, c_ushort
        ]

        self.lib.VCS_ClearFault.argtypes = [
            c_void_p, c_ushort, C.POINTER(c_uint)
        ]

        self.lib.VCS_GetFaultState.argtypes = [
            c_void_p, c_ushort, C.POINTER(c_int), C.POINTER(c_uint)
        ]

        self.lib.VCS_SetEnableState.argtypes = [
            c_void_p, c_ushort, C.POINTER(c_uint)
        ]

        self.lib.VCS_SetDisableState.argtypes = [
            c_void_p, c_ushort, C.POINTER(c_uint)
        ]

        self.lib.VCS_GetEnableState.argtypes = [
            c_void_p, c_ushort, C.POINTER(c_int), C.POINTER(c_uint)
        ]

        self.lib.VCS_ActivateProfileVelocityMode.argtypes = [
            c_void_p, c_ushort, C.POINTER(c_uint)
        ]

        self.lib.VCS_MoveWithVelocity.argtypes = [
            c_void_p, c_ushort, c_long, C.POINTER(c_uint)
        ]

        self.lib.VCS_HaltVelocityMovement.argtypes = [
            c_void_p, c_ushort, C.POINTER(c_uint)
        ]

    def _ok(self, ret, name: str, perr):
        if not ret:
            err_code = perr.value
            buf = C.create_string_buffer(256)
            self.lib.VCS_GetErrorInfo(err_code, buf, c_ushort(255))
            info = buf.value.decode(errors="ignore")
            raise RuntimeError(f"[EPOS:{name}] 0x{err_code:08X} {info}")

    def open(self):
        perr = c_uint(0)

        self.key = self.lib.VCS_OpenDevice(
            DEVICE_NAME,
            PROTOCOL_STACK,
            INTERFACE_NAME,
            PORT_NAME,
            byref(perr)
        )
        self._ok(1 if self.key else 0, "OpenDevice", perr)

        baud = c_uint(0)
        timeout = c_uint(0)

        self._ok(
            self.lib.VCS_GetProtocolStackSettings(
                self.key,
                byref(baud),
                byref(timeout),
                byref(perr)
            ),
            "GetProtocolStackSettings",
            perr,
        )

        self._ok(
            self.lib.VCS_SetProtocolStackSettings(
                self.key,
                baud,
                timeout,
                byref(perr)
            ),
            "SetProtocolStackSettings",
            perr,
        )

        is_fault = c_int(0)

        self._ok(
            self.lib.VCS_GetFaultState(
                self.key,
                c_ushort(NODE_ID),
                byref(is_fault),
                byref(perr)
            ),
            "GetFaultState",
            perr,
        )

        if is_fault.value:
            self._ok(
                self.lib.VCS_ClearFault(
                    self.key,
                    c_ushort(NODE_ID),
                    byref(perr)
                ),
                "ClearFault",
                perr,
            )

        self.enable_velocity_mode()

    def enable_velocity_mode(self):
        if self.key is None:
            raise RuntimeError("EPOS device not opened.")

        perr = c_uint(0)
        enabled = c_int(0)

        self._ok(
            self.lib.VCS_GetEnableState(
                self.key,
                c_ushort(NODE_ID),
                byref(enabled),
                byref(perr)
            ),
            "GetEnableState",
            perr,
        )

        if not enabled.value:
            self._ok(
                self.lib.VCS_SetEnableState(
                    self.key,
                    c_ushort(NODE_ID),
                    byref(perr)
                ),
                "SetEnableState",
                perr,
            )

        self._ok(
            self.lib.VCS_ActivateProfileVelocityMode(
                self.key,
                c_ushort(NODE_ID),
                byref(perr)
            ),
            "ActivateProfileVelocityMode",
            perr,
        )

    def disable(self):
        if self.key is None:
            return

        perr = c_uint(0)

        self._ok(
            self.lib.VCS_SetDisableState(
                self.key,
                c_ushort(NODE_ID),
                byref(perr)
            ),
            "SetDisableState",
            perr,
        )

    def move_velocity(self, rpm: float):
        if self.key is None:
            raise RuntimeError("EPOS device not opened.")

        perr = c_uint(0)
        vel = c_long(int(round(rpm)))

        self._ok(
            self.lib.VCS_MoveWithVelocity(
                self.key,
                c_ushort(NODE_ID),
                vel,
                byref(perr)
            ),
            "MoveWithVelocity",
            perr,
        )

    def halt(self):
        if self.key is None:
            return

        perr = c_uint(0)

        self._ok(
            self.lib.VCS_HaltVelocityMovement(
                self.key,
                c_ushort(NODE_ID),
                byref(perr)
            ),
            "HaltVelocityMovement",
            perr,
        )

    def close(self):
        if self.key is not None:
            perr = c_uint(0)

            self._ok(
                self.lib.VCS_CloseDevice(
                    self.key,
                    byref(perr)
                ),
                "CloseDevice",
                perr,
            )

            self.key = None


# ============================================================
# ROS2 node
# ============================================================

class EposControlNode(Node):
    def __init__(self):
        super().__init__("epos_control_node")

        # 最新入力
        self.index_angle = None
        self.plane_angle = None
        self.slider_value = None

        self.last_index_angle_time = None
        self.last_plane_angle_time = None
        self.last_slider_time = None

        self.last_missing_log_time = 0.0
        self.last_debug_log_time = 0.0

        self.epos = EPOS()
        self.last_cmd_rpm = 0.0
        self.motor_disabled = False

        self.state = "STOP"
        self.last_dir = 0
        self.move_start_time = None
        self.last_stop_time = time.time()

        # PID
        self.pid_integral = 0.0
        self.prev_error = 0.0
        self.prev_pid_time = None

        # rpm smoothing
        self.filtered_rpm = 0.0
        self.prev_rpm_time = None

        try:
            self.epos.open()
            self.motor_disabled = False
            self.get_logger().info("EPOS device opened and ready.")
        except Exception as e:
            self.get_logger().error(f"Failed to open EPOS: {e}")
            raise

        self.create_subscription(Float32, INDEX_ANGLE_TOPIC, self.index_angle_callback, 10)

        if USE_FALLBACK_PLANE_ANGLE:
            self.create_subscription(Float32, FALLBACK_ANGLE_TOPIC, self.plane_angle_callback, 10)

        self.create_subscription(Float32, SLIDER_TOPIC, self.slider_callback, 10)

        self.get_logger().info(
            f"Waiting topics: {INDEX_ANGLE_TOPIC}, {SLIDER_TOPIC}"
            + (f"  fallback={FALLBACK_ANGLE_TOPIC}" if USE_FALLBACK_PLANE_ANGLE else "")
        )

        self.timer = self.create_timer(1.0 / TIMER_HZ, self.control_loop)

    # ------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------

    def index_angle_callback(self, msg: Float32):
        self.index_angle = float(msg.data)
        self.last_index_angle_time = time.time()

    def plane_angle_callback(self, msg: Float32):
        self.plane_angle = float(msg.data)
        self.last_plane_angle_time = time.time()

    def slider_callback(self, msg: Float32):
        new_value = float(msg.data)

        if self.slider_value is not None:
            if abs(new_value - self.slider_value) > SLIDER_JUMP_REJECT:
                self.get_logger().warn(
                    f"Rejected slider jump: old={self.slider_value:.1f}, new={new_value:.1f}"
                )
                return

        self.slider_value = new_value
        self.last_slider_time = time.time()

    # ------------------------------------------------------------
    # utilities
    # ------------------------------------------------------------

    def clamp(self, value: float, vmin: float, vmax: float) -> float:
        return max(vmin, min(vmax, float(value)))

    def normalize(self, value: float, vmin: float, vmax: float) -> float:
        if abs(vmax - vmin) < 1.0e-9:
            return 0.0

        n = (float(value) - vmin) / (vmax - vmin)
        return self.clamp(n, 0.0, 1.0)

    def angle_to_slider_target(self, angle: float) -> float:
        angle_norm = self.normalize(angle, ANGLE_OPEN, ANGLE_CLOSE)

        target = (
            SLIDER_TARGET_AT_OPEN
            + angle_norm * (SLIDER_TARGET_AT_CLOSE - SLIDER_TARGET_AT_OPEN)
        )

        return self.clamp(target, min(SLIDER_MIN, SLIDER_MAX), max(SLIDER_MIN, SLIDER_MAX))

    def slider_to_device_angle(self, slider: float) -> float:
        denom = SLIDER_TARGET_AT_CLOSE - SLIDER_TARGET_AT_OPEN

        if abs(denom) < 1.0e-9:
            return ANGLE_OPEN

        angle_norm = (float(slider) - SLIDER_TARGET_AT_OPEN) / denom
        angle_norm = self.clamp(angle_norm, 0.0, 1.0)

        device_angle = ANGLE_OPEN + angle_norm * (ANGLE_CLOSE - ANGLE_OPEN)
        return self.clamp(device_angle, min(ANGLE_OPEN, ANGLE_CLOSE), max(ANGLE_OPEN, ANGLE_CLOSE))

    def get_target_angle(self):
        now = time.time()

        if self.index_angle is not None and self.last_index_angle_time is not None:
            if now - self.last_index_angle_time <= INPUT_TIMEOUT_SEC:
                return self.index_angle, "index"

        if USE_FALLBACK_PLANE_ANGLE:
            if self.plane_angle is not None and self.last_plane_angle_time is not None:
                if now - self.last_plane_angle_time <= INPUT_TIMEOUT_SEC:
                    return self.plane_angle, "plane_fallback"

        return None, "none"

    def inputs_are_valid(self):
        now = time.time()
        target_angle, source = self.get_target_angle()

        if target_angle is None:
            self.log_missing_input("angle input missing or timeout")
            return False, None, source

        if self.slider_value is None or self.last_slider_time is None:
            self.log_missing_input("slider input missing")
            return False, None, source

        if now - self.last_slider_time > INPUT_TIMEOUT_SEC:
            self.log_missing_input("slider input timeout")
            return False, None, source

        return True, target_angle, source

    def log_missing_input(self, reason: str):
        now = time.time()
        if now - self.last_missing_log_time < MISSING_INPUT_LOG_SEC:
            return

        self.last_missing_log_time = now

        def age_text(t):
            if t is None:
                return "None"
            return f"{now - t:.2f}s"

        self.get_logger().warn(
            f"{reason}. "
            f"age(index)={age_text(self.last_index_angle_time)}, "
            f"age(plane)={age_text(self.last_plane_angle_time)}, "
            f"age(slider)={age_text(self.last_slider_time)}. "
            f"Need {INDEX_ANGLE_TOPIC} and {SLIDER_TOPIC}."
        )

    def reached_mechanical_end(
        self,
        angle: float,
        slider: float,
        target_slider: float
    ) -> bool:
        """
        端位置で止まらない問題への保険。

        重要：
        以前は「開き角度に近い」かつ「スライダが開き端に近い」
        の両方を満たしたときだけ停止していた。
        しかしMediaPipeの角度が少し大きく出ると、
        スライダが開き端まで到達していても止まらない。

        そのため今回は、
        - 目標が開き側に近い
        - スライダが開き端付近に到達している
        なら角度条件に関係なく停止する。

        閉じ側も同様に、目標が閉じ側に近いときだけ端停止を有効にする。
        """
        open_slider_stop_value = SLIDER_TARGET_AT_OPEN - OPEN_SLIDER_END_MARGIN
        close_slider_stop_value = SLIDER_TARGET_AT_CLOSE + CLOSE_SLIDER_END_MARGIN

        # 今の目標が開き端・閉じ端に近いか
        target_is_open_side = target_slider >= open_slider_stop_value
        target_is_close_side = target_slider <= close_slider_stop_value

        is_open_angle = angle <= (ANGLE_OPEN + OPEN_ANGLE_STOP_MARGIN)
        is_open_slider = slider >= open_slider_stop_value

        # 開き目標中にスライダが開き端付近へ来たら停止
        if target_is_open_side and is_open_slider:
            return True

        # 角度も開き側なら、より確実に停止
        if target_is_open_side and is_open_angle and is_open_slider:
            return True

        is_close_angle = angle >= (ANGLE_CLOSE - CLOSE_ANGLE_STOP_MARGIN)
        is_close_slider = slider <= close_slider_stop_value

        # 閉じ目標中にスライダが閉じ端付近へ来たら停止
        if target_is_close_side and is_close_slider:
            return True

        if target_is_close_side and is_close_angle and is_close_slider:
            return True

        return False

    def reset_pid(self):
        self.pid_integral = 0.0
        self.prev_error = 0.0
        self.prev_pid_time = None
        self.filtered_rpm = 0.0
        self.prev_rpm_time = None

    def smooth_rpm_command(self, target_rpm: float) -> float:
        now = time.time()

        if self.prev_rpm_time is None:
            dt = 1.0 / TIMER_HZ
            self.prev_rpm_time = now
        else:
            dt = now - self.prev_rpm_time
            if dt <= 1.0e-6:
                dt = 1.0 / TIMER_HZ
            self.prev_rpm_time = now

        desired = self.filtered_rpm + RPM_FILTER_ALPHA * (target_rpm - self.filtered_rpm)

        max_step = RPM_SLEW_RATE * dt
        step = desired - self.filtered_rpm
        step = self.clamp(step, -max_step, max_step)

        self.filtered_rpm += step

        if abs(self.filtered_rpm) < 1.0:
            self.filtered_rpm = 0.0

        return self.filtered_rpm

    def calc_pid_rpm(self, error_raw: float) -> float:
        abs_error = abs(error_raw)

        if abs_error <= SLIDER_STOP_BAND:
            self.reset_pid()
            return 0.0

        now = time.time()

        if self.prev_pid_time is None:
            dt = 1.0 / TIMER_HZ
            derivative = 0.0
        else:
            dt = now - self.prev_pid_time
            if dt <= 1.0e-6:
                dt = 1.0 / TIMER_HZ
            derivative = (error_raw - self.prev_error) / dt

        self.prev_pid_time = now
        self.prev_error = error_raw

        self.pid_integral += error_raw * dt
        self.pid_integral = self.clamp(self.pid_integral, -PID_I_LIMIT, PID_I_LIMIT)

        sign = 1.0 if error_raw > 0.0 else -1.0

        p_term = PID_KP * abs_error

        if error_raw > 0.0:
            approach_speed = max(0.0, -derivative)
        else:
            approach_speed = max(0.0, derivative)

        d_brake = PID_KD * approach_speed
        i_term = PID_KI * abs(self.pid_integral)

        rpm_abs = max(0.0, p_term + i_term - d_brake)

        is_closing = error_raw < 0.0

        if abs_error > SLOW_ZONE_ERROR:
            min_rpm = CLOSE_FAST_MIN_RPM if is_closing else FAST_MIN_RPM
        else:
            min_rpm = CLOSE_NEAR_MIN_RPM if is_closing else NEAR_MIN_RPM

        rpm_abs = max(min_rpm, rpm_abs)

        speed_limit = CLOSE_RPM_SPEED if is_closing else RPM_SPEED
        rpm_abs = min(speed_limit, rpm_abs)

        rpm = MOTOR_DIR_SIGN * sign * rpm_abs
        rpm = self.clamp(rpm, -MAX_RPM, MAX_RPM)

        return self.smooth_rpm_command(rpm)

    def debug_log(
        self,
        source,
        target_angle,
        device_angle,
        target_slider,
        slider,
        error_angle,
        error_raw,
        rpm
    ):
        now = time.time()

        if now - self.last_debug_log_time < DEBUG_LOG_SEC:
            return

        self.last_debug_log_time = now

        self.get_logger().info(
            f"src={source}, target_angle={target_angle:.2f}, "
            f"device_angle={device_angle:.2f}, error_angle={error_angle:.2f}, "
            f"target_slider={target_slider:.1f}, slider={slider:.1f}, "
            f"error_raw={error_raw:.1f}, rpm={rpm:.1f}, "
            f"state={self.state}, disabled={self.motor_disabled}"
        )

    def _enter_stop_state(self):
        if self.state != "STOP":
            self.last_stop_time = time.time()
        elif self.last_stop_time <= 0.0:
            self.last_stop_time = time.time()

        self.state = "STOP"
        self.last_dir = 0
        self.move_start_time = None
        self.reset_pid()

    def _auto_disable_if_idle(self):
        if not AUTO_DISABLE_WHEN_STOPPED:
            return

        if self.motor_disabled:
            return

        if self.state != "STOP":
            return

        if self.last_cmd_rpm != 0.0:
            return

        if self.last_stop_time <= 0.0:
            return

        if time.time() - self.last_stop_time < AUTO_DISABLE_AFTER_STOP_SEC:
            return

        try:
            self.epos.disable()
            self.motor_disabled = True
            self.get_logger().info("EPOS disabled after stop to reduce motor noise.")
        except Exception as e:
            self.get_logger().error(f"EPOS disable error: {e}")

    def _ensure_motor_enabled(self):
        if not self.motor_disabled:
            return

        try:
            self.epos.enable_velocity_mode()
            self.motor_disabled = False
            self.get_logger().info("EPOS enabled for movement.")
        except Exception as e:
            self.get_logger().error(f"EPOS enable error: {e}")
            raise

    # ------------------------------------------------------------
    # main control
    # ------------------------------------------------------------

    def control_loop(self):
        valid, target_angle, source = self.inputs_are_valid()

        if not valid:
            if self.last_cmd_rpm != 0.0:
                self._stop_motor()
            self._enter_stop_state()
            self._auto_disable_if_idle()
            return

        target_angle = self.clamp(target_angle, min(ANGLE_OPEN, ANGLE_CLOSE), max(ANGLE_OPEN, ANGLE_CLOSE))
        slider = float(self.slider_value)

        target_slider = self.angle_to_slider_target(target_angle)
        device_angle = self.slider_to_device_angle(slider)

        error_angle = target_angle - device_angle
        abs_angle_error = abs(error_angle)

        error_raw = target_slider - slider
        abs_error_raw = abs(error_raw)

        if self.reached_mechanical_end(target_angle, slider, target_slider):
            if self.last_cmd_rpm != 0.0:
                self._stop_motor()
            self._enter_stop_state()
            self.debug_log(source, target_angle, device_angle, target_slider, slider, error_angle, error_raw, 0.0)
            self._auto_disable_if_idle()
            return

        # ---------------- STOP ----------------
        if self.state == "STOP":
            if self.last_cmd_rpm != 0.0:
                self._stop_motor()
                self._enter_stop_state()

            if time.time() - self.last_stop_time < HOLD_AFTER_STOP_SEC:
                self.debug_log(source, target_angle, device_angle, target_slider, slider, error_angle, error_raw, 0.0)
                self._auto_disable_if_idle()
                return

            # 角度差が小さいなら止めたまま
            if abs_angle_error < ANGLE_MOVE_BAND and abs_error_raw < SLIDER_MOVE_BAND:
                self.debug_log(source, target_angle, device_angle, target_slider, slider, error_angle, error_raw, 0.0)
                self._auto_disable_if_idle()
                return

            try:
                self._ensure_motor_enabled()
            except Exception:
                self._enter_stop_state()
                return

            self.reset_pid()
            target_rpm = self.calc_pid_rpm(error_raw)

            if target_rpm == 0.0:
                self._enter_stop_state()
                self.debug_log(source, target_angle, device_angle, target_slider, slider, error_angle, error_raw, 0.0)
                return

            direction = 1 if error_raw > 0.0 else -1

            self.state = "MOVE"
            self.last_dir = direction
            self.move_start_time = time.time()

            self.debug_log(source, target_angle, device_angle, target_slider, slider, error_angle, error_raw, target_rpm)
            self._set_motor(target_rpm)
            return

        # ---------------- MOVE ----------------
        if abs_angle_error <= ANGLE_STOP_BAND or abs_error_raw <= SLIDER_STOP_BAND:
            if self.last_cmd_rpm != 0.0:
                self._stop_motor()

            self._enter_stop_state()
            self.debug_log(source, target_angle, device_angle, target_slider, slider, error_angle, error_raw, 0.0)
            return

        direction = 1 if error_raw > 0.0 else -1

        if direction != self.last_dir:
            self.move_start_time = time.time()
            self.last_dir = direction
            self.reset_pid()

        target_rpm = self.calc_pid_rpm(error_raw)

        if target_rpm == 0.0:
            if self.last_cmd_rpm != 0.0:
                self._stop_motor()
            self._enter_stop_state()
            return

        if self.move_start_time is not None:
            if time.time() - self.move_start_time >= SAFETY_TIMEOUT_SEC:
                if self.last_cmd_rpm != 0.0:
                    self._stop_motor()
                self.get_logger().warn("Safety timeout -> motor stopped")
                self._enter_stop_state()
                return

        self.debug_log(source, target_angle, device_angle, target_slider, slider, error_angle, error_raw, target_rpm)

        if abs(target_rpm - self.last_cmd_rpm) >= RPM_COMMAND_EPS:
            self._set_motor(target_rpm)

    # ------------------------------------------------------------
    # motor helpers
    # ------------------------------------------------------------

    def _set_motor(self, rpm: float):
        rpm = self.clamp(rpm, -MAX_RPM, MAX_RPM)

        try:
            self._ensure_motor_enabled()
            self.epos.move_velocity(rpm)
            self.last_cmd_rpm = rpm
        except Exception as e:
            self.get_logger().error(f"EPOS move_velocity error: {e}")

            try:
                self.epos.halt()
            except Exception:
                pass

            self.last_cmd_rpm = 0.0
            self._enter_stop_state()

    def _stop_motor(self):
        try:
            self.epos.halt()
        except Exception as e:
            self.get_logger().error(f"EPOS halt error: {e}")

        self.last_cmd_rpm = 0.0

    def destroy_node(self):
        self.get_logger().info("Shutting down EPOS control node...")

        try:
            if self.last_cmd_rpm != 0.0:
                self._stop_motor()
        except Exception:
            pass

        try:
            if not self.motor_disabled:
                self.epos.disable()
                self.motor_disabled = True
        except Exception:
            pass

        try:
            self.epos.close()
        except Exception as e:
            self.get_logger().error(f"EPOS close error: {e}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EposControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
