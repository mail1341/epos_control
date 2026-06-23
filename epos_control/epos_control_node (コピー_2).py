#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import ctypes as C
from ctypes import byref, c_uint, c_int, c_char_p, c_void_p, c_long, c_ushort

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32


# ========= EPOS 接続設定 =========
LIB_PATH       = "/usr/local/lib/libEposCmd.so"
DEVICE_NAME    = b"EPOS2"
PROTOCOL_STACK = b"MAXON SERIAL V2"
INTERFACE_NAME = b"USB"
PORT_NAME      = b"USB0"
NODE_ID        = 1


# ========= 制御パラメータ =========
RPM_SPEED = 6000.0
MAX_RPM   = 8000.0
TIMER_HZ  = 50.0

# モータ回転方向の補正
# 1.0  : 通常
# -1.0 : 回転方向を反転
MOTOR_DIR_SIGN = 1.0

# ========= 可変抵抗（スライダ）の実測範囲 =========
SLIDER_MIN = 0.0
SLIDER_MAX = 810.0

# ========= hand_norm の実測範囲 =========
HAND_NORM_MIN = 2.4
HAND_NORM_MAX = 5.9


# ========= 追従制御パラメータ =========
# STOP_BAND 以内なら停止
STOP_BAND = 0.05

# MOVE_BAND 以上離れたら動作開始
MOVE_BAND = 0.12

# 速度を誤差に応じて少し変えるための最小速度
MIN_RPM = 1200.0

# 安全用タイムアウト
SAFETY_TIMEOUT_SEC = 5.0

# ログを出す周期
DEBUG_LOG_SEC = 0.2


# ======================================================================
# EPOS ラッパークラス
# ======================================================================

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


# ======================================================================
# ROS2 ノード本体
# ======================================================================

class EposControlNode(Node):
    def __init__(self):
        super().__init__("epos_control_node")

        # hand_norm の生値
        self.norm_avg = None

        # 可変抵抗の生値
        self.slider_value = None

        self.epos = EPOS()
        self.last_cmd_rpm = 0.0

        # 状態: STOP or MOVE
        self.state = "STOP"

        # 直近の回転方向 +1 / -1 / 0
        self.last_dir = 0

        # 動作開始時刻
        self.move_start_time = None

        # ログ制御
        self.last_debug_log_time = 0.0

        try:
            self.epos.open()
            self.get_logger().info("EPOS device opened and ready.")
        except Exception as e:
            self.get_logger().error(f"Failed to open EPOS: {e}")
            raise

        # /hand_norm: Float32
        self.create_subscription(
            Float32,
            "/hand_norm",
            self.hand_norm_callback,
            10
        )

        # /slider_raw: Int32
        self.create_subscription(
            Int32,
            "/slider_raw",
            self.slider_callback,
            10
        )

        self.timer = self.create_timer(
            1.0 / TIMER_HZ,
            self.control_loop
        )

    # ------------------------------------------------------------------
    # callback
    # ------------------------------------------------------------------

    def hand_norm_callback(self, msg: Float32):
        self.norm_avg = float(msg.data)

    def slider_callback(self, msg: Int32):
        self.slider_value = int(msg.data)

    # ------------------------------------------------------------------
    # utility
    # ------------------------------------------------------------------

    def normalize(self, value: float, vmin: float, vmax: float) -> float:
        if abs(vmax - vmin) < 1e-9:
            return 0.0

        n = (float(value) - vmin) / (vmax - vmin)
        return max(0.0, min(1.0, n))

    def calc_rpm_from_error(self, error: float) -> float:
        """
        誤差に応じて回転速度を決める。
        誤差が小さいときは低速、大きいときはRPM_SPEEDに近づける。
        """
        abs_error = abs(error)

        if abs_error <= STOP_BAND:
            return 0.0

        # MOVE_BAND 以上ではしっかり動かす
        ratio = min(1.0, abs_error / max(MOVE_BAND, 1e-9))

        rpm_abs = MIN_RPM + (RPM_SPEED - MIN_RPM) * ratio
        rpm_abs = min(MAX_RPM, max(MIN_RPM, rpm_abs))

        sign = 1.0 if error > 0.0 else -1.0

        return MOTOR_DIR_SIGN * sign * rpm_abs

    def debug_log(self, norm_raw, hand_norm, slider_raw, slider_norm, error, rpm):
        now = time.time()

        if now - self.last_debug_log_time < DEBUG_LOG_SEC:
            return

        self.last_debug_log_time = now

        self.get_logger().info(
            f"norm_raw={norm_raw:.3f}, hand_norm={hand_norm:.3f}, "
            f"slider_raw={slider_raw}, slider_norm={slider_norm:.3f}, "
            f"error={error:.3f}, rpm={rpm:.1f}, state={self.state}"
        )

    # ------------------------------------------------------------------
    # main control
    # ------------------------------------------------------------------

    def control_loop(self):
        norm_avg = self.norm_avg
        slider = self.slider_value

        # 情報がないときは停止
        if norm_avg is None or slider is None:
            if self.last_cmd_rpm != 0.0:
                self._stop_motor()

            self.state = "STOP"
            self.last_dir = 0
            self.move_start_time = None
            return

        # スライダ現在位置を 0〜1 に正規化
        slider_norm = self.normalize(
            slider,
            SLIDER_MIN,
            SLIDER_MAX
        )

        # 手の目標値を 0〜1 に正規化
        hand_norm = self.normalize(
            norm_avg,
            HAND_NORM_MIN,
            HAND_NORM_MAX
        )

        # 追従誤差
        error = hand_norm - slider_norm

        # 誤差に応じて回転速度を決める
        target_rpm = self.calc_rpm_from_error(error)

        self.debug_log(
            norm_avg,
            hand_norm,
            slider,
            slider_norm,
            error,
            target_rpm
        )

        # --------------------------------------------------------------
        # 停止範囲に入ったら停止
        # --------------------------------------------------------------
        if abs(error) <= STOP_BAND:
            if self.last_cmd_rpm != 0.0:
                self._stop_motor()

            self.state = "STOP"
            self.last_dir = 0
            self.move_start_time = None
            return

        # --------------------------------------------------------------
        # 停止範囲外なら追従動作
        # --------------------------------------------------------------
        direction = 1 if target_rpm > 0.0 else -1

        # 停止状態から動き始めた時刻を記録
        if self.state == "STOP":
            self.move_start_time = time.time()
            self.state = "MOVE"
            self.last_dir = direction

        # 方向が変わった場合はタイムアウトをリセット
        if direction != self.last_dir:
            self.move_start_time = time.time()
            self.last_dir = direction

        # 安全用タイムアウト
        if self.move_start_time is not None:
            if time.time() - self.move_start_time >= SAFETY_TIMEOUT_SEC:
                if self.last_cmd_rpm != 0.0:
                    self._stop_motor()

                self.get_logger().warn("Safety timeout -> motor stopped")

                self.state = "STOP"
                self.last_dir = 0
                self.move_start_time = None
                return

        # 前回と速度が大きく違うときだけ指令を送る
        if abs(target_rpm - self.last_cmd_rpm) >= 50.0:
            self._set_motor(target_rpm)

    # ------------------------------------------------------------------
    # motor helpers
    # ------------------------------------------------------------------

    def _set_motor(self, rpm: float):
        rpm = min(MAX_RPM, max(-MAX_RPM, float(rpm)))

        try:
            self.epos.move_velocity(rpm)
            self.last_cmd_rpm = rpm
        except Exception as e:
            self.get_logger().error(f"EPOS move_velocity error: {e}")

            try:
                self.epos.halt()
            except Exception:
                pass

            self.last_cmd_rpm = 0.0
            self.state = "STOP"
            self.last_dir = 0
            self.move_start_time = None

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
            self.epos.close()
        except Exception as e:
            self.get_logger().error(f"EPOS close error: {e}")

        super().destroy_node()


# ======================================================================
# main
# ======================================================================

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