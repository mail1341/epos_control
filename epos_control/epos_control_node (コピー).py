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
NODE_ID        = 1  # EPOS のノード ID

# ========= 制御パラメータ =========
RPM_SPEED    = 6000.0   # 基本回転速度（正:開く方向, 負:閉じる方向）
MAX_RPM      = 8000.0   # 安全のための上限値
TIMER_HZ     = 50.0     # 制御ループ周期 [Hz]

# 可変抵抗（スライダ）の生値の範囲（最大値=680）
SLIDER_MIN      = 0.0
SLIDER_MAX      = 680.0

# 誤差ヒステリシス（0〜1スケール）
STOP_BAND = 0.03   # ここまで近づいたら停止
MOVE_BAND = 0.20   # ここより離れたら動き出す（STOP_BAND より必ず大きく）

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
        self.lib.VCS_CloseDevice.argtypes = [c_void_p, C.POINTER(c_uint)]
        self.lib.VCS_GetProtocolStackSettings.argtypes = [
            c_void_p, C.POINTER(c_uint), C.POINTER(c_uint), C.POINTER(c_uint)
        ]
        self.lib.VCS_SetProtocolStackSettings.argtypes = [
            c_void_p, c_uint, c_uint, C.POINTER(c_uint)
        ]
        self.lib.VCS_GetErrorInfo.argtypes = [c_uint, c_char_p, c_ushort]
        self.lib.VCS_ClearFault.argtypes = [c_void_p, c_ushort, C.POINTER(c_uint)]
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
            DEVICE_NAME, PROTOCOL_STACK, INTERFACE_NAME, PORT_NAME, byref(perr)
        )
        self._ok(1 if self.key else 0, "OpenDevice", perr)

        baud = c_uint(0)
        timeout = c_uint(0)
        self._ok(
            self.lib.VCS_GetProtocolStackSettings(
                self.key, byref(baud), byref(timeout), byref(perr)
            ),
            "GetProtocolStackSettings",
            perr,
        )
        self._ok(
            self.lib.VCS_SetProtocolStackSettings(
                self.key, baud, timeout, byref(perr)
            ),
            "SetProtocolStackSettings",
            perr,
        )

        is_fault = c_int(0)
        self._ok(
            self.lib.VCS_GetFaultState(
                self.key, c_ushort(NODE_ID), byref(is_fault), byref(perr)
            ),
            "GetFaultState",
            perr,
        )
        if is_fault.value:
            self._ok(
                self.lib.VCS_ClearFault(
                    self.key, c_ushort(NODE_ID), byref(perr)
                ),
                "ClearFault",
                perr,
            )

        enabled = c_int(0)
        self._ok(
            self.lib.VCS_GetEnableState(
                self.key, c_ushort(NODE_ID), byref(enabled), byref(perr)
            ),
            "GetEnableState",
            perr,
        )
        if not enabled.value:
            self._ok(
                self.lib.VCS_SetEnableState(
                    self.key, c_ushort(NODE_ID), byref(perr)
                ),
                "SetEnableState",
                perr,
            )

        self._ok(
            self.lib.VCS_ActivateProfileVelocityMode(
                self.key, c_ushort(NODE_ID), byref(perr)
            ),
            "ActivateProfileVelocityMode",
            perr,
        )

    def move_velocity(self, rpm: float):
        if self.key is None:
            raise RuntimeError("EPOS device not opened.")
        perr = c_uint(0)
        vel = c_long(int(rpm))
        self._ok(
            self.lib.VCS_MoveWithVelocity(
                self.key, c_ushort(NODE_ID), vel, byref(perr)
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
                self.key, c_ushort(NODE_ID), byref(perr)
            ),
            "HaltVelocityMovement",
            perr,
        )

    def close(self):
        if self.key is not None:
            perr = c_uint(0)
            self._ok(
                self.lib.VCS_CloseDevice(self.key, byref(perr)),
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

        # hand_norm の正規化値（0〜1）
        self.norm_avg = None

        # 可変抵抗の生値（0〜680）
        self.slider_value = None

        self.epos = EPOS()
        self.last_cmd_rpm = 0.0

        # 状態: "STOP" or "MOVE"
        self.state = "STOP"

        # 直近の回転方向 +1 / -1 / 0
        self.last_dir = 0

        try:
            self.epos.open()
            self.get_logger().info("EPOS device opened and ready.")
        except Exception as e:
            self.get_logger().error(f"Failed to open EPOS: {e}")
            raise

        # /hand_norm: Float32 の想定
        self.create_subscription(
            Float32,
            "/hand_norm",
            self.hand_norm_callback,
            10
        )

        # /slider_raw: Int32（0〜680）
        self.create_subscription(
            Int32,
            "/slider_raw",
            self.slider_callback,
            10
        )

        self.timer = self.create_timer(1.0 / TIMER_HZ, self.control_loop)

    # --- コールバック: hand_norm を受信 ---
    def hand_norm_callback(self, msg: Float32):
        self.norm_avg = float(msg.data)

    # --- コールバック: スライダ値 ---
    def slider_callback(self, msg: Int32):
        self.slider_value = int(msg.data)

    # --- 制御ループ ---
    def control_loop(self):
        norm_avg = self.norm_avg
        slider = self.slider_value

        # 情報がないときは停止
        if norm_avg is None or slider is None:
            if self.last_cmd_rpm != 0.0:
                self._stop_motor()
            self.state = "STOP"
            self.last_dir = 0
            return

        # スライダ正規化（0〜1）
        slider_norm = (float(slider) - SLIDER_MIN) / (SLIDER_MAX - SLIDER_MIN)
        slider_norm = max(0.0, min(1.0, slider_norm))

        # 誤差
        error = norm_avg - slider_norm

        # 符号（開く:+1, 閉じる:-1, ほぼ同じ:0）
        if error > 0.0:
            sign = 1
        elif error < 0.0:
            sign = -1
        else:
            sign = 0

        # デバッグしたいときはコメントを外す
        # self.get_logger().info(
        #     f"norm={norm_avg:.3f}, slider_raw={slider}, "
        #     f"slider_norm={slider_norm:.3f}, error={error:.3f}, state={self.state}"
        # )

        # =========================
        # 状態: STOP（停止中）
        # =========================
        if self.state == "STOP":
            # 目標から十分離れたら動き出す
            if abs(error) >= MOVE_BAND and sign != 0:
                self.state = "MOVE"
                self.last_dir = sign
                rpm = RPM_SPEED * sign
                self._set_motor(rpm)
            else:
                # 念のため停止しておく
                if self.last_cmd_rpm != 0.0:
                    self._stop_motor()
            return

        # =========================
        # 状態: MOVE（動作中）
        # =========================
        # 1) 目標に十分近づいたら停止
        if abs(error) <= STOP_BAND:
            self.state = "STOP"
            self.last_dir = 0
            if self.last_cmd_rpm != 0.0:
                self._stop_motor()
            return

        # 2) まだ遠い → 回転方向を決める
        desired_dir = self.last_dir

        # 誤差が大きい場合だけ方向変更を許可（ヒステリシスで振動防止）
        if abs(error) >= MOVE_BAND and sign != 0:
            desired_dir = sign

        # 念のためチェック
        if desired_dir == 0:
            if self.last_cmd_rpm != 0.0:
                self._stop_motor()
            self.state = "STOP"
            return

        rpm = RPM_SPEED * desired_dir
        rpm = min(MAX_RPM, max(-MAX_RPM, rpm))

        if rpm != self.last_cmd_rpm:
            self._set_motor(rpm)

    # --- モータ制御ヘルパ ---
    def _set_motor(self, rpm: float):
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