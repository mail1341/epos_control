#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import ctypes as C
from ctypes import byref, c_uint, c_int, c_char_p, c_void_p, c_long, c_ushort

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32

# ========= EPOS 接続設定 =========
# 必要に応じて実機に合わせて変更してください
LIB_PATH       = "/usr/local/lib/libEposCmd.so"
DEVICE_NAME    = b"EPOS2"
PROTOCOL_STACK = b"MAXON SERIAL V2"
INTERFACE_NAME = b"USB"
PORT_NAME      = b"USB0"
BAUDRATE       = 1000000
NODE_ID        = 1

# ========= 制御パラメータ =========
RPM_SPEED    = 7500.0   # 開: +RPM_SPEED, 閉: -RPM_SPEED
MAX_RPM      = 8000.0   # 安全のための上限値
OPEN_NORM_TH = 0.6      # 「開いた」判定（hand_norm 平均）
CLOSE_NORM_TH= 0.4      # 「閉じた」判定
BURST_SEC    = 10.0     # 非常停止用の最大回転時間 [秒]（長めに設定）
TIMER_HZ     = 50.0     # 制御ループ周期 [Hz]


class EPOS:
    """EPOS 制御ラッパ"""
    def __init__(self):
        self.lib = C.cdll.LoadLibrary(LIB_PATH)

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

        self.lib.VCS_GetErrorInfo.argtypes = [
            c_uint, C.c_char_p, c_ushort
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
        self.lib.VCS_SetDisableState.argtypes = [
            c_void_p, c_ushort, C.POINTER(c_uint)
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

        self.key = c_void_p(None)

    def _errstr(self, code: int) -> str:
        buf = C.create_string_buffer(512)
        self.lib.VCS_GetErrorInfo(c_uint(code), buf, c_ushort(512))
        return buf.value.decode("latin1", errors="ignore")

    def _ok(self, ok: int, where: str, perr: c_uint):
        if not ok:
            raise RuntimeError(
                f"[EPOS:{where}] 0x{perr.value:08X} {self._errstr(perr.value)}"
            )

    def open(self):
        perr = c_uint(0)

        # デバイスオープン
        self.key = self.lib.VCS_OpenDevice(
            DEVICE_NAME, PROTOCOL_STACK, INTERFACE_NAME, PORT_NAME, byref(perr)
        )
        self._ok(1 if self.key else 0, "OpenDevice", perr)

        # 通信設定
        b = c_uint(0)
        to = c_uint(0)
        self._ok(
            self.lib.VCS_GetProtocolStackSettings(
                self.key, byref(b), byref(to), byref(perr)
            ),
            "GetProtocolStackSettings",
            perr,
        )
        self._ok(
            self.lib.VCS_SetProtocolStackSettings(
                self.key, c_uint(BAUDRATE), to, byref(perr)
            ),
            "SetProtocolStackSettings",
            perr,
        )

        # フォルト解除
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

        # Enable
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

        # 速度プロファイルモード
        self._ok(
            self.lib.VCS_ActivateProfileVelocityMode(
                self.key, c_ushort(NODE_ID), byref(perr)
            ),
            "ActivateProfileVelocityMode",
            perr,
        )

    def close(self):
        perr = c_uint(0)
        try:
            try:
                self.halt_velocity()
            except Exception:
                pass
            self.lib.VCS_SetDisableState(self.key, c_ushort(NODE_ID), byref(perr))
        finally:
            if self.key:
                self.lib.VCS_CloseDevice(self.key, byref(perr))
            self.key = c_void_p(None)

    def move_with_velocity_rpm(self, rpm: float):
        perr = c_uint(0)
        self._ok(
            self.lib.VCS_MoveWithVelocity(
                self.key,
                c_ushort(NODE_ID),
                c_long(int(round(rpm))),
                byref(perr),
            ),
            f"MoveWithVelocity({rpm})",
            perr,
        )

    def halt_velocity(self):
        perr = c_uint(0)
        self._ok(
            self.lib.VCS_HaltVelocityMovement(
                self.key, c_ushort(NODE_ID), byref(perr)
            ),
            "HaltVelocityMovement",
            perr,
        )


class EposControlNode(Node):
    """
    /hand_norm (Float32MultiArray; [index, middle, ring, pinky] 0〜1) を購読して
    手の開閉で EPOS モータを駆動する。
    さらに /slider_raw (Int32; 0〜1023) を購読し、
    開き動作中は「700以上」で停止、
    閉じ動作中は「10以下」で停止させる。
    """

    def __init__(self):
        super().__init__("epos_control_node")

        # 最新の正規化平均（手の開き具合）
        self.norm_avg = None

        # スライダの最新値（Arduino analogRead 0〜1023 想定）
        self.slider_value = None
        self.slider_stop_threshold  = 680 # 開くときに止めるしきい値
        self.slider_close_threshold = 10   # 閉じるときに止めるしきい値

        # ★ 連続フレームでのしきい値超えカウンタ
        self.required_over_count = 5   # 何回連続で超えたら停止するか
        self.open_over_count  = 0      # RUN_OPEN 用
        self.close_over_count = 0      # RUN_CLOSE 用


        # 状態機械
        self.mode = "WAIT_OPEN"   # WAIT_OPEN, RUN_OPEN, WAIT_CLOSE, RUN_CLOSE
        self.burst_start = None   # 回転開始時刻
        self.last_cmd_rpm = 0.0

        # EPOS 初期化
        self.epos = EPOS()
        try:
            self.epos.open()
            self.get_logger().info("EPOS device opened and ready.")
        except Exception as e:
            self.get_logger().error(f"Failed to open EPOS: {e}")
            raise

        # /hand_norm 購読
        self.sub_norm = self.create_subscription(
            Float32MultiArray,
            "hand_norm",
            self.hand_norm_callback,
            10,
        )

        # /slider_raw 購読（Arduino スライダ）
        self.sub_slider = self.create_subscription(
            Int32,
            "slider_raw",          # slider_node 側の publish と合わせる
            self.slider_callback,
            10,
        )

        # 制御タイマ
        self.timer = self.create_timer(1.0 / TIMER_HZ, self.control_loop)

    # --- コールバック: hand_norm を受信 ---
    def hand_norm_callback(self, msg: Float32MultiArray):
        if not msg.data:
            self.norm_avg = None
            return
        # index, middle, ring, pinky の平均
        self.norm_avg = float(sum(msg.data) / len(msg.data))

    # --- コールバック: スライダ値を受信 ---
    def slider_callback(self, msg: Int32):
        self.slider_value = int(msg.data)

    # --- 制御ループ ---
    def control_loop(self):
        now = time.time()
        norm_avg = self.norm_avg
        slider = self.slider_value

        # 手が見えていないときは停止
        if norm_avg is None:
            if self.last_cmd_rpm != 0.0:
                self._stop_motor()
            return

        is_open  = norm_avg >= OPEN_NORM_TH
        is_close = norm_avg <= CLOSE_NORM_TH

        # ===== 状態機械 =====
        if self.mode == "WAIT_OPEN":
            # 停止して開き待ち
            if self.last_cmd_rpm != 0.0:
                self._stop_motor()

            # ★ カウンタはリセット
            self.open_over_count  = 0
            self.close_over_count = 0

            if is_open:
                rpm = min(MAX_RPM, max(-MAX_RPM, +RPM_SPEED))
                self._set_motor(rpm)
                self.burst_start = now
                self.mode = "RUN_OPEN"
                self.get_logger().info(
                    f"Detected OPEN (norm={norm_avg:.2f}) -> RUN_OPEN (forward)"
                )


        elif self.mode == "RUN_OPEN":
            # スライダ監視（開く動作時）
            if slider is not None:
                if slider >= self.slider_stop_threshold:
                    self.open_over_count += 1
                else:
                    self.open_over_count = 0
            else:
                self.open_over_count = 0

            # 1) 5回連続でしきい値以上なら停止
            if self.open_over_count >= self.required_over_count:
                self._stop_motor()
                self.burst_start = None
                self.mode = "WAIT_CLOSE"
                self.get_logger().info(
                    f"RUN_OPEN stopped by slider (value={slider}, "
                    f"count={self.open_over_count}) -> WAIT_CLOSE"
                )

            # 2) 非常用タイムアウト（保険）
            elif self.burst_start and (now - self.burst_start >= BURST_SEC):
                self._stop_motor()
                self.burst_start = None
                self.mode = "WAIT_CLOSE"
                self.get_logger().warn(
                    "RUN_OPEN emergency stop by time (safety timeout) -> WAIT_CLOSE"
                )


        elif self.mode == "WAIT_CLOSE":
            # 停止して握り待ち
            if self.last_cmd_rpm != 0.0:
                self._stop_motor()

            # ★ カウンタはリセット
            self.open_over_count  = 0
            self.close_over_count = 0

            if is_close:
                rpm = min(MAX_RPM, max(-MAX_RPM, -RPM_SPEED))
                self._set_motor(rpm)
                self.burst_start = now
                self.mode = "RUN_CLOSE"
                self.get_logger().info(
                    f"Detected CLOSE (norm={norm_avg:.2f}) -> RUN_CLOSE (reverse)"
                )


        elif self.mode == "RUN_CLOSE":
            # 経過時間（タイムアウト用）
            elapsed = now - self.burst_start if self.burst_start else 0.0

            # スライダ監視（閉じる動作時）
            if slider is not None:
                if slider <= self.slider_close_threshold:
                    self.close_over_count += 1
                else:
                    self.close_over_count = 0
            else:
                self.close_over_count = 0

            # 1) 5回連続でしきい値以下なら停止
            if self.close_over_count >= self.required_over_count:
                self._stop_motor()
                self.burst_start = None
                self.mode = "WAIT_OPEN"
                self.get_logger().info(
                    f"RUN_CLOSE stopped by slider (value={slider}, "
                    f"count={self.close_over_count}) -> WAIT_OPEN"
                )

            # 2) 非常用タイムアウト（保険）
            elif self.burst_start and (elapsed >= BURST_SEC):
                self._stop_motor()
                self.burst_start = None
                self.mode = "WAIT_OPEN"
                self.get_logger().warn(
                    "RUN_CLOSE emergency stop by time (safety timeout) -> WAIT_OPEN"
                )


    def _set_motor(self, rpm: float):
        try:
            self.epos.move_with_velocity_rpm(rpm)
            self.last_cmd_rpm = rpm
        except Exception as e:
            self.get_logger().error(f"EPOS move error: {e}")
            self.last_cmd_rpm = 0.0

    def _stop_motor(self):
        try:
            self.epos.halt_velocity()
        except Exception as e:
            self.get_logger().error(f"EPOS halt error: {e}")
        self.last_cmd_rpm = 0.0

    def destroy_node(self):
        # 終了時の後始末
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


def main(args=None):
    rclpy.init(args=args)
    node = EposControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
