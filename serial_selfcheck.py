#!/usr/bin/env python3
"""
串口自检脚本 — 定位 STM32 通信问题（真机 RDK X5）

绕过 logger / Web / 摄像头，直接验证：
  1. /dev/ttyS1 是否被别的进程占用
  2. pyserial open + write 是否成功、实际写回多少字节
  3. STM32 有没有任何应答帧

用法（在 RDK X5 上）:
  python3 serial_selfcheck.py [port] [baudrate]
"""
import sys
import time
import struct

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyS1"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200


def _checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def build_frame(cmd_id: int, payload: bytes) -> bytes:
    frame = bytes([0xA5, 0x5A, cmd_id, len(payload)]) + payload
    return frame + bytes([_checksum(frame)])


def encode_velocity(vx: int) -> bytes:
    vx = max(-500, min(500, vx))
    return build_frame(0x01, struct.pack("<h", vx))


def encode_stop() -> bytes:
    return build_frame(0x02, bytes([0x01]))


def main():
    print("=" * 60)
    print(f"串口自检: {PORT} @ {BAUD}")
    print("=" * 60)

    # 1. 检查设备是否存在 + 是否被占用
    import os
    if not os.path.exists(PORT):
        print(f"[失败] {PORT} 不存在！板载 UART 可能未使能")
        import glob
        cands = sorted(set(glob.glob('/dev/ttyS*') + glob.glob('/dev/ttyTHS*')))
        print(f"  实际 /dev 下 UART 候选: {cands if cands else '(空)'}")
        print("  → 请确认 STM32 接的是哪个 UART，并修改 settings.yaml 的 serial.port")
        return 1

    # 2. 检查是否被占用（fuser）
    try:
        import subprocess
        r = subprocess.run(["fuser", PORT], capture_output=True, text=True, timeout=5)
        if r.stdout.strip():
            print(f"[警告] {PORT} 正被进程占用: {r.stdout.strip()}")
    except Exception:
        pass

    # 3. 打开串口
    import serial
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUD,
            timeout=0.05,
            write_timeout=0.05,
        )
        print(f"[成功] 串口已打开: {ser.name}, is_open={ser.is_open}")
    except Exception as e:
        print(f"[失败] 打开串口失败: {e}")
        print("  → 权限不足或驱动未加载")
        return 1

    # 4. 发送一帧速度指令，看实际写回字节数
    vel = encode_velocity(300)
    stop = encode_stop()
    print(f"\n速度帧 (前3字节): {vel[:3].hex(' ')} ... 完整 {len(vel)} 字节: {vel.hex(' ')}")
    print(f"急停帧 ({len(stop)} 字节): {stop.hex(' ')}")

    try:
        for i in range(3):
            n = ser.write(vel)
            print(f"[发送#{i+1}] write 返回 {n} 字节 (期望 {len(vel)})")
            time.sleep(0.5)
    except Exception as e:
        print(f"[失败] 写入串口异常: {e}")
        ser.close()
        return 1

    # 5. 读回显，看 STM32 有没有应答
    print("\n等待 STM32 应答 2 秒...")
    ser.reset_input_buffer()
    deadline = time.time() + 2.0
    total = b""
    while time.time() < deadline:
        if ser.in_waiting > 0:
            chunk = ser.read(ser.in_waiting)
            total += chunk
            print(f"  收到 {len(chunk)} 字节: {chunk.hex(' ')}")
            deadline = time.time() + 0.3  # 收到数据则延长等待，收集完整帧
    if not total:
        print("  (无任何应答字节)")

    # 6. 小结
    print("\n" + "=" * 60)
    print("结论:")
    if total:
        print(f"  ✅ STM32 有应答，共 {len(total)} 字节。通信正常，问题在上层")
    else:
        print("  ⚠️ STM32 无应答。可能原因:")
        print("     a. STM32 固件没跑/没初始化 UART 接收")
        print("     b. TX/RX 接反 (RDK TX→STM32 RX, RDK RX→STM32 TX)")
        print("     c. 波特率不匹配 (settings.yaml 写 115200，确认 STM32 串口同为 115200)")
        print("     d. 地线(共地)没接")
        print("     e. TTL 电平不匹配 (板载 UART 是 3.3V TTL)")
    print("=" * 60)

    ser.close()
    return 0 if total else 2


if __name__ == "__main__":
    sys.exit(main())
