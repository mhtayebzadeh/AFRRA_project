from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass
from typing import Optional
import numpy as np

try:
    import serial  # type: ignore
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pyserial not installed. Run: pip install pyserial") from exc

# ---------------- USER SETTINGS ----------------
PORT = "COM8"
BAUD_RATE = 115200

LOOP_INTERVAL_S = 0.01  # superloop delay to avoid busy loop if no bytes arrive

# Initial parameters pushed once after connect
INIT_KP = 8.5
INIT_KD = 0.0
INIT_KI = 0.0

# Setpoint send options
SINE_AMPLITUDE_RAD = 0.2
SINE_FREQUENCY_HZ = 0.2

# ---------------- PACKET IDs (Core/Src/main.c) ----------------
CMD_SET_KP = 10
CMD_SET_KD = 11
CMD_SET_KI = 12
CMD_SET_ANKLE_SP = 13

SCALE = 1000.0
PACKET_LEN = 8
START_BYTES = (0xAA, 0xAA)
END_BYTE = 0xBB


@dataclass
class McuState:
    time_ms: int
    theta_ankle: float
    theta_dot: Optional[float] = None
    sp_theta_ankle: Optional[float] = None
    loadcell_force: Optional[float] = None
    sp_force: Optional[float] = None
    current_sp_sysID: Optional[float] = None

def build_packet(command_id: int, value_float: float) -> bytes:
    """Build 8-byte packet expected by MCU."""
    scaled = int(round(value_float * SCALE))
    payload = struct.pack(">i", scaled)
    return bytes([START_BYTES[0], START_BYTES[1], command_id]) + payload + bytes([END_BYTE])

def write_packet(ser: serial.Serial, command_id: int, value_float: float) -> None:
    ser.write(build_packet(command_id, value_float))

def set_kp(ser: serial.Serial, kp: float) -> None:
    write_packet(ser, CMD_SET_KP, kp)

def set_kd(ser: serial.Serial, kd: float) -> None:
    write_packet(ser, CMD_SET_KD, kd)

def set_ki(ser: serial.Serial, ki: float) -> None:
    write_packet(ser, CMD_SET_KI, ki)

def set_theta_sp(ser: serial.Serial, theta_rad: float) -> None:
    write_packet(ser, CMD_SET_ANKLE_SP, theta_rad)

def parse_mcu_line(text: str) -> Optional[McuState]:
    """Parse one CSV line from STM32 (sent via USART6).

    Expected current firmware format in main.c:
      time_ms, theta_ankle, theta_dot, sp_theta_ankle, loadcell_force, sp_force, current_sp_sysID

    Also accepts an older 6-field variant.
    """
    parts = [p.strip() for p in text.split(",")]

    try:
        if len(parts) == 7:
            return McuState(
                time_ms=int(parts[0]),
                theta_ankle=float(parts[1]),
                theta_dot=float(parts[2]),
                sp_theta_ankle=float(parts[3]),
                loadcell_force=float(parts[4]),
                sp_force=float(parts[5]),
                current_sp_sysID=float(parts[6]),
            )
    except ValueError:
        return None

    return None

def format_state(s: McuState) -> str:
    # Keep this intentionally short for realtime readability
    if s.theta_dot is not None:
        return (
            f"{s.time_ms:6d} | theta={s.theta_ankle:+.3f} | theta_dot={s.theta_dot:+.3f} | "
            f"sp_theta={None if s.sp_theta_ankle is None else f'{s.sp_theta_ankle:+.3f}'} | "
            f"F={None if s.loadcell_force is None else f'{s.loadcell_force:+.3f}'} | "
            f"spF={None if s.sp_force is None else f'{s.sp_force:+.3f}'}"
        )

    return (
        f"{s.time_ms:6d} | theta={s.theta_ankle:+.3f} | "
        f"sp_theta={None if s.sp_theta_ankle is None else f'{s.sp_theta_ankle:+.3f}'} | "
        f"F={None if s.loadcell_force is None else f'{s.loadcell_force:+.3f}'} | "
        f"spF={None if s.sp_force is None else f'{s.sp_force:+.3f}'}"
    )

def superloop(ser: serial.Serial) -> None:
    while True:
        time.sleep(LOOP_INTERVAL_S)
        line = ser.readline()
        mcu_states = parse_mcu_line(line.decode("ascii", errors="ignore"))
        if mcu_states is None:
            continue
        
        # print values one by one for testing
        # print(format_state(mcu_states))
        # print("time_ms:", mcu_states.time_ms)
        # print("theta_ankle:", mcu_states.theta_ankle)
        # print("theta_dot:", mcu_states.theta_dot)
        # print("sp_theta_ankle:", mcu_states.sp_theta_ankle)
        # print("loadcell_force:", mcu_states.loadcell_force)
        # print("sp_force:", mcu_states.sp_force)
        # print("current_sp_sysID:", mcu_states.current_sp_sysID)
        # print("-" * 40)
        
        ## set impedance and setpoint for testing
        # set_kp(ser, 2.5)
        # set_kd(ser, 0.0)
        # set_theta_sp(ser, 0.0)        

        theta_desired = np.sin(2 * np.pi * SINE_FREQUENCY_HZ * time.time()) * SINE_AMPLITUDE_RAD
        set_theta_sp(ser, theta_desired)


def main():
    print(f"Connecting to {PORT} @ {BAUD_RATE}...")

    # Small timeout keeps the loop responsive even if no bytes arrive
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=0.02, write_timeout=0.05)
    except Exception as e:
        msg = [f"Failed to open {PORT}: {e}"]
        raise SystemExit("\n".join(msg))

    with ser:
        time.sleep(1.0)

        # Discard any partial line after reset/boot
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        # Push initial gains once
        set_kp(ser, INIT_KP)
        set_kd(ser, INIT_KD)
        set_ki(ser, INIT_KI)

        print("Running... Ctrl+C to stop")
        superloop(ser)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
