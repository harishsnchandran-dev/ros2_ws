#!/usr/bin/env python3
"""
Parker Compax3 Drive Enable Sequence Tool
Walks through the full CiA 402 state machine to enable the drive.
Run as: sudo python3 enable_drive.py
"""
import serial
import time
import sys

PORT     = '/dev/ttyUSB0'
BAUDRATE = 115200
TIMEOUT  = 1.0

def talk(ser, cmd, delay=0.1):
    """Send a command and return the stripped response."""
    ser.reset_input_buffer()
    ser.write((cmd + '\r\n').encode())
    time.sleep(delay)
    resp = ser.read(ser.in_waiting or 64).decode(errors='replace').strip()
    return resp

def read_obj(ser, obj):
    """Read an object value from the drive."""
    r = talk(ser, f'O{obj}?')
    return r

def write_obj(ser, obj, val, delay=0.15):
    """Write an object value to the drive."""
    r = talk(ser, f'O{obj}={val}', delay)
    return r

def print_status(ser):
    """Pretty-print the current drive status."""
    sw_raw = read_obj(ser, '700.1')  # Compax3 statusword
    err    = read_obj(ser, '604.1')  # Error code
    en     = read_obj(ser, '300.1')  # Drive enable
    print(f"\n  Error Code  : {err}")
    print(f"  Statusword  : {sw_raw}")
    print(f"  Drive Enable: {en}")
    try:
        sw = int(sw_raw)
        print(f"    Bit 0 Ready to Switch On : {'ON' if sw & (1<<0) else 'OFF'}")
        print(f"    Bit 1 Switched On         : {'ON' if sw & (1<<1) else 'OFF'}")
        print(f"    Bit 2 Operation Enabled   : {'ON' if sw & (1<<2) else 'OFF'}")
        print(f"    Bit 3 Fault               : {'ON' if sw & (1<<3) else 'OFF'}")
        print(f"    Bit 4 Voltage Enabled     : {'ON' if sw & (1<<4) else 'OFF'}")
        print(f"    Bit 5 Quick Stop (Act.Low): {'ON' if sw & (1<<5) else 'OFF'}")
        print(f"    Bit 6 Switch On Inhibited : {'ON' if sw & (1<<6) else 'OFF'}")
        print(f"    Bit 7 Warning             : {'ON' if sw & (1<<7) else 'OFF'}")
    except ValueError:
        print(f"  (Could not parse statusword: '{sw_raw}')")

def main():
    print("=" * 55)
    print("  Parker Compax3 — Drive Enable Tool")
    print("=" * 55)
    print(f"\nOpening {PORT} at {BAUDRATE} baud...")

    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"\n[ERROR] Cannot open serial port: {e}")
        sys.exit(1)

    time.sleep(0.5)

    # ── 1. Basic comms test ────────────────────────────────────────────────────
    print("\n[1/5] Testing communication...")
    resp = read_obj(ser, '604.1')
    if resp == '':
        print("  [FAIL] No response from drive.")
        print("  Check: USB adapter plugged in? RS232 cable connected?")
        print("         Baud rate set to 115200 in C3 ServoManager?")
        ser.close()
        sys.exit(1)
    print(f"  [OK ] Drive responded: '{resp}'")

    # ── 2. Check for existing faults ──────────────────────────────────────────
    print("\n[2/5] Checking for faults...")
    print_status(ser)

    err = read_obj(ser, '604.1')
    if err != '0':
        print(f"\n  [WARN] Error code {err} detected. Attempting fault reset...")
        write_obj(ser, '300.6', '1')  # Fault reset
        time.sleep(0.5)
        write_obj(ser, '300.6', '0')
        time.sleep(0.3)

    # ── 3. STO / hardware enable check hint ───────────────────────────────────
    print("\n[3/5] Hardware STO Check (Manual):")
    print("  ┌─────────────────────────────────────────────────────┐")
    print("  │ The drive will NOT enable if STO pins are open.     │")
    print("  │ On the Compax3, locate connector X12:               │")
    print("  │   Pin 1 (+24V) → Pin 2 (STO1)  must be connected   │")
    print("  │   Pin 3 (+24V) → Pin 4 (STO2)  must be connected   │")
    print("  │ If you have no E-Stop yet: jumper Pin1→Pin2         │")
    print("  │ and jumper Pin3→Pin4 with a wire.                   │")
    print("  └─────────────────────────────────────────────────────┘")
    input("\n  Press ENTER when STO pins are wired (or skip)...")

    # ── 4. CiA 402 enable sequence ────────────────────────────────────────────
    print("\n[4/5] Running CiA 402 Enable Sequence...")

    # Step A: Shutdown (controlword = 6)
    print("  → Sending Shutdown (CW=6)...")
    write_obj(ser, '301.1', '6', delay=0.2)
    time.sleep(0.3)

    # Step B: Switch On (controlword = 7)
    print("  → Sending Switch On (CW=7)...")
    write_obj(ser, '301.1', '7', delay=0.2)
    time.sleep(0.3)

    # Step C: Enable Operation (controlword = 15)
    print("  → Sending Enable Operation (CW=15)...")
    write_obj(ser, '301.1', '15', delay=0.2)
    time.sleep(0.3)

    # Step D: Drive Enable object
    print("  → Setting Drive Enable (O300.1=1)...")
    write_obj(ser, '300.1', '1', delay=0.2)
    time.sleep(0.5)

    # ── 5. Final status ───────────────────────────────────────────────────────
    print("\n[5/5] Final Status:")
    print_status(ser)

    sw_raw = read_obj(ser, '700.1')
    try:
        sw = int(sw_raw)
        op_enabled = bool(sw & (1 << 2))
    except ValueError:
        op_enabled = False

    print("\n" + "=" * 55)
    if op_enabled:
        print("  ✅  Drive is OPERATION ENABLED — ready for motion!")
        print("  You can now launch: ros2 launch parker_actuator")
        print("                      parker_actuator.launch.py sim_mode:=false")
    else:
        print("  ❌  Drive did NOT enable.")
        print("\n  Most likely cause: STO pins not wired.")
        print("  See Step 3 above — jumper X12 Pin1→Pin2 and Pin3→Pin4")
        print("  Then run this script again.\n")
    print("=" * 55)

    ser.close()

if __name__ == '__main__':
    main()
