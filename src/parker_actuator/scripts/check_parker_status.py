#!/usr/bin/env python3
import serial
import sys
import time

def get_error_desc(code):
    errors = {
        "0": "No Error",
        "0x2211": "Undervoltage control voltage",
        "0x2212": "Overvoltage control voltage",
        "0x3210": "DC link overvoltage",
        "0x3221": "DC link undervoltage",
        "0x5111": "Supply phase missing",
        "0x5410": "Output stage current (Overcurrent)",
        "0x6281": "PLC Timeout",
        "0x7121": "Motor stalled",
        "0x7320": "Resolver error",
        "0x8120": "RS232/RS485 Communication Error",
    }
    return errors.get(code, "Unknown Error (Check Manual)")

def decode_statusword(val_str):
    try:
        val = int(val_str)
        bits = {
            0: ("Ready to switch on", (val >> 0) & 1),
            1: ("Switched on", (val >> 1) & 1),
            2: ("Operation enabled", (val >> 2) & 1),
            3: ("Fault", (val >> 3) & 1),
            4: ("Voltage enabled", (val >> 4) & 1),
            5: ("Quick stop (Active Low)", (val >> 5) & 1),
            6: ("Switch on inhibited", (val >> 6) & 1),
            7: ("Warning", (val >> 7) & 1),
        }
        return bits
    except:
        return {}

def main():
    port = '/dev/ttyUSB0'
    baud = 115200
    
    if len(sys.argv) > 1:
        port = sys.argv[1]

    print(f"Connecting to Parker Compax3 on {port}...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except Exception as e:
        print(f"Error: Could not open port {port}. {e}")
        return

    def send_query(cmd):
        ser.write(f"{cmd}\r".encode())
        resp = ser.read_until(b'\r').decode().strip()
        return resp

    print("-" * 40)
    
    # Read Status
    status = send_query("O190.1")
    last_err = send_query("O190.2")
    sub_code = send_query("O190.4")
    statusword = send_query("O150.1")
    
    # Read Enable State
    enabled = send_query("O300.1")
    
    print(f"Current Error Code: {status}")
    print(f"Description:        {get_error_desc(status)}")
    print(f"Statusword:         {statusword}")
    
    sw_bits = decode_statusword(statusword)
    for bit, (name, state) in sw_bits.items():
        print(f"  Bit {bit}: {name:<25} {'[ON]' if state else '[OFF]'}")

    print("-" * 40)
    print(f"Drive Enabled (O300.1): {'Yes' if enabled == '1' else 'No'}")
    print("-" * 40)

    if status != "0":
        print("To clear the error, type 'QT' and press Enter, or use the 'Quit' command.")
        cmd = input("Send command (e.g. QT to reset, enter to exit): ").strip().upper()
        if cmd:
            ser.write(f"{cmd}\r".encode())
            print(f"Sent: {cmd}")
            time.sleep(0.5)
            new_status = send_query("O190.1")
            print(f"New Status: {new_status}")

    ser.close()

if __name__ == "__main__":
    main()
