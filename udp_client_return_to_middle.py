import socket
import struct
import time

SERVER_IP = '127.0.0.1'
SERVER_PORT = 5000

SCOMMAND_FORMAT = '<6B h 6f 6f 6f 6f 6f 6f 3f 3f I'

scommand_data = struct.pack(
    SCOMMAND_FORMAT,
    55,   # ID (must be 55)
    2,    # Cmd (Back to the middle)
    0,    # SubCmd
    0,    # FileNum
    0,    # CyChoose
    0,    # DO
    0,    # JogSpeed
    *([0.0]*6),  # DOFs
    *([0.0]*6),  # Amp
    *([0.0]*6),  # Fre
    *([0.0]*6),  # Pha
    *([0.0]*6),  # Pos
    *([0.0]*6),  # Spd
    *([0.0]*3),  # Rev1
    *([0.0]*3),  # Rev2
    int(time.time())  # Time
)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(3.0)
try:
    sock.sendto(scommand_data, (SERVER_IP, SERVER_PORT))
    print(f"Sent 'return to middle' command to {SERVER_IP}:{SERVER_PORT}")
    try:
        data, addr = sock.recvfrom(1024)
        print(f"Received response from {addr} (length {len(data)} bytes)")
        NETSEND_FORMAT = '<4B 6f 6f 6f 6f I I'
        if len(data) == struct.calcsize(NETSEND_FORMAT):
            unpacked = struct.unpack(NETSEND_FORMAT, data)
            print("Response fields:")
            print(f"  ID: {unpacked[0]}")
            print(f"  DOFStatus: {unpacked[1]}")
            print(f"  DI: {unpacked[2]}")
            print(f"  Rev1: {unpacked[3]}")
            print(f"  Attitudes: {unpacked[4:10]}")
            print(f"  ErrorCode: {unpacked[10:16]}")
            print(f"  MotorCode: {unpacked[16:22]}")
            print(f"  Tor: {unpacked[22:28]}")
            print(f"  Version: {unpacked[28]}")
            print(f"  Time: {unpacked[29]}")
        else:
            print("Response does not match expected NETSEND format.")
    except socket.timeout:
        print("No response received (timeout)")
finally:
    sock.close() 