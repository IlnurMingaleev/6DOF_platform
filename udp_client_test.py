import socket
import struct
import random
import math
import time

# Server address and port (change if needed)
SERVER_IP = '127.0.0.1'
SERVER_PORT = 5000

# SCommand format (must match server)
SCOMMAND_FORMAT = '<6B h 6f 6f 6f 6f 6f 6f 3f 3f I'

# Random pose within limits (degrees/mm)
#pitch = random.uniform(-15, 15)
#roll = random.uniform(-15, 15)
#yaw = random.uniform(-30, 30)
#sway = random.uniform(-100, 100)
#surge = random.uniform(-100, 100)
#heave = random.uniform(-100, 100)

pitch = 0
roll = 0
yaw = 0
sway = 0
surge = 0
heave = 0

# Reasonable speed (deg/s for rotation, mm/s for translation)
spd = [10, 10, 10, 50, 50, 50]

def deg2rad(deg):
    return deg * math.pi / 180.0

dofs = [0.0, 0.0, 0.0, deg2rad(roll), deg2rad(pitch), deg2rad(yaw)]
scommand_data = struct.pack(
    SCOMMAND_FORMAT,
    55,   # ID (must be 55)
    11,   # Cmd (command execution)
    1,    # SubCmd (single-step)
    0,    # FileNum
    0,    # CyChoose
    0,    # DO
    0,    # JogSpeed
    *([0.0]*6),  # DOFs (not used for single-step)
    *([0.0]*6),  # Amp
    *([0.0]*6),  # Fre
    *([0.0]*6),  # Pha
    *( [pitch, roll, yaw, sway, surge, heave] ),  # Pos (target pose)
    *spd,        # Spd (speed)
    *([0.0]*3),  # Rev1
    *([0.0]*3),  # Rev2
    int(time.time())  # Time
)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(3.0)
try:
    sock.sendto(scommand_data, (SERVER_IP, SERVER_PORT))
    print(f"Sent single-step command: pitch={pitch:.2f}, roll={roll:.2f}, yaw={yaw:.2f}, sway={sway:.2f}, surge={surge:.2f}, heave={heave:.2f}")
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