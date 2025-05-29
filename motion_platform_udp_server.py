import socket
import struct
import threading
import time
import math
import logging
import argparse
import sys
from StewartPlatform import StewartPlatform
import numpy as np

# Data structure formats (little-endian, packed)
# SCommand_Typedef: see documentation for field order and types
SCOMMAND_FORMAT = '<6B h 6f 6f 6f 6f 6f 6f 3f 3f I'
# Fields: ID, Cmd, SubCmd, FileNum, CyChoose, DO, JogSpeed, DOFs[6], Amp[6], Fre[6], Pha[6], Pos[6], Spd[6], Rev1[3], Rev2[3], Time
SCOMMAND_SIZE = struct.calcsize(SCOMMAND_FORMAT)

# NetSend_Typedef: see documentation for field order and types
NETSEND_FORMAT = '<4B 6f 6f 6f 6f I I'
# Fields: ID, DOFStatus, DI, Rev1, Attitudes[6], ErrorCode[6], MotorCode[6], Tor[6], Version, Time
NETSEND_SIZE = struct.calcsize(NETSEND_FORMAT)

# Default IP/port (from documentation)
CONTROLLER_IP = '0.0.0.0'  # Listen on all interfaces
CONTROLLER_PORT = 5000

# State codes (from documentation)
STATUS_CODES = {
    0: 'Bottom-finding initialization in progress',
    1: 'Bottom-finding initialization completed',
    2: 'In the process of rising from the bottom to the middle',
    3: 'Complete from the bottom to the middle',
    4: 'In follow-up execution',
    5: 'In command execution',
    8: 'In script execution',
    9: 'Script run completed',
    12: 'In the process of falling from the middle to the bottom',
    13: 'Complete from the middle to the bottom',
    14: 'In the process of returning to the middle',
    32: 'Manual mode',
    33: 'Emergency stops',
    55: 'System initialization completed',
}

# Command codes (from documentation)
CMD_CODES = {
    2: 'Back to the middle',
    3: 'Emergency stops',
    4: 'Finding the bottom initialization',
    5: 'Manual mode',
    6: 'From the bottom to the middle',
    7: 'From the middle to the bottom',
    9: 'Follow-up execution',
    11: 'Command execution',
    13: 'Script execution',
}

class PlatformState:
    def __init__(self):
        # === FILL THESE IN WITH YOUR ACTUAL VALUES ===
        urdf_path = "Stewart/Stewart.urdf"  # Example path
        joint_indices = [(6, 16), (35, 17), (49, 18), (42, 19), (28, 20)]

# Define the actuators for each pair of linked joints
        actuator_indices = [9, 2, 31, 45, 38, 24]     #[1,0,3,5,4,2][9, 2, 31, 45, 38, 24] 
                                        # [0: 2, 1: 31, 2: 45, 3: 38, 4: 24, 5: 9]

        # Define the stewart platform design variables
        radious_platform, radious_base = 0.2, 0.2         # meters
        half_angle_platform, half_angle_base = 24/2, 24/2 # degrees
        design_variable = [radious_platform, radious_base, half_angle_platform, half_angle_base]
        #joint_indices = [(0, 1), (2, 3), (4, 5)]  # Example, replace with your pairs
        #actuator_indices = [0, 1, 2, 3, 4, 5]     # Example, replace with your indices
        #design_variable = (0.1, 0.1, 30, 30)      # Example, replace with your values
        # =============================================
        self.platform = StewartPlatform(
            path=urdf_path,
            joint_indices=joint_indices,
            actuator_indices=actuator_indices,
            design_variable=design_variable
        )
        self.status = 55
        self.error_code = [0.0] * 6
        self.motor_code = [0.0] * 6
        self.tor = [0.0] * 6
        self.version = 1
        self.time = int(time.time())
        self.last_cmd = None
        self.last_update = time.time()
        self.sine_params = None
        self.script_data = None
        self.attitudes = [0.0] * 6

    def update(self):
        # Always update attitudes from the simulation
        self.attitudes = self.platform.get_pose()
        self.time = int(time.time())

    def handle_command(self, cmd):
        self.last_cmd = cmd
        self.sine_params = None
        if cmd['Cmd'] == 2:  # Back to the middle
            self.status = 14
            self.platform.reset_position()
        elif cmd['Cmd'] == 3:  # Emergency stops
            self.status = 33
            # Optionally: self.platform.reset_position()
        elif cmd['Cmd'] == 4:  # Finding the bottom
            self.status = 0
        elif cmd['Cmd'] == 5:  # Manual mode
            self.status = 32
        elif cmd['Cmd'] == 6:  # From the bottom to the middle
            self.status = 2
        elif cmd['Cmd'] == 7:  # From the middle to the bottom
            self.status = 12
        elif cmd['Cmd'] == 9:  # Follow-up execution
            self.status = 4
            # Move to the requested pose (DOFs)
            pose = list(cmd['DOFs'])
            self.platform.set_pose(*pose, duration=1.0)
        elif cmd['Cmd'] == 11:
            if cmd['SubCmd'] == 1:  # Single-step
                self.status = 5
                pose = list(cmd['Pos'])
                self.platform.set_pose(*pose, duration=1.0)
            elif cmd['SubCmd'] == 2:  # Sine wave
                self.status = 5
                amplitudes = list(cmd['Amp'])
                frequencies = list(cmd['Fre'])
                phases = list(cmd['Pha'])
                self.platform.run_sine(amplitudes, frequencies, phases, duration=5.0)
            else:
                self.status = 5
        elif cmd['Cmd'] == 13:  # Script execution
            self.status = 8
            # Not implemented: script file execution
        else:
            self.status = 55
        # Always update attitudes after command
        self.attitudes = self.platform.get_pose()

    def to_netsend(self):
        return struct.pack(
            NETSEND_FORMAT,
            55,
            self.status,
            0,
            0,
            *self.attitudes,
            *self.error_code,
            *self.motor_code,
            *self.tor,
            self.version,
            int(time.time())
        )

def parse_scommand(data):
    if len(data) != SCOMMAND_SIZE:
        raise ValueError(f'Invalid SCommand size: {len(data)}')
    unpacked = struct.unpack(SCOMMAND_FORMAT, data)
    return {
        'ID': unpacked[0],
        'Cmd': unpacked[1],
        'SubCmd': unpacked[2],
        'FileNum': unpacked[3],
        'CyChoose': unpacked[4],
        'DO': unpacked[5],
        'JogSpeed': unpacked[6],
        'DOFs': unpacked[7:13],
        'Amp': unpacked[13:19],
        'Fre': unpacked[19:25],
        'Pha': unpacked[25:31],
        'Pos': unpacked[31:37],
        'Spd': unpacked[37:43],
        'Rev1': unpacked[43:46],
        'Rev2': unpacked[46:49],
        'Time': unpacked[49],
    }

def udp_server(listen_ip, listen_port):
    state = PlatformState()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((listen_ip, listen_port))
    except Exception as e:
        logging.error(f'Failed to bind UDP server to {listen_ip}:{listen_port}: {e}')
        sys.exit(1)
    logging.info(f'UDP server listening on {listen_ip}:{listen_port}')
    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                if len(data) == SCOMMAND_SIZE:
                    cmd = parse_scommand(data)
                    logging.info(f'Received command from {addr}: {cmd}')
                    state.handle_command(cmd)
                    state.update()
                    response = state.to_netsend()
                    sock.sendto(response, addr)
                    logging.info(f'Sent status to {addr}')
                else:
                    logging.warning(f'Ignored packet of size {len(data)} from {addr}')
            except Exception as e:
                logging.error(f'Error handling packet: {e}')
    except KeyboardInterrupt:
        logging.info('UDP server shutting down (Ctrl+C/KeyboardInterrupt)')
    finally:
        sock.close()
        logging.info('Socket closed')

def main():
    parser = argparse.ArgumentParser(description='6DOF Motion Platform UDP Controller Server')
    parser.add_argument('--ip', type=str, default='0.0.0.0', help='IP address to listen on (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=5000, help='UDP port to listen on (default: 5000)')
    parser.add_argument('--log', type=str, default='INFO', help='Logging level (DEBUG, INFO, WARNING, ERROR)')
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log.upper(), logging.INFO),
        format='[%(asctime)s] %(levelname)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    udp_server(args.ip, args.port)

if __name__ == '__main__':
    main() 