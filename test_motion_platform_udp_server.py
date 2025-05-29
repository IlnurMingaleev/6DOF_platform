import unittest
import socket
import struct
import threading
import time

from motion_platform_udp_server import udp_server, SCOMMAND_FORMAT, SCOMMAND_SIZE, NETSEND_FORMAT, NETSEND_SIZE

SERVER_IP = '127.0.0.1'
SERVER_PORT = 6000  # Use a test port

class TestMotionPlatformUDPServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Start the server in a background thread
        cls.server_thread = threading.Thread(
            target=udp_server, args=(SERVER_IP, SERVER_PORT), daemon=True
        )
        cls.server_thread.start()
        time.sleep(1)  # Give the server time to start

    def test_single_step_command(self):
        # Prepare a single-step SCommand packet
        ID = 55
        Cmd = 11  # Command execution
        SubCmd = 1  # Single-step
        FileNum = 0
        CyChoose = 0
        DO = 0
        JogSpeed = 0
        DOFs = [0.0]*6
        Amp = [0.0]*6
        Fre = [0.0]*6
        Pha = [0.0]*6
        # Set a test pose: pitch=5, roll=2, yaw=1, sway=10, surge=20, heave=30
        Pos = [5.0, 2.0, 1.0, 10.0, 20.0, 30.0]
        Spd = [0.0]*6
        Rev1 = [0.0]*3
        Rev2 = [0.0]*3
        TimeVal = int(time.time())
        packet = struct.pack(
            SCOMMAND_FORMAT, ID, Cmd, SubCmd, FileNum, CyChoose, DO, JogSpeed,
            *DOFs, *Amp, *Fre, *Pha, *Pos, *Spd, *Rev1, *Rev2, TimeVal
        )

        # Send the packet to the server
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2)
        sock.sendto(packet, (SERVER_IP, SERVER_PORT))

        # Receive the response
        data, _ = sock.recvfrom(1024)
        self.assertEqual(len(data), struct.calcsize(NETSEND_FORMAT))

        # Unpack the response
        unpacked = struct.unpack(NETSEND_FORMAT, data)
        # The pose should match the command
        pose = unpacked[4:10]  # [pitch, roll, yaw, sway, surge, heave]
        self.assertAlmostEqual(pose[0], 5.0, places=1)
        self.assertAlmostEqual(pose[1], 2.0, places=1)
        self.assertAlmostEqual(pose[2], 1.0, places=1)
        self.assertAlmostEqual(pose[3], 10.0, places=1)
        self.assertAlmostEqual(pose[4], 20.0, places=1)
        self.assertAlmostEqual(pose[5], 30.0, places=1)

        sock.close()

if __name__ == '__main__':
    unittest.main() 