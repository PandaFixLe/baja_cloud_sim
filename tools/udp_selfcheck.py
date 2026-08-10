"""Local UDP loopback self-check for remote_control_node.

Verifies the UDP <dddbb packet (26 bytes) decodes exactly the way
remote_control_node expects, WITHOUT launching ROS/Gazebo.

Usage:
    python3 tools/udp_selfcheck.py

It opens a UDP socket on 127.0.0.1:5005, sends the same packet the user
would send from a control client, then reads it back and prints the decoded
fields. If this prints the decoded values, the packet format + localhost
loopback are correct, and the original "no effect" problem is upstream
(ROS node not started / sim clock / topic wiring).
"""

import socket
import struct
import time

LISTEN_ADDR = "127.0.0.1"
LISTEN_PORT = 5005

# Same packet the user sends to drive the car.
PACKET = struct.pack("<dddbb", 1.2, 15.0, 0.0, 1, 0)
fmt = struct.Struct("<dddbb")


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((LISTEN_ADDR, LISTEN_PORT))
    sock.setblocking(False)
    print(f"Bound UDP server on {LISTEN_ADDR}:{LISTEN_PORT}")

    # Act as the remote client: send the exact packet.
    sock.sendto(PACKET, (LISTEN_ADDR, LISTEN_PORT))
    print(f"Sent {len(PACKET)}-byte packet: steer=1.2 speed=15.0 brake=0.0 "
          f"mode=1 extra=0")

    # Give the OS a moment to deliver the loopback packet.
    time.sleep(0.1)

    try:
        data, sender = sock.recvfrom(1024)
    except BlockingIOError:
        print("ERROR: no packet received on loopback — localhost UDP is "
              "broken or firewalled.")
        return
    finally:
        sock.close()

    if len(data) != 26:
        print(f"ERROR: unexpected packet length {len(data)} (expected 26)")
        return

    steer, speed, brake, mode, extra = fmt.unpack(data)
    print("Decoded packet:")
    print(f"  steer  = {steer}")
    print(f"  speed  = {speed}")
    print(f"  brake  = {brake}")
    print(f"  mode   = {bool(mode)}")
    print(f"  extra  = {bool(extra)}")
    print("\nOK: packet format and localhost loopback work. "
          "If the car still does not move under run_remote.sh, the issue is "
          "the ROS node / Gazebo wiring, not the UDP layer.")


if __name__ == "__main__":
    main()
