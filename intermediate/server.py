import os
import socket
import struct
import threading
import time
from watchdog.observers import Observer
from watchdog.events import PatternMatchingEventHandler

# Scooter data directory
DATA_DIR = 'scooter_data'

# Server data
RECEIVED_FILE = 'server_data.json'

# Server IP and port
HOST = '127.0.0.1'
TCP_PORT = 5001

# Group and ports for receiving updates
MCAST_GRP = '239.192.0.3'
MCAST_PORT = 6000
ACK_PORT = 6001


def send_files_to_server():
    def on_change(event):
        if not event.is_directory:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((HOST, TCP_PORT))
                    filename = os.path.basename(event.src_path).encode() + b'\n'
                    s.sendall(filename)
                    with open(event.src_path, 'rb') as f:
                        s.sendfile(f)
                print(f"[INTERMEDIATE] Sent file to Server: {event.src_path}")
            except Exception as e:
                print(f"[INTERMEDIATE] Error sending file: {e}")

    observer = Observer()
    handler = PatternMatchingEventHandler(patterns=["*"], ignore_directories=True)
    handler.on_modified = on_change
    handler.on_created = on_change
    observer.schedule(handler, path=DATA_DIR, recursive=False)
    observer.start()
    print(f"[INTERMEDIATE] Watching {DATA_DIR} for file changes")
    try:
        while True:
            time.sleep(1)
    finally:
        observer.stop()
        observer.join()

def receive_multicast_json():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.bind(('', MCAST_PORT))
    mreq = struct.pack('4sL', socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    print(f"[INTERMEDIATE] Listening for multicast from server on {MCAST_GRP}:{MCAST_PORT}")
    while True:
        data, addr = sock.recvfrom(65535)
        with open(RECEIVED_FILE, 'wb') as f:
            f.write(data)
        print(f"[INTERMEDIATE] Received {RECEIVED_FILE} from {addr}")

        # Send ACK back to Server
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ack_sock:
                ack_sock.sendto(b"ACK", (addr[0], ACK_PORT))
                print(f"[INTERMEDIATE] Sent ACK to Server at {addr[0]}:{ACK_PORT}")
        except Exception as e:
            print(f"[INTERMEDIATE] Failed to send ACK: {e}")

if __name__ == '__main__':
    try:
        sender_thread = threading.Thread(target=send_files_to_server, daemon=True)
        receiver_thread = threading.Thread(target=receive_multicast_json, daemon=True)
        sender_thread.start()
        receiver_thread.start()
        print("[INTERMEDIATE STARTED]")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[INTERMEDIATE] Exiting...")
