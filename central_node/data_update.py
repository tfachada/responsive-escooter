import os
import socket
import threading
import time
from watchdog.observers import Observer
from watchdog.events import PatternMatchingEventHandler

# Scooter data directory
DATA_DIR = 'scooter_data'

# Server data
FILE_TO_SEND = 'server_data.json'

# Port for receiving data
TCP_PORT = 5001

# Group and ports for updating intermediate nodes
MCAST_GRP = '239.192.0.3'
MCAST_PORT = 6000
ACK_PORT = 6001

# Acknowledgment tolerance
ACK_TIMEOUT = 3
MAX_RETRIES = 3

def receive_tcp_files():
    os.makedirs(DATA_DIR, exist_ok=True)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('', TCP_PORT))
        s.listen()
        print(f"[SERVER] Listening for TCP files on port {TCP_PORT}")
        while True:
            conn, addr = s.accept()
            with conn:
                filename = conn.recv(1024).decode().strip()
                filepath = os.path.join(DATA_DIR, filename)
                with open(filepath, 'wb') as f:
                    while chunk := conn.recv(4096):
                        f.write(chunk)
                print(f"[SERVER] Received {filename} from {addr}")

def multicast_server_data_with_ack():
    def on_change(event):
        if not event.is_directory:
            for attempt in range(1, MAX_RETRIES + 1):
                try:
                    # Send multicast
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
                        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
                        with open(FILE_TO_SEND, 'rb') as f:
                            sock.sendto(f.read(), (MCAST_GRP, MCAST_PORT))
                    print(f"[SERVER] Multicast attempt {attempt} sent")

                    # Wait for ACK
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ack_sock:
                        ack_sock.bind(('', ACK_PORT))
                        ack_sock.settimeout(ACK_TIMEOUT)
                        ack_data, addr = ack_sock.recvfrom(1024)
                        if ack_data == b"ACK":
                            print(f"[SERVER] ACK received from {addr}")
                            break
                except socket.timeout:
                    print(f"[SERVER] No ACK received (Attempt {attempt})")
                except Exception as e:
                    print(f"[SERVER] Error: {e}")

    observer = Observer()
    handler = PatternMatchingEventHandler(patterns=[FILE_TO_SEND], ignore_directories=True)
    handler.on_modified = on_change
    handler.on_created = on_change
    observer.schedule(handler, path='.', recursive=False)
    observer.start()
    print(f"[SERVER] Watching {FILE_TO_SEND}")
    try:
        while True:
            time.sleep(1)
    finally:
        observer.stop()
        observer.join()

if __name__ == '__main__':
    try:
        t1 = threading.Thread(target=receive_tcp_files, daemon=True)
        t2 = threading.Thread(target=multicast_server_data_with_ack, daemon=True)
        t1.start()
        t2.start()
        print("[SERVER STARTED]")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[SERVER] Exiting...")
