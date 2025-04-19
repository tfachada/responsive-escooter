import socket
import struct
import sys
import os
import time


#RECV_OUTPUT_DIR = './'
#RECV_FILENAME = 'received_file2.txt'

#MCAST_GROUP_A = '239.192.0.1'  # Scooter to Intermediate Node
#MCAST_GROUP_B = '239.192.0.2'  # Intermediate Node to Scooter
#MCAST_PORT = 5000

CHUNK_SIZE = 1024  # bytes per packet

def multicast_sender(send_file, mcast_group, mcast_port):

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    ttl = struct.pack('b', 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

    if not os.path.exists(send_file):
        print(f"Error: File {send_file} does not exist!")
        exit

    while True:
        with open(send_file, 'rb') as f:
            # Calculate file size
            f.seek(0, os.SEEK_END)
            filesize = f.tell()
            f.seek(0)
            print(f"[Sender] Sending file '{send_file}' of size {filesize} bytes on group {mcast_group}")

            # Send header message
            header = f"FILE_TRANSFER::{os.path.basename(send_file)}::{filesize}".encode('utf-8')
            sock.sendto(header, (mcast_group, mcast_port))
            
            # Pause briefly to let receivers process header
            time.sleep(0.2)
            
            # Send file data in chunks
            while True:
                chunk = f.read(CHUNK_SIZE)
                if not chunk:
                    break
                sock.sendto(chunk, (mcast_group, mcast_port))
        print(f"[Sender] Finished sending {send_file}")
        time.sleep(5)

def multicast_receiver(output_dir, mcast_group, mcast_port):

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    # Allow multiple sockets on same UDP port.
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('', mcast_port))
    except Exception as e:
        print(f"[Receiver] Bind failed: {e}")
        sys.exit(1)

    # Join the multicast group on all interfaces.
    mreq = struct.pack("4sl", socket.inet_aton(mcast_group), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    current_file = None
    file_data = b''
    filesize = None
    received = 0

    print(f"[Receiver] Listening on multicast group {mcast_group}:{mcast_port} for incoming file...")

    while True:
        data, addr = sock.recvfrom(2048)
        if data.startswith(b"FILE_TRANSFER::"):
            try:
                header_parts = data.decode('utf-8').split("::")

                if len(header_parts) >= 3:

                    _, filename, filesize_str = header_parts
                    filesize = int(filesize_str)

                    current_file = os.path.join(output_dir, filename)
                    file_data = b''
                    received = 0
                    print(f"[Receiver] Starting to receive file '{filename}' (size {filesize} bytes) from {addr}")

                    continue

            except Exception as err:
                print(f"[Receiver] Header parse error: {err}")
                continue

        if current_file is not None:
            file_data += data
            received += len(data)
            print(f"[Receiver] Received {received} / {filesize if filesize else '?'} bytes...", end='\r')
            if filesize and received >= filesize:
                try:
                    with open(current_file, 'wb') as f:
                        f.write(file_data)
                    print(f"\n[Receiver] File saved as {current_file}")
                except Exception as err:
                    print(f"\n[Receiver] Error saving file: {err}")
                current_file = None
                file_data = b''
                filesize = None
                received = 0

        time.sleep(5)
