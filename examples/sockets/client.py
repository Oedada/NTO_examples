import socket
import time
import json

HOST = "192.168.31.77"
PORT = 10101

sock = None

def socket_start():
    global sock
    while True:
        try:
            sock = socket.socket()
            sock.connect((HOST, PORT))
            break
        except ConnectionRefusedError:
            print("connection error")
            time.sleep(2)

def tx(data):
    print(f"sent: {data}")
    global sock
    sock.send(data.encode("utf-8"))

def rx():
    data = sock.recv(8192)
    decoded = data.decode("utf-8")
    print(f"received: {decoded}")
    return decoded

json_data = {
        'byte': 256,
        'hello': "hello"
}

def main():
    socket_start()

    tx(json.dumps(json_data))
    rx()

main()
