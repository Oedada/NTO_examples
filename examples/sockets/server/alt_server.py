import socket
import json

client = None
sock = None


def socket_start(HOST, PORT): 
    global client, sock

    sock = socket.socket()
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((HOST, PORT))
    sock.listen()

    client, addr = sock.accept()
    print(f"{addr} connected\n")

def socket_close():
    global client
    global sock

    if client:
        client.close()
    if sock:
        sock.close()
    
def rx():
    global client
    data = client.recv(8192)
    return data.decode("utf-8")

def tx(data):
    global client
    client.sendall(data.encode("utf-8"))

def rx_json():
    json_data = rx()
    return json.loads(json_data)

def tx_json(python_data):
    tx(json.dumps(python_data))
