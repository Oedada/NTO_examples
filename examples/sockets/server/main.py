import alt_server as serv
import json

random_json = {
        "hello": "helloooo",
        "spisok": [20, 3234, 22],
        "int": 34
}

def main():
    try:
        serv.socket_start("", 10101)
        print("received:", serv.rx())
        serv.tx_json(random_json)
    except Exception as e:
        print("error:", e)
    finally:
        serv.socket_close()

if __name__ == "__main__":
    main()
