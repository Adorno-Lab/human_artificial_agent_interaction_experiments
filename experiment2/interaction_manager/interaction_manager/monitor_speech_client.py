import socket
import time


count = 0
while True:
    try:
        s = socket.socket()
        s.connect(('127.0.0.1', 12345))
        s.send(bytes('hello ' + str(count), 'utf-8'))
        print(str(s.recv(1024).decode('utf-8')))
        s.close()
        count = count + 1
        time.sleep(2)
    except KeyboardInterrupt:
        break