import socket
import time

ip = '192.168.0.103'
port = 1234

s = socket.socket()
s.connect((ip, port))
s.send(bytes('start audio video', 'utf-8'))
print(str(s.recv(1024).decode('utf-8')))
s.close()

start = time.time()
count = 1
while time.time() - start < 3:
    time.sleep(1)
    count = count + 1
    print(count)

s = socket.socket()
s.connect((ip, port))
s.send(bytes('stop', 'utf-8'))
print(str(s.recv(1024).decode('utf-8')))
s.close()