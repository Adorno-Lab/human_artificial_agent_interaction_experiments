#!/usr/bin/env python2.7
from naoqi import ALProxy, ALModule, ALBroker
import socket


def main():

    memory_service = ALProxy("ALMemory", "192.168.0.100", 9559)

    socket_obj = socket.socket()
    socket_obj.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    socket_obj.bind(("", 12345))
    socket_obj.listen(5)

    while True:
        try:
            robot_talking = memory_service.getData("ALTextToSpeech/TextStarted")
            current_sentence = memory_service.getData("ALTextToSpeech/CurrentSentence")
            c, addr = socket_obj.accept()
            message = str(robot_talking) + " " + current_sentence
            c.send(message.encode())
            c.close()
        except KeyboardInterrupt:
            socket_obj.close()
            socket_obj.shutdown()
            break
        except Exception:
            pass


if __name__ == "__main__":
    main()