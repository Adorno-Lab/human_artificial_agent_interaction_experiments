#!/usr/bin/env python2.7
import sys
import socket

from naoqi import ALProxy, ALModule, ALBroker

# Global variable needed when using ALBroker
rs = None


class RobotServer(ALModule):
    def __init__(self, name):
        ALModule.__init__(self, name)

        self.memory_proxy = ALProxy("ALMemory")
        self.leds_proxy = ALProxy("ALLeds")
        self.motion_proxy = ALProxy("ALMotion")
        self.robot_posture_proxy = ALProxy("ALRobotPosture")
        self.autonomous_life_proxy = ALProxy("ALAutonomousLife")
        self.face_tracker_proxy = ALProxy("ALFaceTracker")
        self.audio_device_proxy= ALProxy("ALAudioDevice")

    def check_speech(self):
        """
        Check if robot is speaking or not.

        :return: message to send to the client
        """

        # Checking speaking status
        robot_talking = self.memory_proxy.getData(
            "ALTextToSpeech/TextStarted")
        current_sentence = self.memory_proxy.getData(
            "ALTextToSpeech/CurrentSentence")

        # Writing message to send back to the client
        message = str(robot_talking) + " " + current_sentence

        return message

    def expressions(self, request):
        """
        Execute commands related to the robot facial expressions.

        Request string should be one of the options below:
        - "expression groups": to create the necessary LED groups
        - "expression off": to turn off the face LEDs
        - "expression <expression_name> <duration>: to execute expression

        :param request: a String with the expression command
        """
        if "groups" in request.lower():
            # Creating groups for the available facial expressions

            # LED group for sad expression
            sad_leds = ["FaceLedLeft1", "FaceLedLeft2",
                        "FaceLedLeft3", "FaceLedLeft4",
                        "FaceLedRight1", "FaceLedRight2",
                        "FaceLedRight3", "FaceLedRight4"]
            self.leds_proxy.createGroup("sad_leds", sad_leds)

            # LED groups for happy expression
            happy_leds1 = ["FaceLedLeft0", "FaceLedLeft1",
                           "FaceLedRight0", "FaceLedRight1"]
            self.leds_proxy.createGroup("happy_leds1", happy_leds1)

            happy_leds2 = ["FaceLedLeft2", "FaceLedLeft3",
                           "FaceLedRight2", "FaceLedRight3"]
            self.leds_proxy.createGroup("happy_leds2", happy_leds2)

            happy_leds3 = ["FaceLedLeft4", "FaceLedLeft5",
                           "FaceLedRight4", "FaceLedRight5"]
            self.leds_proxy.createGroup("happy_leds3", happy_leds3)

            happy_leds4 = ["FaceLedLeft6", "FaceLedLeft7",
                           "FaceLedRight6", "FaceLedRight7"]
            self.leds_proxy.createGroup("happy_leds4", happy_leds4)

        elif "off" in request.lower():
            # Turning all eyes leds off
            self.leds_proxy.off("FaceLeds")
        else:
            # Executing facial expression
            command, name, duration = request.split(" ")

            # Turning all eyes leds off
            self.leds_proxy.off("FaceLeds")

            if name.lower() == "happy":
                # Setting the colors for the happy expression
                happy_color1 = 0x00ff00ff  # magenta, decimal: 16711935
                happy_color2 = 0x00969600  # yellow, decimal: 9868800
                happy_color3 = 0x0000ff00  # green, decimal: 65280
                happy_color4 = 0x00ff7800  # orange, decimal: 16742400

                # Duration of the expression and number of steps
                total_duration = float(duration)
                happy_time = 0.21
                steps = int(total_duration/(happy_time*4))

                # Creating the colors and times arrays
                colors1 = []
                colors2 = []
                colors3 = []
                colors4 = []
                times = []
                for i in range(0, steps):
                    colors1.append(happy_color1)
                    colors1.append(happy_color2)
                    colors1.append(happy_color3)
                    colors1.append(happy_color4)

                    colors2.append(happy_color4)
                    colors2.append(happy_color1)
                    colors2.append(happy_color2)
                    colors2.append(happy_color3)

                    colors3.append(happy_color3)
                    colors3.append(happy_color4)
                    colors3.append(happy_color1)
                    colors3.append(happy_color2)

                    colors4.append(happy_color2)
                    colors4.append(happy_color3)
                    colors4.append(happy_color4)
                    colors4.append(happy_color1)

                    times.append(happy_time)
                    times.append(happy_time)
                    times.append(happy_time)
                    times.append(happy_time)

                # Executing the pattern
                self.leds_proxy.post.fadeListRGB("happy_leds1", colors1, times)
                self.leds_proxy.post.fadeListRGB("happy_leds2", colors2, times)
                self.leds_proxy.post.fadeListRGB("happy_leds3", colors3, times)
                self.leds_proxy.fadeListRGB("happy_leds4", colors4, times)

            if name.lower() == "sad":
                # Setting the colors for the happy expression
                sad_color1 = 0x00000046  # decimal: 70
                sad_color2 = 0x00000064  # decimal: 100
                # sad_color1 = 0x000000b3  # decimal: 179
                # sad_color2 = 0x000000ff  # decimal: 255
                # sad_color1 = 0x00000000  # decimal: 0
                # sad_color2 = 0x000000ff  # decimal: 255

                # Duration of the expression and number of steps
                total_duration = float(duration)
                sad_time = 0.3
                steps = int(total_duration/(sad_time*2))

                # Creating the colors and times arrays
                colors = []
                times = []
                for i in range(0, steps):
                    colors.append(sad_color1)
                    times.append(sad_time)
                    colors.append(sad_color2)
                    times.append(sad_time)

                # Executing the pattern
                self.leds_proxy.fadeListRGB("sad_leds", colors, times)

            # Turning all eyes leds off
            self.leds_proxy.off("FaceLeds")

        return "end"

    def motion(self, request):
        """
        Deal with commands to the ALMotion module.

        Request string should be "Motion <method> <parameters>".

        :param request: a String with the command
        """
        if "wakeup" in request.lower():
            # Waking up robot
            self.motion_proxy.wakeUp()

        if "rest" in request.lower():
            # Putting robot to rest
            self.motion_proxy.rest()

        return "end"

    def robot_posture(self, request):
        """
        Deal with commands to the ALRobotPosture module.

        Request string should be "RobotPosture <method> <parameters>".

        :param request: a String with the command
        """
        if "gotoposture" in request.lower():
            # Sending robot to one of the predefined postures
            module, method, name, time = request.split(" ")
            self.robot_posture_proxy.goToPosture(name, float(time))

        return "end"

    def autonomous_life(self, request):
        """
        Deal with commands to the ALAutonomousLife module.

        Request string should be "AutonomousLife <method> <parameters>".

        :param request: a String with the command
        """
        if "getstate" in request.lower():
            # Getting current state of the Autonomous Life module
            message = self.autonomous_life_proxy.getState()

        if "setstate" in request.lower():
            # Setting state of the Autonomous Life module
            command, method, parameter = request.split(" ")
            self.autonomous_life_proxy.setState(parameter)
            message = "end"

        return message

    def face_tracker(self, request):
        """
        Deal with commands to the ALFaceTracker module.

        Request string should be "FaceTracker <method> <parameters>".

        :param request: a String with the command
        """
        if "start" in request.lower():
            # Starting face tracker
            while not self.face_tracker_proxy.isActive():
                self.face_tracker_proxy.startTracker()

        if "stop" in request.lower():
            # Stopping face tracker
            while self.face_tracker_proxy.isActive():
                self.face_tracker_proxy.stopTracker()

        return "end"

    def audio_device(self, request):
        """
        Deal with commands to the ALAudioDevice module.

        Request string should be "ALAudioDevice <method> <parameters>".

        :param request: a String with the command
        """
        if "setoutputvolume" in request.lower():
            command, method, parameter = request.split(" ")
            self.audio_device_proxy.setOutputVolume(int(parameter))

        return "end"

    def leds(self, request):
        """
        Deal with commands to the ALLeds module.

        Request string should be "ALLeds <method> <parameters>".

        :param request: a String with the command
        """
        if "off" in request.lower():
            command, method, parameter = request.split(" ")
            self.leds_proxy.off(parameter)
        if "on" in request.lower():
            command, method, parameter = request.split(" ")
            self.leds_proxy.on(parameter)

        return "end"


def main():

    # Keep trying until it connects with the robot
    while True:
        try:
            print("Connecting with robot at " + sys.argv[1])
            my_broker = ALBroker("myBroker", "0.0.0.0", 0, sys.argv[1], 9559)
        except:
            continue
        else:
            break

    global rs
    rs = RobotServer("rs")

    # Creating server
    socket_obj = socket.socket()
    socket_obj.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    socket_obj.bind(("", 1111))
    socket_obj.listen(100)

    while True:
        try:
            # Accepting and reading client request
            c, addr = socket_obj.accept()
            request = c.recv(64)
            message = None

            if "speech" in request.lower():
                # Check if robot is speaking or not
                message = rs.check_speech()
            if "expression" in request.lower():
                # Facial expression command
                message = rs.expressions(request)
            if "motion" in request.lower():
                # Command to ALMotion module
                message = rs.motion(request)
            if "robotposture" in request.lower():
                # Command to ALRobotPosture module
                message = rs.robot_posture(request)
            if "autonomouslife" in request.lower():
                # Command to ALAutonomousLife module
                message = rs.autonomous_life(request)
            if "facetracker" in request.lower():
                # Command to ALFaceTracker module
                message = rs.face_tracker(request)
            if "audiodevice" in request.lower():
                # Command to ALAudioDevice module
                message = rs.audio_device(request)
            if "leds" in request.lower():
                # Command to ALLeds module
                message = rs.leds(request)

            # Sending message to client
            c.send(message.encode())
            c.close()

        except KeyboardInterrupt:
            socket_obj.close()
            socket_obj.shutdown()
            my_broker.shutdown()
            break
        except Exception:
            pass

    socket_obj.close()
    socket_obj.shutdown()
    my_broker.shutdown()


if __name__ == "__main__":
    main()