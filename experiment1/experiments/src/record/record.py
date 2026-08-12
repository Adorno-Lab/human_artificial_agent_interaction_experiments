#!/usr/bin/env python3.6
import rospy
import roslib
import rospkg
from datetime import datetime
import subprocess
import signal
from std_msgs.msg import Int16
import os


class Data:
    def record_callback(self, msg):
        """
        Function to receive commands.

        If record == 1: starts the image saving.
        If record == 0: stops the image saving.
        If record == 2: edits images and creates the video.
        If record == 3: only creates the video.

        :param msg: a std_msgs/Int16 message with a command.
        :return:
        """
        self.record = msg.data

    def calculating_fps(self, lines):
        frames = []
        current = -1
        for l in lines:
            last_char = len(l) - 1
            sec = int(l[(last_char - 2):last_char])
            if sec != current:
                current = sec
                frames.append(1)
            else:
                frames[len(frames) - 1] = frames[len(frames) - 1] + 1

        fps = max(frames)

        return fps

    def create_video(self, path, camera):
        import cv2
        file_name = path + self.files + ".txt"
        file = open(file_name, "r")
        lines = file.readlines()

        fps = self.calculating_fps(lines)

        total_frames = len(lines)

        if self.record < 3:
            message = "." * 20 + " " + camera.upper() + \
                      " VIDEO: Started editing images" + " " + "." * 20
            print("\033[92m {}\033[00m".format(message))

            # Editing images to add the date and time.
            for l in lines:
                [num, dt] = l.split("\t")
                datetime = dt.split("\n")[0:20][0]

                if int(num) % 100 == 0:
                    message = "." * 20 + " " + camera.upper() + \
                              " VIDEO: Editing image " + num + "/" + str(
                        total_frames) \
                              + " " + "." * 20
                    print("\033[92m {}\033[00m".format(message))

                image_name = path + "temp_images/" + self.files + "/" + f'{int(num):06}' + ".png"
                image = cv2.imread(image_name, cv2.IMREAD_UNCHANGED)

                if image is None:
                    # If the current frame is missing, repeat the last frame and add a
                    # 'missing frame' message.
                    image_name = path + "temp_images/" + self.files + "/" + f'{int(num) - 1:06}' + ".png"
                    image = cv2.imread(image_name, cv2.IMREAD_UNCHANGED)
                    cv2.putText(image, datetime + " (missing frame)", (20, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (17, 190, 252), 2, cv2.LINE_AA)
                    image_name = path + "/temp_images/" + self.files + "/" + f'{int(num):06}' + ".png"
                else:
                    cv2.putText(image, datetime, (20, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (17, 190, 252), 2, cv2.LINE_AA)

                cv2.imwrite(image_name, image)

        message = "." * 20 + " " + camera.upper() + \
                  " VIDEO: Started creating video" + " " + "." * 20
        print("\033[92m {}\033[00m".format(message))

        # Creating video from frame images.
        files_names = path + "temp_images/" + self.files + "/" + '%06d.png'
        video = path + self.files + ".mp4"
        command = ['ffmpeg', '-framerate', str(fps), '-i', files_names, '-c:v',
                   'libx264', '-profile:v', 'high', '-crf', '20', '-pix_fmt',
                   'yuv420p', video]
        ffmpeg_process = subprocess.Popen(command)

        # Waiting for the subprocess to finish.
        while ffmpeg_process.poll() is None:
            continue

        # Sending exit signal to the subprocess.
        ffmpeg_process.send_signal(signal.SIGINT)

        message = "." * 20 + " " + camera.upper() + " VIDEO: Finished video" + \
                  " " + "." * 20
        print("\033[92m {}\033[00m".format(message))

        message = "." * 19 + " " + video.replace(path, "").replace("\n", "") + \
                  " " + "." * 19
        print("\033[92m {}\033[00m".format(message))

        file.close()

    def __init__(self):
        # Name of the frames folder and the text file.
        self.files = 0

        # The first date/time stored in the file.
        self.first_datetime = 0

        self.record = -1
        rospy.Subscriber('/record', Int16, self.record_callback)


def main():
    rospy.init_node('record_node')

    # Getting the path for the ROS package.
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('experiments')
    path = pkg_path + "/videos/"

    # Getting parameters from the launch file.
    topic = rospy.get_param("~image_topic")
    camera = "kinect" + str(rospy.get_param("~camera_number"))
    video = int(rospy.get_param("~video"))  # If video == 1, creates video.
    files = rospy.get_param("~files_param")

    # Creating the directory to store the frames if it does not exist.
    directory = path + "temp_images/" + camera
    if not os.path.exists(directory):
        os.makedirs(directory)

    d = Data()
    process = []

    while not rospy.is_shutdown():
        if d.record == 1:
            if not process:
                # Writing the command.
                # rosrun experiments image_saver '<camera_name>' '<topic_name>' '<path_to_files>'
                command = ["rosrun", "experiments", "image_saver",
                           camera, topic, path]

                # Starting the subprocess.
                process = subprocess.Popen(command)
                d.record = -1
            else:
                d.record = -1

        if d.record == 0 or d.record >= 2:
            break

    # Sending exit signal to the subprocess.
    if process:
        process.send_signal(signal.SIGINT)

    if files == "":
        # Opening the text file.
        file_name = path + camera + ".txt"
        text_file = open(file_name, "r")

        # Getting the first date/time from the file.
        lines = text_file.readlines()
        [num, datetime] = lines[0].split("\t")
        d.first_datetime = datetime
        text_file.close()

        # Rename the folder with the frames and the text file.
        dt = d.first_datetime.replace(" ", "_").replace("/", ".").split("\n")
        current = path + camera + ".txt"
        new = path + dt[0:20][0] + "_" + camera + ".txt"
        os.rename(current, new)

        # Rename the text file.
        current = path + "temp_images/" + camera
        new = path + "temp_images/" + dt[0:20][0] + "_" + camera
        os.rename(current, new)

        d.files = dt[0:20][0] + "_" + camera
    else:
        d.files = files

        # Opening the text file.
        file_name = path + d.files + ".txt"
        text_file = open(file_name, "r")

        # Getting the first date/time from the file.
        lines = text_file.readlines()
        [num, datetime] = lines[0].split("\t")
        d.first_datetime = datetime
        text_file.close()

    if video == 1 or d.record >= 2:
        d.create_video(path, camera)


if __name__ == '__main__':
    main()