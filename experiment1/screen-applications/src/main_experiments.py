#!/usr/bin/env python3.6
from Password import *
from Counting import *
import rospy
import rospkg
import time
from std_msgs.msg import Int16
from tkinter import *
from PIL import Image, ImageTk
import logging
from imp import reload
from datetime import datetime


class Data:
    def screen_callback(self, msg):
        """
        Callback function for commands.

        :param msg: an std_msgs/Int16 message.
        :return:
        """
        self.command = msg.data

    def string_to_list(self, s, separator, type):
        """
        Converts a string in a numerical list.

        :param s: a string to be converted.
        :param separator: the separator character(s) in the string.
        :param type: a string to indicate if the elements are "int" or "float".
        :return: a numerical list.
        """
        s = s[1:len(s)-1]
        s = s.split(separator)
        list = []
        for i in s:
            if type == "int":
                list.append(int(i))
            if type == "float":
                list.append(float(i))

        return list

    def read_shutdown_log(self):
        """
        Reads the log file generated on shutdown and updates the class arguments.

        :return:
        """
        log_file = open(self.pkg_path + "/log/on_shutdown.log", "r")
        lines = log_file.read().splitlines()

        self.window = self.string_to_list(lines[0], ", ", "int")

        # ----------------------- Password application ----------------------- #
        if self.window[0] == 1:
            self.password = lines[1][2:len(lines[1])-2]
            self.password = self.password.split("', '")
            self.virtual_agent = lines[2]
            self.password_state = int(lines[3])
            self.password_accepting = int(lines[4])

        # ----------------------- Counting application ----------------------- #
        if self.window[0] == 2:
            self.counting_images = int(lines[1])

            self.counting_values = lines[2]
            if self.counting_values == "-1":
                self.counting_values = int(self.counting_values)
            else:
                self.counting_values = self.string_to_list(self.counting_values,
                                                           ", ", "int")

            self.counting_fields = lines[3]
            if self.counting_fields == "-1":
                self.counting_fields = int(self.counting_fields)
            else:
                self.counting_fields = self.string_to_list(self.counting_fields,
                                                          ", ", "int")

            self.counting_state = int(lines[4])

    def shutdown(self):
        """
        Function executed on node shutdown. It writes the shutdown log file.

        :return:
        """
        # Writing shutdown log file.
        log_file = open(self.pkg_path + "/log/on_shutdown.log", "w")
        log_file.write(str(self.window) + "\n")

        # ----------------------- Password application ----------------------- #
        if self.window[0] == 1:
            log_file.write(str(self.password) + "\n")
            log_file.write(str(self.virtual_agent) + "\n")
            log_file.write(str(self.password_state) + "\n")
            log_file.write(str(self.password_accepting) + "\n")

        # ----------------------- Counting application ----------------------- #
        if self.window[0] == 2:
            log_file.write(str(self.counting_images) + "\n")
            log_file.write(str(self.counting_values) + "\n")
            log_file.write(str(self.counting_fields) + "\n")
            log_file.write(str(self.counting_state) + "\n")

        log_file.close()
        print("Log file written.")

    def __init__(self):
        # Package path.
        self.pkg_path = -1

        # Current window.
        self.window = [0, 0]

        # Images set og the counting application.
        self.counting_images = -1

        # Correct values in the counting application.
        self.counting_values = -1

        # State of the entry fields in the counting application.
        self.counting_fields = -1

        # State of the counting application.
        self.counting_state = -1

        # Virtual agent.
        self.virtual_agent = -1

        # Password.
        self.password = -1

        # State of the password application.
        self.password_state = -1

        # If the password application is accepting indications.
        self.password_accepting = -1

        # If command == -2: show blank black screen.
        # If command == -1: show/hide message about surrender pose.
        # If command == 1: start password application.
        # If command == 2: start counting application.
        # If command = 3 or command = 4: show message screen.
        # If command = 10: recover last screen with data saved on shutdown.
        self.command = 0
        rospy.Subscriber('/screen_commands', Int16, self.screen_callback)

        # Publisher for commands to the counting application.
        self.count_pub = rospy.Publisher('/counting_commands', Int16,
                                         queue_size=10)


def main():
    rospy.init_node('screen')

    d = Data()

    # Blank window.
    blank_window = Tk()
    blank_window.attributes('-fullscreen', True)
    blank_window.configure(background="#000000")
    d.window[0] = -2

    # Message windows.
    msg_window = []
    pose_window = []

    # Getting the path for the ROS package.
    rospack = rospkg.RosPack()
    d.pkg_path = rospack.get_path('screen')

    # Configuring the message to be shown after phase 2.
    screen_width = blank_window.winfo_screenwidth()
    screen_height = blank_window.winfo_screenheight()
    img_width = screen_width
    img_height = int(2179 * screen_width / 4423)
    images = [Image.open(d.pkg_path + "/src/images/mensagem0.png"),
              Image.open(d.pkg_path + "/src/images/mensagem1.png")]
    images[0] = images[0].resize((img_width, img_height), Image.ANTIALIAS)
    images[1] = images[1].resize((img_width, img_height), Image.ANTIALIAS)

    pose_msg = Image.open(d.pkg_path + "/src/images/pose.png")
    pose_msg = pose_msg.resize((img_width, img_height), Image.ANTIALIAS)

    # Creating and configuring the log file.
    log_file = reload(logging)
    log_path = d.pkg_path + "/log/"
    log_name = datetime.now().strftime("%d.%m.%Y_%H:%M:%S") + '.log'
    format_text = '%(asctime)s %(message)s'
    date_format = '%d/%m/%Y %H:%M:%S'
    log_file.basicConfig(filename=log_path + log_name, format=format_text,
                         datefmt=date_format, level=logging.INFO)

    s = []

    rospy.on_shutdown(d.shutdown)

    while not rospy.is_shutdown():
        comm = d.command

        # ------------------------- Recovering window ------------------------ #
        if comm == 10:
            log_file.info('%s', "Recovering window.")
            d.read_shutdown_log()
            try:
                if blank_window:
                    blank_window.destroy()
                    blank_window = []
                if pose_window:
                    pose_window.destroy()
                    pose_window = []
                if msg_window:
                    msg_window.destroy()
                    msg_window = []
                if s:
                    s.destroy_all()
                    del s
            except:
                log_file.info('%s', "Error when destroying windows.")
                raise

            if d.window[0] == -2:
                blank_window = Tk()
                blank_window.attributes('-fullscreen', True)
                blank_window.configure(background="#000000")
                d.command = 0
            if d.window[1] == -1:
                comm = -1
            if d.window[0] == 1:
                s = Password()
                s.set_password(d.password)
                s.set_virtual_agent(d.virtual_agent)
                s.accepting_indication = d.password_accepting
                s.fill_fields(d.password_state)
                d.command = 0
            if d.window[0] == 2:
                s = Counting()
                sett = [d.counting_images] + d.counting_values
                s.settings(sett)
                if d.counting_state > 0:
                    # Start application.
                    d.count_pub.publish(-1)
                    s.update_all()
                    if d.counting_state == 2 or 0 not in d.counting_fields:
                        # Activate main buttons
                        d.count_pub.publish(0)
                        s.update_all()
                        s.fill_entries(d.counting_fields)
                d.command = 0
            if 2 < d.window[0] <= 2 + len(images):
                msg_window = Tk()
                msg_window.attributes('-fullscreen', True)
                msg_window.configure(background="#000000")

                photo = ImageTk.PhotoImage(images[d.window[0] - 3])
                label = Label(msg_window, image=photo, background="#000000")
                label.place(x=int((screen_width - img_width) / 2),
                            y=int((screen_height - img_height) / 2),
                            width=img_width, height=img_height)
                d.command = 0

        # --------------------------- Blank window --------------------------- #
        if comm == -2:
            log_file.info('%s', "Blank window.")
            try:
                if msg_window:
                    msg_window.destroy()
                    msg_window = []
                if s:
                    s.destroy_all()
                    del s
                    s = []
                if blank_window:
                    blank_window.destroy()
            except:
                log_file.info('%s', "Error when destroying windows.")
                raise

            try:
                blank_window = Tk()
                blank_window.attributes('-fullscreen', True)
                blank_window.configure(background="#000000")
                d.command = 0
                d.window[0] = comm
            except:
                log_file.info('%s', "Error when openning blank window.")
                raise

        # --------------------------- Pose message --------------------------- #
        if comm == -1:
            if pose_window:
                # If the window already exists, close it.
                log_file.info('%s', "Surrender pose message window (close).")
                try:
                    pose_window.destroy()
                    pose_window = []
                    d.window[1] = 0
                except:
                    log_file.info('%s', "Error when destroying surrender pose message window.")
                    raise
            else:
                log_file.info('%s', "Surrender pose message window (open).")
                try:
                    pose_window = Toplevel()
                    pose_window.attributes('-fullscreen', True)
                    pose_window.configure(background="#dcdcdc")
                    pose_window.attributes("-topmost", 1)

                    photo = ImageTk.PhotoImage(pose_msg)
                    label = Label(pose_window, image=photo, background="#dcdcdc")
                    label.place(x=int((screen_width - img_width) / 2),
                                y=int((screen_height - img_height) / 2),
                                width=img_width, height=img_height)
                    d.window[1] = comm
                except:
                    log_file.info('%s', "Error when openning surrender pose message window.")
                    raise

            d.command = 0

        # ----------------------- Password application ----------------------- #
        if comm == 1:
            log_file.info('%s', "Password window.")
            try:
                if blank_window:
                    blank_window.destroy()
                    blank_window = []
                if msg_window:
                    msg_window.destroy()
                    msg_window = []
                if s:
                    s.destroy_all()
                    del s
            except:
                log_file.info('%s', "Error when destroying windows.")
                raise

            try:
                s = Password()
                d.window[0] = comm
            except:
                log_file.info('%s', "Error when openning password window.")
                raise

            d.command = 0

        # ----------------------- Counting application ----------------------- #
        if comm == 2:
            log_file.info('%s', "Counting window.")
            try:
                if blank_window:
                    blank_window.destroy()
                    blank_window = []
                if msg_window:
                    msg_window.destroy()
                    msg_window = []
                if s:
                    s.destroy_all()
                    del s
            except:
                log_file.info('%s', "Error when destroying windows.")
                raise

            try:
                s = Counting()
                d.window[0] = comm
            except:
                log_file.info('%s', "Error when openning counting window.")
                raise

            d.command = 0

        # --------------------------- Form message --------------------------- #
        if 2 < comm <= 2 + len(images):
            log_file.info('%s (%i).', "Message window", comm-3)
            try:
                if s:
                    s.destroy_all()
                    del s
                    s = []
                if blank_window:
                    blank_window.destroy()
                    blank_window = []
                if msg_window:
                    msg_window.destroy()
            except:
                log_file.info('%s', "Error when destroying windows.")
                raise

            try:
                msg_window = Tk()
                msg_window.attributes('-fullscreen', True)
                msg_window.configure(background="#000000")

                photo = ImageTk.PhotoImage(images[comm-3])
                label = Label(msg_window, image=photo, background="#000000")
                label.place(x=int((screen_width - img_width) / 2),
                            y=int((screen_height - img_height) / 2),
                            width=img_width, height=img_height)
                d.window[0] = comm
            except:
                log_file.info('%s', "Error when openning message window.")
                raise

            d.command = 0

        # -------------------------- Updating window ------------------------- #
        try:
            if s:
                s.update_all()
        except:
            log_file.info('%s', "Error when updating application window.")
            raise

        try:
            if blank_window:
                blank_window.update_idletasks()
        except:
            log_file.info('%s', "Error when updating blank window.")
            raise

        try:
            if msg_window:
                msg_window.update_idletasks()
        except:
            log_file.info('%s', "Error when updating message window.")
            raise

        # Saving windows configurations.
        if d.window[0] == 1:
            [d.password, d.virtual_agent,
             d.password_state, d.password_accepting] = s.get_data()
        if d.window[0] == 2:
            [d.counting_images, d.counting_values,
             d.counting_fields, d.counting_state] = s.get_data()

    # Destroying classes objects.
    del d
    if s:
        del s


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass