from tkinter import *
import threading
import webbrowser
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Bool
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

# Global variables to handle the node and GUI objects:
NODE = None
GUI = None


class Screen2Node(Node):

    def __init__(self):
        super().__init__('screen2_node')

        # Flag indicating which screen to be displayed
        # If 0: blank screen
        # If 1: screen with button for intermediate questionnaire
        # If 2: screen with button for final questionnaire
        self.screen = 0

        # Creating subscriber for the screen to be displayed
        self.create_subscription(msg_type=Int64,
                                 topic="/screen2",
                                 callback=self.screen2_callback,
                                 qos_profile=1)

        # Creating publisher for the click of the button
        self.button_click_publisher = self.create_publisher(
            msg_type=Bool, topic="/open_questionnaire_click",
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=50))

    def screen2_callback(self, msg: Int64):
        """
        Callback function to set the screen to be displayed.

        :param msg: an integer setting the screen
        :return:
        """
        self.screen = msg.data
        GUI.update_window()

class Screen2GUI(Frame):
    def __init__(self, root: Tk = None):
        Frame.__init__(self, master=root)

        self.window = root
        self.window.attributes('-fullscreen', True)

        # Getting screen dimensions
        screen_width = self.window.winfo_screenwidth()
        screen_height = self.window.winfo_screenheight()

        # Window dimensions relative to the screen dimensions
        self.window_W = int(screen_width)
        self.window_H = int(screen_height)
        dimensions = str(self.window_W) + "x" + str(self.window_H)
        self.window.geometry(dimensions)
        self.window.configure(background="#000000")

        # Button to open questionnaire
        self.questionnaire_button = Button(self.window,
                                           text="Open questionnaire")
        self.questionnaire_button.configure(command=self.questionnaire_button_callback)

        self.update_window()

    def questionnaire_button_callback(self):
        """
        Callback function for the click of the "Open questionnaire"
        button. It opens it in the web browser.

        :return:
        """
        if NODE.screen == 1:
            # Open intermediate questionnaire
            webbrowser.open_new(r"https://www.qualtrics.manchester.ac.uk/jfe/form/SV_3L65OCUbcP8YM4u")

            msg = Bool()
            msg.data = True
            NODE.button_click_publisher.publish(msg)
        if NODE.screen == 2:
            # Open final questionnaire
            webbrowser.open_new(r"https://www.qualtrics.manchester.ac.uk/jfe/form/SV_37PFRyvdPJUzohg/")

    def update_window(self):
        """
        Update the window according to the flag indicating which screen
        should be displayed.

        """
        self.questionnaire_button.place_forget()

        # Black screen
        if NODE.screen == 0:
            # Forcing window to go on top of others
            self.window.attributes('-topmost', True)
            time.sleep(0.1)
            self.window.attributes('-topmost', False)

            self.window.configure(background="#000000")

        # Screen with button
        if NODE.screen > 0:
            # Forcing window to go on top of others
            self.window.attributes('-topmost', True)
            time.sleep(0.1)
            self.window.attributes('-topmost', False)

            self.window.configure(background="#eeeeee")

            # Placing button
            width = int(self.window_W/3)
            height = int(self.window_H/7)
            x = int(self.window_W/2 - width/2)
            y = int(self.window_H/2 - height/2)
            self.questionnaire_button.config(font=('Helvetica', int(height/5)))
            self.questionnaire_button.place(x=x, y=y, width=width,
                                            height=height)

            if NODE.screen == 1:
                self.questionnaire_button.config(text="Open questionnaire")
            else:
                self.questionnaire_button.config(text="Open final "
                                                      "questionnaire")

def main(args=None):
    try:
        rclpy.init(args=args)

        # ROS 2 node object
        global NODE
        NODE = Screen2Node()
        thread_spin = threading.Thread(target=rclpy.spin, args=(NODE,))
        thread_spin.start()

        # GUI object
        root = Tk()
        global GUI
        GUI = Screen2GUI(root=root)
        root.mainloop()

        thread_spin.join()

    except KeyboardInterrupt:
        NODE.destroy_node()
        GUI.window.destroy()
        rclpy.shutdown()
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()