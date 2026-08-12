from tkinter import *
from PIL import ImageTk, Image
import threading
from playsound import playsound
import webbrowser
import copy

import rclpy
from rclpy.node import Node

from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Int64, Bool, Int16MultiArray, Float64

from package_with_interfaces.msg import RobotSpeech

# Global variables to handle the node and GUI objects:
NODE = None
GUI = None


class ScreenApplicationsNode(Node):
    def __init__(self):
        super().__init__('screen_applications_node')

        # Current status of the interaction
        # 0: not started yet, screen showing button for Instructions
        # 1: practice round
        # 2: task finished, screen showing buttons related to questionnaires
        self.interaction_status = 0

        # Colour sequence info
        # Each number in the sequence corresponds to a colour/symbol:
        # 1: magenta/square
        # 2: green/triangle
        # 3: yellow/circle
        self.sequence = [17]
        self.sequence_length = 1

        # Amount of time that sequence should be displayed (in s)
        self.sequence_time = 1

        # Next slot to be filled
        self.next_square = 0

        # Creating subscribers:
        # interaction_status: current stage of the task
        # monitor_speech: to synchronise screen to robot speech
        # button_state: enable or disable buttons on screen
        # sequence: the correct colour sequence to be added
        # sequence_time: the amount of time to show colour sequence
        # added: the current colour added to the designated area
        # show_sequence: command to show the colour sequence
        # open_questionnaire_click: click in the button to open questionnaire
        self.create_subscription(msg_type=Int64,
                                 topic="/interaction_status",
                                 callback=self.interaction_status_callback,
                                 qos_profile=1)
        self.create_subscription(msg_type=RobotSpeech,
                                 topic="/monitor_speech",
                                 callback=self.monitor_speech_callback,
                                 qos_profile=1)
        self.create_subscription(msg_type=Bool,
                                 topic="/button_state",
                                 callback=self.button_state_callback,
                                 qos_profile=1)
        self.create_subscription(msg_type=Int16MultiArray,
                                 topic="/sequence",
                                 callback=self.sequence_callback,
                                 qos_profile=1)
        self.create_subscription(msg_type=Float64,
                                 topic="/sequence_time",
                                 callback=self.sequence_time_callback,
                                 qos_profile=1)
        self.create_subscription(msg_type=Int64,
                                 topic="/added",
                                 callback=self.added_callback,
                                 qos_profile=1)
        self.create_subscription(msg_type=Float64,
                                 topic="/show_sequence",
                                 callback=self.show_sequence_callback,
                                 qos_profile=1)
        self.create_subscription(msg_type=Bool,
                                 topic="/open_questionnaire_click",
                                 callback=self.open_questionnaire_click_callback,
                                 qos_profile=1)

        # Creating publishers
        # interaction_status: current stage of the task
        # next_instructions_click: click in the Next button
        # show_sequence: click in the Show sequence button
        self.interaction_status_publisher = self.create_publisher(
            msg_type=Int64, topic="/interaction_status",
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=50))
        self.next_instructions_publisher = self.create_publisher(
            msg_type=Int64, topic="/next_instructions_click",
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=50))
        self.show_sequence_click_publisher = self.create_publisher(
            msg_type=Bool, topic="/show_sequence_click",
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   history=HistoryPolicy.KEEP_LAST, depth=50))

    def interaction_status_callback(self, msg: Int64):
        """
        Callback function to get information about changes in the
        status of the interaction. It changes a class attribute and
        updates the GUI window.

        :param msg: an integer indicating the current stage
        :return:
        """
        self.interaction_status = msg.data
        GUI.update_speech_labels(False, " ")
        GUI.update_window()

    def monitor_speech_callback(self, msg: String):
        """
        Callback function to get information about robot speech. It
        calls the method from the GUI class to update the labels
        related to the robot speech.

        :param msg: a package_with_interfaces/RobotSpeech message
        :return:
        """
        GUI.update_speech_labels(status=msg.status, text=msg.text)

    def button_state_callback(self, msg: Bool):
        """
        Callback function to change the state of the buttons on the
        screen. It calls the method to update this from the GUI class.

        :param msg: a std_msg/Bool message to indicate the buttons state
        :return:
        """
        GUI.set_buttons_state(msg.data)

    def sequence_callback(self, msg: Int16MultiArray):
        """
        Callback function to set the correct colour sequence.

        :param msg: an integer array indicating the sequence
        :return:
        """
        self.sequence = msg.data
        self.sequence_length = len(self.sequence)
        GUI.square_status = [0]*self.sequence_length
        self.next_square = 0
        GUI.update_window()

    def sequence_time_callback(self, msg: Int16MultiArray):
        """
        Callback function to set the amount of time the sequence should
        be displayed on screen when required.

        :param msg: an integer with the time duration in seconds
        :return:
        """
        self.sequence_time = msg.data

    def added_callback(self, msg: Int64):
        """
        Callback function for a colour added. It will check if it is
        the correct object and update the screen application accordingly.

        :param msg: an integer indicating the colour added
        :return:
        """
        previous = copy.deepcopy(GUI.feedback_status)
        if msg.data == 0:
            # If there is no colour in the designated area
            GUI.feedback_status = -1
        elif msg.data == self.sequence[self.next_square]:
            # If the correct colour was added
            GUI.square_status[self.next_square] = self.sequence[self.next_square]
            self.next_square = self.next_square + 1
            if self.next_square == len(self.sequence):
                self.next_square = self.next_square - 1
            GUI.feedback_status = 1

            file = (get_package_share_directory("screen_applications") +
                    "/screen_applications/audios/correct_sound.mp3")
            audio_thread = threading.Thread(target=playsound,
                                            args=[file])
            audio_thread.start()
        else:
            # If the wrong colour was added
            GUI.feedback_status = 0

            file = (get_package_share_directory("screen_applications") +
                    "/screen_applications/audios/wrong_sound.mp3")
            audio_thread = threading.Thread(target=playsound,
                                            args=[file])
            audio_thread.start()

        # Update window only if something changed
        if previous != GUI.feedback_status:
            GUI.update_window()

    def show_sequence_callback(self, msg: Float64):
        """
        Callback function to get the command to show the colour
        sequence. It call the GUI class method to do this.

        :param msg: the duration to display the sequence on screen
        :return:
        """
        GUI.show_popup(msg.data)

    def open_questionnaire_click_callback(self, msg: Bool):
        """
        Callback function for the click of the button to open
        questionnaire in the second screen application. It enables the
        button saying the questionnaire is finished.

        :param msg: a boolean indicating the command
        :return:
        """
        if msg.data:
            GUI.finished_button.config(state=NORMAL)


class ScreenApplicationsGUI(Frame):
    def __init__(self, root: Tk = None):
        Frame.__init__(self, master=root)

        # Defining hexadecimal code for colours
        self.window_bg_colour = "#ffffff"
        self.label_bg_colour = "#eeeeee"
        self.label_off_fg_colour = "#aaaaaa"
        self.label_on_fg_colour = "black"
        self.label_off_bg_colour = "#ffffff"
        self.label_on_bg_colour = "#aaaaaa"
        self.green = "green"
        self.magenta = "magenta"
        self.yellow = "yellow"

        # Creating dictionaries to match the numbers with the colours
        # and symbols
        self.colours = {0: self.window_bg_colour,
                        17: self.magenta, 18: self.green, 19: self.yellow}
        self.symbols = {17: '\u25a1', 18: '\u25b3', 19: '\u25cb'}

        # Status of the squares
        # If 0, square has no colour, and if not, it has the respective
        # colour according to the dictionary defined before
        self.square_status = [0]

        # Status of the feedback signal indicating if the object added
        # is correct or not
        # If -1: no sign
        # If 0: sign of wrong object (❌)
        # If 1: sign of correct object (✔)
        self.feedback_status = -1

        # Main window of the application
        self.window = root
        self.window.attributes('-fullscreen', True)

        # Status of the pop-up window
        self.popup_status = False

        # Getting screen dimensions
        screen_width = self.window.winfo_screenwidth()
        screen_height = self.window.winfo_screenheight()

        # Window dimensions relative to the screen dimensions
        self.window_W = int(screen_width)
        self.window_H = int(screen_height)
        dimensions = str(self.window_W) + "x" + str(self.window_H)
        self.window.geometry(dimensions)
        self.window.configure(background=self.window_bg_colour)

        # Creating widgets and updating window
        self.create_widgets()
        self.update_window()

    def next_stage(self):
        """
        Go to the next interaction stage and update the window.

        :return:
        """
        NODE.interaction_status = NODE.interaction_status + 1

        # If it was the last screen, go back to the start one
        #if NODE.interaction_status > :
        #    NODE.interaction_status = 0

        # Publishing the new interaction status
        msg = Int64()
        msg.data = NODE.interaction_status
        NODE.interaction_status_publisher.publish(msg)

        self.update_speech_labels(False, " ")
        self.update_window()

    def next_button_callback(self):
        """
        Callback function for the click of the "Next" button in the
        instructions page. It publishes a message informing about the
        click.

        :return:
        """
        msg = Int64()
        msg.data = 1
        NODE.next_instructions_publisher.publish(msg)

    def show_sequence_button_callback(self):
        """
        Callback function for the click of the "Show sequence" button.
        It sends the command to the interaction manager.

        :return:
        """
        msg = Bool()
        msg.data = True
        NODE.show_sequence_click_publisher.publish(msg)

    def set_buttons_state(self, state: bool):
        """
        Set the state of all buttons to either NORMAL or DISABLED.

        :state: True to set buttons to NORMAL, False to set to DISABLED
        :return:
        """
        if state:
            self.instructions_button.config(state=NORMAL)
            self.start_button.config(state=NORMAL)
            self.show_sequence_button.config(state=NORMAL)
            if self.finished_button is not None:
                self.finished_button.config(state=NORMAL)
        else:
            self.instructions_button.config(state=DISABLED)
            self.start_button.config(state=DISABLED)
            self.show_sequence_button.config(state=DISABLED)
            if self.finished_button is not None:
                self.finished_button.config(state=DISABLED)

    def create_widgets(self):
        """
        Create the widgets such as buttons and text labels to be used
        in the screen applications. They are created in this function,
        but not placed in the window yet.

        :return:
        """
        # Minimum border for the main window
        self.border = int(self.window_H/40)

        # Button to see instructions and start practice round
        self.instructions_button = Button(self.window,
                                          text="Instructions")
        self.instructions_button.configure(command=self.next_stage)

        # Label with instructions text
        self.instructions_label = Label(self.window, text="",
                                        bg=self.label_bg_colour)

        # Button for next step of instructions and practice
        self.next_button = Button(self.window, text="Next")
        self.next_button.configure(command=self.next_button_callback)

        # Button to start task
        self.start_button = Button(self.window, text="START")
        self.start_button.configure(command=self.next_stage)

        # Text label to indicate if robot is speaking
        self.speaking_text_label = Label(self.window, text="ROBOT SPEAKING",
                                         fg=self.label_off_fg_colour,
                                         bg=self.label_off_bg_colour)

        # Label with robot's speech
        self.speech_label = Label(self.window, text="",
                                  bg=self.label_bg_colour)

        # Text label to indicate feedback on object added
        self.feedback_text_label = Label(self.window,
                                         text="OBJECT ADDED:",
                                         bg=self.window_bg_colour)

        # Label with sign indicating correct or wrong object
        self.feedback_label = Label(self.window,
                                    bg=self.window_bg_colour, text="")
        self.feedback_label.config(font=('Helvetica', int(2*self.border)))

        # Labels with squares to show the evolution of the task
        self.square_labels = None

        # Button to show correct sequence
        self.show_sequence_button = Button(self.window, text="Show sequence")
        self.show_sequence_button.configure(command=self.show_sequence_button_callback)

        # Button to indicate that questionnaire is finished
        self.finished_button = None

    def create_popup_window(self):
        """
        Create pop-up window to show the colour sequence.

        :return:
        """
        # Dimensions and position of the pop-up window. Position is
        # relative to the main window.
        popup_width = self.window_W
        popup_height = self.window_H
        popup_x = 0
        popup_y = 0
        popup_settings = (popup_width, popup_height, popup_x, popup_y)

        # Creating the window on top of main window
        self.popup_window = Toplevel(self.window)
        self.popup_window.geometry("%dx%d+%d+%d" % popup_settings)
        self.popup_window.configure(background=self.window_bg_colour)
        self.popup_window.attributes("-fullscreen", True)

        # Creating labels to show the colour sequence
        path = (get_package_share_directory("screen_applications") +
                "/screen_applications/images/")
        image = Image.open(path + "square.png")
        # calculating side of the squares depending on length of
        # colour sequence and resizing image:
        possible_height = popup_height - 22*self.border
        possible_width = int((popup_width - (
                NODE.sequence_length + 1)*self.border)/NODE.sequence_length)
        square_side = min(possible_height, possible_width)
        image = image.resize((square_side, square_side), Image.ANTIALIAS)
        square_image = ImageTk.PhotoImage(image)
        # defining position of first square:
        x = self.border + (popup_width -
                           (2 + (NODE.sequence_length - 1))*self.border -
                           NODE.sequence_length*square_side)/2
        # creating the appropriate number of labels:
        square_labels = NODE.sequence_length*[0]
        for i in range(0, NODE.sequence_length):
            square_labels[i] = Label(self.popup_window,
                                     image=square_image,
                                     bg=self.colours[NODE.sequence[i]],
                                     width=square_side,
                                     height=square_side,
                                     compound='center')
            square_labels[i].config(text=self.symbols[NODE.sequence[i]],
                                    font=('Helvetica', int(square_side/2)),
                                    justify=CENTER)
            square_labels[i].place(x=x, y=int(popup_height/2 - square_side/2))

            x = x + square_side + self.border  # position of next square

        # Not showing the pop-up window yet
        #self.popup_window.withdraw()

    def show_popup(self, duration: float = -1.0):
        """
        Display the pop-up window with the colour sequence for the
        given amount of time.

        :param duration: how long to display the pop-up window (in s)
        :return:
        """
        # If duration is not set, use the data from the NODE class attribute
        if duration < 0:
            duration = NODE.sequence_time

        duration = int(duration*1000)  # duration in ms

        self.create_popup_window()
        self.popup_window.update()
        self.popup_window.deiconify()
        self.window.after(duration, self.hide_popup)

    def hide_popup(self):
        """
        Hide the pop-up window.

        :return:
        """
        self.popup_window.withdraw()

    def update_speech_labels(self, status: bool, text: str):
        """
        Update the widgets monitoring the robot speech.

        :param status: if the robot is speaking (True) or not (False)
        :param text: the text to be displayed on the screen
        :return:
        """
        # Updating speaking sign
        if status:
            self.speaking_text_label.config(bg=self.label_on_bg_colour,
                                            fg=self.label_on_fg_colour)
        else:
            self.speaking_text_label.config(bg=self.label_off_bg_colour,
                                            fg=self.label_off_fg_colour)

        # Updating text shown on screen
        if text != 'None':
            self.speech_label.config(text=text)

    def forget_all(self):
        """
        Remove all widgets from the window.

        :return:
        """
        self.instructions_button.place_forget()
        self.next_button.place_forget()
        self.start_button.place_forget()
        self.speaking_text_label.place_forget()
        self.speech_label.place_forget()
        self.feedback_label.place_forget()
        self.feedback_text_label.place_forget()
        if self.square_labels is not None:
            for i in range(0, len(self.square_labels)):
                self.square_labels[i].place_forget()
        self.show_sequence_button.place_forget()
        if self.finished_button is not None:
            self.finished_button.place_forget()

    def update_window(self):
        """
        Update the window according to the interaction status.

        :return:
        """

        self.forget_all()

        # Initial screen: showing button to start the instructions
        if NODE.interaction_status == 0:
            width = int(self.window_W/3)
            height = int(self.window_H/7)
            x = int(self.window_W/2 - width/2)
            y = int(self.window_H/2 - height/2)
            self.instructions_button.config(font=('Helvetica', int(height/5)))
            self.instructions_button.place(x=x, y=y, width=width, height=height)

        # Instructions and practice round
        if NODE.interaction_status == 1:
            # Label with the instructions text
            x = self.border
            y = 5*self.border
            width = self.window_W - 2*self.border
            height = 15*self.border
            self.speech_label.place(x=x, y=y, width=width, height=height)
            self.speech_label.config(font=('Helvetica', int(height/12)),
                                     wraplength=0.95*width)

            # Button to go to next step of instructions and practice
            width = int(self.window_W/9)
            height = 3*self.border
            x = self.window_W - self.border - width
            y = 21*self.border
            self.next_button.config(font=('Helvetica', int(height/5)))
            self.next_button.place(x=x, y=y, width=width, height=height)

            # Squares to monitor evolution of task
            # loading image:
            path = (get_package_share_directory("screen_applications") +
                    "/screen_applications/images/")
            image = Image.open(path + "square.png")
            # calculating side of the squares depending on length of
            # colour sequence and resizing image:
            possible_height = self.window_H - 25*self.border
            possible_width = int((self.window_W - (
                    NODE.sequence_length + 1)*self.border)/NODE.sequence_length)
            square_side = min(possible_height, possible_width)
            image = image.resize((square_side, square_side), Image.ANTIALIAS)
            self.square_image = ImageTk.PhotoImage(image)
            # defining position of first square
            x = self.border + (self.window_W -
                               (2 + (NODE.sequence_length - 1))*self.border -
                               NODE.sequence_length*square_side)/2
            # creating the appropriate number of labels
            remaining_height = self.window_H - 23*self.border
            self.square_labels = NODE.sequence_length*[0]
            for i in range(0, NODE.sequence_length):
                self.square_labels[i] = Label(self.window,
                                              image=self.square_image,
                                              bg=self.colours[
                                                  self.square_status[i]],
                                              width=square_side,
                                              height=square_side,
                                              compound='center')
                if self.square_status[i] != 0:
                    # If colour was added, add respective symbol to label
                    self.square_labels[i].config(
                        text=u'{unicodes_value}'.format(
                            unicodes_value=self.symbols[NODE.sequence[i]]),
                        font=('Helvetica', int(square_side/2)), justify=CENTER)
                self.square_labels[i].place(x=x,
                                            y=self.window_H - int((remaining_height - square_side)/2) - square_side)

                x = x + square_side + self.border  # position for next square

            # Text label "OBJECT ADDED:"
            x = self.border
            y = self.border
            width = int(self.window_W/7)
            height = 3*self.border
            self.feedback_text_label.place(x=x, y=y,
                                           width=width, height=height)
            self.feedback_text_label.config(font=('Helvetica', int(height/4)),
                                            justify=LEFT)

            # Label with the sign for correct or wrong object
            x = self.border + width
            y = self.border
            width = 3*self.border
            height = 3*self.border
            self.feedback_label.place(x=x, y=y, width=width, height=height)
            # setting sign to be shown
            if self.feedback_status == 1:
                text = u'{unicodes_value}'.format(unicodes_value='\u2714')
            elif self.feedback_status == 0:
                text = u'{unicodes_value}'.format(unicodes_value='\u274c')
            else:
                text = ""
            self.feedback_label.config(text=text,
                                       font=('Helvetica', int(height/2)),
                                       justify=LEFT)

            # Button to show the sequence
            width = int(self.window_W/5)
            height = 3*self.border
            x = int(self.window_W/2 - width/2)
            y = self.border
            self.show_sequence_button.config(font=('Helvetica',
                                                   int(height/5)))
            self.show_sequence_button.place(x=x, y=y, width=width,
                                            height=height)

        # Robot introduction: showing only widgets for robot speech
        if NODE.interaction_status == 2:
            # Text label "ROBOT SPEAKING"
            x = int(5*self.window_W/6) - self.border
            y = self.border
            width = int(self.window_W/6)
            height = 3*self.border
            self.speaking_text_label.place(x=x, y=y,
                                           width=width, height=height)
            self.speaking_text_label.config(font=('Helvetica', int(height/4)),
                                            justify=RIGHT)

            # Label with the robot speech
            x = self.border
            y = 5*self.border
            width = self.window_W - 2*self.border
            height = 15*self.border
            self.speech_label.place(x=x, y=y, width=width, height=height)
            self.speech_label.config(font=('Helvetica', int(height/12)),
                                     wraplength=0.95*width)

            # Button to start task
            y = y + height + self.border
            width = int(self.window_W/6)
            height = int(self.window_H/6)
            x = int(self.window_W/2 - width/2)
            self.start_button.config(font=('Helvetica', int(height/6)))
            self.start_button.place(x=x, y=y, width=width, height=height)

        # Task screen: monitoring the task
        if NODE.interaction_status >= 3:

            # Text label "ROBOT SPEAKING"
            x = int(5*self.window_W/6) - self.border
            y = self.border
            width = int(self.window_W/6)
            height = 3*self.border
            self.speaking_text_label.place(x=x, y=y,
                                           width=width, height=height)
            self.speaking_text_label.config(font=('Helvetica', int(height/4)),
                                            justify=RIGHT)

            # Label with the robot speech
            x = self.border
            y = 5*self.border
            width = self.window_W - 2*self.border
            height = 15*self.border
            self.speech_label.place(x=x, y=y, width=width, height=height)
            self.speech_label.config(font=('Helvetica', int(height/12)),
                                     wraplength=0.95*width)

            # Text label "OBJECT ADDED:"
            x = self.border
            y = self.border
            width = int(self.window_W/7)
            height = 3*self.border
            self.feedback_text_label.place(x=x, y=y,
                                           width=width, height=height)
            self.feedback_text_label.config(font=('Helvetica', int(height/4)),
                                            justify=LEFT)

            # Label with the sign for correct or wrong object
            x = self.border + width
            y = self.border
            width = 3*self.border
            height = 3*self.border
            self.feedback_label.place(x=x, y=y, width=width, height=height)
            # setting sign to be shown
            if self.feedback_status == 1:
                text = u'{unicodes_value}'.format(unicodes_value='\u2714')
            elif self.feedback_status == 0:
                text = u'{unicodes_value}'.format(unicodes_value='\u274c')
            else:
                text = ""
            self.feedback_label.config(text=text,
                                       font=('Helvetica', int(height/2)),
                                       justify=LEFT)

            # Button to show the sequence
            width = int(self.window_W/5)
            height = 3*self.border
            x = int(self.window_W/2 - width/2)
            y = self.border
            self.show_sequence_button.config(font=('Helvetica',
                                                   int(height/5)))
            self.show_sequence_button.place(x=x, y=y, width=width,
                                            height=height)

            # Squares to monitor evolution of task
            # loading image:
            path = (get_package_share_directory("screen_applications") +
                    "/screen_applications/images/")
            image = Image.open(path + "square.png")
            # calculating side of the squares depending on length of
            # colour sequence and resizing image:
            possible_height = self.window_H - 22*self.border
            possible_width = int((self.window_W - (
                    NODE.sequence_length + 1)*self.border)/NODE.sequence_length)
            square_side = min(possible_height, possible_width)
            image = image.resize((square_side, square_side), Image.ANTIALIAS)
            self.square_image = ImageTk.PhotoImage(image)
            # defining position of first square
            x = self.border + (self.window_W -
                               (2 + (NODE.sequence_length - 1))*self.border -
                               NODE.sequence_length*square_side)/2
            # creating the appropriate number of labels
            remaining_height = self.window_H - 20*self.border
            self.square_labels = NODE.sequence_length*[0]
            for i in range(0, NODE.sequence_length):
                self.square_labels[i] = Label(self.window,
                                              image=self.square_image,
                                              bg=self.colours[self.square_status[i]],
                                              width=square_side, height=square_side,
                                              compound='center')
                if self.square_status[i] != 0:
                    # If colour was added, add respective symbol to label
                    self.square_labels[i].config(
                        text=u'{unicodes_value}'.format(unicodes_value=self.symbols[NODE.sequence[i]]),
                        font=('Helvetica', int(square_side/2)), justify=CENTER)
                self.square_labels[i].place(x=x,
                                            y=self.window_H - int((remaining_height - square_side) / 2) - square_side)

                x = x + square_side + self.border  # position for next square

            # Adding a button to indicate that questionnaire is finished
            if NODE.interaction_status >= 4:
                # Button to indicate that questionnaire was finished
                self.finished_button = Button(
                    self.window, text="I finished the questionnaire")
                self.finished_button.configure(command=self.next_stage)

                width = int(3*remaining_height/2)
                height = int(remaining_height/3)
                x = int(self.window_W/2 - width/2)
                y = int(self.window_H - remaining_height/2 - height/2)
                self.finished_button.configure(font=('Helvetica', int(height/6)))
                self.finished_button.place(x=x, y=y, width=width,
                                           height=height)

                # Removing squares from screens with finish button
                for i in range(0, NODE.sequence_length):
                    self.square_labels[i].place_forget()


def main(args=None):

    try:
        rclpy.init(args=args)

        # ROS 2 node object
        global NODE
        NODE = ScreenApplicationsNode()
        thread_spin = threading.Thread(target=rclpy.spin, args=(NODE,))
        thread_spin.start()

        # GUI object
        root = Tk()
        global GUI
        GUI = ScreenApplicationsGUI(root=root)
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
