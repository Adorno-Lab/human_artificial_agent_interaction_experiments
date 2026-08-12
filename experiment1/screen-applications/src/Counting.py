import rospy
import rospkg
from tkinter import *
from tkinter import messagebox
from PIL import Image, ImageTk
from std_msgs.msg import Int16, Int16MultiArray
import time


class Counting:
    """
     Counting is a class with the screen application for the counting phase.

    Private attributes:
        _pkg_path: path for the ROS package.
        _gray: color for the background.
        _main_window: the main tkinter window.
        _screen_width and _screen_height: dimensions of the screen.
        _window_width and _window_height: dimensions of the window.
        _images_set: index for the set of images to be used in the main buttons.
        _values: correct values.
        _border: border for the window.
        _canvas: Canvas object with the virtual agent's space.
        _start_button: button to start.
        _buttons_height and _buttons_width: dimensions of the image buttons.
        _square_photo, _circle_photo, _rhombus_photo, _triangle_photo: PhotoImage objects.
        _buttons: image buttons.
        _text_var: StringVar object to get the entry texts.
        _texts: added texts.
        _text_entries: text entries.
        _ok_buttons: ok buttons to submit answers.
        _current_entry: index of the current active entry.
        _state: state of the application.
        _complete: indicates if the fields should be completed.
        _start_command: indicates if the application should be started.
        _start_with_button: indicates if the main buttons should activate with
                            the click of the start button.
        _control_clicks: indicates if the clicks in the application are enabled or not.
        _check_pub, _entry_pub, _status_pub: publishers.

    Private methods:
        _add_objects: adds objects to the main window.
        _start: starts the application.
        _activate: activates the main buttons.
        _show_entry: shows text entry.
        _hide_entry: hides text entry.
        _get_entry: get added text.
        _complete_entries: completes the entries with the correct values.
        _check_state: checks the state of the application.
        _counting_callback: callback method to get commands for the application.
        _update_buttons: updates the main buttons images.

    Public methods:
        get_data: returns the application data.
        fill_entries: fills some of the text entries.
        settings: configures the application.
        update_all: updates all the windows.
        destroy_all: destroys all the windows.

    Published topics:
        /check_input: to publish if the last input was correct or not.
        /entry_value: to publish the value added in the fields.
        /status_counting: to publish the status of the application.
    Subscribed topics:
        /counting_commands: to get commands for the application.
        /control_clicks: to get commands to control clicks in the application.
        /counting_settings: to configure the application.
    """

    def _add_objects(self):
        """
        Adds the objects to the main window.

        :return:
        """
        # Border size
        self._border = int(self._window_width / 50)

        self._buttons_height = int((self._window_height - 2 * self._border) / 6)
        self._buttons_width = self._buttons_height

        # Adding virtual agent's rectangle.
        self._canvas = Canvas(self._main_window,
                              width=int(self._window_width / 2),
                              height=int(self._window_height / 2))
        self._canvas.place(x=int(self._window_width / 4),
                           y=int(self._window_height / 4))
        self._canvas.create_rectangle(1, 1,
                                      int(self._window_width / 2),
                                      int(self._window_height / 2))

        # Button to start.
        self._start_button = Button(self._main_window, text="COMEÇAR",
                                    font=(
                                    'Helvetica', int(self._buttons_height / 8)),
                                    compound="c")
        self._start_button.place(
            x=int(self._window_width / 2 - self._buttons_width),
            y=int(self._window_height / 2 - self._buttons_width / 4),
            width=2*self._buttons_width,
            height=int(self._buttons_width / 2))
        self._start_button.configure(command=lambda: self._start())

        # Buttons to open the answer text field.
        self._buttons = 4 * [0]

        one_pixel = PhotoImage(width=1, height=1)

        # Top left button.
        self._buttons[0] = Button(self._main_window, image=one_pixel,
                                  compound="c",
                                  width=self._buttons_width,
                                  height=self._buttons_height)
        self._buttons[0].place(x=self._border, y=self._border)
        self._buttons[0].configure(command=lambda: self._show_entry(0))
        self._buttons[0].config(state=DISABLED)

        # Bottom left button.
        self._buttons[1] = Button(self._main_window, image=one_pixel,
                                  compound="c",
                                  width=self._buttons_width,
                                  height=self._buttons_height)
        self._buttons[1].place(x=self._border,
                               y=self._window_height - self._border - self._buttons_height)
        self._buttons[1].configure(command=lambda: self._show_entry(1))
        self._buttons[1].config(state=DISABLED)

        # Top right button.
        self._buttons[2] = Button(self._main_window, image=one_pixel,
                                  compound="c",
                                  width=self._buttons_width,
                                  height=self._buttons_height)
        self._buttons[2].place(
            x=self._window_width - self._border - self._buttons_width,
            y=self._border)
        self._buttons[2].configure(command=lambda: self._show_entry(2))
        self._buttons[2].config(state=DISABLED)

        # Bottom right button.
        self._buttons[3] = Button(self._main_window, image=one_pixel,
                                  compound="c",
                                  width=self._buttons_width,
                                  height=self._buttons_height)
        self._buttons[3].place(x=self._window_width - self._border - self._buttons_width,
                               y=self._window_height - self._border - self._buttons_height)
        self._buttons[3].configure(command=lambda: self._show_entry(3))
        self._buttons[3].config(state=DISABLED)

        # Variables to get the entry texts.
        self._text_var = []
        for i in range(0, 4):
            self._text_var.append(StringVar())

        # List to store the answers.
        self._texts = 4 * [0]

        # Text entries for the answers.
        self._text_entries = 4 * [0]

        # Top left text entry.
        self._text_entries[0] = Entry(self._main_window,
                                      textvariable=self._text_var[0],
                                      font=('Helvetica', int(self._buttons_height / 6)))

        # Bottom left text entry.
        self._text_entries[1] = Entry(self._main_window,
                                      textvariable=self._text_var[1],
                                      font=('Helvetica', int(self._buttons_height / 6)))

        # Top right text entry.
        self._text_entries[2] = Entry(self._main_window,
                                      textvariable=self._text_var[2],
                                      font=('Helvetica',
                                            int(self._buttons_height / 6)))

        # Bottom right text entry.
        self._text_entries[3] = Entry(self._main_window,
                                      textvariable=self._text_var[3],
                                      font=('Helvetica', int(self._buttons_height / 6)))

        # Buttons to submit the answers.
        self._ok_buttons = 4 * [0]

        # Top left ok button.
        self._ok_buttons[0] = Button(self._main_window, text="OK",
                                     font=('Helvetica', int(self._buttons_height / 10)),
                                     compound="c")
        self._ok_buttons[0].configure(command=lambda: self._get_entry(0))

        # Bottom left ok button.
        self._ok_buttons[1] = Button(self._main_window, text="OK",
                                     font=('Helvetica', int(self._buttons_height / 10)),
                                     compound="c")
        self._ok_buttons[1].configure(command=lambda: self._get_entry(1))

        # Top right ok button.
        self._ok_buttons[2] = Button(self._main_window, text="OK",
                                     font=('Helvetica',
                                           int(self._buttons_height / 10)),
                                     compound="c")
        self._ok_buttons[2].configure(command=lambda: self._get_entry(2))

        # Bottom right ok button.
        self._ok_buttons[3] = Button(self._main_window, text="OK",
                                     font=('Helvetica', int(self._buttons_height / 10)),
                                     compound="c")
        self._ok_buttons[3].configure(command=lambda: self._get_entry(3))

    def _start(self):
        """
        Starts the application, but with no active buttons yet.

        :return:
        """
        self._start_button.place_forget()
        self._state = -1
        self._status_pub.publish(self._state)
        if self._start_with_button == 1:
            self._activate()

    def _activate(self):
        """
        Activates the main buttons.

        :return:
        """
        for i in range(0, 4):
            self._buttons[i].config(state=NORMAL)
        self._state = 0
        self._status_pub.publish(self._state)

    def _show_entry(self, index):
        """
        Shows the text entry and the ok button related to the given index.
        All the other entries are hidden.

        :param index: the index of the entry that should be displayed.
        :return:
        """

        if self._control_clicks == 1:
            self._state = 1
            self._current_entry = index
            self._status_pub.publish(self._current_entry + 1)

            self._text_entries[index].config(state=NORMAL)
            for i in range(0, 4):
                if i != index and self._text_entries[i]["state"] == 'normal':
                    self._hide_entry(i)

            if index == 0:
                self._text_entries[index].place(
                    x=2 * self._border + self._buttons_width,
                    y=self._border + int(self._buttons_height / 5),
                    width=self._buttons_width,
                    height=int(self._buttons_height / 4))
                self._ok_buttons[index].place(
                    x=2 * self._border + int(5 * self._buttons_width / 4),
                    y=self._border + int(11 * self._buttons_height / 20),
                    width=(int(self._buttons_width / 2)),
                    height=(int(self._buttons_width / 4)))

            if index == 1:
                self._text_entries[index].place(
                    x=2 * self._border + self._buttons_width,
                    y=self._window_height - self._border - int(
                        4 * self._buttons_height / 5),
                    width=self._buttons_width,
                    height=int(self._buttons_height / 4))
                self._ok_buttons[index].place(
                    x=2 * self._border + int(5 * self._buttons_width / 4),
                    y=self._window_height - self._border - int(
                        9 * self._buttons_height / 20),
                    width=(int(self._buttons_width / 2)),
                    height=(int(self._buttons_width / 4)))

            if index == 2:
                self._text_entries[index].place(
                    x=self._window_width - 2 * self._border - 2 * self._buttons_width,
                    y=self._border + int(self._buttons_height / 5),
                    width=self._buttons_width,
                    height=int(self._buttons_height / 4))
                self._ok_buttons[index].place(
                    x=self._window_width - 2 * self._border - + int(
                        7 * self._buttons_width / 4),
                    y=self._border + int(11 * self._buttons_height / 20),
                    width=(int(self._buttons_width / 2)),
                    height=(int(self._buttons_width / 4)))

            if index == 3:
                self._text_entries[index].place(
                    x=self._window_width - self._border - self._buttons_width - self._border - self._buttons_width,
                    y=self._window_height - self._border - self._buttons_height + int(self._buttons_height / 5),
                    width=self._buttons_width, height=int(self._buttons_height / 4))
                self._ok_buttons[index].place(
                    x=self._window_width - self._border - self._buttons_width - self._border - self._buttons_width + int(
                        self._buttons_width / 4),
                    y=self._window_height - self._border - int(
                        self._buttons_height / 5) - int(self._buttons_height / 4),
                    width=(int(self._buttons_width / 2)),
                    height=(int(self._buttons_width / 4)))

            self._text_entries[index].focus_set()

    def _hide_entry(self, index):
        """
        Hides the text entry and the ok button related to the given index.

        :param index: the index of the entry that should be hidden.
        :return:
        """
        self._text_entries[index].place_forget()
        self._ok_buttons[index].place_forget()

    def _get_entry(self, index):
        """
        Gets the text in the text entry and disables it.

        :param index: the index of the added entry.
        :return:
        """

        if self._control_clicks == 1:
            if self._text_var[index].get() and self._text_var[index].get().isdigit():
                # If the input field is not empty and if the input is a number.
                self._texts[index] = self._text_var[index].get()
                self._buttons[index].config(state=DISABLED)
                self._text_entries[index].config(state=DISABLED)

                if self._complete == 0:
                    self._entry_pub.publish(int(self._texts[index]))
                    if int(self._texts[index]) == self._values[index]:
                        self._check_pub.publish(1)
                    else:
                        self._check_pub.publish(0)

                    self._state = 0
                    self._status_pub.publish(self._state)
                """else:
                    # If the input is not valid, opens an information window.
                    messagebox.showinfo("Entrada inválida",
                                        "Por favor, digite um número.",
                                        icon='error')"""

    def _complete_entries(self):
        """
        Completes the entries with the correct values.

        :return:
        """
        for i in range(0, 4):
            self._buttons[i].invoke()
            self._text_entries[i].delete(0, END)
            self._text_entries[i].insert(0, str(self._values[i]))
            self._ok_buttons[i].invoke()
        self._complete = 0

    def _check_state(self):
        """
        Checks the state of the application.
        If finished, self._state = -1.

        :return:
        """
        if self._state != -1:
            finished = 1
            for i in range(0, 4):
                if self._text_entries[i]["state"] == 'normal':
                    finished = 0
                    break
            if finished == 1:
                self._state = -1
                self._status_pub.publish(self._state)

    def _counting_callback(self, msg):
        """
        Callback method to get commands for the application.

        If msg.data == 0: activate buttons.
        If msg.data == -1: starts application.
        If msg.data == -2: completes remaining fields.

        :param msg: a std_msgs/Int16 message with commands for the application.
        :return:
        """
        if msg.data == 0:
            self._start_command = 2
        if msg.data == -1:
            self._start_command = 1
        if msg.data == -2:
            self._complete = 1

    def _control_clicks_callback(self, msg):
        """
        Callback method to control clicks in the application.

        :param msg: a std_msgs/Int16 message with commands to control clicks.
        :return:
        """
        self._control_clicks = msg.data

    def _update_buttons(self):
        """
        Updates the main buttons images.
        Loads, resizes and adds the images for the buttons.

        :return:
        """
        images_height = self._buttons_height - self._border
        images_width = self._buttons_width - self._border

        path = self._pkg_path + "/src/images/" + str(self._images_set[0]) + "/"

        images = []
        self._buttons_photos = []
        for i in range(0, 4):
            images.append(Image.open(path + str(i+1) + ".png"))
            images[i] = images[i].resize((images_width, images_height),
                                         Image.ANTIALIAS)
            self._buttons_photos.append(ImageTk.PhotoImage(images[i]))
            self._buttons[i].configure(image=self._buttons_photos[i])

    def get_data(self):
        """
        Returns the application data.

        :return: a list with the images set, the correct values and the entered answers.
        """
        answers = []
        for i in self._texts:
            answers.append(int(i))

        if self._start_button.winfo_ismapped():
            state = 0
        else:
            state = 1
        if self._state >= 0:
            state = 2

        return [self._images_set[0], self._values, answers, state]

    def settings(self, settings, start_with_button=0):
        """
        Configures the application, setting the index of the image set for the
        main buttons and the correct values for the entries.

        :param settings: a list or an Int16MultiArray message.
        :param start_with_button: indicates how to activate the main buttons.
        :return:
        """
        if type(settings) == Int16MultiArray:
            # If the parameter is a std_msgs/Int16MultiArray message.
            settings = settings.data
        self._images_set[1] = self._images_set[0]
        self._images_set[0] = settings[0]
        self._values = settings[1:len(settings)]
        self._start_with_button = start_with_button

    def fill_entries(self, list_of_values):
        """
        Fills some of the text entries.

        :param list_of_values: the values that should be entered.
        :return:
        """
        if self._state == -1:
            self._start()
        for i in range(0, len(list_of_values)):
            if list_of_values[i] != 0:
                self._buttons[i].invoke()
                self._text_entries[i].delete(0, END)
                self._text_entries[i].insert(0, str(list_of_values[i]))
                self._ok_buttons[i].invoke()
            else:
                self._buttons[i].config(state=NORMAL)

    def update_all(self):
        """
        Updates all the windows. This method should be called inside a while
        loop in the main script.

        :return:
        """
        self._check_state()

        #self._main_window.update_idletasks()
        self._main_window.update()

        if self._complete == 1:
            self._complete_entries()

        if self._start_command == 1:
            self._start()
            self._start_command = 0
        if self._start_command == 2:
            self._activate()
            self._start_command = 0

        if self._images_set[0] != self._images_set[1]:
            self._update_buttons()

    def destroy_all(self):
        """
        Destroys the main window and hence all the pop-up windows.

        :return:
        """
        self._main_window.destroy()

    def __init__(self, window=0):
        """
        Constructor method of the class.

        :param window: window dimensions.
        """
        # Getting the path for the ROS package.
        rospack = rospkg.RosPack()
        self._pkg_path = rospack.get_path('screen')

        # Defining the codes of background color.
        self._gray = "#dcdcdc"

        # Creating and configuring the main window.
        self._main_window = Tk()
        self._screen_width = self._main_window.winfo_screenwidth()
        self._screen_height = self._main_window.winfo_screenheight()

        if window == 0:
            self._window_width = self._screen_width
            self._window_height = self._screen_height
            self._main_window.attributes('-fullscreen', True)
        else:
            self._window_width = window[0]
            self._window_height = window[1]

        dimensions = str(self._window_width) + "x" + str(self._window_height)
        self._main_window.geometry(dimensions)
        self._main_window.configure(background=self._gray)

        # Index for the set of images to be used in the main buttons.
        self._images_set = [0, 0]

        self._add_objects()

        # Correct values.
        self._values = []

        # Current active entry.
        self._current_entry = -1

        # Current state of the application.
        # If state == -1: application not running.
        # If state == 0: application running, no active entry.
        # If state == 1: application running, entry active.
        self._state = -1

        # Indicates if the fields should be completed (1) or not (0).
        self._complete = 0

        # Indicates the beginning of the application.
        # If start_command == 1: start application, but with no active buttons.
        # If start_command == 2: activate buttons.
        self._start_command = 0

        # Indicates if the main buttons should activate with the click of the
        # start button (1) or wait for the command through topic (0).
        self._start_with_button = 0

        # Indicates if the clicks in the application are enabled (1) or not (0).
        self._control_clicks = 1

        # Publisher to inform if the input was correct (1) or not (-1).
        self._check_pub = rospy.Publisher('/check_input', Int16,
                                          queue_size=1, latch=True)

        # Publisher for the value entered in the fields.
        self._entry_pub = rospy.Publisher('/entry_value', Int16,
                                          queue_size=1, latch=True)

        # Publisher for the status of the application.
        self._status_pub = rospy.Publisher('status_counting', Int16,
                                           queue_size=10, latch=True)

        # Subscriber for commands about the counting application.
        rospy.Subscriber('/counting_commands', Int16, self._counting_callback)

        # Subscriber for commands to control clicks in the application.
        rospy.Subscriber('/control_clicks', Int16, self._control_clicks_callback)

        # Subscriber to configure the application.
        rospy.Subscriber('/counting_settings', Int16MultiArray, self.settings)
