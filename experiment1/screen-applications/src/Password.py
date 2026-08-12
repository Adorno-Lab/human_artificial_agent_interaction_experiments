from tkinter import *
from PIL import Image, ImageTk
import rospy
import rospkg
from std_msgs.msg import String, Int16, Float32, Int16MultiArray
from playsound import playsound
from threading import Thread


class Password:
    """
    Password is a class with the screen application for the password phase.

    Private attributes:
        _pkg_path: path for the ROS package.
        _main_window: the main tkinter window.
        _screen_width and screen_height: screen dimensions.
        _window_width and _window_height: window dimensions.
        _num_fields: number of color fields in the password.
        _popup_index: index of the last opened pop-up window.
        _va_image_photo: ImageTk.PhotoImage object for the virtual agent image.
        _va_image_label: Label object with the face of the virtual agent.
        _va_text_label: Label object with the name of the virtual agent.
        _square_photo: ImageTk.PhotoImage object for the square image.
        _square_labels: the Label objects of the password fields.
        _popup_windows: tkinter windows that pop up to show the password.
        _password: the sequence of color to be used as password.
        _current_field: current field of the password.
        _state: state of the application.
        _square_bg: square labels' background colors.
        _virtual_agent: name of the current virtual agent.
        _password_duration: duration of each color in the screen while showing the password
        _check_pub, status_pub: publishers.

    Public attributes:
        gray, blue, yellow, white, and black: colors.
        accepting_indication: indicates if the application is accepting indication commands.

    Private methods:
        _add_objects: adds objects to the main window.
        _configure_popups: configures the pop-up windows.
        _show_popup: shows one of the pop-up windows.
        _hide_popup: hides one of the pop-up windows.
        _show_password: show the password (a sequence of color windows).
        _complete_password: completes the remaining fields of the password.
        _check_color: checks if given color is the one in the current field.
        _check_state: checks the state of the password.
        _indicated_callback: callback method to get the gesture command.
        _password_callback: callback method to get commands for the application.
        _update_virtual_agent: update the virtual agent's labels.
        _update_squares: updates the square labels.

    Public methods:
        get_data: returns the application data.
        set_password: sets the password for the application.
        set_virtual_agent: set the current virtual agent.
        fill_fields: fills some fields of the password.
        update_all: updates all the windows.
        destroy_all: destroys all the windows.

    Published topics:
        /check_indication: to publish if the last indication was correct or not.
        /status_password: to publish the status of the application.
    Subscribed topics:
        /indicated: to get the gesture indication.
        /password: to get the password.
        /password_commands: to get commands for the application.
        /virtual_agent: to get the current virtual agent.
    """

    def _add_objects(self):
        """
        Adds the objects to the main window.

        :return:
        """
        # Border size
        border = int(self._window_width / 50)

        # Dimensions for the header with information about the virtual agent.
        header_height = int(self._window_height / 3 - 3 * border / 2)
        header_width = self._window_width - 2 * border

        # Adding the virtual agent's face image.
        path = self._pkg_path + '/src/images/'
        va_image = Image.open(path + "luna.png")
        va_image_height = int(header_height / 2)
        va_image_width = int(
            va_image_height * (va_image.width / va_image.height))
        va_image = va_image.resize((va_image_width, va_image_height),
                                   Image.ANTIALIAS)
        self._va_image_photo = ImageTk.PhotoImage(va_image)
        self._va_image_label = Label(self._main_window,
                                     image=self._va_image_photo,
                                     width=va_image_width,
                                     height=va_image_height)
        self._va_image_label.place(x=self._window_width - 3 * border - va_image_width,
                                   y=border + int(va_image_height / 2))

        # Adding the virtual agent's name.
        va_name = "LUNA"
        self._va_text_label = Label(self._main_window, text=va_name,
                                    font=("bold", int(va_image_height / 4)),
                                    background=self.gray)
        self._va_text_label.place(
            x=self._window_width - 4 * border - va_image_width - self._va_text_label.winfo_reqwidth(),
            y=border + va_image_height - int(
                self._va_text_label.winfo_reqheight() / 2))

        # Adding text about to insert the password.
        password_text_label = Label(self._main_window, text="Insira a senha:",
                                    font=("bold", int(va_image_height / 4)),
                                    background=self.gray)
        password_text_label.place(x=border,
                                  y=border + 2 * va_image_height)

        # Adding the empty squares for the password.
        path = self._pkg_path + '/src/images/'
        square_image = Image.open(path + "square.png")
        possible_height = self._window_height - 3 * border - header_height
        possible_width = int((self._window_width - (
                    self._num_fields + 1) * border) / self._num_fields)
        square_side = min(possible_height, possible_width)
        square_image = square_image.resize((square_side, square_side),
                                           Image.ANTIALIAS)
        self._square_photo = ImageTk.PhotoImage(square_image)
        remaining_height = self._window_height - border - header_height - password_text_label.winfo_reqheight()
        self._square_labels = self._num_fields * [0]
        for i in range(0, self._num_fields):
            self._square_labels[i] = Label(self._main_window,
                                           image=self._square_photo,
                                           background=self.gray,
                                           width=square_side,
                                           height=square_side)
            self._square_labels[i].place(x=(i + 1) * border + i * square_side,
                                         y=self._window_height - int((remaining_height - square_side) / 2) - square_side)

    def _configure_popups(self):
        """
        Creates and configures the pop-up windows.

        :return:
        """
        # Dimensions and position.
        popup_width = int(3 * self._square_photo.width() / 2)
        popup_height = popup_width
        popup_x = int(self._window_width / 2 - popup_width / 2)
        popup_y = int(self._window_height / 2 - popup_height / 2)
        popup_settings = (popup_width, popup_height, popup_x, popup_y)

        self._popup_windows = self._num_fields * [0]
        for i in range(0, self._num_fields):
            self._popup_windows[i] = Toplevel(self._main_window)
            self._popup_windows[i].title(str(i + 1))
            self._popup_windows[i].geometry("%dx%d+%d+%d" % popup_settings)
            self._popup_windows[i].configure(background=self.gray)
            self._popup_windows[i].withdraw()

    def _show_popup(self, index):
        """
        Shows one of the pop-up windows.

        :param index: the index of the pop-up window to be shown.
        :return:
        """
        self._popup_index = index
        self._popup_windows[index].update()
        self._popup_windows[index].deiconify()

    def _hide_popup(self, index):
        """
        Hides one of the pop-up windows.

        :param index: the index of the pop-up window to be hidden.
        :return:
        """
        self._popup_windows[index].withdraw()

    def _show_password(self, duration):
        """
        Shows the password. Each color appears during 'duration' seconds.

        :param duration: the time that each color should appear.
        :return:
        """
        if self._password:
            for i in range(0, self._num_fields):
                self._popup_windows[i].configure(background=self._password[i])

            duration = int(duration * 1000)  # Time in milliseconds.

            self._show_popup(0)
            self._main_window.after(duration, lambda: self._hide_popup(0))

            for i in range(1, self._num_fields):
                self._main_window.after(i * duration,
                                        lambda: self._show_popup(self._popup_index + 1))
                self._main_window.after((i + 1) * duration,
                                        lambda: self._hide_popup(self._popup_index))
        else:
            print("\033[93m{}\033[00m".format(
                "ERROR: No password was configured yet."))

    def _complete_password(self):
        """
        Completes the remaining fields of the password.

        :return:
        """
        current = self._current_field
        for i in range(current, self._num_fields):
            self._check_color(self._password[i])

    def _check_color(self, color):
        """
        Checks if a given color is the one in the current field of the password.
        If it is, adds the color and moves to the next one.

        :param color: color to be checked.
        :return:
        """
        if self._current_field != self._num_fields:
            if color == self._password[self._current_field]:
                self._square_bg[self._current_field] = color
                self._current_field = self._current_field + 1

                audio = self._pkg_path + '/src/audios/correct_sound.mp3'
                audio_thread = Thread(target=playsound, args=[audio])
                audio_thread.start()

                self._check_pub.publish(1)
            else:
                audio = self._pkg_path + '/src/audios/wrong_sound.mp3'
                audio_thread = Thread(target=playsound, args=[audio])
                audio_thread.start()

                self._check_pub.publish(-1)

    def _check_state(self):
        """
        Checks the state of the application and updates the state attribute.
        State is 1 if all the answers were added, 0 if not.

        :return:
        """
        if self._current_field == 4:
            self._state = 1
        else:
            self._state = 0

    def _indicated_callback(self, msg):
        """
        Callback method to get the gesture command.

        :param msg: a std_msgs/String message with the indicated color.
        :return:
        """
        if self.accepting_indication == 1:
            if msg.data == 'blue':
                self._check_color(self.blue)
            if msg.data == 'yellow':
                self._check_color(self.yellow)
            if msg.data == 'white':
                self._check_color(self.white)
            if msg.data == 'black':
                self._check_color(self.black)

    def _password_callback(self, msg):
        """
        Callback method to get commands for the application.

        If msg.data > 0: show password with msg.data as duration between colors.
        If msg.data == 0: enable indication commands.
        If msg.data == -1: disable indication commands.
        If msg.data == -2: complete remaining fields of the password.

        :param msg: a std_msgs/Float32 message with commands for the application.
        :return:
        """
        if msg.data > 0:
            self._password_duration = msg.data
        if msg.data == 0:
            self.accepting_indication = 1
        if msg.data == -1:
            self.accepting_indication = 0
        if msg.data == -2:
            self._complete_password()

    def _update_virtual_agent(self):
        """
        Updates the virtual agent's labels.

        :param:
        :return:
        """
        # Updates the text label with the name.
        self._va_text_label.config(text=self._virtual_agent.upper())

        # Updates the image label with the face.
        path = self._pkg_path + '/src/images/'
        va_path = path + self._virtual_agent + '.png'
        va_image = Image.open(va_path)
        self._va_image_label.winfo_reqwidth()
        va_image = va_image.resize((self._va_image_photo.width(),
                                    self._va_image_photo.height()),
                                   Image.ANTIALIAS)
        self._va_image_photo = ImageTk.PhotoImage(va_image)
        self._va_image_label.config(image=self._va_image_photo)

    def _update_squares(self):
        """
        Updates the square labels.

        :return:
        """
        # Updating the background colors.
        for i in range(0, self._num_fields):
            self._square_labels[i].configure(background=self._square_bg[i])

        # Updating the state.
        if self._current_field == 4:
            for i in range(0, self._num_fields):
                self._square_labels[i].config(state=DISABLED)

    def get_data(self):
        """
        Returns the application data.

        :return: a list with the password, the virtual agent and the current state.
        """
        return [self._password, self._virtual_agent,
                self._current_field, self.accepting_indication]

    def set_password(self, password):
        """
        Sets the password for the application.

        :param password: the password.
        :return:
        """
        if type(password) == Int16MultiArray:
            # If the parameter is a std_msgs/Int16MultiArray message.
            data = password.data
            password = []
            for i in data:
                if i == 1:
                    password.append(self.blue)
                if i == 2:
                    password.append(self.yellow)
                if i == 3:
                    password.append(self.white)
                if i == 4:
                    password.append(self.black)
        self._password = password

    def set_virtual_agent(self, name):
        """
        Sets the current virtual agent.

        :param name: the name of the virtual agent.
        :return:
        """
        if type(name) is String:
            self._virtual_agent = name.data
        else:
            self._virtual_agent = name

    def fill_fields(self, fields):
        """
        Fills some fields of the password.

        :param fields: the number of fields that should be filled.
        :return:
        """
        for i in range(0, fields):
            while not self._password:
                pass
            self._square_bg[self._current_field] = self._password[self._current_field]
            self._current_field = self._current_field + 1

    def update_all(self):
        """
        Updates all the windows. This method should be called inside a while
        loop in the main script.

        :return:
        """
        self._update_squares()
        self._check_state()
        if self._virtual_agent != 0:
            self._update_virtual_agent()

        self._main_window.update_idletasks()
        #self._main_window.update()

        if self._password_duration != 0:
            self._show_password(self._password_duration)
            self._password_duration = 0

        for i in range(0, self._num_fields):
            self._popup_windows[i].update_idletasks()
            self._popup_windows[i].update()

        # Publishes the status of the application.
        if self._state == 0:
            self._status_pub.publish(self._current_field)
        else:
            self._status_pub.publish(-1)

    def destroy_all(self):
        """
        Destroys the main window and hence all the pop-up windows.

        :return:
        """
        self._main_window.destroy()

    def __init__(self, window=0):
        """
        Constructor method of the class.
        """
        # Getting the path for the ROS package.
        rospack = rospkg.RosPack()
        self._pkg_path = rospack.get_path('screen')

        # Defining the codes of the colors.
        self.gray = "#dcdcdc"
        self.blue = "#46d6ff"
        self.yellow = "#ffff32"
        self.white = "#ffffff"
        self.black = "#000000"

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
        self._main_window.configure(background=self.gray)

        # Setting the number of fields for the password.
        self._num_fields = 4

        # Index of the last opened pop-up window.
        self._popup_index = -1

        self._add_objects()
        self._configure_popups()

        # Password is initialized empty.
        self._password = []

        # Current field of the password.
        self._current_field = 0

        # Current state of the application.
        # 1 if finished, 0 if not.
        self._state = 0

        # Square labels' background colors.
        self._square_bg = [self.gray] * self._num_fields

        # Current virtual agent.
        self._virtual_agent = 'luna'

        # The duration of each color in the screen while showing the password.
        self._password_duration = 0

        # Indicates if the application is accepting indication commands (1) or
        # not (0).
        self.accepting_indication = 0

        # Publisher to inform if the indication was correct (1) or not (-1).
        self._check_pub = rospy.Publisher('/check_indication', Int16,
                                          queue_size=1, latch=True)

        # Publisher to inform the application status.
        self._status_pub = rospy.Publisher('/status_password', Int16,
                                           queue_size=1, latch=True)

        # Subscriber for the indicated color.
        rospy.Subscriber('/indicated', String, self._indicated_callback)

        # Subscriber to set the password.
        rospy.Subscriber('/password', Int16MultiArray, self.set_password)

        # Subscriber for commands about the password application.
        rospy.Subscriber('/password_commands', Float32, self._password_callback)

        # Subscriber for the current virtual agent.
        rospy.Subscriber('/virtual_agent', String, self.set_virtual_agent)