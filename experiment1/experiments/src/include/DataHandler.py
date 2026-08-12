import rospy
from std_msgs.msg import Int16, Float32, String, Int16MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from dqrobotics import *


class DataHandler:
    """
    DataHandler is a class to handle communication between the nodes of the
    experiment. It is responsible to connect them and make exchange of
    information possible.

    Public attributes:
        detecting_flag: indicates if a human is being detected (1) or not (0)
        pointing_flag: indicates if the human is pointing (1) or not (0).
        pointed_object: indicates the pointed object.
        check_indicated: indicates if the last pointing indication was correct (1) or not (0).
        count_errors: the number or errors during password phase.
        max_errors: maximum number of errors.
        total_errors: total number of errors.
        total_time: total time.
        start_time: to hold the start time for the total time calculation
        password_status: status of the password application.
        count_correct: to count the correct gestures.
        counting_status: status of the counting application.
        phase: current phase.
        configuration: indicates if it is the first (0), one in the middle (1), or the last configuration (2).
        virtual_agent: current virtual agent.
        va_speaking: indicates if the virtual agent is speaking (1) or not (0)
        x0_head: transformation from marker 0 to the human head.
        x0_objects: transformation from marker 0 to each of the objects of phase 1.
        password: password for phase 1.
        counting: counting values for phase 2.
        counting_images_set: images set for the counting application.
        check_input: indicates if the last input was correct (1) or not (0).
        entry_value: the value entered in the field of the counting application.
        added_entries: which entries were already filled in the counting application.
        markers_ids: markers being detected.
        id_counting: ID of the counting phase marker.
        counting_marker: indicates if the counting phase marker should be used (1) or not (0).
        id_help: ID of the help area marker.
        facial_points: indicates if the facial points references were obtained.
        looked_area: detected looked area.
        timer_gaze: timer for when the virtual agent follows the human gaze.
        last_spoken: indicates about what field was the last voice command.
        last_va_gaze: holds the last command for the virtual agent's gaze.
        log_file: log file.
        questionnaire: indicates if the questionnaire was completed.
        detection_fail: number of times that the time limit for human detection was reached in a row.
        max_detection_failures: maximum number of human detection failures to restart the application.

    Public methods:
        set_odometry_msg: sets an Odometry message from a dual quaternion.
        zero_for_phase1: restarts the attributes for the beginning of phase 1.
        zero_for_phase2: restarts the attributes for the beginning of phase 2.
        write_log: writes information in the log file

        The following methods receive messages from the subscribed topics and
        update the correspondent attributes.
            detecting_callback
            pointing_callback
            pointed_object_callback
            check_password_callback
            password_callback
            counting_callback
            check_counting_callback
            entry_value_callback
            facial_points_callback
            looked_area_callback
            speaking_callback
            head_callback
            objects_callback
            markers_ids_callback
            quest_callback

    Published topics:
        /password_commands: to send commands to the password application.
        /password: to set the password.
        /counting_commands: to send commands to the counting application.
        /counting_settings: to configure the buttons and the counting values.
        /human_gaze_commands: to send commands to the human gaze application.
        /indicated: to send the pointed object
        /virtual_agent: to set the virtual agent.
        /voice: to send voice commands to the virtual agent.
        /expression: to send expression commands to the virtual agent.
        /va_gaze: to send gaze commands to the virtual agent.
        /phase: to set the phase.
        /tracker_commands: to send commands to the tracker application.
        /record: to send commands to the video record application.
    Subscribed topics:
        /detecting_flag
        /pointing_flag
        /pointed_object
        /check_indication
        /status_password
        /status_counting
        /check_input
        /entry_value
        /ref_left
        /looked_object
        /va_speaking
        /x0_head
        /x01, /x02, /x03, /x04
        /detected_markers
        /questionnaire
    """

    def set_odometry_msg(self, dq):
        """
        Sets an Odometry message from a dual quaternion.

        :param dq: a DQ object.
        :return: the Odometry message.
        """

        msg = Odometry()
        position = dq.translation().vec4()
        orientation = dq.P().vec4()
        msg.pose.pose.position.x = position[1]
        msg.pose.pose.position.y = position[2]
        msg.pose.pose.position.z = position[3]
        msg.pose.pose.orientation.w = orientation[0]
        msg.pose.pose.orientation.x = orientation[1]
        msg.pose.pose.orientation.y = orientation[2]
        msg.pose.pose.orientation.z = orientation[3]

        return msg

    def zero_for_phase1(self):
        """
        Restarts the attributes for the beginning of phase 1.

        :return:
        """
        self.pointing_flag = 0
        self.pointed_object = 0
        self.check_indicated = 0
        self.count_errors = 0
        self.total_errors = 0
        self.password_status = 0
        self.password = []
        self.count_correct = 0

    def zero_for_phase2(self):
        """
        Restarts the attributes for the beginning of phase 2.

        :return:
        """
        self.counting_status = -2
        self.counting = [0, 0, 0, 0]
        self.facial_points = 0
        self.looked_area = 0
        self.added_entries = [0, [-1, -1, -1, -1]]
        self.last_spoken = 0
        self.timer_gaze = 0
        self.questionnaire = 0

    def write_log(self, tags, message):
        """
        Writes information in the log file.

        :param tags: the set of tags to identify the subject of the message.
        :param message: the message to be written
        :return:
        """
        self.log_file.disable(self.log_file.NOTSET)

        # Formatting the tags.
        tags_formatted = ''
        for s in tags:
            tags_formatted = tags_formatted + '[' + s + '] '

        # Writing in the log file.
        self.log_file.info('%s%s', tags_formatted, message)

        self.log_file.disable(self.log_file.WARNING)

    def detecting_callback(self, msg):
        """
        Receives a message from the topic and updates detecting_flag attribute.

        :param msg: a std_msgs/Int16 message.
        :return:
        """
        self.detecting_flag = msg.data

    def pointing_callback(self, msg):
        """
        Receives a message from the topic and updates pointing_flag attribute.

        :param msg: a std_msgs/Int16 message.
        :return:
        """
        self.pointing_flag = msg.data

    def pointed_object_callback(self, msg):
        """
        Receives a message from the topic and updates pointed_object attribute.

        :param msg: a std_msgs/Int16 message.
        :return:
        """
        self.pointed_object = msg.data

    def check_password_callback(self, msg):
        """
        Receives a message from the topic and updates check_indicated attribute.

        :param msg: a std_msgs/Int16 message.
        :return:
        """
        self.check_indicated = msg.data

    def password_callback(self, msg):
        """
        Receives a message from the topic and updates password_status attribute.

        :param msg: a std_msgs/Int16MultiArray message.
        :return:
        """
        self.password_status = msg.data

    def counting_callback(self, msg):
        """
        Receives a message from the topic and updates counting_status attribute.

        :param msg: a std_msgs/Int16 message.
        :return:
        """
        self.counting_status = msg.data

    def check_counting_callback(self, msg):
        """
        Receives a message from the topic and updates check_input attribute.

        :param msg: a std_msgs/Int16 message.
        :return:
        """
        self.check_input = msg.data

    def entry_value_callback(self, msg):
        """
        Receives a message from the topic and updates entry_value attribute.

        :param msg: a std_msgs/Int16 message.
        :return:
        """
        self.entry_value = msg.data

    def facial_points_callback(self, msg):
        """
        Receives a message from the topic that publishes the reference for the
        left eyes, indicating that the references were obtained, and updates
        facial_points attribute.

        :param msg:
        :return:
        """
        self.facial_points = 1

    def looked_area_callback(self, msg):
        """
        Receives a message from the topic and updates looked_area attribute.

        :param msg: a std_msgs/Int16 message.
        :return:
        """
        self.looked_area = msg.data

    def speaking_callback(self, msg):
        """
        Receives a message from the topic and updates va_speaking attribute.

        :param msg: a std_msgs/Int16 message.
        :return:
        """
        self.va_speaking = msg.data

    def head_callback(self, msg):
        """
        Receives a message from the topic and updates x0_head attribute.

        :param msg: a nav_msgs/Odometry message.
        :return:
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        w = msg.pose.pose.orientation.w
        wx = msg.pose.pose.orientation.x
        wy = msg.pose.pose.orientation.y
        wz = msg.pose.pose.orientation.z
        r = DQ([w, wx, wy, wz])
        p = DQ([0, x, y, z])
        self.x0_head = normalize(r + DQ.E * 0.5 * p * r)

    def objects_callback(self, msg, object):
        """
        Receives a message from the topic and updates the object's attribute.

        :param msg: a nav_msgs/Odometry message.
        :param object: number of the object to be updated.
        :return:
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        w = msg.pose.pose.orientation.w
        wx = msg.pose.pose.orientation.x
        wy = msg.pose.pose.orientation.y
        wz = msg.pose.pose.orientation.z
        r = DQ([w, wx, wy, wz])
        p = DQ([0, x, y, z])
        self.x0_objects[object-1] = normalize(r + DQ.E * 0.5 * p * r)

    def markers_ids_callback(self, msg):
        """
        Receives a message from the topic and updates markers_ids attribute.

        :param msg: a std_msgs/Int16MultiArray message.
        :return:
        """
        self.markers_ids = msg.data

    def quest_callback(self, msg):
        """
        Receives a message from the topic and updates questionnaire attribute.

        :param msg: a std_msgs/Int16 message.
        :return:
        """
        self.questionnaire = msg.data

    def __init__(self):
        # Indicates if a human is being detected (1) or not (0).
        self.detecting_flag = 0

        # Indicates if the human is pointing (1) or not (0).
        self.pointing_flag = 0

        # Indicates the pointed object.
        self.pointed_object = 0

        # Indicates if the last pointing indication was correct (1) or not (0).
        self.check_indicated = 0

        # The number or errors during password phase.
        self.count_errors = 0

        # Maximum number of errors.
        self.max_errors = 3

        # Total number of errors.
        self.total_errors = 0

        # Total time.
        # total_time: accumulated time.
        # total_time[1]: total time of phase 1.
        # total_time[2]: total time of phase 2.
        self.total_time = [0, 0, 0]

        # To hold the start time for the total time calculation.
        self.start_time = 0

        # Status of the password application.
        # If password_status >= 0: the current password field.
        # If password_status == -1: finished password.
        self.password_status = 0

        # To count the correct gestures.
        self.count_correct = 0

        # Status of the counting application.
        # If counting_status > 0: the current active entry field.
        # If counting_status == 0: buttons enabled, no entry active.
        # If counting_status == -1: buttons disabled.
        self.counting_status = -2

        # Current phase.
        self.phase = 0

        # Indicates if it is the first (0), one in the middle (1), or the last
        # configuration (2).
        self.configuration = 0

        # Current virtual agent.
        self.virtual_agent = 0

        # Indicates if the virtual agent is speaking (1) or not (0).
        # If va_speaking == -1: the audio file was not found.
        self.va_speaking = 0

        # Transformation from marker 0 to the human head.
        self.x0_head = DQ([1])

        # Transformation from marker 0 to each of the objects of phase 1.
        self.x0_objects = [DQ([1])] * 4

        # Password for phase 1.
        self.password = []

        # Counting values for phase 2.
        self.counting = [0, 0, 0, 0]

        # Images set for the counting application.
        self.counting_images_set = 0

        # Indicates if the last input was correct (1) or not (0).
        self.check_input = -1

        # The value entered in the field of the counting application.
        self.entry_value = -1

        # Which entries were already filled in the counting application.
        # If added_entries[1][i] == -1: the ith field was not filled yet.
        # If added_entries[1][i] == 0: the ith input was incorrect.
        # If added_entries[1][i] == 1: the ith input was correct.
        self.added_entries = [0, [-1, -1, -1, -1]]

        # Markers being detected.
        self.markers_ids = []

        # ID of the counting phase marker.
        self.id_counting = 7

        # Indicates if the counting phase marker should be used (1) or not (0) to indicate the human position.
        self.counting_marker = 1

        # ID of the help area marker.
        self.id_help = 8

        # Indicates if the facial points references were obtained.
        self.facial_points = 0

        # Detected looked area.
        self.looked_area = 0

        # Timer for when the virtual agent follows the human gaze in phase 2.
        self.timer_gaze = 0

        # Indicates about what field was the last voice command.
        self.last_spoken = 0

        # Holds the last command for the virtual agent's gaze.
        # If last_va_gaze == 1: virtual agent looking to the human.
        # If last_va_gaze == 2: virtual agent looking to the next correct object.
        # If last_va_gaze == 3: virtual agent looking to the open entry field.
        # If last_va_gaze == 4: virtual agent following human gaze to one of the
        # entry fields.
        self.last_va_gaze = 0

        # Log file.
        self.log_file = []

        # Indicates if the questionnaire about the virtual agent was
        # completed (1).
        self.questionnaire = 0

        # Number of times that the time limit for the human detection was
        # reached in a row.
        self.detection_fail = 0

        # Maximum number of human detection failures to restart the application.
        self.max_detection_failures = 1

        # Publisher to send commands to the password application.
        self.password_pub = rospy.Publisher('/password_commands', Float32,
                                            queue_size=1, latch=True)

        # Publisher to set the password.
        self.pass_pub = rospy.Publisher('/password', Int16MultiArray,
                                        queue_size=1, latch=True)

        # Publisher to send commands to the counting application.
        self.counting_pub = rospy.Publisher('/counting_commands', Int16,
                                            queue_size=10)

        # Publisher to send commands to control clicks in the counting application.
        self.control_clicks_pub = rospy.Publisher('/control_clicks', Int16,
                                                  queue_size=1, latch=True)

        # Publisher to configure the buttons and the counting values.
        self.counting_settings_pub = rospy.Publisher('/counting_settings',
                                                     Int16MultiArray,
                                                     queue_size=1, latch=True)

        # Publisher to send commands to the human gaze application.
        self.human_gaze_pub = rospy.Publisher('/human_gaze_commands', Int16,
                                              queue_size=1, latch=True)

        # Publisher for the pointing indication.
        self.indication_pub = rospy.Publisher('/indicated', String,
                                              queue_size=1, latch=True)

        # Publisher to set the virtual agent.
        self.va_pub = rospy.Publisher('/virtual_agent', String,
                                      queue_size=1, latch=True)

        # Publisher to send voice commands to the virtual agent.
        self.voice_pub = rospy.Publisher('/voice', String,
                                         queue_size=10, latch=True)

        # Publisher to send expressions commands to the virtual agent.
        self.expression_pub = rospy.Publisher('/expression', String,
                                              queue_size=1, latch=True)

        # Publisher to gaze commands to the virtual agent.
        self.va_gaze_pub = rospy.Publisher('/va_gaze', Odometry,
                                           queue_size=1)

        # Publisher to set the phase.
        self.phase_pub = rospy.Publisher('/phase', Int16,
                                         queue_size=1, latch=True)

        # Publisher to send commands to the tracker application.
        self.tracker_pub = rospy.Publisher('/tracker_commands', Int16,
                                           queue_size=1, latch=True)

        # Publisher to send commands to the screen main application.
        self.screen_pub = rospy.Publisher('/screen_commands', Int16,
                                          queue_size=1, latch=True)

        # Publisher to send commands to the video record application.
        self.record_pub = rospy.Publisher('/record', Int16,
                                          queue_size=1, latch=True)

        # Subscribers to update the attributes.
        rospy.Subscriber('/detecting_flag', Int16, self.detecting_callback)
        rospy.Subscriber('/pointing_flag', Int16, self.pointing_callback)
        rospy.Subscriber('/pointed_object', Int16, self.pointed_object_callback)
        rospy.Subscriber('/check_indication', Int16, self.check_password_callback)
        rospy.Subscriber('/status_password', Int16, self.password_callback)
        rospy.Subscriber('/status_counting', Int16, self.counting_callback)
        rospy.Subscriber('/check_input', Int16, self.check_counting_callback)
        rospy.Subscriber('/entry_value', Int16, self.entry_value_callback)
        rospy.Subscriber('/ref_left', Point, self.facial_points_callback)
        rospy.Subscriber('/looked_object', Int16, self.looked_area_callback)
        rospy.Subscriber('/va_speaking', Int16, self.speaking_callback)
        rospy.Subscriber('/x0_head', Odometry, self.head_callback)
        rospy.Subscriber('/x01', Odometry, self.objects_callback, 1)
        rospy.Subscriber('/x02', Odometry, self.objects_callback, 2)
        rospy.Subscriber('/x03', Odometry, self.objects_callback, 3)
        rospy.Subscriber('/x04', Odometry, self.objects_callback, 4)
        rospy.Subscriber('/detected_markers', Int16MultiArray,
                         self.markers_ids_callback)
        rospy.Subscriber('/questionnaire', Int16, self.quest_callback)




