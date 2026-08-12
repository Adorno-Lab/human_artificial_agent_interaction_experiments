#!/usr/bin/env python2.7
import roslib
#roslib.load_manifest('tracker')
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16


def set_msg(translation, rotation):
    """
    Set the PoseStamped message.

    :param translation: translation.
    :param rotation: rotation.
    :return: message to be published.
    """
    message = PoseStamped()
    message.header.frame_id = "/openni_depth_frame"
    message.pose.position.x = translation[0]
    message.pose.position.y = translation[1]
    message.pose.position.z = translation[2]
    message.pose.orientation.x = rotation[1]
    message.pose.orientation.y = rotation[2]
    message.pose.orientation.z = rotation[3]
    message.pose.orientation.w = rotation[0]

    return message


def main():
    rospy.init_node('tracker_tf_listener')

    # Once the listener is created, it starts receiving tf transformations
    # over the wire and buffers them.
    listener = tf.TransformListener()

    # PoseStamped publishers for each joint.
    head_pub = rospy.Publisher('head', PoseStamped, queue_size=1)
    neck_pub = rospy.Publisher('neck', PoseStamped, queue_size=1)
    torso_pub = rospy.Publisher('torso', PoseStamped, queue_size=1)
    left_shoulder_pub = rospy.Publisher('left_shoulder', PoseStamped, queue_size=1)
    left_elbow_pub = rospy.Publisher('left_elbow', PoseStamped, queue_size=1)
    left_hand_pub = rospy.Publisher('left_hand', PoseStamped, queue_size=1)
    right_shoulder_pub = rospy.Publisher('right_shoulder', PoseStamped, queue_size=1)
    right_elbow_pub = rospy.Publisher('right_elbow', PoseStamped, queue_size=1)
    right_hand_pub = rospy.Publisher('right_hand', PoseStamped, queue_size=1)
    left_hip_pub = rospy.Publisher('left_hip', PoseStamped, queue_size=1)
    left_knee_pub = rospy.Publisher('left_knee', PoseStamped, queue_size=1)
    left_foot_pub = rospy.Publisher('left_foot', PoseStamped, queue_size=1)
    right_hip_pub = rospy.Publisher('right_hip', PoseStamped, queue_size=1)
    right_knee_pub = rospy.Publisher('right_knee', PoseStamped, queue_size=1)
    right_foot_pub = rospy.Publisher('right_foot', PoseStamped, queue_size=1)

    # Three last values of t_head[0] (two users).
    three_last = [[0, 0, 0], [0, 0, 0]]

    # Publisher for the flag indicating if a human is being tracked (1) or
    # not (0).
    detecting = 0
    detecting_pub = rospy.Publisher('detecting_flag', Int16, queue_size=10)

    user = 1

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:
            # We want to transform from /openni_depth_frame to /joint frame.
            # It will return two lists: first one is the (x,y,z) linear
            # transformation of the child frame with respect to the parent and
            # the second one is the (x,y,z,w) rotation quaternion from the
            # parent frame to the child frame.
            (t_head, r_head) = listener.lookupTransform(
                '/openni_depth_frame', '/head_' + str(user), rospy.Time(0))
            (t_neck, r_neck) = listener.lookupTransform(
                '/openni_depth_frame', '/neck_' + str(user), rospy.Time(0))
            (t_torso, r_torso) = listener.lookupTransform(
                '/openni_depth_frame', '/torso_' + str(user), rospy.Time(0))
            (t_left_shoulder, r_left_shoulder) = listener.lookupTransform(
                '/openni_depth_frame', '/left_shoulder_' + str(user), rospy.Time(0))
            (t_left_elbow, r_left_elbow) = listener.lookupTransform(
                '/openni_depth_frame', '/left_elbow_' + str(user), rospy.Time(0))
            (t_left_hand, r_left_hand) = listener.lookupTransform(
                '/openni_depth_frame', '/left_hand_' + str(user), rospy.Time(0))
            (t_right_shoulder, r_right_shoulder) = listener.lookupTransform(
                '/openni_depth_frame', '/right_shoulder_' + str(user), rospy.Time(0))
            (t_right_elbow, r_right_elbow) = listener.lookupTransform(
                '/openni_depth_frame', '/right_elbow_' + str(user), rospy.Time(0))
            (t_right_hand, r_right_hand) = listener.lookupTransform(
                '/openni_depth_frame', '/right_hand_' + str(user), rospy.Time(0))
            (t_left_hip, r_left_hip) = listener.lookupTransform(
                '/openni_depth_frame', '/left_hip_' + str(user), rospy.Time(0))
            (t_left_knee, r_left_knee) = listener.lookupTransform(
                '/openni_depth_frame', '/left_knee_' + str(user), rospy.Time(0))
            (t_left_foot, r_left_foot) = listener.lookupTransform(
                '/openni_depth_frame', '/left_foot_' + str(user), rospy.Time(0))
            (t_right_hip, r_right_hip) = listener.lookupTransform(
                '/openni_depth_frame', '/right_hip_' + str(user), rospy.Time(0))
            (t_right_knee, r_right_knee) = listener.lookupTransform(
                '/openni_depth_frame', '/right_knee_' + str(user), rospy.Time(0))
            (t_right_foot, r_right_foot) = listener.lookupTransform(
                '/openni_depth_frame', '/right_foot_' + str(user), rospy.Time(0))

            thead = t_head[:]
            rhead = r_head[:]
            tneck = t_neck[:]
            rneck = r_neck[:]
            ttorso = t_torso[:]
            rtorso = r_torso[:]
            tleftshoulder = t_left_shoulder[:]
            rleftshoulder = r_left_shoulder[:]
            tleftelbow = t_left_elbow[:]
            rleftelbow = r_left_elbow[:]
            tlefthand = t_left_hand[:]
            rlefthand = r_left_hand[:]
            trightshoulder = t_right_shoulder[:]
            rrightshoulder = r_right_shoulder[:]
            trightelbow = t_right_elbow[:]
            rrightelbow = r_right_elbow[:]
            trighthand = t_right_hand[:]
            rrighthand = r_right_hand[:]
            tlefthip = t_left_hip[:]
            rlefthip = r_left_hip[:]
            tleftknee = t_left_knee[:]
            rleftknee = r_left_knee[:]
            tleftfoot = t_left_foot[:]
            rleftfoot = r_left_foot[:]
            trighthip = t_right_hip[:]
            rrighthip = r_right_hip[:]
            trightknee = t_right_knee[:]
            rrightknee = r_right_knee[:]
            trightfoot = t_right_foot[:]
            rrightfoot = r_right_foot[:]

            # t_head[0] is used to check if the message is being updated or not.
            # If current value is equal to all the three last ones, then the
            # flag for detecting is turned off (detecting = 0). If the value is
            # different of any of them, the flag is turned on (detecting = 1).
            if three_last[user - 1].count(t_head[0]) < len(three_last[user - 1]) - 1:
                detecting = 1
                for j in range(0, len(three_last[user - 1]) - 1):
                    three_last[user - 1][j] = three_last[user - 1][j + 1]
                three_last[user - 1][len(three_last[user - 1]) - 1] = t_head[0]
            else:
                detecting = 0
                if user == 1:
                    # Try another user.
                    user = 2
                else:
                    user = 1

            detecting_pub.publish(detecting)

            # If detecting, sets the message and publishes the joints poses.
            if detecting == 1:
                msg = set_msg(thead, rhead)
                head_pub.publish(msg)
                msg = set_msg(tneck, rneck)
                neck_pub.publish(msg)
                msg = set_msg(ttorso, rtorso)
                torso_pub.publish(msg)
                msg = set_msg(tleftshoulder, rleftshoulder)
                left_shoulder_pub.publish(msg)
                msg = set_msg(tleftelbow, rleftelbow)
                left_elbow_pub.publish(msg)
                msg = set_msg(tlefthand, rlefthand)
                left_hand_pub.publish(msg)
                msg = set_msg(trightshoulder, rrightshoulder)
                right_shoulder_pub.publish(msg)
                msg = set_msg(trightelbow, rrightelbow)
                right_elbow_pub.publish(msg)
                msg = set_msg(trighthand, rrighthand)
                right_hand_pub.publish(msg)
                msg = set_msg(tlefthip, rlefthip)
                left_hip_pub.publish(msg)
                msg = set_msg(tleftknee, rleftknee)
                left_knee_pub.publish(msg)
                msg = set_msg(tleftfoot, rleftfoot)
                left_foot_pub.publish(msg)
                msg = set_msg(trighthip, rrighthip)
                right_hip_pub.publish(msg)
                msg = set_msg(trightknee, rrightknee)
                right_knee_pub.publish(msg)
                msg = set_msg(trightfoot, rrightfoot)
                right_foot_pub.publish(msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass