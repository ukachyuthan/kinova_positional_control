#!/usr/bin/env python
""" Implements visual assistance for bi-manual control experiment

Author(s): Achyuthan Unni Krishnan

TODO: Create a publisher in the right and left arm control files that publish 
the state of if they used scaling, orientation control or 
engagement/disengagement

TODO: Create stream to Unity
"""

# # Standart libraries:
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import cv2.aruco as aruco
from std_msgs.msg import String, Float64, Float64MultiArray
from geometry_msgs.msg import TransformStamped, Pose
import copy
import socket
import pickle
import os

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (Bool)
from std_srvs.srv import (SetBool)


class VisualAssistance:
    """
    
    """

    def __init__(
        self,
        node_name,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.image_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.key = 0
        self.node_rate = rospy.Rate(1000)
        # self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.server_socket.bind(('130.215.181.94', 2332))
        # self.server_socket.listen(5)

        self.out_send = cv2.VideoWriter(
            'appsrc ! videoconvert ! video/x-raw,format=YUY2 ! jpegenc ! rtpjpegpay ! udpsink host=130.215.181.94 port=2332 sync=false',
            cv2.CAP_GSTREAMER, 0, 25, (640, 480)
        )

        # left parameters:
        self.left_scaling = 0
        self.left_disengage = 0
        self.left_orientation = 0
        self.left_scaling_avail = 0
        self.left_disengage_avail = 0
        self.left_emergency = 0

        # right parameters:
        self.right_scaling = 0
        self.right_disengage = 0
        self.right_orientation = 0
        self.right_scaling_avail = 0
        self.right_disengage_avail = 0
        self.right_emergency = 0

        # # Public variables:
        self.public_variable = 1

        # # Topic publisher:

        # # Topic subscriber:
        rospy.Subscriber(
            '/camera/color/image_raw',
            Image,
            self.callback_image,
        )
        rospy.Subscriber(
            '/scaling_values',
            Float64MultiArray,
            self.callback_scaling_parameters,
        )

        # # Left arm subscriber:
        rospy.Subscriber(
            '/left/ui_parameters',
            Float64MultiArray,
            self.callback_left_parameters,
        )
        rospy.Subscriber(
            '/left/emergency_topic',
            Float64,
            self.callback_emergency_left,
        )

        # # Right arm subscriber:
        rospy.Subscriber(
            '/right/ui_parameters',
            Float64MultiArray,
            self.callback_right_parameters,
        )
        rospy.Subscriber(
            '/right/emergency_topic',
            Float64,
            self.callback_emergency_right,
        )

    # # Topic callbacks:
    def callback_emergency_right(self, message):

        self.right_emergency = message.data

    def callback_emergency_left(self, message):

        self.left_emergency = message.data

    def callback_scaling_parameters(self, message):

        self.left_scaling_avail = message.data[0]
        self.right_scaling_avail = message.data[1]
        self.left_disengage_avail = message.data[2]
        self.right_disengage_avail = message.data[3]

    def callback_left_parameters(self, message):

        self.left_scaling = message.data[0]
        self.left_disengage = message.data[1]
        self.left_orientation = message.data[2]

    def callback_right_parameters(self, message):

        self.right_scaling = message.data[0]
        self.right_disengage = message.data[1]
        self.right_orientation = message.data[2]

    def callback_image(self, message):
        """

        """
        br = CvBridge()

        self.image_frame = br.imgmsg_to_cv2(message, "bgr8")

    def image_gen(self):

        # Receiving and displaying the video frame
        current_frame = copy.deepcopy(self.image_frame)

        # Defining sidebar parameters
        sidebar_width = 150
        sidebar = np.zeros(
            (current_frame.shape[0], sidebar_width, 3), dtype=np.uint8
        )

        # Drawing squares in the sidebar
        square_height = sidebar.shape[0] // 3
        square_size = min(sidebar_width, square_height)
        square_color = (255, 255, 255)  # Changed color to gray

        # Adding black borders to the squares
        border_thickness = 2
        border_color = (0, 0, 0)  # Black border color

        scale_text_color = (128, 128, 128)
        disengage_text_color = (128, 128, 128)
        orientation_text_color = (128, 128, 128)

        if self.left_scaling_avail == 1 or self.right_scaling_avail == 1:

            scale_text_color = (0, 0, 255)

        if self.left_scaling == 1 or self.right_scaling == 1:

            scale_text_color = (0, 255, 0)

        if self.left_disengage_avail == 1 or self.right_disengage_avail == 1:

            disengage_text_color = (0, 0, 255)

        if self.left_disengage == 1 or self.right_disengage == 1:

            disengage_text_color = (0, 255, 0)

        if self.left_orientation == 1 or self.right_orientation == 1:

            orientation_text_color = (0, 255, 0)

        robot_color = cv2.imread(
            "/home/rbemotion/Downloads/robot_color.jpg", cv2.IMREAD_UNCHANGED
        )

        robot_gray_unavailable = cv2.imread(
            "/home/rbemotion/Downloads/robot_gray_unavailable.jpg",
            cv2.IMREAD_UNCHANGED
        )

        robot_gray_available = cv2.imread(
            "/home/rbemotion/Downloads/robot_gray_available.jpg",
            cv2.IMREAD_UNCHANGED
        )

        mirrored_image_color = cv2.flip(robot_color, 1)
        mirrored_image_gray_unavailable = cv2.flip(robot_gray_unavailable, 1)
        mirrored_image_gray_available = cv2.flip(robot_gray_available, 1)

        robot_color_resized = cv2.resize(robot_color, (64, 64))
        robot_gray_unavailable_resized = cv2.resize(
            robot_gray_unavailable,
            (64, 64),
        )
        robot_gray_available_resized = cv2.resize(
            robot_gray_available,
            (64, 64),
        )
        mirrored_image_color_resized = cv2.resize(
            mirrored_image_color,
            (64, 64),
        )
        mirrored_image_gray_unavailable_resized = cv2.resize(
            mirrored_image_gray_unavailable,
            (64, 64),
        )
        mirrored_image_gray_available_resized = cv2.resize(
            mirrored_image_gray_available,
            (64, 64),
        )

        # First square
        cv2.rectangle(
            sidebar, (0, 0), (square_size, square_size), border_color, -1
        )  # Black border
        cv2.rectangle(
            sidebar, (border_thickness, border_thickness),
            (square_size - border_thickness, square_size - border_thickness),
            square_color, -1
        )

        cv2.putText(
            sidebar, "SCALING", (4, 45), cv2.FONT_HERSHEY_SIMPLEX, 1,
            scale_text_color, 3, cv2.LINE_AA
        )

        if self.left_scaling == 1:
            display_image_left = copy.deepcopy(robot_color_resized)

        elif self.left_scaling_avail == 1:
            display_image_left = copy.deepcopy(robot_gray_available_resized)

        else:
            display_image_left = copy.deepcopy(robot_gray_unavailable_resized)

        if self.right_scaling == 1:
            display_image_right = copy.deepcopy(mirrored_image_color_resized)

        elif self.right_scaling_avail == 1:
            display_image_right = copy.deepcopy(
                mirrored_image_gray_available_resized
            )

        else:
            display_image_right = copy.deepcopy(
                mirrored_image_gray_unavailable_resized
            )

        sidebar[84:148, 5:69] = display_image_left
        sidebar[84:148, 81:145] = display_image_right

        # Second square
        cv2.rectangle(
            sidebar, (0, square_height), (square_size, 2 * square_height),
            border_color, -1
        )  # Black border
        cv2.rectangle(
            sidebar, (border_thickness, square_height + border_thickness), (
                square_size - border_thickness,
                2 * square_height - border_thickness
            ), square_color, -1
        )

        cv2.putText(
            sidebar, "ACTIVE", (4, 200), cv2.FONT_HERSHEY_SIMPLEX, 1,
            disengage_text_color, 3, cv2.LINE_AA
        )

        if self.left_emergency == 1:
            display_image_left = copy.deepcopy(robot_gray_unavailable_resized)
        elif self.left_emergency == 0:
            if self.left_disengage == 1:
                display_image_left = copy.deepcopy(robot_color_resized)
            elif self.left_disengage_avail == 1:
                display_image_left = copy.deepcopy(robot_gray_available_resized)
            else:
                display_image_left = copy.deepcopy(
                    robot_gray_unavailable_resized
                )

        if self.right_emergency == 1:
            display_image_right = copy.deepcopy(
                mirrored_image_gray_unavailable_resized
            )
        elif self.right_emergency == 0:
            if self.right_disengage == 1:
                display_image_right = copy.deepcopy(
                    mirrored_image_color_resized
                )
            elif self.right_disengage_avail == 1:
                display_image_right = copy.deepcopy(
                    mirrored_image_gray_available_resized
                )
            else:
                display_image_right = copy.deepcopy(
                    mirrored_image_gray_unavailable_resized
                )

        sidebar[248:312, 5:69] = display_image_left
        sidebar[248:312, 81:145] = display_image_right

        # Third square
        cv2.rectangle(
            sidebar, (0, 2 * square_height), (square_size, 3 * square_height),
            border_color, -1
        )  # Black border
        cv2.rectangle(
            sidebar, (border_thickness, 2 * square_height + border_thickness), (
                square_size - border_thickness,
                3 * square_height - border_thickness
            ), square_color, -1
        )

        cv2.putText(
            sidebar, "T-PAD", (4, 380), cv2.FONT_HERSHEY_SIMPLEX, 1,
            orientation_text_color, 3, cv2.LINE_AA
        )

        if self.left_orientation == 1:
            display_image_left = copy.deepcopy(robot_color_resized)

        else:
            display_image_left = copy.deepcopy(robot_gray_available_resized)

        if self.right_orientation == 1:
            display_image_right = copy.deepcopy(mirrored_image_color_resized)

        else:
            display_image_right = copy.deepcopy(
                mirrored_image_gray_available_resized
            )

        sidebar[398:462, 5:69] = display_image_left
        sidebar[398:462, 81:145] = display_image_right

        # Concatenating the sidebar with the current frame
        output_frame = np.hstack((sidebar, current_frame))

        cv2.namedWindow(
            'Viewpoint', cv2.WINDOW_GUI_NORMAL
        )  # Create named window
        cv2.resizeWindow('Viewpoint', 1280, 960)  # Resize window

        output_frame_resized = cv2.resize(output_frame, (640, 480))
        cv2.imshow('Viewpoint', output_frame_resized)
        self.out_send.write(output_frame_resized)

        self.key = cv2.waitKey(1)

    def main_loop(self):

        while not rospy.is_shutdown():

            self.image_gen()
            self.node_rate.sleep()

            if self.key == 113:

                break

        self.out_send.release()
        cv2.destroyAllWindows()


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'image_node',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = "image_node"

    class_instance = VisualAssistance(node_name=node_name,)

    class_instance.main_loop()


if __name__ == '__main__':
    main()
