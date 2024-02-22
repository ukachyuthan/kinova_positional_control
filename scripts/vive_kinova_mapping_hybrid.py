#!/usr/bin/env python
"""Implements HTC Vive controller to teleoperation mapping module.

TODO: Add detailed description.

Author (s):
    1. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.
    2. Lorena Genua (lorenagenua@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.

"""

import rospy
import numpy as np
import transformations
import copy

from std_msgs.msg import (Bool)
from std_srvs.srv import (SetBool)
from geometry_msgs.msg import (Pose)

from oculus_ros.msg import (ControllerButtons)

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped, Pose
from tf.transformations import quaternion_from_euler, quaternion_multiply


class ViveMapping:
    """
    
    """

    def __init__(
        self,
        robot_name='my_gen3',
        controller_side='right',
        headset_mode='table',
    ):
        """
        
        """

        if controller_side not in ['right', 'left']:
            raise ValueError(
                'controller_side should be either "right" or "left".'
            )

        # # Private constants:

        # # Public constants:
        self.ROBOT_NAME = robot_name
        self.CONTROLLER_SIDE = controller_side
        self.HEADSET_MODE = headset_mode

        self.__control_mode = 'full'
        self.__mode_state_machine_state = 0

        self.__scaling_motion = "regular"
        self.__scaling_state_machine_state = 0
        self.scaled_value = 1
        self.scale_flag = 0
        self.unscale_flag = 0

        self.gripper_val = 0
        self.vive_buttons = [0, 0, 0, 0]
        self.vive_axes = [0, 0, 0]
        self.trigger_press = False

        self.trigger_press = False

        # # Private variables:
        self.__input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__last_input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__scaled_motion_input = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        # # Public variables:
        self.is_initialized = True

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/vive_mapping/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__dependency_status = {}

        self.__dependency_status_topics = {}

        # # Service provider:

        # # Service subscriber:
        self.__teleoperation_enable_tracking = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/teleoperation/enable_tracking',
            SetBool,
        )

        # # Topic publisher:
        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/vive_mapping/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__teleoperation_pose = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/input_pose',
            Pose,
            queue_size=1,
        )
        self.__teleoperation_tracking_button = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/tracking_button',
            Bool,
            queue_size=1,
        )
        self.__teleoperation_gripper_button = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/gripper_button',
            Bool,
            queue_size=1,
        )
        self.__teleoperation_mode_button = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/mode_button',
            Bool,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            '/Right_Hand',
            TransformStamped,
            self.__input_pose_callback,
        )

        rospy.Subscriber(
            '/vive/controller_LHR_FF7FBBC0/joy', Joy, self.callback_vive_b
        )

    # # Dependency status callbacks:
    def __teleoperation_callback(self, message):
        """Monitors teleoperation is_initialized topic.
        
        """

        self.__dependency_status['teleoperation'] = message.data

    # # Service handlers:

    # # Topic callbacks:
    def __input_pose_callback(self, msg):
        """

        """

        self.__input_pose['position'][0] = -msg.transform.translation.x
        self.__input_pose['position'][1] = -msg.transform.translation.y
        self.__input_pose['position'][
            2] = msg.transform.translation.z - 0.96 + 0.25

        self.__input_pose['orientation'][0] = msg.transform.rotation.w
        self.__input_pose['orientation'][1] = msg.transform.rotation.x
        self.__input_pose['orientation'][2] = msg.transform.rotation.y
        self.__input_pose['orientation'][3] = msg.transform.rotation.z

    def callback_vive_b(self, msg):
        """

        """

        self.vive_buttons = msg.buttons
        self.vive_axes = msg.axes

        self.gripper_val = self.vive_axes[2]

        self.trigger_press = False

        self.__scaling_state_machine(self.vive_buttons[3])

        if self.gripper_val == 1:  # Trigger button to hold the gripper state
            self.trigger_press = True

    # # Private methods:

    def __scaling_state_machine(self, button):
        """
        Used to track if continous or discrete orientation control is desired
        """

        # State 0: Button was pressed.
        if (self.__scaling_state_machine_state == 0 and button):

            self.__scaling_state_machine_state = 1
            self.__scaling_motion = 'slow'
            print("______slow")
            self.scale_flag = 1
            self.__teleoperation_enable_tracking(False)

        # State 1: Button was released.
        elif (self.__scaling_state_machine_state == 1 and not button):

            self.__scaling_state_machine_state = 2

        # State 2: Button was released.
        elif (self.__scaling_state_machine_state == 2 and button):

            self.__scaling_state_machine_state = 3
            self.__scaling_motion = 'regular'
            print("___regular")
            self.unscale_flag = 1
            self.__teleoperation_enable_tracking(False)

        elif (self.__scaling_state_machine_state == 3 and not button):

            self.__scaling_state_machine_state = 0

    def __mode_state_machine(self, button):
        """
        Used to track if continous or discrete orientation control is desired
        """

        # State 0: Button was pressed.
        if (self.__mode_state_machine_state == 0 and button):

            self.__mode_state_machine_state = 1
            self.__control_mode = 'full'

        # State 1: Button was released.
        elif (self.__mode_state_machine_state == 1 and not button):

            self.__mode_state_machine_state = 2

        # State 2: Button was pressed.
        elif (self.__mode_state_machine_state == 2 and button):

            self.__mode_state_machine_state = 3
            self.__control_mode = 'position'

        # State 3: Button was released.
        elif (self.__mode_state_machine_state == 3 and not button):

            self.__mode_state_machine_state = 0

    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (
                            f'/{self.ROBOT_NAME}/vive_mapping: '
                            f'lost connection to {key}!'
                        )
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key}...'

            rospy.logwarn_throttle(
                15,
                (
                    f'/{self.ROBOT_NAME}/vive_mapping:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE: Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(
                    f'\033[92m/{self.ROBOT_NAME}/vive_mapping: ready.\033[0m',
                )

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.
                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __publish_teleoperation_pose_regular(self):
        """
        Control input for regular scaling
        """

        corrected_input_pose = copy.deepcopy(self.__input_pose)

        # # STEP 1: Table or Head mode correction.
        # If the headset is located on table invert position X and Y axis,
        # rotate orientation quaternion by 180 degrees around Z.
        if self.HEADSET_MODE == 'table':
            corrected_input_pose['position'][0] = (
                -1 * self.__input_pose['position'][0]
            )
            corrected_input_pose['position'][1] = (
                -1 * self.__input_pose['position'][1]
            )

            corrected_input_pose['orientation'] = (
                transformations.quaternion_multiply(
                    transformations.quaternion_about_axis(
                        np.deg2rad(180),
                        (0, 0, 1),
                    ),
                    corrected_input_pose['orientation'],
                )
            )

        if self.__control_mode == 'full':
            pose_message = Pose()
            pose_message.position.x = self.__input_pose['position'][0]
            pose_message.position.y = self.__input_pose['position'][1]
            pose_message.position.z = self.__input_pose['position'][2]

            pose_message.orientation.w = self.__input_pose['orientation'][0]
            pose_message.orientation.x = self.__input_pose['orientation'][1]
            pose_message.orientation.y = self.__input_pose['orientation'][2]
            pose_message.orientation.z = self.__input_pose['orientation'][3]

            self.__teleoperation_pose.publish(pose_message)

            # flag: activate tracking after triggering compensation for scaling
            if self.unscale_flag == 1:
                self.unscale_flag = 0
                self.__teleoperation_enable_tracking(True)

            self.__last_input_pose = copy.deepcopy(self.__input_pose)

        elif self.__control_mode == 'position':
            angle_z = -self.vive_axes[0] / 7500
            angle_y = self.vive_axes[1] / 7500

            euler_quaternion = quaternion_from_euler(angle_z, angle_y, 0)

            combined_quaternion = quaternion_multiply(
                self.__last_input_pose['orientation'],
                euler_quaternion,
            )

            pose_message = Pose()
            pose_message.position.x = self.__input_pose['position'][0]
            pose_message.position.y = self.__input_pose['position'][1]
            pose_message.position.z = self.__input_pose['position'][2]

            pose_message.orientation.w = combined_quaternion[0]
            pose_message.orientation.x = combined_quaternion[1]
            pose_message.orientation.y = combined_quaternion[2]
            pose_message.orientation.z = combined_quaternion[3]

            self.__teleoperation_pose.publish(pose_message)

            # flag: activate tracking after triggering compensation for scaling
            if self.unscale_flag == 1:
                self.unscale_flag = 0
                self.__teleoperation_enable_tracking(True)

            self.__last_input_pose = copy.deepcopy(self.__input_pose)

            self.__last_input_pose['orientation'] = copy.deepcopy(
                combined_quaternion
            )

    def __publish_teleoperation_pose_scaled(self):
        """
        Control input for scaled motion
        """

        corrected_input_pose = copy.deepcopy(self.__input_pose)

        # # STEP 1: Table or Head mode correction.
        # If the headset is located on table invert position X and Y axis,
        # rotate orientation quaternion by 180 degrees around Z.
        if self.HEADSET_MODE == 'table':
            corrected_input_pose['position'][0] = (
                -1 * self.__input_pose['position'][0]
            )
            corrected_input_pose['position'][1] = (
                -1 * self.__input_pose['position'][1]
            )

            corrected_input_pose['orientation'] = (
                transformations.quaternion_multiply(
                    transformations.quaternion_about_axis(
                        np.deg2rad(180),
                        (0, 0, 1),
                    ),
                    corrected_input_pose['orientation'],
                )
            )

        if self.__control_mode == 'full':
            pose_message = Pose()
            pose_message.position.x = self.__input_pose['position'][0] / 2
            pose_message.position.y = self.__input_pose['position'][1] / 2
            pose_message.position.z = self.__input_pose['position'][2] / 2

            pose_message.orientation.w = self.__input_pose['orientation'][0]
            pose_message.orientation.x = self.__input_pose['orientation'][1]
            pose_message.orientation.y = self.__input_pose['orientation'][2]
            pose_message.orientation.z = self.__input_pose['orientation'][3]

            self.__teleoperation_pose.publish(pose_message)

            self.__last_input_pose = copy.deepcopy(self.__input_pose)

            # flag: activate tracking after triggering compensation for scaling
            if self.scale_flag == 1:

                self.scale_flag = 0
                self.__teleoperation_enable_tracking(True)

        elif self.__control_mode == 'position':
            angle_z = -self.vive_axes[0] / 7500
            angle_y = self.vive_axes[1] / 7500

            euler_quaternion = quaternion_from_euler(angle_z, angle_y, 0)

            combined_quaternion = quaternion_multiply(
                self.__last_input_pose['orientation'],
                euler_quaternion,
            )

            pose_message = Pose()
            pose_message.position.x = self.__input_pose['position'][0] / 2
            pose_message.position.y = self.__input_pose['position'][1] / 2
            pose_message.position.z = self.__input_pose['position'][2] / 2

            pose_message.orientation.w = combined_quaternion[0]
            pose_message.orientation.x = combined_quaternion[1]
            pose_message.orientation.y = combined_quaternion[2]
            pose_message.orientation.z = combined_quaternion[3]

            self.__teleoperation_pose.publish(pose_message)

            # flag: activate tracking after triggering compensation for scaling
            if self.scale_flag == 1:

                self.scale_flag = 0
                self.__teleoperation_enable_tracking(True)

            self.__last_input_pose = copy.deepcopy(self.__input_pose)

            self.__last_input_pose['orientation'] = copy.deepcopy(
                combined_quaternion
            )

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        self.__mode_state_machine(self.vive_buttons[0])
        self.__scaling_state_machine(self.vive_buttons[3])

        if self.__scaling_motion == 'regular':
            self.__publish_teleoperation_pose_regular()
        elif self.__scaling_motion == 'slow':
            self.__publish_teleoperation_pose_scaled()

        self.__teleoperation_tracking_button.publish(self.vive_buttons[2])
        self.__teleoperation_gripper_button.publish(self.trigger_press)
        self.__teleoperation_mode_button.publish(self.vive_buttons[0])

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/vive_mapping: node is shutting down...',
        )

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/vive_mapping: node has shut down.',
        )


def main():
    """

    """

    rospy.init_node(
        'vive_mapping',
        log_level=rospy.INFO,  # TODO: Make this a launch file parameter.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS parameters:
    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=1000,
    )

    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    controller_side = rospy.get_param(
        param_name=f'{rospy.get_name()}/controller_side',
        default='right',
    )

    oculus_kinova_mapping = ViveMapping(
        robot_name=kinova_name,
        controller_side=controller_side,
        headset_mode='table',
    )

    rospy.on_shutdown(oculus_kinova_mapping.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        oculus_kinova_mapping.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
