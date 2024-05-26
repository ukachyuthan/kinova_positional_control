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
import math
from std_msgs.msg import (Bool)
from std_srvs.srv import (
    SetBool,
)
from geometry_msgs.msg import (Pose)

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped, Pose
from tf.transformations import quaternion_from_euler, quaternion_multiply, \
euler_from_quaternion

from std_msgs.msg import Float64MultiArray

from kinova_positional_control.srv import (CalculateCompensation)
import time
from std_msgs.msg import String, Float64, Float64MultiArray


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
        self.__mode_state_machine_state = 2
        self.__tracking_state_machine_state = 0

        self.__scaling_motion = "regular"
        self.__scaling_state_machine_state = 0
        self.scaled_value = 1
        self.last_scaled_value = 1
        self.scale_flag = 0
        self.unscale_flag = 0
        self.scale_change = 0
        self.left_scaling_active = 0
        self.left_orientation_active = 0
        self.left_disengage = 0

        self.last_left_emergency = 0

        self.gripper_val = 0
        self.vive_buttons = [0, 0, 0, 0]
        self.vive_axes = [0, 0, 0]

        self.trigger_press = False

        # Variables for speed check:

        self.threshold_position = 0.05
        self.x_positions = [0] * 10
        self.y_positions = [0] * 10
        self.z_positions = [0] * 10
        self.thresholds_position = [0.006, 0.006, 0.006]

        self.threshold_orientation = 15
        self.roll_positions = [0] * 10
        self.pitch_positions = [0] * 10
        self.yaw_positions = [0] * 10
        self.thresholds_orientation = [0.35, 0.35, 0.35]

        self.constraint_activate = 0
        self.constraint_deactivate = 0

        self.last_time = time.time()

        self.poisition_fixture_condn = None
        self.orientation_fixture_condn = None

        self.left_emergency = 3

        self.__mode_switch = 0  # 0 = full, 1 = intent inference mode

        # # Private variables:
        self.__input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__last_input_pose = {
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
        self.__teleoperation_calculate_compesation = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/teleoperation/calculate_compesation',
            CalculateCompensation,
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

        self.__left_parameter_publisher = rospy.Publisher(
            '/left/ui_parameters',
            Float64MultiArray,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            '/Left_Hand',
            TransformStamped,
            self.__input_pose_callback,
        )

        rospy.Subscriber(
            '/vive/controller_LHR_FFFB7FC3/joy', Joy, self.callback_vive_b
        )

        rospy.Subscriber(
            '/scaling_values', Float64MultiArray,
            self.callback_scaling_parameters
        )

        rospy.Subscriber(
            '/left/emergency_topic',
            Float64,
            self.callback_emergency_right,
        )

    # # Dependency status callbacks:
    def callback_emergency_right(self, message):

        self.last_left_emergency = self.left_emergency

        self.left_emergency = message.data

        if self.left_emergency == 1 and (
            self.left_emergency != self.last_left_emergency
        ):
            # print("_____________________________________________________")
            self.__tracking_state_machine_state = 0
            self.left_disengage = 0

    def __teleoperation_callback(self, message):
        """Monitors teleoperation is_initialized topic.
        
        """

        self.__dependency_status['teleoperation'] = message.data

    # # Service handlers:

    # # Topic callbacks:
    def __input_pose_callback(self, msg):
        """

        """

        if (
            sum(self.x_positions) == 0 and sum(self.y_positions) == 0
            and sum(self.z_positions) == 0
        ):

            self.x_positions = [-msg.transform.translation.x] * 10
            self.y_positions = [-msg.transform.translation.y] * 10
            self.z_positions = [msg.transform.translation.z - 0.96 + 0.25] * 10

        self.__input_pose['position'][0] = -msg.transform.translation.x
        self.__input_pose['position'][1] = -msg.transform.translation.y
        self.__input_pose['position'][
            2] = msg.transform.translation.z - 0.96 + 0.25

        self.x_positions.append(-msg.transform.translation.x)
        self.x_positions.pop(0)

        self.y_positions.append(-msg.transform.translation.y)
        self.y_positions.pop(0)

        self.z_positions.append(msg.transform.translation.z - 0.96 + 0.25)
        self.z_positions.pop(0)

        r, p, y = euler_from_quaternion(
            [
                msg.transform.rotation.x,
                msg.transform.rotation.y,
                msg.transform.rotation.z,
                msg.transform.rotation.w,
            ]
        )

        if (
            sum(self.roll_positions) == 0 and sum(self.pitch_positions) == 0
            and sum(self.yaw_positions) == 0
        ):

            self.roll_positions = [math.degrees(r)] * 10
            self.pitch_positions = [math.degrees(p)] * 10
            self.yaw_positions = [math.degrees(y)] * 10

        self.roll_positions.append(math.degrees(r))
        self.roll_positions.pop(0)

        self.pitch_positions.append(math.degrees(p))
        self.pitch_positions.pop(0)

        self.yaw_positions.append(math.degrees(y))
        self.yaw_positions.pop(0)

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

    def callback_scaling_parameters(self, scaling_array):

        self.last_scaled_value = self.scaled_value
        self.scaled_value = scaling_array.data[0]

    # # Private methods:
    def __speedchecker_position(self, position_list, axis, threshold):

        distances = [
            position_list[i + 1] - position_list[i]
            for i in range(len(position_list) - 1)
        ]

        avg_position = np.mean(distances)

        # if axis == 'x':

        #     print(avg_position)

        if np.abs(avg_position) >= threshold:

            # print(avg_position, "position", axis)

            return 1

        return 0

    def __speedchecker_orientation(self, position_list, axis, threshold):

        distances = [
            position_list[i + 1] - position_list[i]
            for i in range(len(position_list) - 1)
        ]

        avg_position = np.mean(distances)

        # print(avg_position)

        # if axis == 'y':

        #     print('====================================')

        if np.abs(avg_position) >= threshold:

            # print(avg_position, "orientation", axis)

            return 1, avg_position

        return 0, avg_position

    def __check_axis(self, a, b, c, condn):

        count = 0
        axis = []  # 0 = all, 1 = x/roll, 2 = y/pitch, 3 = z/yaw

        if a == 1:

            count += 1
            axis.append(1)

        if b == 1:

            count += 1
            axis.append(2)

        if c == 1:

            count += 1
            axis.append(3)

        # if condn in "p":
        #     print(axis)

        if count == 3:
            return None
        elif count < 3 and count > 0:
            return axis
        else:
            return [4]

    def __scaling_value_state_machine(self):
        """
        
        """
        if self.last_scaled_value != self.scaled_value and \
            self.scale_change == 0:

            self.scale_change = 1

    def __tracking_state_machine(self, button):
        """
        
        """

        # State 0: Grip button was pressed.
        if (self.__tracking_state_machine_state == 0 and button):

            self.left_disengage = 1
            self.__tracking_state_machine_state = 1

        # State 1: Grip button was released. Tracking is activated.
        elif (self.__tracking_state_machine_state == 1 and not button):

            self.__tracking_state_machine_state = 2

        # State 2: Grip button was pressed. Tracking is deactivated.
        elif (self.__tracking_state_machine_state == 2 and button):

            self.__tracking_state_machine_state = 3
            self.left_disengage = 0

        # State 3: Grip button was released.
        elif (self.__tracking_state_machine_state == 3 and not button):

            self.__tracking_state_machine_state = 0

    def __scaling_state_machine(self, button):
        """
        Used to track if continous or discrete orientation control is desired
        """

        # State 0: Button was pressed.
        if (self.__scaling_state_machine_state == 0 and button):

            self.__scaling_state_machine_state = 1
            self.__scaling_motion = 'slow'
            self.scale_flag = 1
            self.left_scaling_active = 1

        # State 1: Button was released.
        elif (self.__scaling_state_machine_state == 1 and not button):

            self.__scaling_state_machine_state = 2

        # State 2: Button was released.
        elif (self.__scaling_state_machine_state == 2 and button):

            self.__scaling_state_machine_state = 3
            self.__scaling_motion = 'regular'
            self.unscale_flag = 1
            self.left_scaling_active = 0

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
            self.left_orientation_active = 0

        # State 1: Button was released.
        elif (self.__mode_state_machine_state == 1 and not button):

            self.__mode_state_machine_state = 2

        # State 2: Button was pressed.
        elif (self.__mode_state_machine_state == 2 and button):

            self.__mode_state_machine_state = 3
            self.__control_mode = 'position'
            self.left_orientation_active = 1

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

        current_time = time.time()

        if current_time - self.last_time >= 0.1:
            x_fast = self.__speedchecker_position(
                self.x_positions,
                'x',
                self.thresholds_position[0],
            )
            y_fast = self.__speedchecker_position(
                self.y_positions,
                'y',
                self.thresholds_position[1],
            )
            z_fast = self.__speedchecker_position(
                self.z_positions,
                'z',
                self.thresholds_position[2],
            )

            self.position_fixture_condn = self.__check_axis(
                x_fast,
                y_fast,
                z_fast,
                'p',
            )

            roll_fast, rr = self.__speedchecker_orientation(
                self.roll_positions,
                'r',
                self.thresholds_orientation[0],
            )
            pitch_fast, rp = self.__speedchecker_orientation(
                self.pitch_positions,
                'p',
                self.thresholds_orientation[1],
            )
            yaw_fast, ry = self.__speedchecker_orientation(
                self.yaw_positions,
                'y',
                self.thresholds_orientation[2],
            )

            self.orientation_fixture_condn = self.__check_axis(
                roll_fast,
                pitch_fast,
                yaw_fast,
                'o',
            )

            # current_time = time.time()
            # if current_time - self.last_time > 0.1:

            #     print(self.orientation_fixture_condn)
            #     self.last_time = current_time

            if self.orientation_fixture_condn or self.position_fixture_condn and self.__mode_switch == 0:
                self.__mode_switch = 1
                self.constraint_activate = 1
                # self.__teleoperation_calculate_compesation(compensation_pose)
                # print(self.__mode_switch)

            elif self.__mode_switch == 1:
                self.__mode_switch = 0
                self.constraint_deactivate = 1
                # self.__teleoperation_calculate_compesation(compensation_pose)
                # print(self.__mode_switch)

            self.last_time = current_time

            print(self.__mode_switch, self.position_fixture_condn)

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

        if self.__mode_switch == 0:

            pose_message = Pose()
            pose_message.position.x = self.__input_pose['position'][0]
            pose_message.position.y = self.__input_pose['position'][1]
            pose_message.position.z = self.__input_pose['position'][2]

            pose_message.orientation.w = self.__input_pose['orientation'][0]
            pose_message.orientation.x = self.__input_pose['orientation'][1]
            pose_message.orientation.y = self.__input_pose['orientation'][2]
            pose_message.orientation.z = self.__input_pose['orientation'][3]

            # flag: activate tracking after triggering compensation for scaling
            if self.unscale_flag == 1:
                self.unscale_flag = 0
                # self.__teleoperation_enable_tracking(True)
                self.__teleoperation_calculate_compesation(pose_message)

            if self.constraint_deactivate == 1:

                self.constraint_deactivate = 0
                self.__teleoperation_calculate_compesation(pose_message)

            self.__teleoperation_pose.publish(pose_message)

            self.__last_input_pose = copy.deepcopy(self.__input_pose)

        elif self.__mode_switch == 1:

            send_x = None
            send_y = None
            send_z = None

            if self.position_fixture_condn:

                for axis in self.position_fixture_condn:
                    if axis == 1:
                        send_x = 1
                    if axis == 2:
                        send_y = 1
                    if axis == 3:
                        send_z = 1
                    if axis == 4:
                        send_x = 1
                        send_y = 1
                        send_z = 1

                if send_x is None:
                    send_x = 0.01
                if send_y is None:
                    send_y = 0.01
                if send_z is None:
                    send_z = 0.01

            else:
                send_x = self.__input_pose['position'][0]
                send_y = self.__input_pose['position'][1]
                send_z = self.__input_pose['position'][2]

            if self.orientation_fixture_condn:

                r, p, y = euler_from_quaternion(
                    [
                        self.__input_pose['orientation'][1],
                        self.__input_pose['orientation'][2],
                        self.__input_pose['orientation'][3],
                        self.__input_pose['orientation'][0]
                    ]
                )

                last_r, last_p, last_y = euler_from_quaternion(
                    [
                        self.__last_input_pose['orientation'][1],
                        self.__last_input_pose['orientation'][2],
                        self.__last_input_pose['orientation'][3],
                        self.__last_input_pose['orientation'][0]
                    ]
                )

                send_angle_roll = None
                send_angle_pitch = None
                send_angle_yaw = None

                for axis in self.orientation_fixture_condn:
                    if axis == 1:
                        send_angle_roll = 1
                    if axis == 2:
                        send_angle_pitch = 1
                    if axis == 3:
                        send_angle_yaw = 1
                    if axis == 4:
                        send_angle_roll = 1
                        send_angle_pitch = 1
                        send_angle_yaw = 1

                if send_angle_roll is None:
                    send_angle_roll = 0.01
                if send_angle_pitch is None:
                    send_angle_pitch = 0.01
                if send_angle_yaw is None:
                    send_angle_yaw = 0.01

                quaternion_scaled = quaternion_from_euler(
                    r * send_angle_roll,
                    p * send_angle_pitch,
                    y * send_angle_yaw,
                )

                quaternion_added = quaternion_from_euler(
                    math.radians(send_angle_roll),
                    math.radians(send_angle_pitch),
                    math.radians(send_angle_yaw),
                )

            else:

                quaternion_scaled = [
                    self.__input_pose['orientation'][1],
                    self.__input_pose['orientation'][2],
                    self.__input_pose['orientation'][3],
                    self.__input_pose['orientation'][0],
                ]

            pose_message = Pose()
            # pose_message.position.x = send_x
            # pose_message.position.y = send_y
            # pose_message.position.z = send_z

            pose_message.position.x = self.__input_pose['position'][0] * send_x
            pose_message.position.y = self.__input_pose['position'][1] * send_y
            pose_message.position.z = self.__input_pose['position'][2] * send_z

            pose_message.orientation.w = quaternion_scaled[3]
            pose_message.orientation.x = quaternion_scaled[0]
            pose_message.orientation.y = quaternion_scaled[1]
            pose_message.orientation.z = quaternion_scaled[2]

            # flag: activate tracking after triggering compensation for scaling
            if self.unscale_flag == 1:
                self.unscale_flag = 0
                self.__teleoperation_calculate_compesation(pose_message)

            if self.constraint_activate == 1:

                self.constraint_activate = 0
                self.__teleoperation_calculate_compesation(pose_message)

            self.__teleoperation_pose.publish(pose_message)

            self.__last_input_pose = copy.deepcopy(self.__input_pose)
            # self.__last_input_pose['position'] = copy.deepcopy(
            #     np.asarray([
            #         send_x,
            #         send_y,
            #         send_z,
            #     ])
            # )
            self.__last_input_pose['orientation'] = copy.deepcopy(
                np.asarray(
                    [
                        quaternion_scaled[3],
                        quaternion_scaled[0],
                        quaternion_scaled[1],
                        quaternion_scaled[2],
                    ]
                )
            )

    def __publish_teleoperation_pose_scaled(self):
        """
        Control input for scaled motion
        """
        # print(self.scaled_value)
        current_time = time.time()

        if current_time - self.last_time >= 0.1:
            x_fast = self.__speedchecker_position(
                self.x_positions,
                'x',
                self.thresholds_position[0],
            )
            y_fast = self.__speedchecker_position(
                self.y_positions,
                'y',
                self.thresholds_position[1],
            )
            z_fast = self.__speedchecker_position(
                self.z_positions,
                'z',
                self.thresholds_position[2],
            )

            self.position_fixture_condn = self.__check_axis(
                x_fast,
                y_fast,
                z_fast,
                'p',
            )

            roll_fast, rr = self.__speedchecker_orientation(
                self.roll_positions,
                'r',
                self.thresholds_orientation[0],
            )
            pitch_fast, rp = self.__speedchecker_orientation(
                self.pitch_positions,
                'p',
                self.thresholds_orientation[1],
            )
            yaw_fast, ry = self.__speedchecker_orientation(
                self.yaw_positions,
                'y',
                self.thresholds_orientation[2],
            )

            self.orientation_fixture_condn = self.__check_axis(
                roll_fast,
                pitch_fast,
                yaw_fast,
                'o',
            )

            # current_time = time.time()
            # if current_time - self.last_time > 0.1:

            #     print(self.orientation_fixture_condn)
            #     self.last_time = current_time

            if self.orientation_fixture_condn or self.position_fixture_condn and self.__mode_switch == 0:
                self.__mode_switch = 1
                self.constraint_activate = 1
                # self.__teleoperation_calculate_compesation(compensation_pose)
                # print(self.__mode_switch)

            elif self.__mode_switch == 1:
                self.__mode_switch = 0
                self.constraint_deactivate = 1
                # self.__teleoperation_calculate_compesation(compensation_pose)
                # print(self.__mode_switch)

            self.last_time = current_time

            # print(
            #     "Scaled", self.scaled_value, self.__mode_switch,
            #     self.orientation_fixture_condn
            # )

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

        # if self.__mode_switch == 0:

        #     pose_message = Pose()
        #     pose_message.position.x = self.__input_pose['position'][
        #         0] * self.scaled_value
        #     pose_message.position.y = self.__input_pose['position'][
        #         1] * self.scaled_value
        #     pose_message.position.z = self.__input_pose['position'][
        #         2] * self.scaled_value

        #     eulers = euler_from_quaternion(
        #         [
        #             self.__input_pose['orientation'][1],
        #             self.__input_pose['orientation'][2],
        #             self.__input_pose['orientation'][3],
        #             self.__input_pose['orientation'][0]
        #         ]
        #     )

        #     euler_half = [angle * self.scaled_value for angle in eulers]

        #     quaternion_scaled = quaternion_from_euler(
        #         euler_half[0], euler_half[1], euler_half[2]
        #     )

        #     pose_message.orientation.w = quaternion_scaled[3]
        #     pose_message.orientation.x = quaternion_scaled[0]
        #     pose_message.orientation.y = quaternion_scaled[1]
        #     pose_message.orientation.z = quaternion_scaled[2]

        #     # flag: activate tracking after triggering compensation for scaling
        #     if self.scale_flag == 1:

        #         self.scale_flag = 0
        #         self.__teleoperation_calculate_compesation(pose_message)

        #     elif self.scale_change == 1:

        #         self.scale_change = 0
        #         self.__teleoperation_calculate_compesation(pose_message)

        #     if self.constraint_deactivate == 1:

        #         self.constraint_deactivate = 0
        #         self.__teleoperation_calculate_compesation(pose_message)

        #     self.__teleoperation_pose.publish(pose_message)

        #     self.__last_input_pose = copy.deepcopy(self.__input_pose)
        #     self.__last_input_pose['orientation'] = copy.deepcopy(
        #         np.asarray(
        #             [
        #                 quaternion_scaled[3],
        #                 quaternion_scaled[0],
        #                 quaternion_scaled[1],
        #                 quaternion_scaled[2],
        #             ]
        #         )
        #     )

        # elif self.__mode_switch == 1:

        pose_message = Pose()
        pose_message.position.x = self.__input_pose['position'][
            0] * self.scaled_value
        pose_message.position.y = self.__input_pose['position'][
            1] * self.scaled_value
        pose_message.position.z = self.__input_pose['position'][
            2] * self.scaled_value

        if self.orientation_fixture_condn:

            send_angle_roll = None
            send_angle_pitch = None
            send_angle_yaw = None

            for axis in self.orientation_fixture_condn:
                if axis == 1:
                    send_angle_roll = 1
                if axis == 2:
                    send_angle_pitch = 1
                if axis == 3:
                    send_angle_yaw = 1
                if axis == 4:
                    send_angle_roll = 1
                    send_angle_pitch = 1
                    send_angle_yaw = 1

            if send_angle_roll is None:
                send_angle_roll = 0.0000001
            if send_angle_pitch is None:
                send_angle_pitch = 0.0000001
            if send_angle_yaw is None:
                send_angle_yaw = 0.0000001

        else:

            send_angle_roll = 1
            send_angle_pitch = 1
            send_angle_yaw = 1

        eulers = euler_from_quaternion(
            [
                self.__input_pose['orientation'][1],
                self.__input_pose['orientation'][2],
                self.__input_pose['orientation'][3],
                self.__input_pose['orientation'][0]
            ]
        )

        scaled_array = [
            self.scaled_value * send_angle_roll,
            self.scaled_value * send_angle_pitch,
            self.scaled_value * send_angle_yaw,
        ]

        euler_half = [x * y for x, y in zip(eulers, scaled_array)]

        quaternion_scaled = quaternion_from_euler(
            euler_half[0], euler_half[1], euler_half[2]
        )

        send_x = None
        send_y = None
        send_z = None

        if self.position_fixture_condn:

            for axis in self.position_fixture_condn:
                if axis == 1:
                    send_x = 1
                if axis == 2:
                    send_y = 1
                if axis == 3:
                    send_z = 1
                if axis == 4:
                    send_x = 1
                    send_y = 1
                    send_z = 1

            if send_x is None:
                send_x = 0.01
            if send_y is None:
                send_y = 0.01
            if send_z is None:
                send_z = 0.01

            pose_message.position.x *= send_x
            pose_message.position.y *= send_y
            pose_message.position.z *= send_z

        else:
            pose_message.position.x = self.__input_pose['position'][0]
            pose_message.position.y = self.__input_pose['position'][1]
            pose_message.position.z = self.__input_pose['position'][2]

        # if self.orientation_fixture_condn:

        #     r, p, y = euler_from_quaternion(
        #         [
        #             quaternion_scaled[1],
        #             quaternion_scaled[2],
        #             quaternion_scaled[3],
        #             quaternion_scaled[0],
        #         ]
        #     )

        #     send_angle_roll = None
        #     send_angle_pitch = None
        #     send_angle_yaw = None

        #     for axis in self.orientation_fixture_condn:
        #         if axis == 1:
        #             send_angle_roll = 1
        #         if axis == 2:
        #             send_angle_pitch = 1
        #         if axis == 3:
        #             send_angle_yaw = 1
        #         if axis == 4:
        #             send_angle_roll = 1
        #             send_angle_pitch = 1
        #             send_angle_yaw = 1

        #     if send_angle_roll is None:
        #         send_angle_roll = 0.0000001
        #     if send_angle_pitch is None:
        #         send_angle_pitch = 0.0000001
        #     if send_angle_yaw is None:
        #         send_angle_yaw = 0.0000001

        #     quaternion_constraint = quaternion_from_euler(
        #         r * send_angle_roll,
        #         p * send_angle_pitch,
        #         y * send_angle_yaw,
        #     )

        #     # print(
        #     #     math.degrees(r * send_angle_roll),
        #     #     math.degrees(p * send_angle_pitch),
        #     #     math.degrees(y * send_angle_yaw),
        #     # )

        # else:

        #     quaternion_constraint = copy.deepcopy(quaternion_scaled)

        pose_message.orientation.w = quaternion_scaled[3]
        pose_message.orientation.x = quaternion_scaled[0]
        pose_message.orientation.y = quaternion_scaled[1]
        pose_message.orientation.z = quaternion_scaled[2]

        # flag: activate tracking after triggering compensation for scaling
        if self.scale_flag == 1:

            self.scale_flag = 0
            self.__teleoperation_calculate_compesation(pose_message)

        elif self.scale_change == 1:

            self.scale_change = 0
            self.__teleoperation_calculate_compesation(pose_message)

        if self.constraint_activate == 1:

            self.constraint_activate = 0
            self.__teleoperation_calculate_compesation(pose_message)

        self.__teleoperation_pose.publish(pose_message)

        self.__last_input_pose = copy.deepcopy(self.__input_pose)
        self.__last_input_pose['orientation'] = copy.deepcopy(
            np.asarray(
                [
                    quaternion_scaled[3],
                    quaternion_scaled[0],
                    quaternion_scaled[1],
                    quaternion_scaled[2],
                ]
            )
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
        self.__scaling_value_state_machine()
        self.__tracking_state_machine(self.vive_buttons[2])

        if self.__scaling_motion == 'regular':
            self.__publish_teleoperation_pose_regular()
        elif self.__scaling_motion == 'slow':
            self.__publish_teleoperation_pose_scaled()

        self.__teleoperation_tracking_button.publish(self.vive_buttons[2])
        self.__teleoperation_gripper_button.publish(self.trigger_press)
        self.__teleoperation_mode_button.publish(self.vive_buttons[0])
        left_data = Float64MultiArray()
        left_data.data = np.asarray(
            [
                self.left_scaling_active,
                self.left_disengage,
                self.left_orientation_active,
            ]
        )
        self.__left_parameter_publisher.publish(left_data)

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
        'vive_mapping_left',
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
