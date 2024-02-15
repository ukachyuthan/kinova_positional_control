#!/usr/bin/env python
"""

"""

import rospy
import numpy as np
import transformations

from geometry_msgs.msg import (Pose)

from oculus_ros.msg import (ControllerButtons)
from kinova_positional_control.srv import (
    GripperForceGrasping,
    GripperPosition,
)
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped, Pose

from std_msgs.msg import Float64MultiArray


class scalingdeterminer:
    """
    
    """

    def __init__(self,):
        """
        
        """

        self.scaling_arm_left_prox = 1
        self.scaling_arm_right_prox = 1

        self.scaling_arm_left_table = 1
        self.scaling_arm_right_table = 1

        self.left_arm_moved = 1
        self.right_arm_moved = 1

        self.left_arm_physical = 0
        self.right_arm_physical = 0

        self.left_arm_motion_physical = 0
        self.right_arm_motion_physical = 0

        self.left_disengage = 0
        self.right_disengage = 0

        # self.old_left_arm_position = np.asarray([0,0,0])
        # self.old_right_arm_position = np.asarray([0,0,0])

        self.left_dwell = 0
        self.right_dwell = 0

        self.last_left_arm_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.last_right_arm_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.last_movement_time_left = rospy.Time.now()
        self.last_movement_time_right = rospy.Time.now()

        self.left_arm_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.right_arm_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        rospy.Subscriber(
            'right/relaxed_ik/commanded_pose_gcs',
            Pose,
            self.__commanded_pose_callback_right,
        )

        rospy.Subscriber(
            'left/relaxed_ik/commanded_pose_gcs',
            Pose,
            self.__commanded_pose_callback_left,
        )

        rospy.Subscriber(
            '/comparison_workload_vive',
            Float64MultiArray,
            self.physical_workload_callback,
        )

        self.scaling_pub = rospy.Publisher(
            '/scaling_values',
            Float64MultiArray,
            queue_size=10,
        )

    def __commanded_pose_callback_left(self, msg):
        """
        
        """

        self.last_left_arm_pose = self.left_arm_pose

        self.left_arm_pose['position'][0] = msg.position.x
        self.left_arm_pose['position'][1] = msg.position.y
        self.left_arm_pose['position'][2] = msg.position.z

        self.left_arm_pose['orientation'][0] = msg.orientation.w
        self.left_arm_pose['orientation'][1] = msg.orientation.x
        self.left_arm_pose['orientation'][2] = msg.orientation.y
        self.left_arm_pose['orientation'][3] = msg.orientation.z

        distance_change = np.linalg.norm(
            np.array([msg.position.x, msg.position.y, msg.position.z]) - [
                self.last_left_arm_pose['position'][0],
                self.last_left_arm_pose['position'][1],
                self.last_left_arm_pose['position'][2]
            ]
        )
        orientation_change = np.arccos(
            np.dot(
                np.array(
                    [
                        msg.orientation.w, msg.orientation.x, msg.orientation.y,
                        msg.orientation.z
                    ]
                ), [
                    self.last_left_arm_pose['orientation'][0],
                    self.last_left_arm_pose['orientation'][1],
                    self.last_left_arm_pose['orientation'][2],
                    self.last_left_arm_pose['orientation'][3]
                ]
            )
        ) * 2 * 180 / np.pi

        if distance_change > 0.05 or orientation_change > 10:

            self.last_movement_time_left = rospy.Time.now()
            self.arm_hasnt_moved()

        else:
            self.left_dwell = 0

        self.left_arm_motion_physical = self.left_arm_physical

    def __commanded_pose_callback_right(self, msg):
        """
        
        """

        self.last_right_arm_pose = self.right_arm_pose

        self.right_arm_pose['position'][0] = msg.position.x
        self.right_arm_pose['position'][1] = msg.position.y
        self.right_arm_pose['position'][2] = msg.position.z

        self.right_arm_pose['orientation'][0] = msg.orientation.w
        self.right_arm_pose['orientation'][1] = msg.orientation.x
        self.right_arm_pose['orientation'][2] = msg.orientation.y
        self.right_arm_pose['orientation'][3] = msg.orientation.z

        distance_change = np.linalg.norm(
            np.array([msg.position.x, msg.position.y, msg.position.z]) - [
                self.last_right_arm_pose['position'][0],
                self.last_right_arm_pose['position'][1],
                self.last_right_arm_pose['position'][2]
            ]
        )
        orientation_change = np.arccos(
            np.dot(
                np.array(
                    [
                        msg.orientation.w, msg.orientation.x, msg.orientation.y,
                        msg.orientation.z
                    ]
                ), [
                    self.last_right_arm_pose['orientation'][0],
                    self.last_right_arm_pose['orientation'][1],
                    self.last_right_arm_pose['orientation'][2],
                    self.last_right_arm_pose['orientation'][3]
                ]
            )
        ) * 2 * 180 / np.pi

        if distance_change > 0.05 or orientation_change > 10:

            self.last_movement_time_right = rospy.Time.now()
            self.arm_hasnt_moved()

        else:
            self.right_dwell = 0

        self.right_arm_motion_physical = self.right_arm_physical * self.right_dwell

    def arm_hasnt_moved(self):

        self.left_dwell = rospy.Time.now() - self.last_movement_time_right
        self.right_dwell = rospy.Time.now() - self.last_movement_time_left

    def proximity_scaling(self):

        array_1 = np.array(
            [
                self.left_arm_pose['position'][0],
                self.left_arm_pose['position'][1] + 0.8,
                self.left_arm_pose['position'][2]
            ]
        )
        array_2 = np.array(
            [
                self.right_arm_pose['position'][0],
                self.right_arm_pose['position'][1],
                self.right_arm_pose['position'][2]
            ]
        )

        distance = np.linalg.norm(array_1 - array_2)

        if distance <= 0.3:

            self.scaling_arm_left_prox = 0.5
            self.scaling_arm_right_prox = 0.5

        elif distance <= 0.45:

            self.scaling_arm_left_prox = 0.75
            self.scaling_arm_right_prox = 0.75

        else:

            self.scaling_arm_left_prox = 1
            self.scaling_arm_right_prox = 1

    def table_scaling(self):

        if self.left_arm_pose['position'][2] <= -0.3:

            self.scaling_arm_left_table = 0.5

        elif self.left_arm_pose['position'][2] <= -0.15:

            self.scaling_arm_left_table = 0.75

        else:

            self.scaling_arm_left_table = 1

        if self.right_arm_pose['position'][2] <= -0.3:

            self.scaling_arm_right_table = 0.5

        elif self.right_arm_pose['position'][2] <= -0.15:

            self.scaling_arm_right_table = 0.75

        else:

            self.scaling_arm_right_table = 1

    def physical_workload_callback(self, message):

        self.left_arm_physical = message.data[0]
        self.right_arm_physical = message.data[1]

    def main_loop(self):
        """
        
        """

        self.proximity_scaling()
        self.table_scaling()

        scaling_value_left = min(
            self.scaling_arm_left_prox, self.scaling_arm_left_table
        )

        scaling_value_right = min(
            self.scaling_arm_right_prox, self.scaling_arm_right_table
        )

        array_data = Float64MultiArray()

        if self.left_arm_motion_physical >= 80:

            self.left_disengage = 1

        if self.right_arm_motion_physical >= 80:

            self.right_disengage = 1

        array_data.data = [
            scaling_value_left, scaling_value_right, self.left_disengage,
            self.right_disengage
        ]

        print(scaling_value_left, scaling_value_right)

        self.scaling_pub.publish(array_data)


def main():
    """

    """

    # # ROS node:
    rospy.init_node('scaling_tracker')

    print('\nScaling tracker up.\n')

    class_obj = scalingdeterminer()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        class_obj.main_loop()
        rate.sleep()


if __name__ == '__main__':
    main()
