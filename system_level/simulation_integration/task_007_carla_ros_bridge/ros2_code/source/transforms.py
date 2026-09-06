#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Tool functions to convert transforms from carla to ROS coordinate system
"""

import math
import numpy

try:
    import carla
except ImportError:
    pass

from geometry_msgs.msg import Vector3, Quaternion, Transform, Pose, Point, Twist, Accel  # pylint: disable=import-error

# Use a local pure-Python fallback so we don't need the transforms3d package at
# runtime (it is not available as a ros-humble apt package).

def _euler2mat(ai, aj, ak):
    """Rotation matrix from Euler angles (sxyz convention)."""
    si, ci = math.sin(ai), math.cos(ai)
    sj, cj = math.sin(aj), math.cos(aj)
    sk, ck = math.sin(ak), math.cos(ak)
    return numpy.array([
        [cj*ck,  si*sj*ck - ci*sk,  ci*sj*ck + si*sk],
        [cj*sk,  si*sj*sk + ci*ck,  ci*sj*sk - si*ck],
        [-sj,    si*cj,             ci*cj            ]
    ])


def _euler2quat(ai, aj, ak):
    """Quaternion (w, x, y, z) from Euler angles (sxyz convention)."""
    ai2 = ai / 2.0
    aj2 = aj / 2.0
    ak2 = ak / 2.0
    ci, si = math.cos(ai2), math.sin(ai2)
    cj, sj = math.cos(aj2), math.sin(aj2)
    ck, sk = math.cos(ak2), math.sin(ak2)
    w = ci*cj*ck + si*sj*sk
    x = si*cj*ck - ci*sj*sk
    y = ci*sj*ck + si*cj*sk
    z = ci*cj*sk - si*sj*ck
    return numpy.array([w, x, y, z])


def _quat2euler(q):
    """Euler angles (sxyz) from quaternion (w, x, y, z)."""
    w, x, y, z = q
    # Rotation matrix elements needed
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def _quat2mat(q):
    """Rotation matrix from quaternion (w, x, y, z)."""
    w, x, y, z = q
    n = w*w + x*x + y*y + z*z
    s = 0.0 if n == 0.0 else 2.0 / n
    wx = s * w * x; wy = s * w * y; wz = s * w * z
    xx = s * x * x; xy = s * x * y; xz = s * x * z
    yy = s * y * y; yz = s * y * z; zz = s * z * z
    return numpy.array([
        [1.0 - (yy + zz), xy - wz,          xz + wy],
        [xy + wz,          1.0 - (xx + zz),  yz - wx],
        [xz - wy,          yz + wx,          1.0 - (xx + yy)]
    ])


def _mat2quat(M):
    """Quaternion (w, x, y, z) from 3x3 rotation matrix."""
    t = numpy.trace(M)
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        w = 0.25 / s
        x = (M[2, 1] - M[1, 2]) * s
        y = (M[0, 2] - M[2, 0]) * s
        z = (M[1, 0] - M[0, 1]) * s
    elif M[0, 0] > M[1, 1] and M[0, 0] > M[2, 2]:
        s = 2.0 * math.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2])
        w = (M[2, 1] - M[1, 2]) / s
        x = 0.25 * s
        y = (M[0, 1] + M[1, 0]) / s
        z = (M[0, 2] + M[2, 0]) / s
    elif M[1, 1] > M[2, 2]:
        s = 2.0 * math.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2])
        w = (M[0, 2] - M[2, 0]) / s
        x = (M[0, 1] + M[1, 0]) / s
        y = 0.25 * s
        z = (M[1, 2] + M[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1])
        w = (M[1, 0] - M[0, 1]) / s
        x = (M[0, 2] + M[2, 0]) / s
        y = (M[1, 2] + M[2, 1]) / s
        z = 0.25 * s
    return numpy.array([w, x, y, z])


# Provide the names that the rest of the file (and tests) expect
euler2mat = _euler2mat
euler2quat = _euler2quat
quat2euler = _quat2euler
quat2mat = _quat2mat
mat2quat = _mat2quat


def carla_location_to_numpy_vector(carla_location):
    """
    Convert a carla location to a ROS vector3

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a numpy.array with 3 elements
    :rtype: numpy.array
    """
    return numpy.array([
        carla_location.x,
        -carla_location.y,
        carla_location.z
    ])


def carla_location_to_ros_vector3(carla_location):
    """
    Convert a carla location to a ROS vector3

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a ROS vector3
    :rtype: geometry_msgs.msg.Vector3
    """
    ros_translation = Vector3()
    ros_translation.x = carla_location.x
    ros_translation.y = -carla_location.y
    ros_translation.z = carla_location.z

    return ros_translation


def carla_location_to_ros_point(carla_location):
    """
    Convert a carla location to a ROS point

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a ROS point
    :rtype: geometry_msgs.msg.Point
    """
    ros_point = Point()
    ros_point.x = carla_location.x
    ros_point.y = -carla_location.y
    ros_point.z = carla_location.z

    return ros_point


def carla_rotation_to_RPY(carla_rotation):
    """
    Convert a carla rotation to a roll, pitch, yaw tuple

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a tuple with 3 elements (roll, pitch, yaw)
    :rtype: tuple
    """
    roll = math.radians(carla_rotation.roll)
    pitch = -math.radians(carla_rotation.pitch)
    yaw = -math.radians(carla_rotation.yaw)

    return (roll, pitch, yaw)


def carla_rotation_to_ros_quaternion(carla_rotation):
    """
    Convert a carla rotation to a ROS quaternion

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a ROS quaternion
    :rtype: geometry_msgs.msg.Quaternion
    """
    roll, pitch, yaw = carla_rotation_to_RPY(carla_rotation)
    quat = euler2quat(roll, pitch, yaw)
    ros_quaternion = Quaternion(w=float(quat[0]), x=float(quat[1]), y=float(quat[2]), z=float(quat[3]))
    return ros_quaternion


def carla_rotation_to_numpy_rotation_matrix(carla_rotation):
    """
    Convert a carla rotation to a ROS quaternion

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a numpy.array with 3x3 elements
    :rtype: numpy.array
    """
    roll, pitch, yaw = carla_rotation_to_RPY(carla_rotation)
    numpy_array = euler2mat(roll, pitch, yaw)
    rotation_matrix = numpy_array[:3, :3]
    return rotation_matrix


def carla_rotation_to_directional_numpy_vector(carla_rotation):
    """
    Convert a carla rotation (as orientation) into a numpy directional vector

    ros_quaternion = np_quaternion_to_ros_quaternion(quat)
    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a numpy.array with 3 elements as directional vector
        representation of the orientation
    :rtype: numpy.array
    """
    rotation_matrix = carla_rotation_to_numpy_rotation_matrix(carla_rotation)
    directional_vector = numpy.array([1, 0, 0])
    rotated_directional_vector = rotation_matrix.dot(directional_vector)
    return rotated_directional_vector


def carla_vector_to_ros_vector_rotated(carla_vector, carla_rotation):
    """
    Rotate carla vector, return it as ros vector

    :param carla_vector: the carla vector
    :type carla_vector: carla.Vector3D
    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: rotated ros vector
    :rtype: Vector3
    """
    rotation_matrix = carla_rotation_to_numpy_rotation_matrix(carla_rotation)
    tmp_array = rotation_matrix.dot(numpy.array([carla_vector.x, carla_vector.y, carla_vector.z]))
    ros_vector = Vector3()
    ros_vector.x = float(tmp_array[0])
    ros_vector.y = float(-tmp_array[1])
    ros_vector.z = float(tmp_array[2])
    return ros_vector


def carla_velocity_to_ros_twist(carla_linear_velocity, carla_angular_velocity, carla_rotation=None):
    """
    Convert a carla velocity to a ROS twist

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS).
    Considers the conversion from degrees (carla) to radians (ROS) for angular velocity.

    :param carla_linear_velocity: the carla linear velocity
    :type carla_linear_velocity: carla.Vector3D
    :param carla_angular_velocity: the carla angular velocity
    :type carla_angular_velocity: carla.Vector3D
    :param carla_rotation: the carla rotation (optional, used to rotate linear velocity)
    :type carla_rotation: carla.Rotation
    :return: a ROS twist
    :rtype: geometry_msgs.msg.Twist
    """
    ros_twist = Twist()

    if carla_rotation:
        ros_twist.linear = carla_vector_to_ros_vector_rotated(carla_linear_velocity, carla_rotation)
    else:
        ros_twist.linear.x = carla_linear_velocity.x
        ros_twist.linear.y = -carla_linear_velocity.y
        ros_twist.linear.z = carla_linear_velocity.z

    ros_twist.angular.x = math.radians(carla_angular_velocity.x)
    ros_twist.angular.y = -math.radians(carla_angular_velocity.y)
    ros_twist.angular.z = -math.radians(carla_angular_velocity.z)

    return ros_twist


def carla_velocity_to_numpy_vector(carla_velocity):
    """
    Convert a carla velocity to a numpy array

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)

    :param carla_velocity: the carla velocity
    :type carla_velocity: carla.Vector3D
    :return: a numpy.array with 3 elements
    :rtype: numpy.array
    """
    return numpy.array([
        carla_velocity.x,
        -carla_velocity.y,
        carla_velocity.z
    ])


def carla_acceleration_to_ros_accel(carla_acceleration):
    """
    Convert a carla acceleration to a ROS accel

    Considers the conversion from left-handed system (unreal) to right-handed
    system (ROS)
    The angular accelerations remain zero.

    :param carla_acceleration: the carla acceleration
    :type carla_acceleration: carla.Vector3D
    :return: a ROS accel
    :rtype: geometry_msgs.msg.Accel
    """
    ros_accel = Accel()
    ros_accel.linear.x = carla_acceleration.x
    ros_accel.linear.y = -carla_acceleration.y
    ros_accel.linear.z = carla_acceleration.z

    return ros_accel


def carla_transform_to_ros_transform(carla_transform):
    """
    Convert a carla transform to a ROS transform

    See carla_location_to_ros_vector3() and carla_rotation_to_ros_quaternion() for details

    :param carla_transform: the carla transform
    :type carla_transform: carla.Transform
    :return: a ROS transform
    :rtype: geometry_msgs.msg.Transform
    """
    ros_transform = Transform()

    ros_transform.translation = carla_location_to_ros_vector3(
        carla_transform.location)
    ros_transform.rotation = carla_rotation_to_ros_quaternion(
        carla_transform.rotation)

    return ros_transform


def carla_transform_to_ros_pose(carla_transform):
    """
    Convert a carla transform to a ROS pose

    See carla_location_to_ros_point() and carla_rotation_to_ros_quaternion() for details

    :param carla_transform: the carla transform
    :type carla_transform: carla.Transform
    :return: a ROS pose
    :rtype: geometry_msgs.msg.Pose
    """
    ros_pose = Pose()

    ros_pose.position = carla_location_to_ros_point(
        carla_transform.location)
    ros_pose.orientation = carla_rotation_to_ros_quaternion(
        carla_transform.rotation)

    return ros_pose


def carla_location_to_pose(carla_location):
    """
    Convert a carla location to a ROS pose

    See carla_location_to_ros_point() for details.
    pose quaternion remains zero.

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a ROS pose
    :rtype: geometry_msgs.msg.Pose
    """
    ros_pose = Pose()
    ros_pose.position = carla_location_to_ros_point(carla_location)
    ros_pose.orientation.w = 1.0
    return ros_pose


def ros_point_to_carla_location(ros_point):
    try:
        return carla.Location(ros_point.x, -ros_point.y, ros_point.z)
    except NameError:
        pass


def RPY_to_carla_rotation(roll, pitch, yaw):
    try:
        return carla.Rotation(roll=math.degrees(roll),
                              pitch=-math.degrees(pitch),
                              yaw=-math.degrees(yaw))
    except NameError:
        pass


def ros_quaternion_to_carla_rotation(ros_quaternion):
    roll, pitch, yaw = quat2euler([ros_quaternion.w,
                                   ros_quaternion.x,
                                   ros_quaternion.y,
                                   ros_quaternion.z])
    return RPY_to_carla_rotation(roll, pitch, yaw)


def ros_pose_to_carla_transform(ros_pose):
    """
    Convert a ROS pose a carla transform.
    """
    try:
        return carla.Transform(
            ros_point_to_carla_location(ros_pose.position),
            ros_quaternion_to_carla_rotation(ros_pose.orientation))
    except NameError:
        pass


def transform_matrix_to_ros_pose(mat):
    """
    Convert a transform matrix to a ROS pose.
    """
    quat = mat2quat(mat[:3, :3])
    msg = Pose()
    msg.position = Point(x=float(mat[0, 3]), y=float(mat[1, 3]), z=float(mat[2, 3]))
    msg.orientation = Quaternion(w=float(quat[0]), x=float(quat[1]), y=float(quat[2]), z=float(quat[3]))
    return msg


def ros_pose_to_transform_matrix(msg):
    """
    Convert a ROS pose to a transform matrix
    """
    mat44 = numpy.eye(4)
    mat44[:3, :3] = quat2mat([msg.orientation.w, msg.orientation.x,
                              msg.orientation.y, msg.orientation.z])
    mat44[0:3, -1] = [msg.position.x, msg.position.y, msg.position.z]
    return mat44