import math

import numpy as np

from pycram.datastructures.enums import Grasp, Arms, ObjectType, AxisIdentifier
from pycram.datastructures.pose import Pose
from pycram.designator import ObjectDesignatorDescription
from pycram.local_transformer import LocalTransformer
from pycram.robot_description import RobotDescription, GraspDescription
from scipy.spatial.transform import Rotation as R
from typing_extensions import List, Union, Optional

from pycram.ros.viz_marker_publisher import AxisMarkerPublisher, ManualMarkerPublisher, ArrowArrayPublisher
from pycram.world_concepts.world_object import Object

"""Implementation of helper functions and classes for internal usage only.

Classes:
Singleton -- implementation of singleton metaclass
"""


class Singleton(type):
    """
    Metaclass for singletons
    """

    _instances = {}
    """
    Dictionary of singleton child classes inheriting from this metaclass, keyed by child class objects.
    """

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


INDEX_TO_AXIS = {0: 'x', 1: 'y', 2: 'z'}
AXIS_TO_INDEX = {'x': 0, 'y': 1, 'z': 2}
AXIS_INDEX_TO_FACE = {
    ('x', -1): Grasp.FRONT,
    ('x', 1): Grasp.BACK,
    ('y', 1): Grasp.LEFT,
    ('y', -1): Grasp.RIGHT,
    ('z', -1): Grasp.TOP,
    ('z', 1): Grasp.BOTTOM
}

FACE_TO_AXIS_INDEX = {
    Grasp.FRONT: ('x', -1),
    Grasp.BACK: ('x', 1),
    Grasp.LEFT: ('y', 1),
    Grasp.RIGHT: ('y', -1),
    Grasp.TOP: ('z', -1),
    Grasp.BOTTOM: ('z', 1)
}


def calculate_object_faces(object_to_robot_vector: List, specified_grasp_axis: Optional[AxisIdentifier] = None) -> List[
    Grasp]:
    """
    Determines the faces of the object based on the input vector.

    If `specified_grasp_axis` is None, it calculates the primary and secondary faces based on the vector's magnitude
    in the x and y directions, determining which sides of the object are most aligned with the robot.
    If `specified_grasp_axis` is provided, it only considers the specified axis and calculates the faces aligned
    with that axis.

    Args:
        object_to_robot_vector (List): A 3D vector representing one of the robot's axes in the object's frame, with
                       irrelevant components set to np.nan.
        specified_grasp_axis (Optional[AxisIdentifier]): Specifies a specific axis (e.g., X, Y, Z) to focus on.

    Returns:
        List[Grasp]: A list of two Grasp enums representing the primary and secondary faces.
    """
    epsilon = 1e-15

    object_to_robot_vector = np.where(np.isnan(object_to_robot_vector), np.nan, object_to_robot_vector + epsilon)

    if specified_grasp_axis is None:
        valid_indices = [AXIS_TO_INDEX['x'], AXIS_TO_INDEX['y'], AXIS_TO_INDEX['z']]
    else:
        valid_indices = [AXIS_TO_INDEX[specified_grasp_axis.name.lower()]]

    valid_indices = [i for i in valid_indices if not np.isnan(object_to_robot_vector[i])]

    abs_vector = np.abs(object_to_robot_vector)
    sorted_indices = sorted(valid_indices, key=lambda i: abs_vector[i], reverse=True)

    primary_index = sorted_indices[0]
    primary_sign = int(np.sign(object_to_robot_vector[primary_index]))
    primary_axis = INDEX_TO_AXIS[primary_index]
    primary_face = AXIS_INDEX_TO_FACE[(primary_axis, primary_sign)]

    if len(sorted_indices) > 1:
        secondary_index = sorted_indices[1]
        secondary_sign = int(np.sign(object_to_robot_vector[secondary_index]))
        secondary_axis = INDEX_TO_AXIS[secondary_index]
        secondary_face = AXIS_INDEX_TO_FACE[(secondary_axis, secondary_sign)]
    else:
        secondary_sign = -primary_sign
        secondary_axis = primary_axis
        secondary_face = AXIS_INDEX_TO_FACE[(secondary_axis, secondary_sign)]

    return [primary_face, secondary_face]


def calculate_grasp_descriptions(target_object: ObjectDesignatorDescription.Object, robot: Optional[Object] = None) -> \
List[GraspDescription]:
    """
    Calculates the grasp configurations of an object relative to the robot based on orientation and position.

    This method determines the possible grasp configurations (side and top/bottom faces) of the object,
    taking into account the object's orientation, position, and whether horizontal alignment is preferred.

    Args:
        target_object (ObjectDesignatorDescription.Object): The object whose grasp configurations are to be calculated.
        robot (Optional[Object]): The robot for which the grasp configurations are being calculated.

    Returns:
        List[GraspConfig]: A sorted list of GraspConfig instances representing all grasp permutations.
    """

    obj_desig = target_object if isinstance(target_object,
                                            (ObjectDesignatorDescription.Object, Pose)) else target_object.resolve()

    oTm = obj_desig.pose
    base_link = RobotDescription.current_robot_description.base_link
    if robot is None:
        base_link_pose = obj_desig.world_object.world.robot.get_link_pose(base_link)
    else:
        base_link_pose = robot.get_link_pose(base_link)

    side_axis, horizontal, top = get_preferred_grasp_alignment(target_object)

    object_position = [oTm.position.x, oTm.position.y, oTm.position.z]
    robot_position = base_link_pose.position_as_list()
    vector_to_robot_world = [robot_position[i] - object_position[i] for i in range(3)]

    orientation = [oTm.orientation.x, oTm.orientation.y, oTm.orientation.z, oTm.orientation.w]
    rotation_matrix = R.from_quat(orientation).as_matrix()
    o_R_w = rotation_matrix.T

    vector_to_robot_local = o_R_w.dot(vector_to_robot_world)

    vector_x, vector_y, vector_z = vector_to_robot_local

    vector_facing = np.array([vector_x, vector_y, np.nan], dtype=float)
    side_faces = calculate_object_faces(vector_facing, side_axis)

    vector_z = np.array([np.nan, np.nan, vector_z], dtype=float)
    top_faces = calculate_object_faces(vector_z) if top else [None]

    grasp_configs = [
        GraspDescription(side_face=side, top_face=top_face, horizontal=horizontal)
        for top_face in top_faces
        for side in side_faces
    ]

    if obj_desig.world_object.world.current_world.allow_publish_debug_poses:
        # Assign colors based on side grasp and publish poses
        marker = ArrowArrayPublisher()
        poses = []
        colors = []
        labels = []
        label_poses = []

        color_map = {
            Grasp.FRONT: [0.0, 1.0, 0.0, 1.0],  # Green
            Grasp.LEFT: [0.0, 0.0, 1.0, 1.0],  # Blue
            Grasp.RIGHT: [1.0, 1.0, 0.0, 1.0],  # Yellow
            Grasp.BACK: [1.0, 0.0, 0.0, 1.0]  # Red
        }

        for idx, grasp_config in enumerate(grasp_configs):
            oTm_copy = oTm.copy()
            side_grasp, top_grasp, horizontal = grasp_config.side_face, grasp_config.top_face, grasp_config.horizontal
            end_effector = RobotDescription.current_robot_description.get_arm_chain(Arms.LEFT).end_effector
            grasp_descriptions: dict = end_effector.calculate_grasp_descriptions([0, 0, 0, 1])
            grasp_orientation = grasp_descriptions.get((side_grasp, top_grasp, horizontal))
            palm_axis = [1, 0, 0]

            if obj_desig.obj_type == ObjectType.BOWL:
                rim_offset = calculate_rim_grasp(obj_desig.world_object.get_object_dimensions(), side_grasp)
                rim_direction = grasp_descriptions.get((side_grasp, None, False))
                rim_adjustment = adjust_grasp_for_object_rotation(oTm_copy, rim_direction)
                rim_pose = translate_relative_to_object(rim_adjustment, palm_axis, rim_offset)
                oTm_copy.position = rim_pose.position

            adjusted_oTm = adjust_grasp_for_object_rotation(oTm_copy, grasp_orientation)
            translation_value = max(obj_desig.world_object.get_object_dimensions()) * 0.75
            pose = translate_relative_to_object(adjusted_oTm, palm_axis, translation_value)
            color = color_map.get(side_grasp, None)
            colors.append(color)
            poses.append(pose)

            labels.append(f"Try {idx + 1}")
            label_pose = pose.copy()
            label_pose = translate_relative_to_object(label_pose, palm_axis, 0.05)
            label_poses.append(label_pose)

        marker.publish(poses, colors, labels, label_poses)

    return grasp_configs


def calculate_grasp_offset(object_dim: List, arm, grasp):
    """
    Calculates the grasp offset of an object based on its dimensions and the desired grasp type.

    This method adjusts the object's position along the specified axis to account for grasping
    constraints, based on the arm's tool frame offset and the object's half-dimensions.

    Args:
        object (Object): The object to be grasped, with a pose attribute that includes position.
        object_dim (List[float]): Dimensions of the object in each axis [x, y, z].
        arm (Enum): The arm used for grasping, needed to determine tool frame offset.
        grasp (Enum): The desired grasp type, which determines the grasp axis and direction.

    Returns:
        offset: Translation offset of the object for grasping.
    """
    axis, _ = FACE_TO_AXIS_INDEX[grasp]

    object_half_dimension = object_dim[AXIS_TO_INDEX[axis]] / 2

    tool_frame_offset = RobotDescription.current_robot_description.get_distance_palm_to_tool_frame(arm) / 2

    offset_value = max(0, object_half_dimension - tool_frame_offset)

    return offset_value


def calculate_rim_grasp(object_dim: List, grasp):
    """
    Calculates the grasp offset of an object based on its dimensions and the desired grasp type.

    This method adjusts the object's position along the specified axis to account for grasping
    constraints, based on the arm's tool frame offset and the object's half-dimensions.

    Args:
        object_dim (List[float]): Dimensions of the object in each axis [x, y, z].
        grasp (Enum): The desired grasp type, which determines the grasp axis and direction.

    Returns:
        offset: Translation offset of the object for grasping.
    """
    axis, _ = FACE_TO_AXIS_INDEX[grasp]

    object_half_dimension = object_dim[AXIS_TO_INDEX[axis]] / 2

    return object_half_dimension


def adjust_grasp_for_object_rotation(object_pose: Pose, grasp_quaternion: List[float]) -> Pose:
    """
    Adjusts the grasp orientation based on the object's rotation.

    Args:
        object_pose (Pose): The object's pose, including position and orientation.
        grasp_quaternion (List[float]): The grasp quaternion to be adjusted.
    Returns:
        Pose: The adjusted grasp orientation.
    """
    obj_pose = Pose(object_pose.position_as_list(), grasp_quaternion)
    obj_pose.multiply_quaternions(object_pose.orientation_as_list())
    return obj_pose


def get_preferred_grasp_alignment(object: Object) -> (bool, bool):
    """
    Determines the preferred grasp alignment for an object.
    Args:
        object (Object): The object to be grasped.
    Returns:
        tuple: A tuple of two booleans, indicating whether the object should be grasped horizontally and/or from the top
    """
    object_type = object.obj_type if isinstance(object,
                                                ObjectDesignatorDescription.Object) else ObjectType.GENERIC_OBJECT
    preferred_alignment_dict = {ObjectType.BOWL: (None, True, True),
                                ObjectType.SPOON: (AxisIdentifier.X, False, True),
                                ObjectType.BREAKFAST_CEREAL: (AxisIdentifier.X, False, False), }

    sidegrasp_axis, grasp_horizontal, grasp_top = preferred_alignment_dict.get(object_type, (None, False, False))

    return sidegrasp_axis, grasp_horizontal, grasp_top


def translate_relative_to_object(obj_pose, palm_axis, translation_value) -> Pose:
    """
    Applies the translation directly along the palm axis returned by get_palm_axis().

    Args:
        oTg: The current pose of the object relative to the gripper.
        palm_axis: A list [x, y, z] where one value is 1 or -1, and the others are 0.
        translation_value: The magnitude of the retreat in meters.
        gripper_pose: The current pose of the gripper.

    Returns:
        None: Modifies the oTg.pose in place.
    """
    object_pose = obj_pose.copy()
    local_retraction = np.array([palm_axis[0] * translation_value,
                                 palm_axis[1] * translation_value,
                                 palm_axis[2] * translation_value])

    quat = object_pose.orientation_as_list()

    rotation_matrix = R.from_quat(quat)
    retraction_world = rotation_matrix.apply(local_retraction)

    object_pose.pose.position.x -= retraction_world[0]
    object_pose.pose.position.y -= retraction_world[1]
    object_pose.pose.position.z -= retraction_world[2]

    return object_pose


def round_pose(pose: Pose, decimals: int = 8) -> Pose:
    """
    Rounds the position and orientation of a pose to the specified number of decimal places.

    Args:
        pose (Pose): The pose to be rounded.
        decimals (int): The number of decimal places to round to.

    Returns:
        Pose: The rounded pose.
    """
    rounded_pose = Pose(frame=pose.frame,
                        position=[round(pose.position.x, decimals),
                                  round(pose.position.y, decimals),
                                  round(pose.position.z, decimals)],
                        orientation=[round(pose.orientation.x, decimals),
                                     round(pose.orientation.y, decimals),
                                     round(pose.orientation.z, decimals),
                                     round(pose.orientation.w, decimals)])

    return rounded_pose
