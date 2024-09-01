from threading import Lock

import pycram.bullet_world_reasoning as btr
import numpy as np

from ..robot_descriptions import robot_description
from ..process_module import ProcessModule, ProcessModuleManager
from ..bullet_world import BulletWorld
from ..external_interfaces.ik import request_ik, IKError
from ..helper import _apply_ik
from ..local_transformer import LocalTransformer
from ..designators.motion_designator import *
from ..enums import JointType, ObjectType
from ..ros.viz_marker_publisher import AxisMarkerPublisher


class JustinNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion.Motion):
        robot = BulletWorld.robot
        robot.set_pose(desig.target)


class JustinPickUp(ProcessModule):
    """
    This process module is for picking up a given object.
    The object has to be reachable for this process module to succeed.
    """

    def _execute(self, desig: PickUpMotion.Motion):
        object = desig.object_desig.bullet_world_object
        robot = BulletWorld.robot
        grasp = robot_description.grasps.get_orientation_for_grasp(desig.grasp)
        target = object.get_pose()
        target.orientation.x = grasp[0]
        target.orientation.y = grasp[1]
        target.orientation.z = grasp[2]
        target.orientation.w = grasp[3]

        arm = desig.arm

        _move_arm_tcp(target, robot, arm)
        tool_frame = robot_description.get_tool_frame(arm)
        robot.attach(object, tool_frame)


class JustinPlace(ProcessModule):
    """
    This process module places an object at the given position in world coordinate frame.
    """

    def _execute(self, desig: PlaceMotion.Motion):
        """

        :param desig: A PlaceMotion
        :return:
        """
        object = desig.object.bullet_world_object
        robot = BulletWorld.robot
        arm = desig.arm

        _move_arm_tcp(desig.target, robot, arm)
        robot.detach(object)


class JustinMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig: LookingMotion.Motion):
        from scipy.spatial.transform import Rotation as R
        target = desig.target
        robot = BulletWorld.robot

        local_transformer = LocalTransformer()

        pan_link = robot_description.chains["neck"].links[0]
        tilt_link = robot_description.chains["neck"].links[1]

        pan_joint = robot_description.chains["neck"].joints[0]
        tilt_joint = robot_description.chains["neck"].joints[1]
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame(pan_link)).position_as_list()
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame(tilt_link)).position_as_list()

        new_pan = np.arctan2(pose_in_pan[1], pose_in_pan[0])
        tilt_offset = robot_description.get_offset(tilt_joint)

        rotation_tilt_offset = R.from_euler('xyz', tilt_offset.rotation).apply(pose_in_tilt)

        new_tilt = -np.arctan2(rotation_tilt_offset[2],
                               np.sqrt(rotation_tilt_offset[0] ** 2 + rotation_tilt_offset[1] ** 2))

        current_pan = robot.get_joint_state(pan_joint)
        current_tilt = robot.get_joint_state(tilt_joint)

        robot.set_joint_state(pan_joint, new_pan + current_pan)
        robot.set_joint_state(tilt_joint, new_tilt + current_tilt)
        marker = AxisMarkerPublisher()
        marker.publish(robot.get_link_pose("head2"), name="r_gripper_tool_frame", length=1)


class JustinMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """

    def _execute(self, desig: MoveGripperMotion.Motion):
        robot = BulletWorld.robot
        gripper = desig.gripper
        motion = desig.motion
        for joint, state in robot_description.get_static_gripper_chain(gripper, motion).items():
            robot.set_joint_state(joint, state)


# class JustinMoveHead(ProcessModule):
#     """
#     This process module moves the head to look at a specific point in the world coordinate frame.
#     This point can either be a position or an object.
#     """
#
#     def _execute(self, desig: LookingMotion.Motion):
#         from scipy.spatial.transform import Rotation as R
#
#         # Helper function to convert Pose to transformation matrix
#         def pose_to_transform(pose):
#             rotation = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
#             transform_matrix = np.eye(4)
#             transform_matrix[:3, :3] = rotation.as_matrix()
#             transform_matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
#             return transform_matrix
#
#         # Helper function to apply an rpy rotation
#         def rpy_to_transform(rpy, xyz):
#             rotation = R.from_euler('xyz', rpy)
#             transform_matrix = np.eye(4)
#             transform_matrix[:3, :3] = rotation.as_matrix()
#             transform_matrix[:3, 3] = xyz
#             return transform_matrix
#
#         # Get the target pose and robot instance
#         target_pose = desig.target
#         robot = BulletWorld.robot
#         marker = AxisMarkerPublisher()
#
#         pan_link = robot_description.chains["neck"].links[0]
#         tilt_link = robot_description.chains["neck"].links[1]
#         pan_joint = robot_description.chains["neck"].joints[0]
#         tilt_joint = robot_description.chains["neck"].joints[1]
#
#         # Get the poses in the map frame
#         mapTpan_pose = robot.get_link_pose(pan_link)
#         mapTtilt_pose = robot.get_link_pose(tilt_link)
#
#         # Convert poses to transformation matrices
#         T_map_to_pan = pose_to_transform(mapTpan_pose)
#         T_map_to_tilt = pose_to_transform(mapTtilt_pose)
#         T_map_to_target = pose_to_transform(target_pose)
#
#         # Normal pan calculation without offsets
#         T_pan_to_map = np.linalg.inv(T_map_to_pan)
#         T_pan_to_target = T_pan_to_map @ T_map_to_target
#         pan_position = T_pan_to_target[:3, 3]
#         new_pan = np.arctan2(pan_position[1], pan_position[0])
#         current_pan = robot.get_joint_state(pan_joint)
#         robot.set_joint_state(pan_joint, new_pan + current_pan)
#
#         # Apply the tilt offset specified in the URDF
#         T_tilt_offset = rpy_to_transform([-1.57079, 0, 0], [0, 0, 0])
#         T_map_to_tilt = T_map_to_tilt @ T_tilt_offset
#
#         # Invert the tilt transformation and calculate the target's position relative to tilt link
#         T_tilt_to_map = np.linalg.inv(T_map_to_tilt)
#         T_tilt_to_target = T_tilt_to_map @ T_map_to_target
#         tilt_position = T_tilt_to_target[:3, 3]
#
#         # Adjust the tilt angle to correct the sign if the robot is looking up instead of down
#         new_tilt = -np.arctan2(tilt_position[2], np.sqrt(tilt_position[0] ** 2 + tilt_position[1] ** 2))
#         new_tilt *= -1
#         current_tilt = robot.get_joint_state(tilt_joint)
#         robot.set_joint_state(tilt_joint, new_tilt + current_tilt)
#
#         # Publish the marker for visualization
#         marker.publish(robot.get_link_pose(tilt_link), name="r_gripper_tool_frame", length=1)
#         print()

class JustinDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig: DetectingMotion.Motion):
        robot = BulletWorld.robot
        object_type = desig.object_type
        # Should be "wide_stereo_optical_frame"
        cam_frame_name = robot_description.get_camera_frame()
        # should be [0, 0, 1]
        front_facing_axis = robot_description.front_facing_axis

        if desig.technique == 'all':
            objects = BulletWorld.current_bullet_world.get_all_objets_not_robot()
        elif desig.technique == 'human':
            human = []
            human.append(Object("human", ObjectType.HUMAN, "human_male.stl", pose=Pose([0, 0, 0])))
            object_dict = {}

            # Iterate over the list of objects and store each one in the dictionary
            for i, obj in enumerate(human):
                object_dict[obj.name] = obj
            return object_dict

        else:
            objects = BulletWorld.current_bullet_world.get_objects_by_type(object_type)

        object_dict = {}

        perceived_objects = []
        for obj in objects:
            perceived_objects.append(ObjectDesignatorDescription.Object(obj.name, obj.type, obj))

            # todo: commented out since the visualisation is not working good bc of rendering one object
            # if btr.visible(obj, robot.get_link_pose(cam_frame_name), front_facing_axis):

        # Iterate over the list of objects and store each one in the dictionary
        for i, obj in enumerate(perceived_objects):
            object_dict[obj.name] = obj

        return object_dict


class JustinMoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig: MoveTCPMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot

        _move_arm_tcp(target, robot, desig.arm)


class JustinMoveArmJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig: MoveArmJointsMotion.Motion):

        robot = BulletWorld.robot
        if desig.right_arm_poses:
            for joint, pose in desig.right_arm_poses.items():
                robot.set_joint_state(joint, pose)
        if desig.left_arm_poses:
            for joint, pose in desig.left_arm_poses.items():
                robot.set_joint_state(joint, pose)


class JustinMoveJoints(ProcessModule):
    def _execute(self, desig: MoveJointsMotion.Motion):
        robot = BulletWorld.robot
        for joint, pose in zip(desig.names, desig.positions):
            robot.set_joint_state(joint, pose)


class JustinWorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion.Motion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.type == obj_type, BulletWorld.current_bullet_world.objects))[0]


class JustinOpen(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion.Motion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(
                                                                  container_joint)[1])


class JustinClose(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """

    def _execute(self, desig: ClosingMotion.Motion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(
                                                                  container_joint)[0])


def _move_arm_tcp(target: Pose, robot: Object, arm: str) -> None:
    gripper = robot_description.get_tool_frame(arm)

    joints = robot_description.chains[arm].joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv, joints)


class JustinManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("rollin-justin")
        self._navigate_lock = Lock()
        self._pick_up_lock = Lock()
        self._place_lock = Lock()
        self._looking_lock = Lock()
        self._detecting_lock = Lock()
        self._move_tcp_lock = Lock()
        self._move_arm_joints_lock = Lock()
        self._world_state_detecting_lock = Lock()
        self._move_joints_lock = Lock()
        self._move_gripper_lock = Lock()
        self._open_lock = Lock()
        self._close_lock = Lock()

    def navigate(self):
        if ProcessModuleManager.execution_type == "simulated":
            return JustinNavigation(self._navigate_lock)

    def pick_up(self):
        if ProcessModuleManager.execution_type == "simulated":
            return JustinPickUp(self._pick_up_lock)

    def place(self):
        if ProcessModuleManager.execution_type == "simulated":
            return JustinPlace(self._place_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return JustinMoveHead(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return JustinDetecting(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return JustinMoveTCP(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return JustinMoveArmJoints(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return JustinWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return JustinMoveJoints(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return JustinMoveGripper(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return JustinOpen(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return JustinClose(self._close_lock)
