from typing import Any

import rospy

from ..external_interfaces.robokudo import query
from .default_process_modules import *
from ..external_interfaces.giskard import init_giskard_interface
from ..designators.motion_designator import *
from ..enums import JointType, ObjectType
from ..external_interfaces import giskard
from ..external_interfaces.ik import request_ik
from ..helper import _apply_ik
from ..local_transformer import LocalTransformer
from ..process_module import ProcessModule


class StretchNavigate(ProcessModule):
    """
    Process module for the simulated Stretch that sends a cartesian goal to the robot to move the robot base
    """
    def _execute(self, desig: MoveMotion.Motion):
        robot = BulletWorld.robot
        robot.set_pose(desig.target)



class StretchMoveHead(ProcessModule):
    """
    Process module for the simulated Stretch that moves the head such that it looks at the given position
    """

    def _execute(self, desig: LookingMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_pan"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("link_head_tilt"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)

        # For some reason the values for position.y and position.z are swapped, so for now the formula is adjusted accordingly.
        # Not guaranteed to work in all cases, depending on the reason why the values are swapped for this robot (maybe wrong urdf or something?)
        new_tilt = - np.arctan2(pose_in_tilt.position.y,
                                np.sqrt(pose_in_tilt.position.x ** 2 + pose_in_tilt.position.z ** 2))

        current_pan = robot.get_joint_state("joint_head_pan")
        current_tilt = robot.get_joint_state("joint_head_tilt")

        robot.set_joint_state("joint_head_pan", new_pan + current_pan)
        robot.set_joint_state("joint_head_tilt", new_tilt + current_tilt)


class StretchMoveGripper(ProcessModule):
    """
    Process module for the simulated Stretch that opens or closes the gripper
    """
    def _execute(self, desig: MoveGripperMotion.Motion):
        robot = BulletWorld.robot
        gripper = desig.gripper
        motion = desig.motion
        for joint, state in robot_description.get_static_gripper_chain(gripper, motion).items():
            robot.set_joint_state(joint, state)


class StretchDetecting(ProcessModule):
    """
    Process Module for the simulated Stretch that tries to detect an object fitting the given object description
    """

    def _execute(self, desig: DetectingMotion.Motion):
        robot = BulletWorld.robot
        object_type = desig.object_type
        # Should be "wide_stereo_optical_frame"
        cam_frame_name = robot_description.get_camera_frame()
        # should be [0, 0, 1]
        front_facing_axis = robot_description.front_facing_axis
        if desig.technique == 'all':
            # rospy.loginfo("Fake detecting all generic objects")
            objects = BulletWorld.current_bullet_world.get_all_objets_not_robot()
        elif desig.technique == 'human':
            # rospy.loginfo("Fake detecting human -> spawn 0,0,0")
            human = []
            human.append(Object("human", ObjectType.HUMAN, "human_male.stl", pose=Pose([0, 0, 0])))
            object_dict = {}

            # Iterate over the list of objects and store each one in the dictionary
            for i, obj in enumerate(human):
                object_dict[obj.name] = obj
            return object_dict

        else:
            # rospy.loginfo("Fake -> Detecting specific object type")
            objects = BulletWorld.current_bullet_world.get_objects_by_type(object_type)

        object_dict = {}

        perceived_objects = []
        for obj in objects:
            # if btr.visible(obj, robot.get_link_pose(cam_frame_name), front_facing_axis):
            perceived_objects.append(ObjectDesignatorDescription.Object(obj.name, obj.type, obj))

        # Iterate over the list of objects and store each one in the dictionary
        for i, obj in enumerate(perceived_objects):
            object_dict[obj.name] = obj

        # rospy.loginfo("returning dict objects")
        return object_dict


class StretchMoveTCP(ProcessModule):
    """
    Process module for the simulated Stretch that moves the tool center point of the robot
    """

    def _execute(self, desig: MoveTCPMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot

        _move_arm_tcp(target, robot, desig.arm)


class StretchMoveArmJoints(ProcessModule):
    """
    Process module for the simulated Stretch that moves the arm joints of the robot
    """
    def _execute(self, desig: MoveArmJointsMotion.Motion):

        robot = BulletWorld.robot
        if desig.right_arm_poses:
            robot.set_joint_states(desig.right_arm_poses)
        if desig.left_arm_poses:
            robot.set_joint_states(desig.left_arm_poses)


class StretchMoveJoints(ProcessModule):
    """
    Process module for the simulated Stretch that moves any joint of the robot
    """

    def _execute(self, desig: MoveJointsMotion.Motion):
        robot = BulletWorld.robot
        robot.set_joint_states(dict(zip(desig.names, desig.positions)))


class StretchWorldStateDetecting(ProcessModule):
    """
    Process Module for the simulated Stretch that tries to detect an object using the world state
    """

    def _execute(self, desig: WorldStateDetectingMotion.Motion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.type == obj_type, BulletWorld.current_bullet_world.objects))[0]


class StretchOpen(ProcessModule):
    """
    Process module for the simulated Stretch that opens an already grasped container
    """

    def _execute(self, desig: OpeningMotion.Motion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(container_joint)[1])


class StretchClose(ProcessModule):
    """
    Process module for the simulated Stretch that closes an already grasped container
    """

    def _execute(self, desig: ClosingMotion.Motion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(container_joint)[0])


def _move_arm_tcp(target: Pose, robot: Object, arm: str) -> None:
    gripper = robot_description.get_tool_frame(arm)

    joints = robot_description.chains[arm].joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv, joints)



###########################################################
########## Process Modules for the Real Stretch ###########
###########################################################

# @TODO Update the following process modules to work with stretch in this pycram version

class StretchNavigationReal(ProcessModule):
    """
    Process module for the real Stretch that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        giskard.achieve_cartesian_goal(designator.target, robot_description.base_link, "map")


class StretchMoveHeadReal(ProcessModule):
    """
    Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
    as the simulated one
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target

        local_transformer = LocalTransformer()
        goal_point = local_transformer.transform_pose(target, 'map')


        giskard.giskard_wrapper.allow_all_collisions()
        from geometry_msgs.msg import Vector3Stamped, PointStamped
        pointing_axis = Vector3Stamped()
        pointing_axis.header.frame_id = 'link_head_tilt'
        pointing_axis.vector.x = 1
        goal = PointStamped()
        goal.header.frame_id = 'map'
        goal.point = goal_point.pose.position
        giskard.giskard_wrapper.set_pointing_goal(goal_point=goal,
                                                  tip_link='link_head_tilt',
                                                  pointing_axis=pointing_axis,
                                                  root_link='link_head')
        giskard.giskard_wrapper.execute()


class StretchDetectingReal(ProcessModule):
    """
    Process Module for the real Stretch that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, designator: DetectingMotion) -> Any:
        query_result = query(ObjectDesignatorDescription(types=[designator.object_type]))
        # print(query_result)
        obj_pose = query_result["ClusterPoseBBAnnotator"]

        lt = LocalTransformer()
        obj_pose = lt.transform_pose(obj_pose, BulletWorld.robot.get_link_tf_frame("torso_lift_link"))
        obj_pose.orientation = [0, 0, 0, 1]
        obj_pose.position.x += 0.05

        bullet_obj = BulletWorld.current_world.get_objects_by_type(designator.object_type)
        if bullet_obj:
            bullet_obj[0].set_pose(obj_pose)
            return bullet_obj[0]
        elif designator.object_type == ObjectType.JEROEN_CUP:
            cup = Object("cup", ObjectType.JEROEN_CUP, "jeroen_cup.stl", pose=obj_pose)
            return cup
        elif designator.object_type == ObjectType.BOWL:
            bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=obj_pose)
            return bowl

        return bullet_obj[0]


class StretchMoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real Stretch while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPMotion) -> Any:
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")

        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)
        giskard.achieve_cartesian_goal(pose_in_map, robot_description.get_tool_frame(designator.arm),
                                       robot_description.base_link)


class StretchMoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real Stretch to the given configuration while avoiding all collisions
    """

    def _execute(self, designator: MoveArmJointsMotion) -> Any:
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
        if designator.right_arm_poses:
            joint_goals.update(designator.right_arm_poses)
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(joint_goals)


class StretchMoveJointsReal(ProcessModule):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """

    def _execute(self, designator: MoveJointsMotion) -> Any:
        name_to_position = dict(zip(designator.names, designator.positions))
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(name_to_position)


class StretchMoveGripperReal(ProcessModule):
    """
    Opens or closes the gripper of the real Stretch, gripper uses an action server for this instead of giskard
    """

    def _execute(self, designator: MoveGripperMotion) -> Any:
        chain = robot_description.chains[designator.gripper].gripper.get_static_joint_chain(designator.motion)
        giskard.achieve_joint_goal(chain)


class StretchOpenReal(ProcessModule):
    """
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion) -> Any:
        giskard.achieve_open_container_goal(robot_description.get_tool_frame(designator.arm),
                                            designator.object_part.name)


class StretchCloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion) -> Any:
        giskard.achieve_close_container_goal(robot_description.get_tool_frame(designator.arm),
                                             designator.object_part.name)


class StretchManager(ProcessModuleManager):
    def __init__(self):
        super().__init__("stretch_description")
        self._navigate_lock = Lock()
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
            return StretchNavigate(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated" or ProcessModuleManager.execution_type == "real":
            return StretchWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return StretchClose(self._close_lock)
        elif ProcessModuleManager.execution_type == "real":
            return StretchCloseReal(self._close_lock)