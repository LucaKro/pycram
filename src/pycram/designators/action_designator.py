import itertools
import math

import numpy as np
import roslibpy.tf
import sqlalchemy.orm
from typing import Any, Union
import rospy
from roslibpy import tf

import pycram.plan_failures
from .. import helper
from ..context_knowledge import ContextConfig
from .location_designator import CostmapLocation, find_reachable_location_and_nav_pose, AccessingLocation
from .motion_designator import *
from .object_designator import ObjectDesignatorDescription, BelieveObject, ObjectPart
from ..bullet_world import BulletWorld, Use_shadow_world
from ..designator import ActionDesignatorDescription
from ..enums import Arms
from ..helper import multiply_quaternions, axis_angle_to_quaternion
from ..local_transformer import LocalTransformer
from ..orm.action_designator import (ParkArmsAction as ORMParkArmsAction, NavigateAction as ORMNavigateAction,
                                     PickUpAction as ORMPickUpAction, PlaceAction as ORMPlaceAction,
                                     MoveTorsoAction as ORMMoveTorsoAction, SetGripperAction as ORMSetGripperAction,
                                     Action as ORMAction, LookAtAction as ORMLookAtAction,
                                     DetectAction as ORMDetectAction, TransportAction as ORMTransportAction,
                                     OpenAction as ORMOpenAction, CloseAction as ORMCloseAction,
                                     GraspingAction as ORMGraspingAction)

from ..orm.base import Quaternion, Position, Base
from ..pose import Pose
from ..robot_descriptions import robot_description
from ..ros.viz_marker_publisher import ManualMarkerPublisher
from ..task import with_tree
from pycram.enums import ObjectType


class MoveTorsoAction(ActionDesignatorDescription):
    """
    Action Designator for Moving the torso of the robot up and down
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        """
        Performable Move Torso Action designator.
        """

        position: float
        """
        Target position of the torso joint
        """

        @with_tree
        def perform(self) -> None:
            if robot_description.name == "rollin-justin":
                MoveJointsMotion(robot_description.chains["torso"].joints,
                                 robot_description.chains["torso"].static_joint_states["down"]).resolve().perform()
            else:
                MoveJointsMotion([robot_description.torso_joint], [self.position]).resolve().perform()

        def to_sql(self) -> ORMMoveTorsoAction:
            return ORMMoveTorsoAction(self.position)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs) -> ORMMoveTorsoAction:
            action = super().insert(session)
            session.add(action)
            session.commit()
            return action

    def __init__(self, positions: List[float], resolver=None):
        """
        Create a designator description to move the torso of the robot up and down.

        :param positions: List of possible positions of the robots torso, possible position is a float of height in metres
        :param resolver: An optional resolver that returns a performable designator for a designator description.
        """
        super().__init__(resolver)
        self.positions: List[float] = positions

    def ground(self) -> Action:
        """
        Creates a performable action designator with the first element from the list of possible torso heights.

        :return: A performable action designator
        """
        return self.Action(self.positions[0])

    def __iter__(self):
        """
        Iterates over all possible values for this designator and returns a performable action designator with the value.

        :return: A performable action designator
        """
        for position in self.positions:
            yield self.Action(position)


class SetGripperAction(ActionDesignatorDescription):
    """
    Set the gripper state of the robot
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        gripper: str
        """
        The gripper that should be set 
        """
        motion: str
        """
        The motion that should be set on the gripper
        """

        @with_tree
        def perform(self) -> None:
            MoveGripperMotion(gripper=self.gripper, motion=self.motion).resolve().perform()

        def to_sql(self) -> ORMSetGripperAction:
            return ORMSetGripperAction(self.gripper, self.motion)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMSetGripperAction:
            action = super().insert(session)
            session.add(action)
            session.commit()
            return action

    def __init__(self, grippers: List[str], motions: List[str], resolver=None):
        """
        Sets the gripper state, the desired state is given with the motion. Motion can either be 'open' or 'close'.

        :param grippers: A list of possible grippers
        :param motions: A list of possible motions
        :param resolver: An alternative resolver that returns a performable designator for a designator description
        """
        super().__init__(resolver)
        self.grippers: List[str] = grippers
        self.motions: List[str] = motions

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first element in the grippers and motions list.

        :return: A performable designator
        """
        return self.Action(self.grippers[0], self.motions[0])

    def __iter__(self):
        """
        Iterates over all possible combinations of grippers and motions

        :return: A performable designator with a combination of gripper and motion
        """
        for parameter_combination in itertools.product(self.grippers, self.motions):
            yield self.Action(*parameter_combination)


class ReleaseAction(ActionDesignatorDescription):
    """
    Releases an Object from the robot.

    Note: This action can not be used yet.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        gripper: str
        object_designator: ObjectDesignatorDescription.Object

        @with_tree
        def perform(self) -> Any:
            raise NotImplementedError()

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, grippers: List[str], object_designator_description: ObjectDesignatorDescription, resolver=None):
        super().__init__(resolver)
        self.grippers: List[str] = grippers
        self.object_designator_description = object_designator_description

    def ground(self) -> Action:
        return self.Action(self.grippers[0], self.object_designator_description.ground())


class GripAction(ActionDesignatorDescription):
    """
    Grip an object with the robot.

    :ivar grippers: The grippers to consider
    :ivar object_designator_description: The description of objects to consider
    :ivar efforts: The efforts to consider

    Note: This action can not be used yet.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        gripper: str
        object_designator: ObjectDesignatorDescription.Object
        effort: float

        @with_tree
        def perform(self) -> Any:
            raise NotImplementedError()

        def to_sql(self) -> Base:
            raise NotImplementedError()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> Base:
            raise NotImplementedError()

    def __init__(self, grippers: List[str], object_designator_description: ObjectDesignatorDescription,
                 efforts: List[float], resolver=None):
        super().__init__(resolver)
        self.grippers: List[str] = grippers
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.efforts: List[float] = efforts

    def ground(self) -> Action:
        return self.Action(self.grippers[0], self.object_designator_description.ground(), self.efforts[0])


class ParkArmsAction(ActionDesignatorDescription):
    """
    Park the arms of the robot.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):

        arm: Arms
        """
        Entry from the enum for which arm should be parked
        """

        @with_tree
        def perform(self) -> None:
            # create the keyword arguments
            kwargs = dict()

            # add park left arm if wanted
            if self.arm in [Arms.LEFT, Arms.BOTH]:
                kwargs["left_arm_config"] = "park"
                MoveArmJointsMotion(**kwargs).resolve().perform()
                if robot_description.name == "Armar6" or robot_description.name == "iai_donbot":
                    MoveTorsoAction([-0.1]).resolve().perform()
                elif robot_description.name == "rollin-justin":
                    pass
                else:
                    MoveTorsoAction([
                        0.25]).resolve().perform()  # MoveTorsoAction([0.005]).resolve().perform()  # MoveTorsoAction([0.2]).resolve().perform()
            # add park right arm if wanted
            if self.arm in [Arms.RIGHT, Arms.BOTH]:
                kwargs["right_arm_config"] = "park"
                MoveArmJointsMotion(**kwargs).resolve().perform()

        def to_sql(self) -> ORMParkArmsAction:
            return ORMParkArmsAction(self.arm.name)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs) -> ORMParkArmsAction:
            print("in insert parkArms")
            action = super().insert(session)
            session.add(action)
            session.commit()
            return action

    def __init__(self, arms: List[Arms], resolver=None):
        """
        Moves the arms in the pre-defined parking position. Arms are taken from pycram.enum.Arms

        :param arms: A list of possible arms, that could be used
        :param resolver: An optional resolver that returns a performable designator from the designator description
        """
        super().__init__(resolver)
        self.arms: List[Arms] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first element of the list of possible arms

        :return: A performable designator
        """
        return self.Action(self.arms[0])


class PickUpAction(ActionDesignatorDescription):
    """
    A class representing a designator for a pick-up action, allowing a robot to pick up a specified object.

    This class encapsulates the details of the pick-up action, including the object to be picked up, the arm to be used,
    and the grasp type. It defines the sequence of operations for the robot to execute the pick-up action, such as opening
    the gripper, moving to the object, closing the gripper, and lifting the object.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):

        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be picked up
        """

        arm: str
        """
        The arm that should be used for pick up
        """

        grasp: str
        """
        The grasp that should be used. For example, 'top' or 'front'
        """

        @with_tree
        def perform(self) -> None:
            # Initialize the local transformer and robot reference

            lt = LocalTransformer()
            robot = BulletWorld.robot
            # Retrieve object and robot from designators
            object = self.object_designator.bullet_world_object
            # Calculate the object's pose in the map frame
            oTm = object.get_pose().copy()
            execute = True

            # Adjust object pose for top-grasping, if applicable
            if self.grasp == "top":
                # Handle special cases for certain object types (e.g., Cutlery, Bowl)
                # Note: This includes hardcoded adjustments and should ideally be generalized
                # if self.object_designator.type == "Cutlery":
                #     # todo: this z is the popcorn-table height, we need to define location to get that z otherwise it
                #     #  is hardcoded
                #     oTm.pose.position.z = 0.718
                oTm.pose.position.z += 0.02

            # Determine the grasp orientation and transform the pose to the base link frame
            grasp_rotation = robot_description.grasps.get_orientation_for_grasp(self.grasp)
            if robot_description.name == "Armar6":
                oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("platform"))
            else:
                oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
            # otbtest = oTb.copy()
            testmulti = helper.multiply_quaternions(
                [oTb.orientation.x, oTb.orientation.y, oTb.orientation.z, oTb.orientation.w], grasp_rotation)
            # otbtest.orientation = testmulti
            # oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
            # otbtest = oTb.copy()
            testmulti = helper.multiply_quaternions(
                [oTb.orientation.x, oTb.orientation.y, oTb.orientation.z, oTb.orientation.w], grasp_rotation)
            # otbtest.orientation = testmulti
            # Set pose to the grasp rotation
            oTb.orientation = testmulti
            # Transform the pose to the map frame
            oTmG = lt.transform_pose(oTb, "map")
            BulletWorld.current_bullet_world.add_vis_axis(oTmG)
            marker = ManualMarkerPublisher()
            marker.clear_all_marker()
            marker.publish(pose=oTm, name="target")
            # BulletWorld.current_bullet_world.add_vis_axis(otbtest)
            #
            #
            # Open the gripper before picking up the object
            # rospy.logwarn("Opening Gripper")
            MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

            # Move to the pre-grasp position and visualize the action
            # rospy.logwarn("Picking up now")
            # BulletWorld.current_bullet_world.add_vis_axis(oTmG)
            # Execute Bool, because sometimes u only want to visualize the poses to test things
            if execute:
                if robot_description.name == "Armar6" or robot_description.name == "iai_donbot":
                    MoveTorsoAction([-0.1]).resolve().perform()
                else:
                    MoveTorsoAction([0.25]).resolve().perform()
                MoveTCPMotion(oTmG, self.arm).resolve().perform()

            # Calculate and apply any special knowledge offsets based on the robot and object type
            # Note: This currently includes robot-specific logic that should be generalized
            tool_frame = robot_description.get_tool_frame(self.arm)
            special_knowledge_offset = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))

            # todo: this is for hsrb only at the moment we will need a function that returns us special knowledge
            #  depending on robot
            if robot.name == "hsrb":
                if self.grasp == "top":
                    if self.object_designator.type == "Bowl":
                        special_knowledge_offset.pose.position.y += 0.06
                        special_knowledge_offset.pose.position.x -= 0.03  # 0.022
                    if self.object_designator.type == "Cutlery":
                        special_knowledge_offset.pose.position.x -= 0.0115  # 0.11 before, fork needs more

            elif robot.name == "pr2":
                if self.grasp == "top":
                    if self.object_designator.type == ObjectType.BOWL:
                        special_knowledge_offset.pose.position.x -= 0.06
                        special_knowledge_offset.pose.position.y += 0.03
                    if self.object_designator.type == "cutting_tool":
                        special_knowledge_offset.pose.position.z -= 0.09  # 0.11 before, fork needs more  # 0.022  # if self.object_designator.type == "Fork":  #     special_knowledge_offset.pose.position.x -= 0.02

            push_base = special_knowledge_offset
            # todo: this is for hsrb only at the moment we will need a function that returns us special knowledge
            #  depending on robot if we dont generlize this we will have a big list in the end of all robots
            if robot.name == "hsrb":
                z = 0.04
                if self.grasp == "top":
                    z = 0.039
                    if self.object_designator.type == "Bowl":
                        z = 0.045  # 0.05
                push_base.pose.position.z += z
            elif robot.name == "pr2":
                x = 0.01
                if self.grasp == "top":
                    x = 0.01
                    if self.object_designator.type == ObjectType.BOWL:
                        x = 0.01  # 0.05
                push_base.pose.position.x += x
            push_baseTm = lt.transform_pose(push_base, "map")
            special_knowledge_offsetTm = lt.transform_pose(push_base, "map")

            # Grasping from the top inherently requires calculating an offset, whereas front grasping involves
            # slightly pushing the object forward.
            # if self.grasp == "top":
            # rospy.logwarn("Offset now")
            BulletWorld.current_bullet_world.add_vis_axis(special_knowledge_offsetTm)
            if execute:
                MoveTCPMotion(special_knowledge_offsetTm, self.arm).resolve().perform()

            # rospy.logwarn("Pushing now")
            BulletWorld.current_bullet_world.add_vis_axis(push_baseTm)
            if execute:
                MoveTCPMotion(push_baseTm, self.arm).resolve().perform()
            tool_frame = robot_description.get_tool_frame(self.arm)
            robot.attach(object=self.object_designator.bullet_world_object, link=tool_frame)
            # Finalize the pick-up by closing the gripper and lifting the object
            # rospy.logwarn("Close Gripper")
            MoveGripperMotion(motion="close", gripper=self.arm).resolve().perform()

            # rospy.logwarn("Lifting now")
            liftingTm = push_baseTm
            liftingTm.pose.position.z += 0.02
            BulletWorld.current_bullet_world.add_vis_axis(liftingTm)
            if execute:
                MoveTCPMotion(liftingTm, self.arm).resolve().perform()
            BulletWorld.current_bullet_world.remove_vis_axis()

        def to_sql(self) -> ORMPickUpAction:
            return ORMPickUpAction(self.arm, self.grasp)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs):
            action = super().insert(session)
            # try to create the object designator
            if self.object_at_execution:
                od = self.object_at_execution.insert(session, )
                action.object = od.id
            else:
                action.object = None

            session.add(action)
            session.commit()

            return action

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 arms: List[str], grasps: List[str], resolver=None):
        """
        Lets the robot pick up an object. The description needs an object designator describing the object that should be
        picked up, an arm that should be used as well as the grasp from which side the object should be picked up.

        :param object_designator_description: List of possible object designator
        :param arms: List of possible arms that could be used
        :param grasps: List of possible grasps for the object
        :param resolver: An optional resolver that returns a performable designator with elements from the lists of possible paramter
        """
        super().__init__(resolver)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[str] = arms
        self.grasps: List[str] = grasps

    def ground(self) -> Action:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()

        return self.Action(obj_desig, self.arms[0], self.grasps[0])


class PlaceAction(ActionDesignatorDescription):
    """
    Places an Object at a position using an arm.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be placed
        """
        arm: str
        """
        Arm that is currently holding the object
        """

        grasp: str
        """
        Grasp that was used to pick up the object
        """
        target_location: Pose
        """
        Pose in the world at which the object should be placed
        """

        @with_tree
        def perform(self) -> None:
            lt = LocalTransformer()
            robot = BulletWorld.robot
            # Retrieve object and robot from designators
            # object = self.object_designator.bullet_world_object
            # oTm = Object Pose in Frame map
            oTm = self.target_location

            # if self.grasp == "top":
            # oTm.pose.position.z += 0.05

            grasp_rotation = robot_description.grasps.get_orientation_for_grasp(self.grasp)
            if robot_description.name == "Armar6":
                oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("platform"))
            else:
                oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))

            grasp_orientation = helper.multiply_quaternions(
                [oTb.orientation.x, oTb.orientation.y, oTb.orientation.z, oTb.orientation.w], grasp_rotation)
            oTb.orientation = grasp_orientation

            oTmG = lt.transform_pose(oTb, "map")

            # rospy.logwarn("Placing now")
            BulletWorld.current_bullet_world.add_vis_axis(oTmG)
            MoveTCPMotion(oTmG, self.arm).resolve().perform()

            tool_frame = robot_description.get_tool_frame(self.arm)
            push_base = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))
            if robot.name == "hsrb":
                z = 0.03
                if self.grasp == "top":
                    z = 0.07
                push_base.pose.position.z += z
            # if robot.name == "pr2":
            #     x = 0.03
            #     if self.grasp == "top":
            #         x = 0.07
            #     push_base.pose.position.x += x
            # todo: make this for other robots
            push_baseTm = lt.transform_pose(push_base, "map")

            # rospy.logwarn("Pushing now")
            MoveTCPMotion(push_baseTm, self.arm).resolve().perform()

            # rospy.logwarn("Opening Gripper")
            robot.detach(object=self.object_designator.bullet_world_object)
            MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()

            # rospy.logwarn("Lifting now")
            liftingTm = push_baseTm
            liftingTm.pose.position.z += 0.03
            BulletWorld.current_bullet_world.add_vis_axis(liftingTm)

            MoveTCPMotion(liftingTm, self.arm).resolve().perform()
            BulletWorld.current_bullet_world.remove_vis_axis()

        def to_sql(self) -> ORMPlaceAction:
            return ORMPlaceAction(self.arm)

        def insert(self, session, *args, **kwargs) -> ORMPlaceAction:
            action = super().insert(session)

            if self.object_designator:
                od = self.object_designator.insert(session, )
                action.object = od.id
            else:
                action.object = None

            if self.target_location:
                position = Position(*self.target_location.position_as_list())
                orientation = Quaternion(*self.target_location.orientation_as_list())
                session.add(position)
                session.add(orientation)
                session.commit()
                action.position = position.id
                action.orientation = orientation.id
            else:
                action.position = None
                action.orientation = None

            session.add(action)
            session.commit()
            return action

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 arms: List[str], grasps: List[str], target_locations: List[Pose], resolver=None):
        """
        Lets the robot place an object. The description needs an object designator describing the object that should be
        placed, an arm that should be used as well as the grasp from which side the object was picked up.

        :param object_designator_description: List of possible object designator
        :param arms: List of possible arms that could be used
        :param grasps: List of possible grasps for the object
        :param target_locations: List of possible target locations for the object to be placed
        :param resolver: An optional resolver that returns a performable designator with elements from the lists of possible paramter
        """
        super().__init__(resolver)
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[str] = arms
        self.grasps: List[str] = grasps
        self.target_locations: List[Pose] = target_locations

    def ground(self) -> Action:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()

        return self.Action(obj_desig, self.arms[0], self.grasps[0], self.target_locations[0])


class PlaceGivenObjAction(ActionDesignatorDescription):
    """
    Arm movement of the robot for placing human given objects.
    """

    # Todo: erweitern ums placing

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        # object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be placed
        """

        arm: str
        """
        Arm that is currently holding the object
        """

        target_location: Pose
        """
        Pose in the world at which the object should be placed
        """

        @with_tree
        def perform(self) -> None:
            lt = LocalTransformer()
            robot = BulletWorld.robot
            # Retrieve object and robot from designators
            # object = self.object_designator.bullet_world_object

            # self.target_location.pose.position.x = 4.86
            # self.target_location.pose.position.y = 2.2
            self.target_location.pose.position.z = 0.9

            # oTm = Object Pose in Frame map
            oTm = self.target_location

            grasp_rotation = robot_description.grasps.get_orientation_for_grasp("front")
            oTb = lt.transform_pose(oTm, robot.get_link_tf_frame("base_link"))
            oTb.orientation = grasp_rotation
            oTmG = lt.transform_pose(oTb, "map")

            rospy.logwarn("Placing now")
            MoveTCPMotion(oTmG, self.arm).resolve().perform()

            MoveTorsoAction([0.62]).resolve().perform()  # 0.62

            MoveJointsMotion(["arm_roll_joint"], [0]).resolve().perform()

            MoveJointsMotion(["wrist_roll_joint"], [-1.5]).resolve().perform()
            MoveJointsMotion(["wrist_flex_joint"], [-0.5]).resolve().perform()
            MoveJointsMotion(["arm_flex_joint"], [-1.8]).resolve().perform()
            MoveJointsMotion(["wrist_flex_joint"], [-0.8]).resolve().perform()
            NavigateAction([Pose([robot.get_pose().position.x, robot.get_pose().position.y, 0])]).resolve().perform()

            print(f" wrist_roll: {robot.get_joint_state('wrist_roll_joint')}")
            MoveGripperMotion(motion="open", gripper="left").resolve().perform()
            # MoveTorsoAction([0.4]).resolve().perform()
            NavigateAction(
                [Pose([robot.get_pose().position.x - 0.1, robot.get_pose().position.y, 0])]).resolve().perform()

            # NavigateAction([Pose([robot.get_pose().position.x - 0.1, robot.get_pose().position.y, 0])]).resolve().perform()

            # navigate close to the table  #  self.target_location.pose.position.x = 4.08  # # self.target_location.pose.position.y = 2.6  #  # because its the left arm of the hsr subtract from y position  #  self.target_location.pose.position.y -= 0.14  #  self.target_location.pose.position.z = 0  #  NavigateAction([self.target_location]).resolve().perform()  #  #  # create the keyword arguments  #  kwargs = dict()  #  #  # taking in the predefined arm position for placing  #  if self.arm in ["left", "both"]:  #       MoveTorsoAction([0.4]).resolve().perform()  #       kwargs["left_arm_config"] = "place_human_given_obj"  #       MoveArmJointsMotion(**kwargs).resolve().perform()  #       print("moveArmJointsMotion in Designator")  #  #  #  rospy.logwarn("Open Gripper")  #  MoveGripperMotion(motion="open", gripper=self.arm).resolve().perform()  #  robot.detach(object=self.object_designator.bullet_world_object)

    def __init__(self, arms: List[str], target_locations: List[Pose], resolver=None):
        """
        Lets the robot place a human given object. The description needs an object designator describing the object that should be
        placed, an arm that should be used as well as the target location where the object should be placed.

        :param object_designator_description: List of possible object designator
        :param arms: List of possible arms that could be used
        :param target_locations: List of possible target locations for the object to be placed
        :param resolver: An optional resolver that returns a performable designator with elements from the lists of possible paramter
        """
        super().__init__(resolver)
        # self.object_designator_description: Union[
        # ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[str] = arms
        self.target_locations: List[Pose] = target_locations

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first element of the list of possible arms

        :return: A performable designator
        """
        # obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
        #   ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()

        return self.Action(self.arms[0], self.target_locations[0])


class NavigateAction(ActionDesignatorDescription):
    """
    Navigates the Robot to a position.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        target_location: Pose
        """
        Location to which the robot should be navigated
        """

        @with_tree
        def perform(self) -> None:
            MoveMotion(self.target_location).resolve().perform()

        def to_sql(self) -> ORMNavigateAction:
            return ORMNavigateAction()

        def insert(self, session, *args, **kwargs) -> ORMNavigateAction:
            action = super().insert(session)

            pose = self.target_location.insert(session)
            action.pose_id = pose.id

            session.add(action)
            session.commit()

            return action

    def __init__(self, target_locations: List[Pose], resolver=None):
        """
        Navigates the robot to a location.

        :param target_locations: A list of possible target locations for the navigation.
        :param resolver: An alternative resolver that creates a performable designator from the list of possible parameter
        """
        super().__init__(resolver)
        self.target_locations: List[Pose] = target_locations

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entry of possible target locations.

        :return: A performable designator
        """
        return self.Action(self.target_locations[0])


class SearchAction(ActionDesignatorDescription):
    """
    Search for an object in the environment.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        target_location: Pose
        """
        Location to which the robot should be navigated
        """

        @with_tree
        def perform(self) -> None:
            MoveMotion(self.target_location).resolve().perform()

    def __init__(self, target_locations: List[Pose], resolver=None):
        """
        Navigates the robot to a location.

        :param target_locations: A list of possible target locations for the navigation.
        :param resolver: An alternative resolver that creates a performable designator from the list of possible parameter
        """
        super().__init__(resolver)
        self.target_locations: List[Pose] = target_locations

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entry of possible target locations.

        :return: A performable designator
        """
        return self.Action(self.target_locations[0])


class TransportAction(ActionDesignatorDescription):
    """
    Transports an object to a position using an arm
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        current_context: ContextConfig
        target_location: Optional[str] = None
        hold: Optional[bool] = None  # Add this line
        target_object: Optional[str] = None  # And this line

        @with_tree
        def perform(self) -> None:
            world = BulletWorld.current_bullet_world
            robot = BulletWorld.robot
            envi = None
            try:
                envi = BulletWorld.current_bullet_world.get_objects_by_name("environment")[0]
            except IndexError:
                rospy.logerr("No environment found")
            objects = self.current_context.get_all_objects()
            robot_desig = BelieveObject(names=[robot.name])
            envi_desig = BelieveObject(names=[envi.name])

            def search_for_object(current_context, obj_name):
                location_to_search, typeloc = current_context.search_locations(obj_name)
                open_container = False
                if typeloc in ["acces"]:
                    open_container = True

                obj_type = self.current_context.get_object_type(obj_name)

                grasp, arm, detected_object, handle_desig = access_and_pickup(current_context, location_to_search, obj_type,
                                                                open_container=open_container)
                return grasp, arm, detected_object, handle_desig

            def get_nav_pose_pick(object_type, location_to_search):
                tiago_poses = {
                    ('bowl',"island_countertop"): Pose([4.8, 3.8, 0.8]),
                    ('breakfast_cereal', "island_countertop"): Pose([1.7, 1.9, 0], [0, 0, 0, 1]),
                    ('milk', "island_countertop"): Pose([1.7, 1.5, 0], [0, 0, 0, 1]),
                    ('spoon', "island_countertop"): Pose([3, 3.7, 1.02]),
                    ('jeroen_cup', "island_countertop"): Pose([1.7, 1.7, 0], [0, 0, 0, 1]),
                }

                justin_poses = {
                    ('bowl', 'table_area_main'): Pose([3.5, 4.8, 0]),
                    ('breakfast_cereal', "island_countertop"): Pose([1.3, 2.9, 0], [0, 0, 0, 1]),
                    ('breakfast_cereal', 'table_area_main'): Pose([3.5, 4.8, 0]),
                    ('milk', "island_countertop"): Pose([1.3, 2.3, 0], [0, 0, 0, 1]),
                    ('milk', 'table_area_main'): Pose([3.5, 3.3, 0]),
                    ('spoon', "island_countertop"): Pose([3, 3.7, 0]),
                    ('spoon', 'table_area_main'): Pose([3.5, 4.3, 0]),
                    ('jeroen_cup', "island_countertop"): Pose([1.3, 2.7, 0], [0, 0, 0, 1]),
                    ('jeroen_cup', 'table_area_main'): Pose([3.5, 3.9, 0]),
                }

                if robot_description.name == "tiago_dual":
                    pose = tiago_poses.get((object_type,location_to_search))
                elif robot_description.name == "rollin-justin":
                    pose = justin_poses.get((object_type,location_to_search))
                else:
                    pose = tiago_poses.get((object_type,location_to_search))

                return pose

            def access_and_pickup(current_context, location_to_search, target_object, open_container=False):
                """
                Accesses a specified location and picks a specified object. Optionally, it can also navigate to and open a container.

                Args:
                - location_to_search: The location to search for the object.
                - target_object: The object to pick up.
                - open_container: A boolean indicating whether to navigate to and open a container before picking up the object.
                """
                # todo maybe a check which arm is free?
                global detected_object, grasp
                only_one_arm = True if ("left" in robot_description.chains) != ("right" in robot_description.chains) else False
                arm = "right"  # Default arm, can be made dynamic or parameterized
                handle_desig = None
                link_pose = current_context.environment_object.get_link_pose(location_to_search)
                if open_container:
                    link_name = current_context.get_handle(location_to_search)
                    location_to_search = link_name

                if open_container:
                    handle_desig = ObjectPart(names=[location_to_search], part_of=envi_desig.resolve())
                    # drawer_open_location = AccessingLocation(handle_desig=handle_desig.resolve(),
                    #                                          robot_desig=robot_desig.resolve()).resolve()
                    #
                    # print("drawer_open_location", drawer_open_location)

                    # hardcoded position for presentations
                    if robot_description.name == "Armar6":
                        pose = Pose([1.7, 1.7, 0], [0, 0, 0.533512180079847, 0.8457923821520558])
                    elif robot_description.name == "tiago_dual":
                        #pose = Pose([1.44, 1.8, 0], [0, 0, 0, 1])
                        pose = Pose([1.45, 2.7, 0], [0, 0, 0, 1])
                    elif robot_description.name == "iai_donbot":
                        pose = Pose([1.7674915790557861, 2.7073597526550293, 0], [0, 0, 0.2040033016133158, 0.9789702002261697])
                    elif robot_description.name == "rollin-justin":
                        pose = Pose([1.4, 1.6, 0], [0, 0, 0.2040033016133158, 0.9789702002261697])
                    else:
                        pose = Pose([1.75, 1.79, 0], [0, 0, 0.533512180079847, 0.8457923821520558])
                    if robot_description.name == "tiago_dual":
                        drawer_open_location = AccessingLocation.Location(pose, ["right"])
                    else:
                        drawer_open_location = AccessingLocation.Location(pose, ["left"])
                    NavigateAction([drawer_open_location.pose]).resolve().perform()
                    OpenAction(object_designator_description=handle_desig,
                               arms=[drawer_open_location.arms[0]]).resolve().perform()
                    arm = select_alternate_arm(drawer_open_location.arms[0])
                    current_context.environment_object.detach(
                        current_context.spoon_)  # Assuming spoon_ needs to be detached

                else:
                    # todo this should be from KB depending on the location
                    if location_to_search == "island_countertop":
                        location_pose = Pose([1.3, 2, 0])
                    elif location_to_search == "table_area_main":
                        location_pose = Pose([3.5, 3.5, 0])
                    else:
                        location_pose = Pose([1.3, 2, 0])
                    NavigateAction(target_locations=[location_pose]).resolve().perform()

                ParkArmsAction([Arms.BOTH]).resolve().perform()
                LookAtAction([link_pose]).resolve().perform()
                # todo i guess i should do "all" and then check for types
                status, object_dict = DetectAction(technique='specific', object_type=target_object).resolve().perform()
                if status:
                    for key, value in object_dict.items():
                        detected_object = object_dict[key]
                        if not open_container:
                            if robot_description.name == "tiago_dual" or robot_description.name == "rollin-justin":
                                NavigateAction([get_nav_pose_pick(target_object, location_to_search)]).resolve().perform()
                                arm = "right" # was left for tiago
                            else:
                                reachable_location = CostmapLocation(target=detected_object.pose,
                                                                     reachable_for=robot_desig.resolve(),
                                                                     reachable_arm="left").resolve()
                                NavigateAction([reachable_location.pose]).resolve().perform()
                                arm = reachable_location.reachable_arms[0]
                        else:
                            if robot_description.name == "tiago_dual":

                                NavigateAction([Pose([1.44, 1.8, 0], [0, 0, 0, 1])]).resolve().perform()
                                arm = "left"
                        if robot_description.name == "iai_donbot":
                            arm = "left"
                        grasp = pickup_target_object(detected_object, arm)
                else:
                    rospy.logerr("Object not found")
                    grasp, detected_object = None, None

                if open_container and not only_one_arm:
                    if robot_description.name == "tiago_dual":
                        NavigateAction([drawer_open_location.pose]).resolve().perform()
                    CloseAction(object_designator_description=handle_desig,
                                arms=[drawer_open_location.arms[0]]).resolve().perform()

                ParkArmsAction([Arms.BOTH]).resolve().perform()


                # for some reason handle_desig makes donbot grasp the middle of the drawer if i create it from scratch
                # for closing the drawer, so I need to return the handle_desig back for later usage. Should investigate.
                return grasp, arm, detected_object, handle_desig

            def get_place_pose(object_type, location, robot_name):
                pr2_poses = {
                    ('bowl', 'table_area_main'): Pose([4.8, 3.8, 0.8]),
                    ('bowl', 'island_countertop'): Pose([3, 3.8, 1.02], [0, 0, 1, 0]),
                    ('breakfast_cereal', 'table_area_main'): Pose([4.8, 3.6, 0.8]),
                    ('breakfast_cereal', 'island_countertop'): Pose([3, 3.6, 1.02], [0, 0, 1, 0]),
                    ('milk', 'table_area_main'): Pose([4.8, 4, 0.8]),
                    ('milk', 'island_countertop'): Pose([3, 4, 1.02], [0, 0, 1, 0]),
                    ('spoon', 'table_area_main'): Pose([4.8, 3.7, 0.8], [0, 0, 1, 0]),
                    ('spoon', 'island_countertop'): Pose([3, 3.7, 1.02]),
                    ('jeroen_cup', 'table_area_main'): Pose([4.9, 3.9, 0.74]),
                    ('jeroen_cup', 'island_countertop'): Pose([2.9, 3.9, 0.95], [0, 0, 1, 0]),
                }

                armar_poses = {
                    ('bowl', 'table_area_main'): Pose([4.8, 3.8, 0.8]),
                    ('bowl', 'island_countertop'): Pose([3, 3.8, 1.02], [0, 0, 1, 0]),
                    ('breakfast_cereal', 'table_area_main'): Pose([4.8, 3.6, 0.8]),
                    ('breakfast_cereal', 'island_countertop'): Pose([3, 3.6, 1.02], [0, 0, 1, 0]),
                    ('milk', 'table_area_main'): Pose([4.8, 4, 0.8]),
                    ('milk', 'island_countertop'): Pose([3, 4, 1.02], [0, 0, 1, 0]),
                    ('spoon', 'table_area_main'): Pose([4.8, 3.7, 0.8], [0, 0, 0, 1]),
                    ('spoon', 'island_countertop'): Pose([3, 3.7, 1.02]),
                    ('jeroen_cup', 'table_area_main'): Pose([4.9, 3.9, 0.74]),
                    ('jeroen_cup', 'island_countertop'): Pose([2.9, 3.9, 0.95], [0, 0, 1, 0]),
                }

                tiago_poses = {
                    ('bowl', 'table_area_main'): Pose([4.8, 3.8, 0.8]),
                    ('bowl', 'island_countertop'): Pose([3, 3.8, 1.02], [0, 0, 1, 0]),
                    ('breakfast_cereal', 'table_area_main'): Pose([4.8, 3.6, 0.8]),
                    ('breakfast_cereal', 'island_countertop'): Pose([3, 3.6, 1.02], [0, 0, 1, 0]),
                    ('milk', 'table_area_main'): Pose([4.8, 4, 0.8]),
                    ('milk', 'island_countertop'): Pose([3, 4, 1.02], [0, 0, 1, 0]),
                    ('spoon', 'table_area_main'): Pose([4.8, 3.7, 0.8], [0, 0, 0, 1]),
                    ('spoon', 'island_countertop'): Pose([3, 3.7, 0.975]),
                    ('jeroen_cup', 'table_area_main'): Pose([4.9, 3.9, 0.74]),
                    ('jeroen_cup', 'island_countertop'): Pose([2.9, 3.9, 0.95], [0, 0, 1, 0]),
                }

                donbot_poses = {
                    ('bowl', 'table_area_main'): Pose([4.8, 3.8, 0.8]),
                    ('bowl', 'island_countertop'): Pose([3, 3.8, 1.02], [0, 0, 1, 0]),
                    ('breakfast_cereal', 'table_area_main'): Pose([4.8, 3.6, 0.8]),
                    ('breakfast_cereal', 'island_countertop'): Pose([3, 3.6, 1.02], [0, 0, 1, 0]),
                    ('milk', 'table_area_main'): Pose([4.8, 4, 0.8]),
                    ('milk', 'island_countertop'): Pose([3, 4, 1.02], [0, 0, 1, 0]),
                    ('spoon', 'table_area_main'): Pose([4.8, 3.7, 0.8], [0, 0, 0, 1]),
                    ('spoon', 'island_countertop'): Pose([3, 3.7, 1.02], [0, 0, 1, 0]),
                    ('jeroen_cup', 'table_area_main'): Pose([4.9, 3.9, 0.74]),
                    ('jeroen_cup', 'island_countertop'): Pose([2.9, 3.9, 0.95], [0, 0, 1, 0]),
                }

                justin_poses = {
                    ('bowl', 'table_area_main'): Pose([4.8, 3.8, 0.8]),
                    ('bowl', 'island_countertop'): Pose([3, 3.8, 1.02], [0, 0, 1, 0]),
                    ('breakfast_cereal', 'table_area_main'): Pose([4.8, 3.6, 0.8]),
                    ('breakfast_cereal', 'island_countertop'): Pose([3, 3.6, 1.02], [0, 0, 1, 0]),
                    ('milk', 'table_area_main'): Pose([4.8, 4, 0.8]),
                    ('milk', 'island_countertop'): Pose([3, 4, 1.02], [0, 0, 1, 0]),
                    ('spoon', 'table_area_main'): Pose([4.8, 3.7, 0.8], [0, 0, 0, 1]),
                    ('spoon', 'island_countertop'): Pose([3, 3.7, 1.02], [0, 0, 1, 0]),
                    ('jeroen_cup', 'table_area_main'): Pose([4.9, 3.9, 0.74]),
                    ('jeroen_cup', 'island_countertop'): Pose([2.9, 3.9, 0.95], [0, 0, 1, 0]),
                }

                if robot_name == "Armar6":
                    pose = armar_poses.get((object_type, location))
                elif robot_name == "tiago_dual":
                    pose = tiago_poses.get((object_type, location))
                elif robot_name == "iai_donbot":
                    pose = donbot_poses.get((object_type, location))
                elif robot_name == "rollin-justin":
                    pose = justin_poses.get((object_type, location))
                else:
                    pose = pr2_poses.get((object_type, location))
                return pose

            def get_nav_pose(object_type, location, robot_name):
                global current_location
                pr2_poses = {('bowl', 'table_area_main'): Pose([4, 3.8, 0]),
                             ('bowl', 'island_countertop'): Pose([3.9, 3.8, 0], [0, 0, 1, 0]),
                             ('breakfast_cereal', 'table_area_main'): Pose([4, 3.6, 0]),
                             ('breakfast_cereal', 'island_countertop'): Pose([3.9, 3.6, 0], [0, 0, 1, 0]),
                             ('milk', 'table_area_main'): Pose([4, 4, 0]),
                             ('milk', 'island_countertop'): Pose([3.9, 4, 0], [0, 0, 1, 0]),
                             ('spoon', 'table_area_main'): Pose([4.2, 3.7, 0]),
                             ('spoon', 'island_countertop'): Pose([3.7, 3.7, 0], [0, 0, 1, 0]),
                             ('jeroen_cup', 'table_area_main'): Pose([4.3, 3.9, 0]),
                             ('jeroen_cup', 'island_countertop'): Pose([3.8, 3.9, 0], [0, 0, 1, 0]), }

                armar_poses = {('bowl', 'table_area_main'): Pose([3.8, 3.8, 0]),
                               ('bowl', 'island_countertop'): Pose([3.9, 3.8, 0], [0, 0, 1, 0]),
                               ('breakfast_cereal', 'table_area_main'): Pose([3.8, 3.6, 0]),
                               ('breakfast_cereal', 'island_countertop'): Pose([4.1, 3.6, 0], [0, 0, 1, 0]),
                               ('milk', 'table_area_main'): Pose([4.1, 3.8, 0]),
                               ('milk', 'island_countertop'): Pose([3.8, 4.2, 0], [0, 0, 1, 0]),
                               ('spoon', 'table_area_main'): Pose([4.3, 4.3, 0]),
                               ('spoon', 'island_countertop'): Pose([3.6, 3.1, 0], [0, 0, 1, 0]),
                               ('jeroen_cup', 'table_area_main'): Pose([4.1, 3.9, 0]),
                               ('jeroen_cup', 'island_countertop'): Pose([4, 3.9, 0], [0, 0, 1, 0]), }

                tiago_poses = {('bowl', 'table_area_main'): Pose([3.8, 3.8, 0]),
                               ('bowl', 'island_countertop'): Pose([3.9, 3.8, 0], [0, 0, 1, 0]),
                               ('breakfast_cereal', 'table_area_main'): Pose([4, 3.2, 0]),
                               ('breakfast_cereal', 'island_countertop'): Pose([3.8, 3.6, 0], [0, 0, 1, 0]),
                               ('milk', 'table_area_main'): Pose([4.1, 3.6, 0]),
                               ('milk', 'island_countertop'): Pose([3.8, 4.2, 0], [0, 0, 1, 0]),
                               ('spoon', 'table_area_main'): Pose([4.2, 3.3, 0.0]),
                               ('spoon', 'island_countertop'): Pose([3.53, 4.1, 0], [0, 0, 1, 0]),
                               ('jeroen_cup', 'table_area_main'): Pose([4.1, 3.4, 0]),
                               ('jeroen_cup', 'island_countertop'): Pose([3.7, 4.3, 0], [0, 0, 1, 0]), }

                donbot_poses = {('bowl', 'table_area_main'): Pose([4, 3.8, 0], [0, 0, 1, 1]),
                             ('bowl', 'island_countertop'): Pose([3.7, 3.8, 0], [0, 0, 1, -1]),
                             ('breakfast_cereal', 'table_area_main'): Pose([4, 3.6, 0]),
                             ('breakfast_cereal', 'island_countertop'): Pose([3.9, 3.6, 0], [0, 0, 1, -1]),
                             ('milk', 'table_area_main'): Pose([4, 4, 0]),
                             ('milk', 'island_countertop'): Pose([3.9, 4, 0], [0, 0, 1, -1]),
                             ('spoon', 'table_area_main'): Pose([4.2, 3.7, 0], [0, 0, 1, 1]),
                             ('spoon', 'island_countertop'): Pose([3.7, 3.7, 0], [0, 0, 1, -1]),
                             ('jeroen_cup', 'table_area_main'): Pose([4.3, 3.9, 0]),
                             ('jeroen_cup', 'island_countertop'): Pose([3.8, 3.9, 0], [0, 0, 1, -1]), }

                justin_poses = {('bowl', 'table_area_main'): Pose([3.5, 3.8, 0]),
                               ('bowl', 'island_countertop'): Pose([4.3, 3.8, 0], [0, 0, 1, 0]),
                               ('breakfast_cereal', 'table_area_main'): Pose([3.5, 3.6, 0]),
                               ('breakfast_cereal', 'island_countertop'): Pose([4.3, 3.6, 0], [0, 0, 1, 0]),
                               ('milk', 'table_area_main'): Pose([3.5, 3.8, 0]),
                               ('milk', 'island_countertop'): Pose([4.3, 4.2, 0], [0, 0, 1, 0]),
                               ('spoon', 'table_area_main'): Pose([3.5, 4.3, 0]),
                               ('spoon', 'island_countertop'): Pose([4, 3.4, 0], [0, 0, 1, 0]),
                               ('jeroen_cup', 'table_area_main'): Pose([3.5, 3.9, 0]),
                               ('jeroen_cup', 'island_countertop'): Pose([4.2, 3.9, 0], [0, 0, 1, 0]), }

                if robot_name == "Armar6":
                    pose = armar_poses.get((object_type, location))
                elif robot_name == "tiago_dual":
                    pose = tiago_poses.get((object_type, location))
                elif robot_name == "iai_donbot":
                    pose = donbot_poses.get((object_type, location))
                elif robot_name == "rollin-justin":
                    pose = justin_poses.get((object_type, location))
                else:
                    pose = pr2_poses.get((object_type, location))
                return pose

            def place_object(grasp_type, target_location, perceived_obj, arm):
                """
                Places an object at a specified location based on the grasp type and the target location.

                Args:
                - grasp_type: The type of grasp used ('top' or 'front').
                - target_location: The target location to place the object (e.g., 'table').
                - detected_object: The object that has been detected and is to be placed.
                - arm: The arm used to perform the operation.
                """
                # todo add placing in drawer
                # Set margin based on grasp type
                if grasp_type == "top":
                    margin_cm = 0.08
                elif grasp_type == "front":
                    margin_cm = 0.2
                else:
                    margin_cm = 0.1  # Default margin if grasp type is unspecified

                # Set environment link based on target location
                environment_link = target_location
                # Find a reachable location and navigation pose
                #place_pose, nav_pose = find_reachable_location_and_nav_pose(enviroment_link=environment_link,
                #                                                            enviroment_desig=envi_desig.resolve(),
                #                                                            object_desig=perceived_obj,
                #                                                            robot_desig=robot_desig.resolve(), arm=arm,
                #                                                            world=world, margin_cm=margin_cm)
                nav_pose = get_nav_pose(perceived_obj.type, target_location, robot_description.name)
                place_pose = get_place_pose(perceived_obj.type, target_location, robot_description.name)

                # Check if a navigation pose was found
                if not nav_pose:
                    rospy.logerr("No navigable location found")
                    return False

                # Navigate to the location, adjust the torso, and place the object
                NavigateAction(target_locations=[nav_pose]).resolve().perform()
                if robot_description.name == "Armar6" or robot_description.name == "iai_donbot":
                    MoveTorsoAction([-0.1]).resolve().perform()
                else:
                    MoveTorsoAction([0.25]).resolve().perform()  # Adjust torso height as needed
                PlaceAction(perceived_obj, [arm], [grasp_type], [place_pose]).resolve().perform()
                ParkArmsAction([Arms.BOTH]).resolve().perform()
                return True

            def pickup_target_object(detected_object, arm):
                ParkArmsAction([arm]).resolve().perform()
                grasp = None
                if detected_object.type in ["spoon", "bowl", "cutlery", "cutting_tool"]:
                    grasp = "top"
                else:
                    grasp = "front"
                PickUpAction(detected_object, [arm], [grasp]).resolve().perform()

                ParkArmsAction([Arms.BOTH]).resolve().perform()
                return grasp

            def select_alternate_arm(current_arm):
                # Selects the alternate arm based on the current arm.
                return "right" if current_arm == "left" else "left"

            if self.target_object:
                objects = [self.target_object]

            for obj in objects:
                if robot_description.name == "Armar6" or robot_description.name == "iai_donbot":
                    MoveTorsoAction([-0.1]).resolve().perform()
                else:
                    MoveTorsoAction([0.25]).resolve().perform()
                ParkArmsAction([Arms.BOTH]).resolve().perform()
                grasp, arm, detected_object, handle_desig = search_for_object(self.current_context, obj)
                if not self.hold:
                    place_object(grasp, self.target_location, detected_object, arm)
                if ("left" in robot_description.chains) != ("right" in robot_description.chains):
                    if obj in ["spoon"]:
                        if robot_description.name == "iai_donbot":
                            pose = Pose([1.6074915790557861, 2.7073597526550293, 0], [0, 0, 0.2040033016133158, 0.9789702002261697])
                        else:
                            pose = Pose([1.75, 1.79, 0], [0, 0, 0.533512180079847, 0.8457923821520558])
                        drawer_open_location = AccessingLocation.Location(pose, ["left"])
                        NavigateAction([drawer_open_location.pose]).resolve().perform()
                        CloseAction(object_designator_description=handle_desig,
                                   arms=[drawer_open_location.arms[0]]).resolve().perform()
                        ParkArmsAction([Arms.BOTH]).resolve().perform()


    def __init__(self, current_context: ContextConfig, target_location: Optional[str] = None,
                 hold: Optional[bool] = None, target_object: Optional[str] = None, resolver=None):
        """
        Designator representing a pick and place plan.

        :param object_designator_description: Object designator description or a specified Object designator that should be transported
        :param arms: A List of possible arms that could be used for transporting
        :param target_locations: A list of possible target locations for the object to be placed
        :param resolver: An alternative resolver that returns a performable designator for the list of possible parameter
        """
        super().__init__(resolver)
        self.current_context: ContextConfig = current_context
        self.target_locations: Optional[str] = target_location
        self.hold: Optional[bool] = hold
        self.target_object: Optional[str] = target_object

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return self.Action(self.current_context, self.target_locations, self.hold, self.target_object)


class LookAtAction(ActionDesignatorDescription):
    """
    Lets the robot look at a position.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        target: Pose
        """
        Position at which the robot should look, given as 6D pose
        """

        @with_tree
        def perform(self) -> None:
            LookingMotion(target=self.target).resolve().perform()

        def to_sql(self) -> ORMLookAtAction:
            return ORMLookAtAction()

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMLookAtAction:
            action = super().insert(session)

            pose = self.target.insert(session)
            action.pose_id = pose.id

            session.add(action)
            session.commit()
            return action

    def __init__(self, targets: List[Pose], resolver=None):
        """
        Moves the head of the robot such that it points towards the given target location.

        :param targets: A list of possible locations to look at
        :param resolver: An alternative resolver that returns a performable designator for a list of possible target locations
        """
        super().__init__(resolver)
        self.targets: List[Pose] = targets

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the first entry in the list of possible targets

        :return: A performable designator
        """
        return self.Action(self.targets[0])


class DetectAction(ActionDesignatorDescription):
    """
    Detects an object with given technique.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        technique: str
        """
        Technique means how the object should be detected, e.g. 'color', 'shape', etc. 
        Or 'all' if all objects should be detected
        """

        object_type: Optional[str] = None
        """
        Object designator loosely describing the object, e.g. only type. 
        """

        state: Optional[str] = None
        """
        The state instructs our perception system to either start or stop the search for an object or human.
        """

        @with_tree
        def perform(self) -> Any:
            return DetectingMotion(technique=self.technique, object_type=self.object_type,
                                   state=self.state).resolve().perform()

        # def to_sql(self) -> ORMDetectAction:  #     return ORMDetectAction()  #  # def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMDetectAction:  #     action = super().insert(session)  #  #     od = self.object_type.insert(session)  #     action.object_id = od.id  #  #     session.add(action)  #     session.commit()  #  #     return action

    def __init__(self, technique, resolver=None, object_type: Optional[str] = None, state: Optional[str] = None):
        """
        Tries to detect an object in the field of view (FOV) of the robot.

        :param technique: Technique means how the object should be detected, e.g. 'color', 'shape', etc.
        :param object_designator_description: Object designator describing the object
        :param state: The state instructs our perception system to either start or stop the search for an object or human.
        :param resolver: An alternative resolver
        """
        super().__init__(resolver)
        self.technique: str = technique
        self.object_type: Optional[str] = object_type
        self.state: Optional[str] = state

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the resolved object description.

        :return: A performable designator
        """
        return self.Action(technique=self.technique, object_type=self.object_type, state=self.state)


class OpenAction(ActionDesignatorDescription):
    """
    Opens a container like object

    Can currently not be used
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_designator: ObjectPart.Object
        """
        Object designator describing the object that should be opened
        """
        arm: str
        """
        Arm that should be used for opening the container
        """

        @with_tree
        def perform(self) -> Any:
            GraspingAction.Action(self.arm, self.object_designator).perform()
            OpeningMotion(self.object_designator, self.arm).resolve().perform()

            MoveGripperMotion("open", self.arm, allow_gripper_collision=True).resolve().perform()

        def to_sql(self) -> ORMOpenAction:
            return ORMOpenAction(self.arm)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMOpenAction:
            action = super().insert(session)

            op = self.object_designator.insert(session)
            action.object_id = op.id

            session.add(action)
            session.commit()

            return action

    def __init__(self, object_designator_description: ObjectPart, arms: List[str], resolver=None):
        """
        Moves the arm of the robot to open a container.

        :param object_designator_description: Object designator describing the handle that should be used to open
        :param arms: A list of possible arms that should be used
        :param resolver: A alternative resolver that returns a performable designator for the lists of possible parameter.
        """
        super().__init__(resolver)
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[str] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the resolved object description and the first entries
        from the lists of possible parameter.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.resolve(), self.arms[0])


class CloseAction(ActionDesignatorDescription):
    """
    Closes a container like object.

    Can currently not be used
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        object_designator: ObjectPart.Object
        """
        Object designator describing the object that should be closed
        """
        arm: str
        """
        Arm that should be used for closing
        """

        @with_tree
        def perform(self) -> Any:
            GraspingAction.Action(self.arm, self.object_designator).perform()
            ClosingMotion(self.object_designator, self.arm).resolve().perform()

            MoveGripperMotion("open", self.arm, allow_gripper_collision=True).resolve().perform()

        def to_sql(self) -> ORMCloseAction:
            return ORMCloseAction(self.arm)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMCloseAction:
            action = super().insert(session)

            op = self.object_designator.insert(session)
            action.object_id = op.id

            session.add(action)
            session.commit()

            return action

    def __init__(self, object_designator_description: ObjectPart, arms: List[str], resolver=None):
        """
        Attempts to close an open container

        :param object_designator_description: Object designator description of the handle that should be used
        :param arms: A list of possible arms to use
        :param resolver: An alternative resolver that returns a performable designator for the list of possible parameter
        """
        super().__init__(resolver)
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[str] = arms

    def ground(self) -> Action:
        """
        Default resolver that returns a performable designator with the resolved object designator and the first entry from
        the list of possible arms.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.resolve(), self.arms[0])


class GraspingAction(ActionDesignatorDescription):
    """
    Grasps an object described by the given Object Designator description
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        arm: str
        """
        The arm that should be used to grasp
        """
        object_desig: Union[ObjectDesignatorDescription.Object, ObjectPart.Object]
        """
        Object Designator for the object that should be grasped
        """

        def perform(self) -> Any:
            if isinstance(self.object_desig, ObjectPart.Object):
                object_pose = self.object_desig.part_pose
            else:
                object_pose = self.object_desig.bullet_world_object.get_pose()
            lt = LocalTransformer()
            gripper_name = robot_description.get_tool_frame(self.arm)
            # oTb = object_pose
            # grasp_rotation = robot_description.grasps.get_orientation_for_grasp("front")
            #
            # grasp_orientation = helper.multiply_quaternions(
            #     [oTb.orientation.x, oTb.orientation.y, oTb.orientation.z, oTb.orientation.w], grasp_rotation)
            # oTb.orientation = grasp_orientation

            object_pose_in_gripper = lt.transform_pose(object_pose, BulletWorld.robot.get_link_tf_frame(gripper_name))

            pre_grasp = object_pose_in_gripper.copy()
            pre_grasp.pose.position.x -= 0.1

            # MoveTCPMotion(pre_grasp, self.arm).resolve().perform()
            MoveGripperMotion("open", self.arm).resolve().perform()

            MoveTCPMotion(object_pose, self.arm, allow_gripper_collision=True).resolve().perform()
            MoveGripperMotion("close", self.arm, allow_gripper_collision=True).resolve().perform()

        def to_sql(self) -> ORMGraspingAction:
            return ORMGraspingAction(self.arm)

        def insert(self, session: sqlalchemy.orm.session.Session, *args, **kwargs) -> ORMGraspingAction:
            action = super().insert(session)

            od = self.object_desig.insert(session)
            action.object_id = od.id

            session.add(action)
            session.commit()

            return action

    def __init__(self, arms: List[str], object_description: Union[ObjectDesignatorDescription, ObjectPart],
                 resolver: Callable = None):
        """
        Will try to grasp the object described by the given description. Grasping is done by moving into a pre grasp
        position 10 cm before the object, opening the gripper, moving to the object and then closing the gripper.

        :param arms: List of Arms that should be used for grasping
        :param object_description: Description of the object that should be grasped
        :param resolver: An alternative resolver to get a specified designator from the designator description
        """
        super().__init__(resolver)
        self.arms: List[str] = arms
        self.object_description: ObjectDesignatorDescription = object_description

    def ground(self) -> Action:
        """
        Default resolver that takes the first element from the list of arms and the first solution for the object
        designator description ond returns it.

        :return: A performable action designator that contains specific arguments
        """
        return self.Action(self.arms[0], self.object_description.resolve())


class CuttingAction(ActionDesignatorDescription):
    """
    A designator for robotic cutting actions. This class facilitates the specification and execution of
    cutting tasks using a robot.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        """
        Represents a specific cutting action to be performed by the robot. This class encapsulates
        all necessary details for executing the cutting task, including the object to be cut, the cutting tool,
        the arm to use, and the cutting technique.
        """

        object_to_be_cut: ObjectDesignatorDescription.Object
        """
        The object to be cut.
        """

        tool: ObjectDesignatorDescription.Object
        """
        The tool used for cutting.
        """

        arm: str
        """
        The robot arm designated for the cutting task.
        """

        technique: Optional[str] = None
        """
        The technique used for cutting (default is None).
        """

        slice_thickness: Optional[float] = 0.03
        """
        The upper bound thickness of the slices (default is 0.03f).
        """

        object_to_be_cut_at_execution: Optional[ObjectDesignatorDescription.Object] = dataclasses.field(init=False)
        """
        The object_to_be_cut at the time this Action got executed. It is used to be a static, information holding entity. It is
        not updated when the BulletWorld object is changed.
        """

        tool_at_execution: Optional[ObjectDesignatorDescription.Object] = dataclasses.field(init=False)
        """
        The object_tool at the time this Action got executed. It is used to be a static, information holding entity. It is
        not updated when the BulletWorld object is changed.
        """

        def calculate_slices(self, obj_length, technique, thickness):
            """
            Calculates the number of slices and the starting offset based on the cutting technique and slice thickness.
            """
            # slicing
            num_slices = int(obj_length // thickness)
            start_offset = (-obj_length / 2) + (thickness / 2)

            # Calculate the starting position for slicing, adjusted based on the chosen technique
            if technique in ['Halving']:
                start_offset = 0  # No offset needed for halving
                num_slices = 1  # Only one slice for halving
                return num_slices, start_offset
            elif technique in ['Cutting Action', 'Sawing', 'Paring', 'Cutting', 'Carving']:
                # Calculate number of slices and initial offset for regular slicing
                num_slices = 1

            return num_slices, start_offset

        def calculate_slice_pose(self, object_pose, slice_position, obj_width):
            """
            Determines the pose for each slice based on the object's current pose, the slice position, and the object's dimensions.
            """
            slice_pose = object_pose.copy()
            slice_pose.position.x = slice_position

            return slice_pose

        def adjust_for_lifting(self, pose, height):
            """
            Adjusts the given pose to lift the cutting tool above the object before and after cutting.
            """
            lift_pose = pose.copy()
            lift_pose.position.z += 2 * height  # Lift the tool above the object
            return lift_pose

        def facing_robot(self, pose, robot):
            """
            Checks if the object is facing the robot.
            """
            if helper.facing_robot(pose, robot):
                rotate_q = helper.axis_angle_to_quaternion([0, 0, 1], 180)
                resulting_q = helper.multiply_quaternions(pose.pose, rotate_q)
                pose.pose.orientation.x = resulting_q[0]
                pose.pose.orientation.y = resulting_q[1]
                pose.pose.orientation.z = resulting_q[2]
                pose.pose.orientation.w = resulting_q[3]
            return pose

        def perpendicular_pose(self, pose):
            perpendicular_pose = pose.copy()
            rotation_axis = [0, 0, 1]
            rotation_quaternion = helper.axis_angle_to_quaternion(rotation_axis, 90)
            resulting_q = helper.multiply_quaternions(perpendicular_pose.pose, rotation_quaternion)
            perpendicular_pose.pose.orientation.x = resulting_q[0]
            perpendicular_pose.pose.orientation.y = resulting_q[1]
            perpendicular_pose.pose.orientation.z = resulting_q[2]
            perpendicular_pose.pose.orientation.w = resulting_q[3]
            return perpendicular_pose

        @with_tree
        def perform(self) -> None:
            """
            Executes the cutting action using the specified parameters. It involves determining the
            grasp orientation, computing slice positions, and orchestrating the necessary robot motions
            for executing the cutting task.
            """
            local_tf = LocalTransformer()
            # Access the designated object for cutting and retrieve its dimensions
            object = self.object_to_be_cut.bullet_world_object
            obj_length, obj_width, object_height = object.get_object_dimensions()

            # Retrieve the current pose of the object and transform it to the object frame
            oTm = object.get_pose()
            # BulletWorld.current_bullet_world.add_vis_axis(oTm)
            object_pose = local_tf.transform_to_object_frame(oTm, object)

            num_slices, start_offset = self.calculate_slices(obj_length, self.technique, self.slice_thickness)

            # Generate coordinates for each slice along the object's length
            slice_coordinates = [start_offset + i * self.slice_thickness for i in range(num_slices)]

            # Transform the coordinates of each slice to the map frame, maintaining orientation
            slice_poses = []
            for x in slice_coordinates:
                # Adjust slice pose based on object dimensions and orientation
                tmp_pose = object_pose.copy()
                tmp_pose.pose.position.y += obj_width  # plus tool länge  # Offset position for slicing
                tmp_pose.pose.position.x = x  # Set slicing position
                # tmp_pose.pose.position.z
                sTm = local_tf.transform_pose(tmp_pose, "map")
                # BulletWorld.current_bullet_world.add_vis_axis(sTm)
                slice_poses.append(sTm)

            # Process each slice pose for the cutting action
            for slice_pose in slice_poses:
                # check if obj is facing the object
                slice_pose = self.facing_robot(slice_pose, BulletWorld.robot)

                perpendicular_pose = self.perpendicular_pose(slice_pose)

                object_pose = self.tool.pose

                # Transformations such that the target position is the position of the object and not the tcp
                tcp_to_object = local_tf.transform_pose(object_pose,
                                                        BulletWorld.robot.get_link_tf_frame(
                                                            robot_description.get_tool_frame(self.arm)))
                target_diff = (perpendicular_pose.to_transform("target").inverse_times(
                    tcp_to_object.to_transform("object")).to_pose())

                lift_pose = target_diff.copy()
                lift_pose.pose.position.z += object_height  # Lift the tool above the object

                BulletWorld.current_bullet_world.add_vis_axis(target_diff)
                # BulletWorld.current_bullet_world.add_vis_axis(lift_pose)
                MoveTCPMotion(lift_pose, self.arm).resolve().perform()
                MoveTCPMotion(target_diff, self.arm).resolve().perform()
                MoveTCPMotion(lift_pose, self.arm).resolve().perform()

    def __init__(self, object_to_be_cut: ObjectDesignatorDescription, tool: ObjectDesignatorDescription,
                 arms: List[str], technique: Optional[str] = None):
        """
        Initializes a CuttingAction with specified object and tool designators, arms, and an optional cutting technique.

        Args:
            object_to_be_cut: Designator for the object to be cut.
            tool: Designator for the cutting tool.
            arms: List of possible arms to be used for the cutting action.
            technique: Optional cutting technique to be used.
        """
        super(CuttingAction, self).__init__()
        self.object_to_be_cut: ObjectDesignatorDescription = object_to_be_cut
        self.tool: ObjectDesignatorDescription = tool
        self.arms: List[str] = arms
        self.technique: Optional[str] = technique

    def __iter__(self):
        """
        Iterator for generating all possible action combinations based on the provided object, tool, and arms.

        :yields:  A possible cutting action with a specific combination of object, tool, and arm.
        """
        yield self.Action(self.object_to_be_cut, self.tool, self.arms[0], self.technique)

    def ground(self) -> Action:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return next(iter(self))


class MixingAction(ActionDesignatorDescription):
    """
    Designator to let the robot perform a mixing action.
    """

    @dataclasses.dataclass
    class Action(ActionDesignatorDescription.Action):
        """
        Action class for the Mixing action.
        """

        object_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the object that should be mixed.
        """

        object_tool_designator: ObjectDesignatorDescription.Object
        """
        Object designator describing the mixing tool.
        """

        arm: str
        """
        The arm that should be used for mixing.
        """

        grasp: str
        """
        The grasp that should be used for mixing. For example, 'left' or 'right'.
        """

        object_at_execution: Optional[ObjectDesignatorDescription.Object] = dataclasses.field(init=False)
        """
        The object at the time this Action got created. It is used to be a static, information holding entity. It is
        not updated when the BulletWorld object is changed.
        """

        @with_tree
        def perform(self) -> None:
            """
            Perform the mixing action using the specified object, tool, arm, and grasp.
            """
            # Store the object's data copy at execution
            self.object_at_execution = self.object_designator.data_copy()
            # Retrieve object and robot from designators
            object = self.object_designator.bullet_world_object

            obj_dim = object.get_object_dimensions()

            dim = [max(obj_dim[0], obj_dim[1]), min(obj_dim[0], obj_dim[1]), obj_dim[2]]
            obj_height = dim[2]
            oTm = object.get_pose()
            object_pose = object.local_transformer.transform_to_object_frame(oTm, object)

            def generate_spiral(pose, upward_increment, radial_increment, angle_increment, steps):
                x_start, y_start, z_start = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
                spiral_poses = []

                for t in range(2 * steps):
                    tmp_pose = pose.copy()

                    r = radial_increment * t
                    a = angle_increment * t
                    h = upward_increment * t

                    x = x_start + r * math.cos(a)
                    y = y_start + r * math.sin(a)
                    z = z_start + h

                    tmp_pose.pose.position.x += x
                    tmp_pose.pose.position.y += y
                    tmp_pose.pose.position.z += z

                    spiralTm = object.local_transformer.transform_pose(tmp_pose, "map")
                    spiral_poses.append(spiralTm)
                    BulletWorld.current_bullet_world.add_vis_axis(spiralTm)

                return spiral_poses

            # this is a very good one but takes ages
            # spiral_poses = generate_spiral(object_pose, 0.0004, 0.0008, math.radians(10), 100)
            spiral_poses = generate_spiral(object_pose, 0.001, 0.0035, math.radians(30), 10)

            BulletWorld.current_bullet_world.remove_vis_axis()
            for spiral_pose in spiral_poses:
                oriR = axis_angle_to_quaternion([1, 0, 0], 180)
                ori = multiply_quaternions(
                    [spiral_pose.orientation.x, spiral_pose.orientation.y, spiral_pose.orientation.z,
                     spiral_pose.orientation.w], oriR)
                adjusted_slice_pose = spiral_pose.copy()
                # # Set the orientation of the object pose by grasp in MAP
                adjusted_slice_pose.orientation.x = ori[0]
                adjusted_slice_pose.orientation.y = ori[1]
                adjusted_slice_pose.orientation.z = ori[2]
                adjusted_slice_pose.orientation.w = ori[3]

                # Adjust the position of the object pose by grasp in MAP
                lift_pose = adjusted_slice_pose.copy()
                lift_pose.pose.position.z += (obj_height + 0.08)
                # Perform the motion for lifting the tool
                # BulletWorld.current_bullet_world.add_vis_axis(lift_pose)
                MoveTCPMotion(lift_pose, self.arm).resolve().perform()

        # def to_sql(self) -> ORMMixingAction:
        #     """
        #     Convert the action to a corresponding SQL representation for storage.
        #     """
        #     return ORMMixingAction(self.arm, self.grasp)

        def insert(self, session: sqlalchemy.orm.session.Session, **kwargs):
            """
            Insert the mixing action into the database session.
            """
            action = super().insert(session)
            # Additional logic for inserting mixing action data goes here
            session.add(action)
            session.commit()

            return action

    def __init__(self, object_designator_description: ObjectDesignatorDescription,
                 object_tool_designator_description: ObjectDesignatorDescription, arms: List[str], grasps: List[str],
                 resolver=None):
        """
        Initialize the MixingAction with object and tool designators, arms, and grasps.

        :param object_designator_description: Object designator for the object to be mixed.
        :param object_tool_designator_description: Object designator for the mixing tool.
        :param arms: List of possible arms that could be used.
        :param grasps: List of possible grasps for the mixing action.
        :param resolver: An optional resolver for dynamic parameter selection.
        """
        super(MixingAction, self).__init__(resolver)
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.object_tool_designator_description: ObjectDesignatorDescription = object_tool_designator_description
        self.arms: List[str] = arms
        self.grasps: List[str] = grasps

    def ground(self) -> Action:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return self.Action(self.object_designator_description.ground(),
                           self.object_tool_designator_description.ground(), self.arms[0], self.grasps[0])
