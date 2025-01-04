import dataclasses
import math
import time
from typing import Tuple, Any

from tqdm import tqdm
from typing_extensions import List, Union, Iterable, Optional, Callable, Tuple

from .object_designator import ObjectDesignatorDescription, ObjectPart
from ..costmaps import OccupancyCostmap, VisibilityCostmap, SemanticCostmap, GaussianCostmap, \
    DirectionalCostmap
from ..datastructures.enums import JointType, Arms, Grasp, AccessingMode
from ..datastructures.pose import Pose
from ..datastructures.world import World, UseProspectionWorld
from ..designator import DesignatorError, LocationDesignatorDescription
from ..helper import calculate_grasp_descriptions
from ..local_transformer import LocalTransformer
from ..plan_failures import ReachabilityFailure
from ..pose_generator_and_validator import PoseGenerator, visibility_validator, reachability_validator, \
    MultiCostmapPoseGenerator, OrientationGenerator, collision_check
from ..robot_description import RobotDescription, GraspDescription
from ..ros.viz_marker_publisher import AxisMarkerPublisher
from ..world_concepts.world_object import Object
from ..world_reasoning import link_pose_for_joint_config


class Location(LocationDesignatorDescription):
    """
    Default location designator which only wraps a pose.
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        pass

    def __init__(self, pose: Pose, resolver=None):
        """
        Basic location designator that represents a single pose.

        :param pose: The pose that should be represented by this location designator
        :param resolver: An alternative specialized_designators that returns a resolved location
        """
        super().__init__(resolver)
        self.pose: Pose = pose

    def ground(self) -> Location:
        """
        Default specialized_designators which returns a resolved designator which contains the pose given in init.

        :return: A resolved designator
        """
        return self.Location(self.pose)


# TODO Maybe delete this
class ObjectRelativeLocation(LocationDesignatorDescription):
    """
    Location relative to an object
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        relative_pose: Pose
        """
        Pose relative to the object
        """
        reference_object: ObjectDesignatorDescription.Object
        """
        Object to which the pose is relative
        """

    def __init__(self, relative_pose: Pose = None, reference_object: ObjectDesignatorDescription = None,
                 resolver=None):
        """
        Location designator representing a location relative to a given object.

        :param relative_pose: Pose that should be relative, in world coordinate frame
        :param reference_object: Object to which the pose should be relative
        :param resolver: An alternative specialized_designators that returns a resolved location for the input parameter
        """
        super().__init__(resolver)
        self.relative_pose: Pose = relative_pose
        self.reference_object: ObjectDesignatorDescription = reference_object

    def ground(self) -> Location:
        """
        Default specialized_designators which returns a resolved location for description input. Resolved location is the first result
        of the iteration of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    def __iter__(self) -> Iterable[Location]:
        """
        Iterates over all possible solutions for a resolved location that is relative to the given object.

        :yield: An instance of ObjectRelativeLocation.Location with the relative pose
        """
        if self.relative_pose is None or self.reference_object is None:
            raise DesignatorError(
                "Could not ground ObjectRelativeLocation: (Relative) pose and reference object must be given")
        # Fetch the object pose and yield the grounded description
        obj_grounded = self.reference_object.resolve()

        lt = LocalTransformer()
        pose = lt.transform_to_object_frame(self.relative_pose, obj_grounded)

        yield self.Location(self.relative_pose, pose, self.reference_object)


class CostmapLocation(LocationDesignatorDescription):
    """
    Uses costmaps to create locations based on complex constraints, such as reachability and visibility.
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        reachable_arms: List[Arms]
        """
        List of arms with which the pose can be reached; only relevant when the `reachable_for` parameter is specified.
        """

        used_grasp_config: GraspDescription
        """
        The grasp configuration used to reach the pose
        """

    def __init__(self, target: Union[Pose, ObjectDesignatorDescription.Object],
                 reachable_for: Optional[ObjectDesignatorDescription.Object] = None,
                 visible_for: Optional[ObjectDesignatorDescription.Object] = None,
                 reachable_arms: Optional[List[Arms]] = None, resolver: Optional[Callable] = None,
                 used_grasp_config: Optional[GraspDescription] = None,
                 object_in_hand: Optional[ObjectDesignatorDescription.Object] = None):
        """
        Initializes a location designator that uses costmaps to calculate locations based on complex constraints,
        such as reachability and visibility.

        Args:
            target (Union[Pose, ObjectDesignatorDescription.Object]): The location for which visibility or reachability
                should be calculated.
            reachable_for (Optional[ObjectDesignatorDescription.Object]): The object (usually a robot) for which
                reachability is calculated.
            visible_for (Optional[ObjectDesignatorDescription.Object]): The object (usually a robot) for which
                visibility is calculated.
            reachable_arms (Optional[List[Arms]]): A list of arms that can be used to reach the target.
            resolver (Optional[Callable]): An alternative function that resolves a location for the input of this description.
            used_grasp_config (Optional[GraspConfig]): The grasp configuration used to reach the pose.
            object_in_hand (Optional[ObjectDesignatorDescription.Object]): The object currently held by the robot, if any.
        """
        super().__init__(resolver)
        self.target: Union[Pose, ObjectDesignatorDescription.Object] = target
        self.reachable_for: ObjectDesignatorDescription.Object = reachable_for
        self.visible_for: ObjectDesignatorDescription.Object = visible_for
        self.reachable_arms: Optional[List[Arms]] = reachable_arms
        self.used_grasp_config: Optional[List[Union[Grasp, bool]]] = used_grasp_config
        self.object_in_hand: Optional[ObjectDesignatorDescription.Object] = object_in_hand

    def ground(self) -> Location:
        """
        Default specialized_designators which returns the first result from the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    def __iter__(self):
        """
        Generates valid positions based on constraints derived from a costmap, yielding each
        that satisfies the given requirements.

        This method merges multiple costmaps, each with a unique purpose (e.g., occupancy,
        visibility, reachability) to form a comprehensive costmap. From this combined map,
        candidate poses are generated and validated against constraints defined in the designator.
        Each pose that meets the specified constraints is yielded as a valid location.

        Yields:
            CostmapLocation.Location: A location instance with a valid position that satisfies
            the constraints, including reachability and visibility as applicable.
        """
        min_height = RobotDescription.current_robot_description.get_default_camera().minimal_height
        max_height = RobotDescription.current_robot_description.get_default_camera().maximal_height
        if self.reachable_for and not self.reachable_arms:
            self.reachable_arms = [Arms.RIGHT, Arms.LEFT]
        # This ensures that the costmaps always get a position as their origin.
        if isinstance(self.target, ObjectDesignatorDescription.Object):
            target_pose = self.target.world_object.get_pose()
        else:
            target_pose = self.target.copy()

        top_grasp = None
        if isinstance(self.target, ObjectDesignatorDescription.Object) and not self.used_grasp_config:
            top_grasp = calculate_grasp_descriptions(self.target)[0].top_face

        if self.used_grasp_config:
            top_grasp = self.used_grasp_config.top_face

        ground_pose = Pose(target_pose.position_as_list())
        ground_pose.position.z = 0
        distance_to_obstacle = RobotDescription.current_robot_description.get_costmap_offset()

        max_reach = RobotDescription.current_robot_description.get_max_reach()
        map_size: int = int(max_reach * 100 * 3)
        map_resolution = 0.15

        occupancy = OccupancyCostmap(distance_to_obstacle, False, map_size * 2, map_resolution, ground_pose)
        final_map = occupancy
        if self.reachable_for:
            if top_grasp:
                distance = (distance_to_obstacle + max_reach) / 1.5
            else:
                distance = (distance_to_obstacle + max_reach) / 1.25
            gaussian = GaussianCostmap(map_size, 1.5, map_resolution, ground_pose, True, distance)
            final_map += gaussian
        if self.visible_for:
            visible = VisibilityCostmap(min_height, max_height, map_size, map_resolution,
                                        Pose(target_pose.position_as_list()))
            final_map += visible

        if self.used_grasp_config:
            directional = DirectionalCostmap(map_size * 3, self.used_grasp_config.side_face, map_resolution,
                                             target_pose,
                                             self.object_in_hand is not None)
            final_map *= directional

        if final_map.world.allow_publish_debug_poses:
            final_map.publish(weighted=True)

        if self.visible_for or self.reachable_for:
            robot_object = self.visible_for.world_object if self.visible_for else self.reachable_for.world_object
            test_robot = World.current_world.get_prospection_object_for_object(robot_object)

        with UseProspectionWorld():
            for maybe_pose in PoseGenerator(final_map, number_of_samples=600):
                if final_map.world.allow_publish_debug_poses:
                    gripper_pose = World.robot.get_link_pose(
                        RobotDescription.current_robot_description.get_arm_chain(
                            self.reachable_arms[0]).get_tool_frame())
                    marker = AxisMarkerPublisher()
                    marker.publish([maybe_pose, target_pose, gripper_pose], length=0.3)
                res = True
                arms = None
                grasp_config = self.used_grasp_config

                if self.visible_for or self.reachable_for:
                    maybe_pose.position.z = 0
                    test_robot.set_pose(maybe_pose)

                if self.visible_for:
                    res = res and visibility_validator(maybe_pose, test_robot, target_pose,
                                                       World.current_world)
                if self.reachable_for:

                    if self.used_grasp_config:
                        grasp_configurations = [self.used_grasp_config]
                    else:
                        grasp_configurations = calculate_grasp_descriptions(self.target, test_robot)

                    for grasp_configuration in grasp_configurations:
                        grasp_config = grasp_configuration
                        valid, arms, _ = reachability_validator(robot=test_robot, target=self.target,
                                                                arms=self.reachable_arms,
                                                                object_in_hand=self.object_in_hand,
                                                                used_grasp_config=grasp_configuration,
                                                                with_lifting=not self.object_in_hand)
                        if arms:
                            res = res and valid
                            if res:
                                break
                    if arms:
                        res = res and valid
                    else:
                        res = False

                if res:
                    yield self.Location(maybe_pose, arms, grasp_config)


class AccessingLocation(LocationDesignatorDescription):
    """
    Designates a location for accessing and opening drawers.

    This designator provides the robot with a pose for interacting with drawer handles.
    Calculating this position before the drawer is opened is recommended to avoid
    potential issues with pose estimation.
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        arms: List[Arms]
        """
        List of arms that can be used for accessing from this pose.
        """

        used_grasp_config: GraspDescription
        """
        The grasp configuration used to reach the pose
        """

    def __init__(self, handle_desig: ObjectPart.Object, robot_desig: ObjectDesignatorDescription.Object, resolver=None,
                 accessing_mode: AccessingMode = AccessingMode.OPENING, arms: List[Arms] = None):
        """
        Initializes a location designator for accessing a drawer handle.

        Args:
            handle_desig (ObjectPart.Object): The designator for the drawer handle to be accessed.
            robot_desig (ObjectDesignatorDescription.Object): The designator for the robot that will perform the action.
            resolver (Optional[Callable]): An optional custom resolver function for location creation.
            accessing_mode (AccessingMode): The mode in which the drawer is accessed (opening or closing).
            arms (List[Arms]): The arms that can be used for accessing the drawer.

        """
        super().__init__(resolver)
        self.handle: ObjectPart.Object = handle_desig
        self.robot: ObjectDesignatorDescription.Object = robot_desig.world_object
        self.accessing_mode = accessing_mode
        self.arms = arms if arms else [Arms.RIGHT, Arms.LEFT]

    def ground(self) -> Location:
        """
        Default specialized_designators for this location designator, just returns the first element from the iteration

        :return: A location designator for a pose from which the drawer can be opened
        """
        try:
            return next(iter(self))
        except StopIteration:
            raise ReachabilityFailure(
                f"No reachable location found for the target location: {self.handle}"
            )

    def __iter__(self) -> Tuple[Location, Location]:
        """
        Generates poses for the robot to access and open a drawer specified by the handle
        designator. Poses are validated to ensure the robot can grasp the handle while the drawer
        is closed and that the handle can still be grasped when the drawer is open.

        The process involves generating candidate poses using a merged costmap that incorporates
        occupancy, Gaussian, and directional costmaps. For each candidate pose, reachability
        validation checks are performed for both the initial and goal (fully opened) positions
        of the drawer. Only poses that satisfy all constraints are yielded.

        Yields:
            Tuple[AccessingLocation.Location, AccessingLocation.Location]: A tuple containing
            two location designators, one for the initial and one for the goal pose, with
            the arms that can be used for each.
        """
        ground_pose = Pose(self.handle.part_pose.position_as_list())
        ground_pose.position.z = 0
        test_robot = World.current_world.get_prospection_object_for_object(self.robot)

        if self.handle.name in ["handle_cab1_top_door", "handle_cab2_door", "handle_cab3_door_top",
                                "handle_cab3_door_bottom", "handle_cab4_door_bottom", "handle_cab7"]:
            container_joint = self.handle.world_object.find_joint_above_link(self.handle.name, JointType.REVOLUTE)
        else:
            container_joint = self.handle.world_object.find_joint_above_link(self.handle.name, JointType.PRISMATIC)

        prospection_world = World.current_world.get_prospection_object_for_object(self.handle.world_object)
        prev_state = prospection_world.get_joint_position(container_joint)

        if self.handle.name == "handle_cab7":
            joint_safety_offset = 0.60
        else:
            joint_safety_offset = 0.05

        init_joint_state = self.handle.world_object.get_joint_position(container_joint)

        if self.accessing_mode == AccessingMode.OPENING:
            goal_joint_state = self.handle.world_object.get_joint_limits(container_joint)[1] - joint_safety_offset
        else:
            goal_joint_state = self.handle.world_object.get_joint_limits(container_joint)[0]

        half_joint_state = init_joint_state + goal_joint_state / 2

        init_pose = link_pose_for_joint_config(self.handle.world_object, {
            container_joint: init_joint_state}, self.handle.name)

        half_pose = link_pose_for_joint_config(self.handle.world_object, {
            container_joint: half_joint_state}, self.handle.name)

        goal_pose = link_pose_for_joint_config(self.handle.world_object, {
            container_joint: goal_joint_state}, self.handle.name)

        prospection_world.set_joint_position(container_joint, prev_state)
        grasp_config = GraspDescription(side_face=Grasp.FRONT, top_face=None, horizontal=False)

        distance_to_obstacle = RobotDescription.current_robot_description.get_costmap_offset()
        max_reach = RobotDescription.current_robot_description.get_max_reach()
        map_size: int = int(max_reach * 100 * 2.5)
        map_resolution = 0.15

        # TODO: find better strategy for distance_to_obstacle
        occupancy = OccupancyCostmap(distance_to_obstacle, False, map_size * 2, map_resolution, ground_pose)
        distance = (distance_to_obstacle + max_reach) / 1.25
        gaussian = GaussianCostmap(map_size, 1.5, map_resolution, ground_pose, True, distance)
        final_map = occupancy + gaussian

        directional = DirectionalCostmap(map_size * 3, Grasp.FRONT, map_resolution, init_pose)
        final_map *= directional

        if final_map.world.allow_publish_debug_poses:
            final_map.publish(weighted=True)

        prev_robot_state = test_robot.get_positions_of_all_joints()
        with (UseProspectionWorld()):
            for init_maybe_pose in PoseGenerator(final_map, number_of_samples=600,
                                                 orientation_generator=lambda p,
                                                                              o: OrientationGenerator.generate_origin_orientation(
                                                     p,
                                                     init_pose)):
                if final_map.world.allow_publish_debug_poses:
                    marker = AxisMarkerPublisher()
                    marker.publish([init_pose, half_pose, goal_pose, init_maybe_pose], length=0.5)

                test_robot.set_pose(init_maybe_pose)

                in_contact = collision_check(test_robot, {})
                if in_contact:
                    continue
                prospection_world.set_joint_position(container_joint, init_joint_state)
                valid_init, arms_init, init_joint_states = reachability_validator(robot=test_robot, target=self.handle,
                                                                                  arms=self.arms,
                                                                                  used_grasp_config=grasp_config,
                                                                                  translation_value=0.05)
                if not valid_init:
                    test_robot.set_joint_positions(prev_robot_state)
                    prospection_world.set_joint_position(container_joint, init_joint_state)
                    continue

                prospection_world.set_joint_position(container_joint, goal_joint_state)
                valid_goal, arms_goal = False, []
                for arm, joints_states in zip(arms_init, init_joint_states):
                    test_robot.set_joint_positions(joints_states)
                    _valid_goal, _arms_goal, _ = reachability_validator(robot=test_robot, target=self.handle,
                                                                        arms=[arm], used_grasp_config=grasp_config,
                                                                        translation_value=0.05, retract_first=False)
                    if _valid_goal:
                        valid_goal = _valid_goal
                        arms_goal.append(_arms_goal[0])
                    test_robot.set_joint_positions(prev_robot_state)
                goal_maybe_pose = init_maybe_pose.copy()

                if not valid_goal:
                    mapThandle = init_pose.to_transform("init_handle")
                    mapTmaybe = init_maybe_pose.to_transform("init_maybe")
                    handleTmaybe = mapThandle.invert() * mapTmaybe
                    goal_maybe_pose = (goal_pose.to_transform("goal_handle") * handleTmaybe).to_pose()

                    if math.isclose(goal_maybe_pose.position.z, 0, abs_tol=0.01):
                        if final_map.world.allow_publish_debug_poses:
                            marker = AxisMarkerPublisher()
                            marker.publish([goal_maybe_pose], length=0.5)

                        test_robot.set_pose(goal_maybe_pose)

                        valid_goal, arms_goal = False, []
                        for arm, joints_states in zip(arms_init, init_joint_states):
                            test_robot.set_joint_positions(joints_states)
                            hand_links = RobotDescription.current_robot_description.get_arm_chain(arm).end_effector.links
                            in_contact = collision_check(test_robot, {test_robot: hand_links})
                            if in_contact:
                                continue
                            _valid_goal, _arms_goal, _ = reachability_validator(robot=test_robot, target=self.handle,
                                                                                arms=[arm],
                                                                                used_grasp_config=grasp_config,
                                                                                translation_value=0.05,
                                                                                retract_first=False)
                            if _valid_goal:
                                valid_goal = _valid_goal
                                arms_goal.append(_arms_goal[0])
                            test_robot.set_joint_positions(prev_robot_state)

                if not valid_goal:
                    goal_ground_pose = goal_pose.copy()
                    goal_ground_pose.position.z = 0
                    gaussian2 = GaussianCostmap(int(map_size / 2), 1.5, map_resolution, test_robot.pose.copy(),
                                                True, 0)
                    occupancy2 = OccupancyCostmap(distance_to_obstacle, False, map_size * 2, map_resolution,
                                                  test_robot.pose.copy())
                    goal_final_map2 = occupancy2 + gaussian2

                    for goal_maybe_pose in tqdm(PoseGenerator(goal_final_map2, number_of_samples=600,
                                                              orientation_generator=lambda p,
                                                                                           o: OrientationGenerator.generate_origin_orientation(
                                                                  p,
                                                                  goal_pose))):
                        test_robot.set_pose(goal_maybe_pose)

                        valid_goal, arms_goal = False, []
                        for arm, joints_states in zip(arms_init, init_joint_states):
                            test_robot.set_joint_positions(joints_states)
                            _valid_goal, _arms_goal, _ = reachability_validator(robot=test_robot, target=self.handle,
                                                                                arms=[arm],
                                                                                used_grasp_config=grasp_config,
                                                                                translation_value=0.05,
                                                                                retract_first=False)
                            if _valid_goal:
                                valid_goal = _valid_goal
                                arms_goal.append(_arms_goal[0])
                            test_robot.set_joint_positions(prev_robot_state)
                        if valid_goal:
                            break

                test_robot.set_joint_positions(prev_robot_state)
                prospection_world.set_joint_position(container_joint, init_joint_state)

                if valid_goal:
                    break

            if valid_init and valid_goal:
                common_arms = list(set(arms_init) & set(arms_goal))
                if common_arms:
                    yield self.Location(init_maybe_pose, common_arms, grasp_config), \
                        self.Location(goal_maybe_pose, common_arms, grasp_config)


class SemanticCostmapLocation(LocationDesignatorDescription):
    """
    Locations over semantic entities, like a table surface
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        pass

    def __init__(self, urdf_link_name, part_of, for_object=None, seed=None, resolver=None, ):
        """
        Creates a distribution over a urdf link to sample poses which are on this link. Can be used, for example, to find
        poses that are on a table. Optionally an object can be given for which poses should be calculated, in that case
        the poses are calculated such that the bottom of the object is on the link.

        :param urdf_link_name: Name of the urdf link for which a distribution should be calculated
        :param part_of: Object of which the urdf link is a part
        :param for_object: Optional object that should be placed at the found location
        :param resolver: An alternative specialized_designators that creates a resolved location for the input parameter of this description

        """
        super().__init__(resolver)
        self.urdf_link_name: str = urdf_link_name
        self.part_of: ObjectDesignatorDescription.Object = part_of
        self.for_object: Optional[ObjectDesignatorDescription.Object] = for_object
        self.seed: Optional[int] = seed

    def ground(self) -> Location:
        """
        Default specialized_designators which returns the first element of the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    def __iter__(self):
        """
        Creates a costmap on top of a link of an Object and creates positions from it. If there is a specific Object for
        which the position should be found, a height offset will be calculated which ensures that the bottom of the Object
        is at the position in the Costmap and not the origin of the Object which is usually in the centre of the Object.

        :yield: An instance of SemanticCostmapLocation.Location with the found valid position of the Costmap.
        """
        sem_costmap = SemanticCostmap(self.part_of.world_object, self.urdf_link_name, resolution=0.05)
        # sem_costmap.publish(scale=1)

        height_offset = 0
        if self.for_object:
            min_p, max_p = self.for_object.world_object.get_axis_aligned_bounding_box().get_min_max_points()
            height_offset = (max_p.z - min_p.z) / 2
        for maybe_pose in PoseGenerator(sem_costmap, seed=self.seed):
            maybe_pose.position.z += height_offset
            yield self.Location(maybe_pose)


class MultiSurfaceCostmapLocation(LocationDesignatorDescription):
    """
    Locations over multiple semantic entities, like a table surface
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        pass

    def __init__(self, urdf_link_names: List[str], part_of: ObjectDesignatorDescription.Object,
                 for_object: Object = None,
                 seed=42, resolver=None):
        """
        Creates a distribution over multiple urdf links to sample poses which are on these links. Can be used, for example, to find
        poses that are on a table. Optionally an object can be given for which poses should be calculated, in that case
        the poses are calculated such that the bottom of the object is on the link.

        Args:
            urdf_link_names: Names of the urdf links for which a distribution should be calculated
            part_of: Object of which the urdf links are a part
            for_object: Optional object that should be placed at the found location
            seed: Seed for the random generator
            resolver: An alternative specialized_designators that creates a resolved location for the input parameter of this description
        """
        super().__init__(resolver)
        self.urdf_link_names: List[str] = urdf_link_names
        self.part_of: ObjectDesignatorDescription.Object = part_of
        self.for_object: Optional[ObjectDesignatorDescription.Object] = for_object
        self.seed: Optional[int] = seed

    def ground(self) -> Tuple[Location, Any]:
        """
        Default specialized_designators which returns the first element of the iterator of this instance.

        Returns:
            Tuple[MultiSurfaceCostmapLocation.Location, Any]: An instance of
        """
        return next(iter(self))

    def __iter__(self):
        """
        Creates a costmap on top of a link of an Object and creates positions from it. If there is a specific Object for
        which the position should be found, a height offset will be calculated which ensures that the bottom of the Object
        is at the position in the Costmap and not the origin of the Object which is usually in the centre of the Object.

        Yields:
            Tuple[MultiSurfaceCostmapLocation.Location, Any]: An instance of MultiSurfaceCostmapLocation.Location, as
            well as the surface sampled from.
        """
        surface_costmaps = []
        for urdf_link_name in self.urdf_link_names:
            sem_costmap = SemanticCostmap(self.part_of.world_object, urdf_link_name, resolution=0.05)
            surface_costmaps.append(sem_costmap)

        height_offset = 0
        if self.for_object:
            min_p, max_p = self.for_object.get_axis_aligned_bounding_box().get_min_max_points()
            height_offset = (max_p.z - min_p.z) / 2
        for sampled_pose, surface in MultiCostmapPoseGenerator(surface_costmaps, seed=self.seed):
            sampled_pose.position.z += height_offset
            yield self.Location(sampled_pose), surface
