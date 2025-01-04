import functools
import random

import numpy as np
import tf
from typing_extensions import Tuple, List, Union, Dict, Iterable, Callable

from .costmaps import Costmap
from .datastructures.enums import Grasp, ObjectType, Arms
from .datastructures.pose import Pose, Transform
from .datastructures.world import World
from .designator import ObjectDesignatorDescription
from .designators.object_designator import ObjectPart
from .external_interfaces.ik import request_ik
from .helper import adjust_grasp_for_object_rotation, calculate_grasp_offset, \
    calculate_rim_grasp, translate_relative_to_object, round_pose
from .local_transformer import LocalTransformer
from .plan_failures import IKError
from .robot_description import RobotDescription
from .ros.viz_marker_publisher import AxisMarkerPublisher
from .world_concepts.world_object import Object
from .world_reasoning import contact


class OrientationGenerator:

    @staticmethod
    def generate_origin_orientation(position: List[float], origin: Pose) -> List[float]:
        """
        Generates an orientation such that the robot faces the origin of the costmap.

        :param position: The position in the costmap, already converted to the world coordinate frame.
        :param origin: The origin of the costmap, the point which the robot should face.
        :return: A quaternion of the calculated orientation.
        """
        if RobotDescription.current_robot_description.name == "iai_donbot":
            angle = np.arctan2(position[1] - origin.position.y, position[0] - origin.position.x) - np.pi / 2
        else:
            angle = np.arctan2(position[1] - origin.position.y, position[0] - origin.position.x) + np.pi
        quaternion = list(tf.transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))
        return quaternion

    @staticmethod
    def generate_random_orientation(*_, rng: random.Random = random.Random(42)) -> List[float]:
        """
        Generates a random orientation rotated around the z-axis (yaw).
        A random angle is sampled using a provided RNG instance to ensure reproducibility.

        Args:
            *_: Ignored parameters to maintain compatibility with other orientation generators.
            rng (random.Random): Random number generator instance for reproducible sampling.

        Returns:
            List[float]: A quaternion representing the random orientation.
        """
        random_yaw = rng.uniform(0, 2 * np.pi)
        quaternion = list(tf.transformations.quaternion_from_euler(0, 0, random_yaw, axes="sxyz"))
        return quaternion


class PoseGenerator:
    """
    Crates pose candidates from a given costmap. The generator
    selects the highest values, amount is given by number_of_sample, and returns the corresponding positions.
    Orientations are calculated such that the Robot faces the center of the costmap.
    """

    current_orientation_generator = None
    """
    If no orientation generator is given, this generator is used to generate the orientation of the robot.
    """
    override_orientation_generator = None
    """
    Override the orientation generator with a custom generator, which will be used regardless of the current_orientation_generator.
    """

    def __init__(self, costmap: Costmap, number_of_samples=100,
                 orientation_generator: Callable = OrientationGenerator.generate_origin_orientation,
                 seed=42):
        """
        Initializes a PoseGenerator for sampling poses from a given costmap.

        This class generates sampled poses within the specified costmap, using a defined number of samples
        and an optional orientation generator. If no orientation generator is provided, a default generator
        is used.

        Args:
            costmap (Costmap): The costmap from which poses should be sampled.
            number_of_samples (int, optional): Maximum number of samples to be returned from the costmap. Defaults to 100.
            orientation_generator (callable, optional): Function that generates an orientation given a position and the origin of the costmap.

        Returns:
            None
        """

        self.costmap = costmap
        self.number_of_samples = number_of_samples
        self.orientation_generator = orientation_generator if orientation_generator else PoseGenerator.current_orientation_generator
        self.orientation_generator = orientation_generator
        self.seed = seed

    def __iter__(self) -> Iterable:
        """
        Generates pose candidates from a costmap, selecting the highest 100 values and
        returning the corresponding positions. The orientations are calculated such that
        the robot faces the center of the costmap.

        Yields:
            Pose: A Pose object containing position and orientation.
        """

        np.random.seed(self.seed)

        if self.number_of_samples == -1:
            self.number_of_samples = self.costmap.map.flatten().shape[0]
        number_of_samples = min(self.number_of_samples, self.costmap.map.size)

        height, width = self.costmap.map.shape
        odd_height = height % 2 == 1
        odd_width = width % 2 == 1
        center = np.array([height // 2 + odd_height, width // 2 + odd_width])

        flat_values = self.costmap.map.flatten()

        non_zero_indices = np.nonzero(flat_values)[0]
        if len(non_zero_indices) == 0:
            return
        non_zero_weights = flat_values[non_zero_indices]
        number_of_samples = min(number_of_samples, len(non_zero_indices))

        if non_zero_weights.sum() > 0:
            sampled_indices = np.random.choice(non_zero_indices, size=number_of_samples, replace=False,
                                               p=non_zero_weights / non_zero_weights.sum())
        else:
            sampled_indices = []

        indices = np.dstack(np.unravel_index(sampled_indices, self.costmap.map.shape)).reshape(number_of_samples, 2)

        sampled_weights = flat_values[sampled_indices]
        sorted_indices = indices[np.argsort(-sampled_weights)]

        for ind in sorted_indices:
            if self.costmap.map[ind[0]][ind[1]] == 0:
                continue
            # The position is calculated by creating a vector from the 2D position in the costmap (given by x and y)
            # and the center of the costmap (since this is the origin). This vector is then turned into a transformation
            # and muiltiplied with the transformation of the origin.
            vector_to_origin = (center - ind) * self.costmap.resolution
            point_to_origin = Transform([*vector_to_origin, 0], frame="point", child_frame="origin")
            origin_to_map = self.costmap.origin.to_transform("origin").invert()
            point_to_map = point_to_origin * origin_to_map
            map_to_point = point_to_map.invert()

            orientation = self.orientation_generator(map_to_point.translation_as_list(), self.costmap.origin)
            yield Pose(map_to_point.translation_as_list(), orientation)

    @staticmethod
    def height_generator() -> float:
        pass


class MultiCostmapPoseGenerator:
    def __init__(self, costmaps: List[Costmap], seed: int = None):
        """
        A pose generator that samples poses from multiple costmaps, interleaving poses between
        costmaps based on their proportional weights.

        Args:
            costmaps (List[Costmap]): List of costmaps from which poses should be sampled.
            seed (int, optional): Seed for the random number generator. Defaults to None.
        """
        self.costmaps = costmaps
        self.seed = seed
        self.weights = self.calculate_weights()
        self.rng = random.Random(self.seed)
        self.orientation_generator = functools.partial(
            OrientationGenerator.generate_random_orientation, rng=self.rng
        )
        self.generators = [iter(PoseGenerator(costmap, number_of_samples=-1, seed=self.seed,
                                              orientation_generator=self.orientation_generator)) for costmap in
                           costmaps]

    def calculate_weights(self):
        """
        Calculate sampling weights for each costmap based purely on its area (height * width).
        """
        costmap_areas = [cm.map.shape[0] * cm.map.shape[1] for cm in self.costmaps]
        total_area = sum(costmap_areas)
        return [area / total_area for area in costmap_areas]

    def __iter__(self):
        """
        Iteratively samples poses from the combined costmaps, interleaving poses between costmaps.
        """
        active_generators = self.generators.copy()  # Track active generators

        while active_generators:
            chosen_index = self.rng.choices(range(len(active_generators)), weights=self.weights, k=1)[0]
            chosen_generator = active_generators[chosen_index]

            try:
                yield next(chosen_generator), active_generators[chosen_index].gi_frame.f_locals[
                    'self'].costmap.link.name
            except StopIteration:
                active_generators.pop(chosen_index)
                self.weights.pop(chosen_index)


def visibility_validator(pose: Pose,
                         robot: Object,
                         object_or_pose: Union[Object, Pose],
                         world: World) -> bool:
    """
    This method validates if the robot can see the target position from a given
    pose candidate. The target position can either be a position, in world coordinate
    system, or an object in the World. The validation is done by shooting a
    ray from the camera to the target position and checking that it does not collide
    with anything else.

    :param pose: The pose candidate that should be validated
    :param robot: The robot object for which this should be validated
    :param object_or_pose: The target position or object for which the pose candidate should be validated.
    :param world: The World instance in which this should be validated.
    :return: True if the target is visible for the robot, None in any other case.
    """
    robot_pose = robot.get_pose()
    if isinstance(object_or_pose, Object):
        camera_pose = robot.get_link_pose(RobotDescription.current_robot_description.get_camera_frame())
        robot.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]))
        ray = world.ray_test(camera_pose.position_as_list(), object_or_pose.get_position_as_list())
        res = ray == object_or_pose.id
    else:
        camera_pose = robot.get_link_pose(RobotDescription.current_robot_description.get_camera_frame())
        robot.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]))
        # TODO: Check if this is correct
        ray = world.ray_test(camera_pose.position_as_list(), object_or_pose)
        res = ray == -1
    robot.set_pose(robot_pose)
    return res


def _in_contact(robot: Object, obj: Object, allowed_collision: Dict[Object, List[str]],
                allowed_robot_links: List[str], safety_distance: float = 0.0) -> bool:
    """
    Checks if the specified robot is in contact with or dangerously close to a given object,
    while accounting for allowed collisions on specific links.

    Args:
        robot (Object): The robot to check for contact.
        obj (Object): The object to check for contact with the robot.
        allowed_collision (Dict[Object, List[str]]): Allowed collisions with link names.
        allowed_robot_links (List[str]): Robot link names allowed to contact the object.
        safety_distance (float): Minimum safe distance between the robot and the object.

    Returns:
        bool: True if the robot is in contact or too close, False otherwise.
    """
    in_contact, contact_links = contact(robot, obj, return_links=True)
    if in_contact:
        allowed_links = allowed_collision.get(obj.id, [])
        for link in contact_links:
            if link[0].name not in allowed_robot_links and link[1].name not in allowed_links:
                return True

    min_distance = World.current_world.calculate_min_distance(robot, obj, allowed_robot_links, safety_distance)
    if abs(min_distance) < safety_distance:
        return True

    return False


def reachability_validator(robot: Object,
                           target: Union[Object, Pose],
                           arms: List[Arms],
                           object_in_hand: ObjectDesignatorDescription.Object = None,
                           used_grasp_config: List[Union[Grasp, bool]] = None,
                           translation_value: float = 0.1,
                           retract_first=None,
                           with_lifting=False) -> Tuple[bool, List, List]:
    """
    Validates if a target position is reachable for a given pose candidate.

    This method uses an IK solver to determine if a valid solution exists for the robot
    standing at the specified pose. The collisions allowed are the links of the currently used end-effector.
    If a solution is found, the validator returns `True`;
    otherwise, it returns `False`.

    Args:
        robot (Object): The robot object in the world for which reachability is being validated.
        target (Union[Object, Pose]): The target position or object that should be reachable.
        arms (List[Arms]): The arms for which reachability should be validated.
        object_in_hand (ObjectDesignatorDescription.Object, optional): The object that the robot is holding, if there is one.
        used_grasp_config (List[Union[Grasp, bool]], optional): The grasp configuration used for the validation.
        translation_value (float, optional): The distance by which the target position should be translated.
        retract_first (bool, optional): Whether the retract pose should be validated before or after validating if the final target is reachable.
        with_lifting (bool, optional): Whether the robot should lift the object after validating reachability.

    Returns:
        Tuple[bool, List, List]: A tuple where the first element is `True` if the target is reachable
        and `False` otherwise. The second element is a list of details about the solution or issues
        encountered during validation. The third element is a list of Dicts of joint states for the robot, used if the solution
        of this validation is needed later calculations.
    """
    if isinstance(target, ObjectPart.Object):
        prospection_world = World.current_world.get_prospection_object_for_object(target.world_object)
        goal_pose = prospection_world.get_link_pose(target.name)
    elif isinstance(target, ObjectDesignatorDescription.Object):
        goal_pose = target.world_object.get_pose()
    else:
        goal_pose = target
        target = None

    if not arms:
        arms = [Arms.LEFT, Arms.RIGHT]

    manipulator_descs = [
        RobotDescription.current_robot_description.get_arm_chain(arm)
        for arm in arms
        if RobotDescription.current_robot_description.get_arm_chain(arm) is not None
    ]

    side_grasp, top_grasp, horizontal = used_grasp_config.side_face, used_grasp_config.top_face, used_grasp_config.horizontal
    res = False
    valid_arms = []
    validated_joint_states = []

    for description in manipulator_descs:

        joints = description.joints
        tool_frame = description.end_effector.tool_frame
        target_pose = goal_pose.copy()

        if object_in_hand:
            local_transformer = LocalTransformer()
            object_pose = World.current_world.get_prospection_object_for_object(object_in_hand.world_object).get_pose()
            tcp_to_object = local_transformer.transform_pose(object_pose,
                                                             robot.get_link_tf_frame(
                                                                 RobotDescription.current_robot_description.get_arm_chain(
                                                                     description.arm_type).get_tool_frame()))

            target_pose = target_pose.to_transform("target").inverse_times(
                tcp_to_object.to_transform("object")).to_pose()
        else:
            grasp_orientation = RobotDescription.current_robot_description.get_arm_chain(
                description.arm_type).end_effector.get_grasp(side_grasp, top_grasp, horizontal)
            palm_axis = description.end_effector.get_palm_axis()
            if hasattr(target, "obj_type") and target.obj_type == ObjectType.BOWL:
                rim_offset = calculate_rim_grasp(target.world_object.get_object_dimensions(), side_grasp)
                rim_direction = RobotDescription.current_robot_description.get_arm_chain(
                    description.arm_type).end_effector.get_grasp(side_grasp, None, False)
                rim_adjustment = adjust_grasp_for_object_rotation(target_pose, rim_direction)
                rim_pose = translate_relative_to_object(rim_adjustment, palm_axis, rim_offset)
                target_pose.position = rim_pose.position
            target_pose = adjust_grasp_for_object_rotation(target_pose, grasp_orientation)

            if hasattr(target, "world_object"):
                grasp_offset = calculate_grasp_offset(target.world_object.get_object_dimensions(), description.arm_type,
                                                      top_grasp if top_grasp else side_grasp)
                target_pose = translate_relative_to_object(target_pose, palm_axis, grasp_offset)
            retract_first = True if retract_first is None else retract_first

        palm_axis = description.end_effector.get_palm_axis()
        target_pose = round_pose(target_pose)
        retract_target_pose = translate_relative_to_object(target_pose, palm_axis, translation_value)
        retract_target_pose = LocalTransformer().transform_pose(retract_target_pose, "map")
        retract_target_pose = round_pose(retract_target_pose)

        joint_state_before_ik = robot.get_positions_of_all_joints()

        # This currently causes the robot to sometimes collide with the apartment. Allow collision specifically with hand_links and object instead
        hand_links = [link for link in description.end_effector.links]
        allowed_collision = {robot: hand_links}

        try:
            marker = AxisMarkerPublisher()
            # test the possible solution and apply it to the robot
            pose, joint_states = request_ik(retract_target_pose if retract_first else target_pose, robot, joints,
                                            tool_frame)
            robot.set_pose(pose)
            robot.set_joint_positions(joint_states)
            # _apply_ik(robot, resp, joints)

            in_contact = collision_check(robot, allowed_collision)

            if not in_contact:

                pose2, joint_states2 = request_ik(target_pose if retract_first else retract_target_pose, robot,
                                                  joints,
                                                  tool_frame)
                robot.set_pose(pose2)
                robot.set_joint_positions(joint_states2)
                # _apply_ik(robot, resp, joints)
                in_contact = collision_check(robot, allowed_collision)

                if not in_contact and with_lifting:
                    target_pose.position.z += 0.03
                    pose3, joint_states3 = request_ik(target_pose, robot, joints, tool_frame)
                    robot.set_pose(pose3)
                    robot.set_joint_positions(joint_states3)
                    in_contact = collision_check(robot, allowed_collision)

                if not in_contact:
                    valid_arms.append(description.arm_type)
                    validated_joint_states.append(joint_states)

        except IKError:
            pass
        finally:
            robot.set_joint_positions(joint_state_before_ik)
    if valid_arms:
        res = True
    return res, valid_arms, validated_joint_states


def collision_check(robot: Object, allowed_collision: Dict[Object, List]):
    """
    This method checks if a given robot collides with any object within the world
    which it is not allowed to collide with.
    This is done checking iterating over every object within the world and checking
    if the robot collides with it. Careful the floor will be ignored.
    If there is a collision with an object that was not within the allowed collision
    list the function returns True else it will return False

    :param robot: The robot object in the (Bullet)World where it should be checked if it collides with something
    :param allowed_collision: dict of objects with which the robot is allowed to collide each object correlates to a list of links of which this object consists
    :return: True if the target is reachable for the robot and False in any other case.
    """
    in_contact = False
    allowed_robot_links = []
    if robot in allowed_collision.keys():
        allowed_robot_links = allowed_collision[robot]

    for obj in World.current_world.objects:
        if obj.name == "floor":
            continue
        in_contact = _in_contact(robot, obj, allowed_collision, allowed_robot_links)
        if in_contact:
            break

    return in_contact
