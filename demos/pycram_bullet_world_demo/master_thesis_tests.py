import logging
import random
from collections import defaultdict

from matplotlib import pyplot as plt
from tqdm import tqdm
from typing_extensions import List

from pycram.plan_failures import IKError
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.ros.viz_marker_publisher import VizMarkerPublisher, AxisMarkerPublisher, CostmapPublisher
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType, WorldMode, TorsoState
from pycram.datastructures.pose import Pose, Transform
from pycram.process_module import simulated_robot, with_simulated_robot, ProcessModule
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color

import os
import math
import xml.etree.ElementTree as ET
from pycram.datastructures.enums import GripperType, WorldMode
from jinja2 import Template
import ipywidgets as widgets
from IPython.display import display
import numpy as np
from scipy.spatial.transform import Rotation as R

# Import pycram modules for simulation
from pycram.ros.viz_marker_publisher import VizMarkerPublisher, AxisMarkerPublisher
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.worlds.bullet_world import BulletWorld
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.datastructures.pose import Pose
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.datastructures.dataclasses import Color
from pycram.world_concepts.world_object import Object
from pycram.datastructures.enums import ObjectType

extension = ObjectDescription.get_file_extension()

world = BulletWorld(WorldMode.DIRECT)
viz = VizMarkerPublisher()
# viz2 = VizMarkerPublisher(as_prospection_world=True)
tfviz = TFBroadcaster()

world.allow_publish_debug_poses = False
ProcessModule.execution_delay = False
surface_tests = False
open_tests = True
robot_name = "Armar6"
robot = Object(robot_name, ObjectType.ROBOT, f"{robot_name}{extension}", pose=Pose([1, 2, 0]))

apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment-small{extension}")

milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([0, 0, 0]),
              color=Color(1, 0, 0, 1))

cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                pose=Pose([0, 0, 0]), color=Color(0, 1, 0, 1))

bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([0, 0, 0]),
              color=Color(1, 1, 0, 1))

spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([0, 0, 0]),
               color=Color(0, 0, 1, 1))

robot_desig = BelieveObject(names=[robot_name])
apartment_desig = BelieveObject(names=["apartment"])
test_seed = 0
test_objects = [milk, cereal, bowl, spoon]
object_rng = random.Random(test_seed)
test_arms = [Arms.LEFT, Arms.RIGHT]
arms_rng = random.Random(test_seed)

test_surfaces = ["island_countertop", "countertop", "table_area_main_temp_raytest_fix"]

test_handles = [
    "handle_cab1_top_door",
    "handle_cab1_drawer_mid",
    "handle_cab1_drawer_bottom",
    "handle_cab2_door",
    "handle_cab3_door_top",
    "handle_cab3_door_bottom",
    "handle_cab4_door_bottom",
    "handle_cab5_t",
    "handle_cab5_m",
    "handle_cab5_b",
    "handle_cab6_t",
    "handle_cab6_m",
    "handle_cab6_b",
    "handle_cab7",
    "handle_cab7_middle",
    "handle_cab8",
    "handle_cab9_t",
    "handle_cab9_m",
    "handle_cab9_b",
    "handle_cab10_t",
    "handle_cab10_m",
    "handle_cab10_b",
    "handle_cab11_t",
    "handle_cab11_m",
    "handle_cab11_b"
]


def mock_detect(obj_type):
    world_object = World.current_world.get_object_by_type(obj_type)[0]
    obj_desig = ObjectDesignatorDescription.Object(world_object.name, world_object.obj_type, world_object)
    return obj_desig


def pairwise(iterable):
    while True:
        location1, surface1 = next(iterable, (None, None))
        location2, surface2 = next(iterable, (None, None))
        if location2 is None:
            break
        yield (location1, surface1), (location2, surface2)


import csv
from collections import defaultdict
from dataclasses import dataclass, field
from tqdm import tqdm


@dataclass
class SuccessMetrics:
    attempts: int = 0
    successful_actions: int = 0


@dataclass
class Results:
    object_pickup_rate: defaultdict = field(default_factory=lambda: defaultdict(SuccessMetrics))
    object_completion_rate: defaultdict = field(default_factory=lambda: defaultdict(SuccessMetrics))
    surface_pickup_rate: defaultdict = field(default_factory=lambda: defaultdict(SuccessMetrics))
    surface_place_rate: defaultdict = field(default_factory=lambda: defaultdict(SuccessMetrics))
    overall_pickup_attempts: int = 0
    overall_pickup_success: int = 0
    overall_completion_attempts: int = 0
    overall_completion_success: int = 0

    overall_open_rate: defaultdict = field(default_factory=lambda: defaultdict(SuccessMetrics))
    overall_close_rate: defaultdict = field(default_factory=lambda: defaultdict(SuccessMetrics))
    overlall_open_close_rate: defaultdict = field(default_factory=lambda: defaultdict(SuccessMetrics))


def update_metrics(metrics, key):
    setattr(metrics, key, getattr(metrics, key) + 1)


def update_successful_pickup(results, test_object, start_location, start_surface, target_surface):
    """Update metrics for successful pickups."""
    if test_object.pose.position_as_list() != start_location.position_as_list():
        results.overall_pickup_success += 1
        update_metrics(results.object_pickup_rate[test_object.obj_type], "successful_actions")
        update_metrics(results.surface_pickup_rate[start_surface], "successful_actions")
        update_metrics(results.object_completion_rate[test_object.obj_type], "attempts")
        update_metrics(results.surface_place_rate[target_surface], "attempts")


def save_results_to_csv(results, robot_name, test_type):
    """
    Save the evaluation results to a CSV file.

    Args:
        results (Results): The results object containing the metrics.
        robot_name (str): The name of the robot.
        test_type (str): The type of test conducted (e.g., "surface_tests").
    """
    filename = f"{robot_name}_{test_type}_results.csv"

    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)

        # Write overall metrics if data exists
        if results.overall_pickup_attempts > 0 or results.overall_completion_attempts > 0:
            writer.writerow(["Metric", "Value"])
            if results.overall_pickup_attempts > 0:
                overall_pickup_rate = (results.overall_pickup_success / results.overall_pickup_attempts)
                writer.writerow(["Overall Pickup Rate", f"{overall_pickup_rate:.2%}, ({results.overall_pickup_success}/{results.overall_pickup_attempts} attempts)"])
            if results.overall_pickup_attempts > 0:
                overall_placing_rate = (results.overall_completion_success / results.overall_pickup_success)
                writer.writerow(["Overall Placing Rate", f"{overall_placing_rate:.2%}", f"({results.overall_completion_success}/{results.overall_pickup_success} attempts)"])
            if results.overall_completion_attempts > 0:
                overall_completion_rate = (results.overall_completion_success / results.overall_completion_attempts)
                writer.writerow(["Overall Completion Rate", f"{overall_completion_rate:.2%}", f"({results.overall_completion_success}/{results.overall_completion_attempts} attempts)"])

        # Write object success rates if data exists
        if results.object_pickup_rate:
            writer.writerow([])
            writer.writerow(["Object Type", "Pickup Rate", "Completion Rate"])
            for obj, data in results.object_pickup_rate.items():
                pickup_rate = (data.successful_actions / data.attempts) if data.attempts > 0 else 0
                completion_rate = (
                        results.object_completion_rate[obj].successful_actions / results.object_completion_rate[
                    obj].attempts) if results.object_completion_rate[obj].attempts > 0 else 0
                writer.writerow([obj, f"{pickup_rate:.2%}", f"{completion_rate:.2%}", f"({results.object_completion_rate[obj].successful_actions}/{results.object_completion_rate[obj].attempts} attempts)"])

        # Write surface success rates if data exists
        if results.surface_pickup_rate:
            writer.writerow([])
            writer.writerow(["Surface", "Pickup Rate", "Place Rate"])
            for loc, data in results.surface_pickup_rate.items():
                pickup_rate = (data.successful_actions / data.attempts) if data.attempts > 0 else 0
                place_rate = (results.surface_place_rate[loc].successful_actions / results.surface_place_rate[
                    loc].attempts) if results.surface_place_rate[loc].attempts > 0 else 0
                writer.writerow([loc, f"{pickup_rate:.2%}", f"{place_rate:.2%}", f"({results.surface_place_rate[loc].successful_actions}/{results.surface_place_rate[loc].attempts} attempts)"])

        if results.overall_open_rate or results.overall_close_rate or results.overlall_open_close_rate:
            writer.writerow([])
            writer.writerow(["Metric", "Open Rate", "Close Rate", "Completion Rate"])
            overall_open_rate = sum(data.successful_actions for data in results.overall_open_rate.values()) / sum(
                data.attempts for data in results.overall_open_rate.values()) if sum(
                data.attempts for data in results.overall_open_rate.values()) > 0 else 0
            overall_close_rate = sum(data.successful_actions for data in results.overall_close_rate.values()) / sum(
                data.attempts for data in results.overall_close_rate.values()) if sum(
                data.attempts for data in results.overall_close_rate.values()) > 0 else 0
            overall_completion_rate = sum(
                data.successful_actions for data in results.overlall_open_close_rate.values()) / sum(
                data.attempts for data in results.overlall_open_close_rate.values()) if sum(
                data.attempts for data in results.overlall_open_close_rate.values()) > 0 else 0
            writer.writerow(["Overall",
                             f"{overall_open_rate:.2%}, ({sum(data.successful_actions for data in results.overall_open_rate.values())}/{sum(data.attempts for data in results.overall_open_rate.values())} attempts)",
                             f"{overall_close_rate:.2%}, ({sum(data.successful_actions for data in results.overall_close_rate.values())}/{sum(data.attempts for data in results.overall_close_rate.values())} attempts)",
                             f"{overall_completion_rate:.2%}, ({sum(data.successful_actions for data in results.overlall_open_close_rate.values())}/{sum(data.attempts for data in results.overlall_open_close_rate.values())} attempts)"])

        # Write open/close success rates if data exists
        if results.overall_open_rate or results.overall_close_rate or results.overlall_open_close_rate:
            writer.writerow([])
            writer.writerow(["Action", "Open Rate", "Close Rate", "Completion Rate"])
            joint_types = ["REVOLUTE", "PRISMATIC"]
            for joint_type in joint_types:
                open_attempts = results.overall_open_rate[joint_type].attempts
                open_success = results.overall_open_rate[joint_type].successful_actions
                close_attempts = results.overall_close_rate[joint_type].attempts
                close_success = results.overall_close_rate[joint_type].successful_actions
                open_close_attempts = results.overlall_open_close_rate[joint_type].attempts
                open_close_success = results.overlall_open_close_rate[joint_type].successful_actions
                writer.writerow([f"{joint_type} Joint",
                                 f"{open_success / open_attempts:.2%} ({open_success}/{open_attempts} attempts)",
                                 f"{close_success / close_attempts:.2%} ({close_success}/{close_attempts} attempts)",
                                 f"{open_close_success / open_close_attempts:.2%} ({open_close_success}/{open_close_attempts} attempts)"])


def print_results(results):
    print("\n--- Overall Metrics ---")
    if results.overall_pickup_attempts > 0:
        print(
            f"Overall Pickup Rate: {results.overall_pickup_success / results.overall_pickup_attempts:.2%} ({results.overall_pickup_success}/{results.overall_pickup_attempts} attempts)")
        print(
            f"Overall Placing Rate: {results.overall_completion_success / results.overall_pickup_attempts:.2%} ({results.overall_completion_success}/{results.overall_pickup_attempts} attempts)")
        print(
            f"Overall Completion Rate: {results.overall_completion_success / results.overall_completion_attempts:.2%} ({results.overall_completion_success}/{results.overall_completion_attempts} attempts)")
    else:
        overall_open_attempts = sum([data.attempts for data in results.overall_open_rate.values()])
        overall_open_success = sum([data.successful_actions for data in results.overall_open_rate.values()])
        overall_close_attempts = sum([data.attempts for data in results.overall_close_rate.values()])
        overall_close_success = sum([data.successful_actions for data in results.overall_close_rate.values()])
        overall_open_close_attempts = sum([data.attempts for data in results.overlall_open_close_rate.values()])
        overall_open_close_success = sum(
            [data.successful_actions for data in results.overlall_open_close_rate.values()])
        print(
            f"Overall Open Rate: {overall_open_success / overall_open_attempts:.2%} ({overall_open_success}/{overall_open_attempts} attempts)")
        print(
            f"Overall Close Rate: {overall_close_success / overall_close_attempts:.2%} ({overall_close_success}/{overall_close_attempts} attempts)")
        print(
            f"Overall Open Close Rate: {overall_open_close_success / overall_open_close_attempts:.2%} ({overall_open_close_success}/{overall_open_close_attempts} attempts)")

    print("\n--- Object Success Rates ---")
    for obj, data in results.object_pickup_rate.items():
        pickup_rate = (data.successful_actions / data.attempts) if data.attempts > 0 else 0
        placing_rate = (results.object_completion_rate[obj].successful_actions / results.object_completion_rate[
            obj].attempts) if results.object_completion_rate[obj].attempts > 0 else 0
        completion_rate = (results.object_completion_rate[obj].successful_actions / data.attempts) if \
            results.object_completion_rate[obj].attempts > 0 else 0
        print(f"Object: {obj}, Pickup Rate: {pickup_rate:.2%} ({data.successful_actions}/{data.attempts} attempts)")
        print(
            f"Object: {obj}, Placing Rate: {placing_rate:.2%} ({results.object_completion_rate[obj].successful_actions}/{results.object_completion_rate[obj].attempts} attempts)")
        print(
            f"Object: {obj}, Completion Rate: {completion_rate:.2%} ({results.object_completion_rate[obj].successful_actions}/{data.attempts} attempts)")

    print("\n--- Surface Success Rates ---")
    for loc, data in results.surface_pickup_rate.items():
        pickup_rate = (data.successful_actions / data.attempts) if data.attempts > 0 else 0
        place_rate = (results.surface_place_rate[loc].successful_actions / results.surface_place_rate[loc].attempts) if \
            results.surface_place_rate[loc].attempts > 0 else 0
        print(f"Surface: {loc}, Pickup Rate: {pickup_rate:.2%} ({data.successful_actions}/{data.attempts} attempts)")
        print(
            f"Surface: {loc}, Place Rate: {place_rate:.2%} ({results.surface_place_rate[loc].successful_actions}/{results.surface_place_rate[loc].attempts} attempts)")

    print("\n--- Joint Open/Close Success Rates ---")
    if results.overall_open_rate or results.overall_close_rate or results.overlall_open_close_rate:
        joint_types = ["REVOLUTE", "PRISMATIC"]
        for joint_type in joint_types:
            open_attempts = results.overall_open_rate[joint_type].attempts
            open_success = results.overall_open_rate[joint_type].successful_actions
            close_attempts = results.overall_close_rate[joint_type].attempts
            close_success = results.overall_close_rate[joint_type].successful_actions
            open_close_attempts = results.overlall_open_close_rate[joint_type].attempts
            open_close_success = results.overlall_open_close_rate[joint_type].successful_actions
            if open_attempts > 0:
                print(
                    f"Overall {joint_type} Open Rate: {open_success / open_attempts:.2%} ({open_success}/{open_attempts} attempts)")
            if close_attempts > 0:
                print(
                    f"Overall {joint_type} Close Rate: {close_success / close_attempts:.2%} ({close_success}/{close_attempts} attempts)")
            if open_close_attempts > 0:
                print(
                    f"Overall {joint_type} Open Close Rate: {open_close_success / open_close_attempts:.2%} ({open_close_success}/{open_close_attempts} attempts)")


# Updated tests
with simulated_robot:
    NavigateAction([Pose([0, 0, 0])]).resolve().perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
    state_id = apartment.world.save_state()

    marker = AxisMarkerPublisher()
    # link1pose = robot.get_link_pose("r_hand")
    # link2pose = robot.get_link_pose("r_gripper_tool_frame")
    # link3pose = robot.get_link_pose("l_hand")
    # link4pose = robot.get_link_pose("l_gripper_tool_frame")
    # marker.publish([link1pose, link2pose, link3pose, link4pose], length=0.3, duration=20)

    if surface_tests:
        location_desig = iter(MultiSurfaceCostmapLocation(test_surfaces, apartment_desig.resolve(), seed=test_seed))
        results = Results()
        counter = 0
        for (start_location, start_surface), (target_location, target_surface) in tqdm(pairwise(location_desig)):
            counter += 1
            if counter > 500:
                break
            if target_location is None:
                break
            try:
                test_object = object_rng.choice(test_objects)
                object_type = test_object.obj_type
                min_p, max_p = test_object.get_axis_aligned_bounding_box().get_min_max_points()
                height_offset = (max_p.z - min_p.z) / 2
                start_location.pose.position.z += height_offset + 0.02
                target_location.pose.position.z += height_offset + 0.02

                # start_location.pose = Pose([2.2, 2.4, 1.02], [0, 0, 0, 1])

                # this comment should stay, because i sometimes use it for debugging
                if object_type != ObjectType.BOWL:
                    continue
                # if counter < 10:
                #     continue

                test_object.set_pose(start_location.pose)
                object_desig = mock_detect(test_object.obj_type)

                update_metrics(results.object_pickup_rate[object_type], "attempts")
                update_metrics(results.surface_pickup_rate[start_surface], "attempts")
                results.overall_pickup_attempts += 1
                results.overall_completion_attempts += 1

                # test_arms = [Arms.LEFT]
                TransportAction(object_desig, test_arms, [target_location.pose]).resolve().perform()

                update_metrics(results.object_completion_rate[object_type], "successful_actions")
                update_metrics(results.surface_place_rate[target_surface], "successful_actions")
                results.overall_completion_success += 1

            except ReachabilityFailure:
                pass

            finally:
                update_successful_pickup(results, test_object, start_location.pose, start_surface, target_surface)
                robot.detach_all()
                apartment.world.restore_state(state_id)

        print_results(results)
        save_results_to_csv(results, robot_name, "surface_tests")

    if open_tests:
        results = Results()
        revolute_joints = ["handle_cab1_top_door", "handle_cab2_door", "handle_cab3_door_top",
                           "handle_cab3_door_bottom", "handle_cab4_door_bottom", "handle_cab7"]
        for handle in test_handles:
            try:
                handle_desig = ObjectPart(names=[handle], part_of=apartment_desig.resolve())
                handle_desig_resolve = handle_desig.resolve()

                joint_type = JointType.REVOLUTE.name if handle_desig_resolve.name in revolute_joints else JointType.PRISMATIC.name

                update_metrics(results.overlall_open_close_rate[joint_type], "attempts")
                open_success, close_success = True, True

                try:
                    update_metrics(results.overall_open_rate[joint_type], "attempts")

                    closed_location, opened_location = AccessingLocation(handle_desig=handle_desig_resolve,
                                                                         robot_desig=robot_desig.resolve()).resolve()

                    NavigateAction([closed_location.pose]).resolve().perform()

                    OpenAction(object_designator_description=handle_desig, arms=[closed_location.arms[0]],
                               start_goal_location=[closed_location, opened_location]).resolve().perform()
                    update_metrics(results.overall_open_rate[joint_type], "successful_actions")

                except Exception:
                    open_success = False

                ParkArmsAction([Arms.BOTH]).resolve().perform()

                if handle_desig_resolve.name in revolute_joints:
                    container_joint = handle_desig_resolve.world_object.find_joint_above_link(handle_desig_resolve.name,
                                                                                              JointType.REVOLUTE)
                else:
                    container_joint = handle_desig_resolve.world_object.find_joint_above_link(handle_desig_resolve.name,
                                                                                              JointType.PRISMATIC)

                if handle_desig_resolve.name == "handle_cab7":
                    joint_safety_offset = 0.60
                else:
                    joint_safety_offset = 0.05

                init_joint_state = handle_desig_resolve.world_object.get_joint_limits(container_joint)[
                                       1] - joint_safety_offset

                apartment.set_joint_position(container_joint, init_joint_state)

                try:
                    update_metrics(results.overall_close_rate[joint_type], "attempts")

                    closed_location, opened_location = AccessingLocation(handle_desig=handle_desig.resolve(),
                                                                         robot_desig=robot_desig.resolve(),
                                                                         accessing_mode=AccessingMode.CLOSING).resolve()

                    CloseAction(object_designator_description=handle_desig, arms=[closed_location.arms[0]],
                                start_goal_location=[closed_location, opened_location]).resolve().perform()
                    update_metrics(results.overall_close_rate[joint_type], "successful_actions")
                except Exception:
                    close_success = False

                if open_success and close_success:
                    update_metrics(results.overlall_open_close_rate[joint_type], "successful_actions")

            finally:
                robot.detach_all()
                apartment.world.restore_state(state_id)

        print_results(results)
        save_results_to_csv(results, robot_name, "open_tests")
