import logging
import random
from collections import defaultdict

from matplotlib import pyplot as plt
from tqdm import tqdm
from typing_extensions import List

from pycram.plan_failures import IKError
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

extension = ObjectDescription.get_file_extension()

world = BulletWorld(WorldMode.DIRECT)
viz = VizMarkerPublisher()
viz2 = VizMarkerPublisher(as_prospection_world=True)

world.allow_publish_debug_poses = False
ProcessModule.execution_delay = False
surface_tests = True
open_tests = True
robot_name = "pr2"
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


logging.basicConfig(level=logging.INFO)
logging.info = print
import csv
import logging
from collections import defaultdict
from dataclasses import dataclass, field
from time import time
from tqdm import tqdm

@dataclass
class FailureMetrics:
    failures: int = 0
    attempts: int = 0
    successful_actions: int = 0

@dataclass
class Results:
    successful_completion: int = 0
    successful_action: int = 0
    failure: int = 0
    completion_time: float = 0.0
    action_times: list = field(default_factory=list)  # Stores tuples of (elapsed_time, action_type, action_stage)
    object_failure_rate: defaultdict = field(default_factory=lambda: defaultdict(FailureMetrics))
    location_failure_rate: defaultdict = field(default_factory=lambda: defaultdict(FailureMetrics))

def update_object_results(results, object_type, key):
    metrics = results.object_failure_rate[object_type]
    setattr(metrics, key, getattr(metrics, key) + 1)

def update_location_results(results, location, key):
    metrics = results.location_failure_rate[location]
    setattr(metrics, key, getattr(metrics, key) + 1)

def log_action_time(results, start_time, action_type, action_stage):
    elapsed_time = time() - start_time
    results.action_times.append((elapsed_time, action_type, action_stage))

def save_results_to_csv(results, robot_name, test_type):
    filename = f"{robot_name}_{test_type}_results.csv"
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Metric", "Value"])
        writer.writerow(["Successful Completion", results.successful_completion])
        writer.writerow(["Successful Actions", results.successful_action])
        writer.writerow(["Failures", results.failure])
        writer.writerow(["Total Completion Time", results.completion_time])
        writer.writerow(["Average Action Time", results.completion_time / len(results.action_times) if results.action_times else 0])
        writer.writerow([])
        writer.writerow(["Action Time", "Action Type", "Action Stage"])
        for action_time, action_type, action_stage in results.action_times:
            writer.writerow([action_time, action_type, action_stage])

        writer.writerow([])
        writer.writerow(["Object Type", "Attempts", "Failures", "Successful Actions"])
        for obj, data in results.object_failure_rate.items():
            writer.writerow([obj, data.attempts, data.failures, data.successful_actions])

        writer.writerow([])
        writer.writerow(["Location", "Attempts", "Failures", "Successful Actions"])
        for loc, data in results.location_failure_rate.items():
            writer.writerow([loc, data.attempts, data.failures, data.successful_actions])

def update_successful_action(results, entity, entity_type, key):
    if entity_type == "object":
        update_object_results(results, entity.obj_type, key)
    elif entity_type == "location":
        update_location_results(results, entity, key)
    results.successful_action += 1

def print_results(results):
    total_attempts = results.successful_completion + results.failure
    average_time = results.completion_time / results.successful_completion if results.successful_completion > 0 else 0
    failure_rate = results.failure / total_attempts if total_attempts > 0 else 0

    logging.info("\n--- Overall Metrics ---")
    logging.info(f"Average completion time: {average_time:.2f} seconds")
    logging.info(f"Failure rate: {failure_rate:.2%} ({results.failure}/{total_attempts} attempts)")

    logging.info("\n--- Object Failure Rate ---")
    for obj, data in results.object_failure_rate.items():
        failure_rate = data.failures / data.attempts if data.attempts > 0 else 0
        logging.info(f"Object: {obj}, Failure Rate: {failure_rate:.2%} ({data.failures}/{data.attempts} attempts)")

    logging.info("\n--- Location Failure Rate ---")
    for location, data in results.location_failure_rate.items():
        failure_rate = data.failures / data.attempts if data.attempts > 0 else 0
        logging.info(f"Location: {location}, Failure Rate: {failure_rate:.2%} ({data.failures}/{data.attempts} attempts)")

# Updated tests
with simulated_robot:
    NavigateAction([Pose([0, 0, 0])]).resolve().perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
    state_id = apartment.world.save_state()

    marker = AxisMarkerPublisher()

    if surface_tests:
        location_desig = iter(MultiSurfaceCostmapLocation(test_surfaces, apartment_desig.resolve(), seed=test_seed))

        results = Results()
        counter = 0
        for (start_location, start_surface), (target_location, target_surface) in tqdm(pairwise(location_desig)):
            counter += 1
            if counter > 100:
                break
            if target_location is None:
                break
            try:
                start_time_pickup = time()
                test_object = object_rng.choice(test_objects)
                object_type = test_object.obj_type
                min_p, max_p = test_object.get_axis_aligned_bounding_box().get_min_max_points()
                height_offset = (max_p.z - min_p.z) / 2
                start_location.pose.position.z += height_offset + 0.02
                target_location.pose.position.z += height_offset + 0.02

                # this comment should stay, because i sometimes use it for debugging
                # if object_type != ObjectType.BREAKFAST_CEREAL:
                #     continue
                # if counter < 52:
                #     continue

                test_object.set_pose(start_location.pose)
                object_desig = mock_detect(test_object.obj_type)

                update_object_results(results, object_type, "attempts")
                update_location_results(results, start_surface, "attempts")

                TransportAction(object_desig, test_arms, [target_location.pose]).resolve().perform()

                log_action_time(results, start_time_pickup, "Transport", "Pickup")

                start_time_placing = time()
                log_action_time(results, start_time_placing, "Transport", "Placing")

            except ReachabilityFailure:
                results.failure += 1
                update_object_results(results, object_type, "failures")
                update_location_results(results, start_surface, "failures")
                log_action_time(results, start_time_pickup, "Transport", "Failure")

            else:
                end_time = time()
                results.successful_completion += 1
                results.completion_time += end_time - start_time_pickup

            finally:
                update_successful_action(results, test_object, "object", "successful_actions")
                robot.detach_all()
                apartment.world.restore_state(state_id)

        print_results(results)
        save_results_to_csv(results, robot_name, "surface_tests")

    if open_tests:
        results = Results()
        for handle in test_handles:
            try:
                start_time_opening = time()
                handle_desig = ObjectPart(names=[handle], part_of=apartment_desig.resolve())
                closed_location, opened_location = AccessingLocation(handle_desig=handle_desig.resolve(),
                                                                     robot_desig=robot_desig.resolve()).resolve()

                NavigateAction([closed_location.pose]).resolve().perform()

                OpenAction(object_designator_description=handle_desig, arms=[closed_location.arms[0]],
                           start_goal_location=[closed_location, opened_location]).resolve().perform()

                log_action_time(results, start_time_opening, "Open", "Opening")

                ParkArmsAction([Arms.BOTH]).resolve().perform()
                NavigateAction([opened_location.pose]).resolve().perform()

                start_time_closing = time()

                opened_location, closed_location = AccessingLocation(handle_desig=handle_desig.resolve(),
                                                                     robot_desig=robot_desig.resolve(),
                                                                     accessing_mode=AccessingMode.CLOSING).resolve()

                CloseAction(object_designator_description=handle_desig, arms=[closed_location.arms[0]],
                            start_goal_location=[opened_location, closed_location]).resolve().perform()

                log_action_time(results, start_time_closing, "Close", "Closing")

                ParkArmsAction([Arms.BOTH]).resolve().perform()

            except ReachabilityFailure:
                marker = AxisMarkerPublisher()
                marker.publish([apartment.get_link_pose(handle)])
                results.failure += 1
                log_action_time(results, start_time_opening, "Open/Close", "Failure")

            else:
                end_time = time()
                results.successful_completion += 1
                results.completion_time += end_time - start_time_opening

            finally:
                update_successful_action(results, handle, "location", "successful_actions")
                robot.detach_all()
                apartment.world.restore_state(state_id)

        print_results(results)
        save_results_to_csv(results, robot_name, "open_tests")
