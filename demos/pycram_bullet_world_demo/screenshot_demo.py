import rospy

from pycram.plan_failures import IKError
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.ros.viz_marker_publisher import VizMarkerPublisher, ManualMarkerPublisher, CurvedArrowMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType, WorldMode, TorsoState
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot, ProcessModule
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color

extension = ObjectDescription.get_file_extension()

world = BulletWorld(WorldMode.DIRECT)
viz = VizMarkerPublisher()
tfviz = TFBroadcaster()

world.allow_publish_debug_poses = True
ProcessModule.execution_delay = False
robot_name = "Armar6"
robot = Object(robot_name, ObjectType.ROBOT, f"{robot_name}{extension}", pose=Pose([1, 2, 0]))

apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment-small{extension}")

if robot.name == "iCub":
    milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([4.7, 4.6, 0.8]),
                  color=Color(1, 0, 0, 1))
    milk_target_pose = Pose([4.8, 3.45, 0.8])

    cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                    pose=Pose([4.65, 4.75, 0.8]), color=Color(0, 1, 0, 1))
    cereal_target_pose = Pose([4.85, 3.3, 0.8], [0, 0, 1, 1])

    bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([4.4, 4.4, 0.75], [0, 0, -1, 1]),
                  color=Color(1, 1, 0, 1))
    bowl_target_pose = Pose([5, 3.3, 0.75], [0, 0, 0, 1])

    spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([4.7, 4.2, 0.75], [0, 0, -1, 1]),
                   color=Color(0, 0, 1, 1))
    spoon_target_pose = Pose([5.2, 3.3, 0.8], [0, 0, 1, 1])

    pick_pose = Pose([4.7, 4.5, 0.8])
    nav_pose = Pose([4, 4.5, 0])
else:
    milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 1.75, 1.32], [0, 0, 0, 1]),
                  color=Color(1, 0, 0, 1))
    milk_target_pose = Pose([4.7, 3.3, 0.81])

    cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                    pose=Pose([2.5, 2.5, 1.35], [0, 0, 0, 1]), color=Color(0, 1, 0, 1))
    cereal_target_pose = Pose([5.2, 3.3, 0.81], [0, 0, 1, 1])

    bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.4, 2.2, 1.28], [0, 0, 1, 1]),
                  color=Color(1, 1, 0, 1))
    bowl_target_pose = Pose([5, 3.3, 0.8], [0, 0, 1, 1])

    spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([2.47, 2.2, 0.86]),
                   color=Color(0, 0, 1, 1))
    spoon_target_pose = Pose([4.85, 3.3, 0.8], [0, 0, 1, 1])

    apartment.attach(spoon, 'cabinet10_drawer_top')

    pick_pose = Pose([2.7, 2.15, 1])
    nav_pose = Pose([1.4, 2, 0])

robot_desig = BelieveObject(names=[robot_name])
apartment_desig = BelieveObject(names=["apartment"])


@with_simulated_robot
def move_and_detect(obj_type):
    NavigateAction(target_locations=[nav_pose]).resolve().perform()

    LookAtAction(targets=[milk.pose]).resolve().perform()

    object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

    return object_desig


with simulated_robot:
    NavigateAction([Pose([0, 0, 0])]).resolve().perform()

    milk.set_position([1, -0.45, 1])

    nav_pose = Pose([0, 0, 0])
    milk_desig = move_and_detect(ObjectType.MILK)
    grasp = GraspDescription(side_face=Grasp.FRONT, top_face=None, horizontal=False)
    PickUpAction(milk_desig, [Arms.RIGHT], grasp).resolve().perform()

    marker = AxisMarkerPublisher()
    marker.publish([milk.pose], length=0.2)

    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()

    nav_pose2 = Pose([1.4, 2, 0], [0, 0, 1, 1])

    NavigateAction([nav_pose2]).resolve().perform()

    # move_and_detect(ObjectType.MILK)

    LookAtAction([milk.pose]).resolve().perform()

    print()
    # def multi(q1: List[float], q2: List[float]) -> List[float]:
    #     """
    #     Multiplies two quaternions q1 and q2.
    #
    #     Args:
    #         q1, q2 (List): Quaternions in [x, y, z, w] format.
    #
    #     Returns:
    #         List: Resulting quaternion [x, y, z, w].
    #     """
    #     x1, y1, z1, w1 = q1
    #     x2, y2, z2, w2 = q2
    #     return [
    #         w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1,
    #         w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1,
    #         w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1,
    #         w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1
    #     ]
    #
    # palm_axis = [0, 0, -1]
    # palm_axis = [1, 0, 0]
    # test = RobotDescription.current_robot_description.get_arm_chain(Arms.LEFT).end_effector.grasps
    # milk_pose_copy = bowl.pose.copy()
    # new_rotation = multi(test[(Grasp.LEFT, None, False)], milk_pose_copy.orientation_as_list())
    # milk_pose_copy.orientation = new_rotation
    # milk_pose_copy = translate_relative_to_object(milk_pose_copy, palm_axis, 0.3)
    # test_poses = [milk_pose_copy]
    # marker = AxisMarkerPublisher()
    #
    # for (side_grasp, top_bot_grasp, horizontal), orientation in test.items():
    #
    #     # if side_grasp == Grasp.FRONT:
    #     # if top_bot_grasp == Grasp.TOP:
    #     if side_grasp == Grasp.LEFT and horizontal and top_bot_grasp == Grasp.TOP:
    #         temp_pose = bowl.pose.copy()
    #         new_rotation = multi(orientation, temp_pose.orientation_as_list())
    #         temp_pose.orientation = new_rotation
    #         temp_pose = translate_relative_to_object(temp_pose, palm_axis, 0.3)
    #         #temp_pose = Pose([temp_pose.position.x, temp_pose.position.y, temp_pose.position.z+0.1], new_rotation)
    #         test_poses.append(temp_pose)
    #
    # marker.publish(test_poses, duration=20.0, length=0.05)
    #
    # print()

    # handle_desig = ObjectPart(names=["handle_cab3_door_top"], part_of=apartment_desig.resolve())
    # closed_location, opened_location = AccessingLocation(handle_desig=handle_desig.resolve(),
    #                                                      robot_desig=robot_desig.resolve()).resolve()
    # OpenAction(object_designator_description=handle_desig, arms=[closed_location.arms[0]], start_goal_location=[closed_location, opened_location]).resolve().perform()
    # NavigateAction([Pose([0, 0, 0])]).resolve().perform()
    # CloseAction(object_designator_description=handle_desig, arms=[closed_location.arms[0]], start_goal_location=[opened_location, closed_location]).resolve().perform()
    # ParkArmsAction([Arms.BOTH]).resolve().perform()



    milk_desig = move_and_detect(ObjectType.MILK)
    cereal_desig = move_and_detect(ObjectType.BREAKFAST_CEREAL)
    bowl_desig = move_and_detect(ObjectType.BOWL)


    # marker = CurvedArrowMarkerPublisher()
    # marker2 = AxisMarkerPublisher()
    # test_pose = Pose([2.5, 1.5, 1.3],  [0.0, 0.0, 0, 1])
    # marker2.publish([test_pose])
    # rotation = [0.0, 0.0, -0.7071067811865476, 0.7071067811865476]
    # marker.publish(center_pose=test_pose, goal_quaternion=rotation)
    # rospy.sleep(1)
    # marker.clear_all_markers()
    # rospy.sleep(1)
    #
    # test_pose.set_orientation(rotation)
    # marker2.publish([test_pose])
    # rotation = [0.5000000000000001, 0.5000000000000001, -0.5000000000000001, 0.5000000000000001]
    # marker.publish(center_pose=test_pose, goal_quaternion=rotation)
    # rospy.sleep(1)
    # marker.clear_all_markers()
    # rospy.sleep(1)
    #
    # test_pose.set_orientation(rotation)
    # marker2.publish([test_pose])
    # rotation = [0.7071067811865477, 0.0, -0.7071067811865477, 0.0]
    # marker.publish(center_pose=test_pose, goal_quaternion=rotation)
    # rospy.sleep(1)
    # marker.clear_all_markers()
    # rospy.sleep(1)
    #
    # test_pose.set_orientation(rotation)
    # marker2.publish([test_pose])
    # rospy.sleep(1)

    # calculate_grasp_configs([milk_desig, cereal_desig, bowl_desig])

    MoveTorsoAction([TorsoState.LOW]).resolve().perform()

    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
    #
    # TransportAction(milk_desig, [Arms.LEFT], [milk_target_pose]).resolve().perform()
    #
    # cereal_desig = move_and_detect(ObjectType.BREAKFAST_CEREAL)
    #
    # TransportAction(cereal_desig, [Arms.LEFT], [cereal_target_pose]).resolve().perform()

    if not robot.name == "tiago_dual":
        bowl_desig = move_and_detect(ObjectType.BOWL)

        TransportAction(bowl_desig, [Arms.LEFT], [bowl_target_pose]).resolve().perform()
    print()
    # if robot.name == "iCub":
    #     spoon_desig = move_and_detect(ObjectType.SPOON)
    #     TransportAction(spoon_desig, [Arms.LEFT], [spoon_target_pose]).resolve().perform()
    # else:
    #     # Finding and navigating to the drawer holding the spoon
    #     handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig.resolve())
    #     closed_location, opened_location = AccessingLocation(handle_desig=handle_desig.resolve(),
    #                                                          robot_desig=robot_desig.resolve()).resolve()
    #
    #     NavigateAction([closed_location.pose]).resolve().perform()
    #
    #     OpenAction(object_designator_description=handle_desig, arms=[closed_location.arms[0]],
    #                start_goal_location=[closed_location, opened_location]).resolve().perform()
    #     spoon.detach(apartment)
    #
    #     # Detect and pickup the spoon
    #     ParkArmsAction([Arms.BOTH]).resolve().perform()
    #     MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
    #     LookAtAction([apartment.get_link_pose("handle_cab10_t")]).resolve().perform()
    #
    #     spoon_desig = DetectAction(BelieveObject(types=[ObjectType.SPOON])).resolve().perform()
    #
    #     if robot.name == "iai_donbot":
    #         ParkArmsAction([Arms.BOTH]).resolve().perform()
    #         PickUpAction(spoon_desig, [Arms.LEFT], [Grasp.TOP]).resolve().perform()
    #
    #         ParkArmsAction([Arms.BOTH]).resolve().perform()
    #
    #         # Find a pose to place the spoon, move and then place it
    #         grasp_config = GraspDescription(side_face=Grasp.FRONT, top_face=Grasp.TOP, horizontal=False)
    #
    #         placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve(),
    #                                       reachable_arms=[Arms.LEFT],
    #                                       used_grasp_config=[grasp_config], object_in_hand=spoon_desig).resolve()
    #
    #         NavigateAction([placing_loc.pose]).resolve().perform()
    #
    #         PlaceAction(spoon_desig, [spoon_target_pose], [Arms.LEFT]).resolve().perform()
    #
    #         ParkArmsAction([Arms.BOTH]).resolve().perform()
    #
    #         NavigateAction([closed_location.pose]).resolve().perform()
    #
    #         CloseAction(object_designator_description=handle_desig, arms=[closed_location.arms[0]],
    #                     start_goal_location=[opened_location, closed_location]).resolve().perform()
    #
    #         ParkArmsAction([Arms.BOTH]).resolve().perform()
    #
    #     else:
    #
    #         pickup_arm = Arms.LEFT if closed_location.arms[0] == Arms.RIGHT else Arms.RIGHT
    #         try:
    #             grasp_config = GraspDescription(side_face=Grasp.FRONT, top_face=Grasp.TOP, horizontal=False)
    #             PickUpAction(spoon_desig, [pickup_arm], grasp_config).resolve().perform()
    #         except IKError:
    #             ParkArmsAction([Arms.BOTH]).resolve().perform()
    #             pickup_loc = CostmapLocation(target=spoon_desig, reachable_for=robot_desig.resolve(),
    #                                          reachable_arms=[pickup_arm], used_grasp_config=grasp_config).resolve()
    #             NavigateActionPerformable(pickup_loc.pose).perform()
    #             PickUpAction(spoon_desig, [pickup_arm], pickup_loc.used_grasp_config).resolve().perform()
    #
    #         ParkArmsAction([Arms.LEFT if pickup_arm == Arms.LEFT else Arms.RIGHT]).resolve().perform()
    #
    #         NavigateAction([opened_location.pose]).resolve().perform()
    #
    #         CloseAction(object_designator_description=handle_desig, arms=[closed_location.arms[0]],
    #                     start_goal_location=[opened_location, closed_location]).resolve().perform()
    #
    #         ParkArmsAction([Arms.BOTH]).resolve().perform()
    #
    #         MoveTorsoAction([TorsoState.MID]).resolve().perform()
    #
    #         # Find a pose to place the spoon, move and then place it
    #         placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve(),
    #                                       reachable_arms=[pickup_arm], used_grasp_config=grasp_config,
    #                                       object_in_hand=spoon_desig).resolve()
    #
    #         NavigateAction([placing_loc.pose]).resolve().perform()
    #         PlaceAction(spoon_desig, [spoon_target_pose], [pickup_arm]).resolve().perform()
    #
    #         ParkArmsAction([Arms.BOTH]).resolve().perform()
