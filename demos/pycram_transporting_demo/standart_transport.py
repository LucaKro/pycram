import rospy
from IPython.display import display, clear_output, HTML

from init_setup import breakfast_context_apartment
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose
from pycram.process_module import simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher

pick_pose = Pose([2.7, 2.15, 1])

world = BulletWorld()
VizMarkerPublisher()
robot = Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=Pose([1.1, 2, 0]))
robot_desig = BelieveObject(names=["pr2"])
apart_desig = BelieveObject(names=["apartment"])
current_context = breakfast_context_apartment  # Or dinner_context, depending on the scenario
current_context.spawn_objects()

with (simulated_robot):
    MoveTorsoAction([0.25]).resolve().perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()
    LookAtAction(targets=[pick_pose]).resolve().perform()
    MoveTorsoAction([0.25]).resolve().perform()

    status, object_dict = DetectAction(technique='specific', object_type="milk").resolve().perform()
    # print(object_dict[0])
    # for key, value in object_dict.items():
    #     if object_dict[key].type == "Cutlery" or object_dict[key].type == ObjectType.BOWL:
    #         grasp = "top"
    #         marging_cm = 0.08
    #     else:
    #         grasp = "front"
    #         marging_cm = 0.2
    #     arm = "left"
    #     PickUpAction(object_dict[key], ["left"], [grasp]).resolve().perform()
    #     ParkArmsAction([Arms.BOTH]).resolve().perform()
    #
    #     place_pose, nav_pose = find_reachable_location_and_nav_pose(enviroment_link="table_area_main",
    #                                                                 enviroment_desig=apart_desig.resolve(),
    #                                                                 object_desig=object_dict[key],
    #                                                                 robot_desig=robot_desig.resolve(), arm=arm,
    #                                                                 world=world, marging_cm=marging_cm)
    #     if not nav_pose:
    #         rospy.logerr("No location found")
    #     else:
    #         NavigateAction(target_locations=[nav_pose]).resolve().perform()
    #         MoveTorsoAction([0.25]).resolve().perform()
    #         PlaceAction(object_dict[key], [arm], [grasp], [place_pose]).resolve().perform()
    #
    #         ParkArmsAction([Arms.BOTH]).resolve().perform()
    #         NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()
    #         MoveTorsoAction([0.25]).resolve().perform()  # Clear the animation
    # rospy.loginfo("Transporting task completed!")
