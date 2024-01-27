import time

from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.external_interfaces.interrupt_actionclient import InterruptClient
from pycram.failure_handling import Retry, RetryMonitor
from pycram.fluent import Fluent
from pycram.language import Monitor, Code
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher

world = BulletWorld("DIRECT")
viz = VizMarkerPublisher()
robot = Object("pr2", "robot", "pr2.urdf", pose=Pose([1, 2, 0]))
apartment = Object("apartment", "environment", "apartment.urdf")

milk = Object("milk", "milk", "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([2.5, 2.3, 1.05]), color=[1, 1, 0, 1])

pick_pose = Pose([2.7, 2.15, 1])

robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])

fluent = InterruptClient()
obj_type = "milk"
obj_color = "blue"
obj_name = ""
obj_location = ""
obj_size = "Normal"
handled = True


def color_map(color):
    if color == "red":
        return [1, 1, 0, 1]
    elif color == "blue":
        return [1, 0, 0, 1]


def monitor_func():
    global obj_type, obj_color, obj_name, obj_location, obj_size, handled
    if fluent.minor_interrupt.get_value():
        handled = False
        for obj in fluent.objects_in_use:
            obj_type = obj.type
            obj_color = obj.color
            obj_name = obj.name
            obj_location = obj.location
            obj_size = obj.size
        fluent.minor_interrupt.set_value(False)
        return True
    return False


def monitor():
    global x
    while x < 10:
        x += 1
        return False
    return True


@with_simulated_robot
def move_and_detect(obj_type, obj_size, obj_color):
    obj_color = color_map(obj_color)
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(BelieveObject(types=[obj_type] if obj_type else None,
                                              sizes=[obj_size] if obj_size else None,
                                              colors=[obj_color] if obj_color else None)).resolve().perform()

    return object_desig


def announce_pick(name: str, type: str, color: str, location: str, size: str):
    print(f"I will now pick up the {size.lower()} {color.lower()} {type.lower()} at location {location.lower()} ")
    print(f"I am now interruptable for 5 seconds")
    time.sleep(5)


def announce_bring(name: str, type: str, color: str, location: str, size: str):
    print(f"I will now bring the {size.lower()} {color.lower()} {type.lower()} to you")
    print(f"I am now interruptable for 5 seconds")
    time.sleep(5)


def announce_recovery():
    print("Recovering from Interrupt")
    print("I am not interruptable here at the moment")


def announce_pick_place(case: str, type: str, color: str, size: str):
    print(f"I will now {case.lower()} the {size.lower()} {color.lower()} {type.lower()}")
    print("This action cannot be interrupted")


def place_and_pick_new_obj(old_desig, location, obj_type, obj_size, obj_color):
    PlaceAction.Action(old_desig, "left", Pose(location)).perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    _, new_desig = move_and_detect(obj_type, obj_size, obj_color)
    PickUpAction.Action(new_desig, "left", "front").perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()


with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([0.25]).resolve().perform()

    milk_desig = Code(lambda: move_and_detect(obj_type, obj_size, obj_color))

    announce = Code(lambda: announce_pick(obj_name, obj_type, obj_color, obj_location, obj_size))

    plan = announce + milk_desig >> Monitor(monitor_func)

    _, [_, milk_desig] = RetryMonitor(plan, max_tries=5).perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

    PickUpAction.Action(milk_desig, "left", "front").perform()

    ParkArmsAction.Action(Arms.BOTH).perform()

    plan = NavigateAction([Pose([4, 2.8, 0])]) + announce >> Monitor(monitor_func)
    announce = Code(lambda: announce_bring(obj_name, obj_type, obj_color, obj_location, obj_size))

    recover = Code(lambda: announce_recovery()) + NavigateAction([Pose([1.7, 2, 0])]) + Code(
        lambda: place_and_pick_new_obj(milk_desig, [2.5, 2, 1.02], obj_type, obj_size, obj_color))

    RetryMonitor(plan, max_tries=5, recovery=recover).perform()

    MoveTCPMotion(arm="left", target=Pose([4.6, 3, 1.3])).resolve().perform()
