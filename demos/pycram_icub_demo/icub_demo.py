from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.pose import Pose
from pycram.bullet_world import BulletWorld, Object
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher, ManualMarkerPublisher, AxisMarkerPublisher, \
    AxisIdentifier

world = BulletWorld("direct")
viz = VizMarkerPublisher()
robot = Object("iCub", "robot", "iCub.urdf", pose=Pose([1, 2, 0]))
apartment = Object("apartment", "environment", "apartment-small.urdf")

milk = Object("milk", "milk", "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([2.5, 2.4, 1.05]), color=[0, 1, 0, 1])
spoon = Object("spoon", "spoon", "spoon.stl", pose=Pose([2.4, 2.2, 0.85]), color=[0, 0, 1, 1])
bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([2.5, 2.2, 1.02]), color=[1, 1, 0, 1])

# milk = Object("milk", "milk", "milk.stl", pose=Pose([4.8, 4.2, 0.8]), color=[1, 0, 0, 1])
# cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([4.8, 4, 0.8]), color=[0, 1, 0, 1])
# spoon = Object("spoon", "spoon", "spoon.stl", pose=Pose([4.8, 3.5, 0.8]), color=[0, 0, 1, 1])
# bowl = Object("bowl", "bowl", "bowl.stl", pose=Pose([4.8, 3.7, 0.8]), color=[1, 1, 0, 1])

apartment.attach(spoon, 'cabinet10_drawer_top')

pick_pose_milk = Pose([2.7, 2.15, 1])
pick_pose_cereal = Pose([2.7, 2.35, 1])

robot_desig = BelieveObject(names=["Armar6"])
apartment_desig = BelieveObject(names=["apartment"])


@with_simulated_robot
def move_and_detect(obj_type, pick_pose):
    if robot_description.name == "iai_donbot":
        NavigateAction(target_locations=[Pose([1.7, 2.5, 0])]).resolve().perform()
    else:
        NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    status, object_dict = DetectAction(technique='specific', object_type=obj_type).resolve().perform()
    if status:
        for key, value in object_dict.items():
            detected_object = object_dict[key]

    return detected_object


def explore_joint_combinations(robot, joint_limits: dict, resolution: float, locked_joints: dict = None):
    """
    Explores possible joint state combinations with some joints locked at specific positions.

    :param robot: The robot object with the set_joint_states method.
    :param joint_limits: A dictionary where the key is the joint name and the value is a tuple of (lower_limit, upper_limit).
    :param resolution: The step size for the joint angles in radians.
    :param locked_joints: A dictionary of joints to lock with their corresponding positions.
    """
    if locked_joints is None:
        locked_joints = {}

    # Filter out the joints that are locked
    movable_joints = {joint: limits for joint, limits in joint_limits.items() if joint not in locked_joints}

    # Generate a list of possible positions for each movable joint within its limits
    joint_positions = {
        joint: np.arange(limits[0], limits[1] + resolution, resolution)
        for joint, limits in movable_joints.items()
    }

    # Get the movable joint names and possible positions
    joint_names = list(joint_positions.keys())
    positions_combinations = list(itertools.product(*joint_positions.values()))

    # Iterate over all combinations and set the joint states
    for combination in positions_combinations:
        joint_states = dict(zip(joint_names, combination))

        # Merge the locked joints with the current combination
        joint_states.update(locked_joints)

        # Set the joint states
        robot.set_joint_states(joint_states)

        # Optional: Add any evaluation or processing here
        print(f"Set joint states to: {joint_states}")


with simulated_robot:
    # MoveTorsoAction([-0.1]).resolve().perform()
    NavigateAction([Pose([1.3, 2.15, 0], [0, 0, 0, 1])]).resolve().perform()

    marker = AxisMarkerPublisher()
    look_pose = robot.get_link_pose("root_link")
    marker.publish(look_pose, name="head1", length=1)
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    # MoveGripperMotion("open", "left", allow_gripper_collision=True).resolve().perform()
    #pose = robot.get_link_pose("l_gripper_tool_frame")
    #marker.publish(pose, name="r_gripper_tool_frame", length=1)
    
  # <joint name="head1_joint" type="revolute">
  #   <parent link="torso4"/>
  #   <child link="head1"/>
  #   <origin rpy="1.57079  0       1.57079" xyz="0.235  0.088  0"/>
  #   <axis xyz="0  0  1"/>
  #   <limit effort="150.0" lower="-0.785398" upper="0.785398" velocity="0.85"/>
  # </joint>
  # <joint name="head2_joint" type="revolute">
  #   <parent link="head1"/>
  #   <child link="head2"/>
  #   <origin rpy="-1.57079  0       0" xyz="0  0  0"/>
  #   <axis xyz="0  0  1"/>
  #   <limit effort="150.0" lower="-0.436332" upper="0.785398" velocity="0.85"/>
  # </joint>

    # robot.set_joint_states({"head1_joint": -0.785398, "head2_joint": 0.785398})
    robot_description.get_tool_frame("r_gripper_tool_frame")
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    NavigateAction([Pose([1.3, 2.15, 0], [0, 0, 0, 1])]).resolve().perform()
    #look_pose.position.z += 0.5


    MoveTorsoAction([-0.1]).resolve().perform()

    # milk
    milk_desig = move_and_detect("milk", pick_pose_milk)

    NavigateAction([Pose([1.3, 1.35, 0], [0, 0, 0, 1])]).resolve().perform()
    # robot.set_joint_states({"ur5_shoulder_pan_joint": -1.6, "ur5_shoulder_lift_joint": -1.3, "ur5_elbow_joint": 1.3,
    #                         "ur5_wrist_1_joint": -0, "ur5_wrist_2_joint": 1.5, "ur5_wrist_3_joint": 1.5})
    joint_limits = {
        "ur5_shoulder_pan_joint": (-np.pi, np.pi),
        "ur5_shoulder_lift_joint": (-np.pi, np.pi),
        "ur5_elbow_joint": (-np.pi, np.pi),
        "ur5_wrist_1_joint": (-np.pi, np.pi),
        "ur5_wrist_2_joint": (-np.pi, np.pi),
        "ur5_wrist_3_joint": (-np.pi, np.pi)
    }

    # Example resolution (in radians)
    resolution = 0.5  # Adjust this value as needed

    # Specify the joints you want to lock and their fixed positions
    locked_joints = {
        "ur5_shoulder_pan_joint": 0.0,  # Locking the shoulder pan joint at 0 radians
        "ur5_wrist_3_joint": np.pi / 2  # Locking the wrist 3 joint at π/2 radians
    }

    # Assuming `robot` is your robot object with the `set_joint_states` method
    # explore_joint_combinations(robot, joint_limits, resolution, locked_joints)

    PickUpAction.Action(milk_desig, "left", "front").perform()

    #PlaceAction(milk_desig, ["left"], ["front"], [milk_desig.pose]).resolve().perform()

    pose = robot.get_link_pose("r_gripper_tool_frame")
    marker.publish(pose, name="r_gripper_tool_frame", length=1)

    ParkArmsAction.Action(Arms.BOTH).perform()

    NavigateAction([Pose([3.8, 4.4, 0], [0, 0, 0, 1])]).resolve().perform()

    place_pose = Pose([4.8, 4.2, 0.8])

    PlaceAction(milk_desig, ["right"], ["front"], [place_pose]).resolve().perform()

    ParkArmsAction.Action(Arms.BOTH).perform()

    # cereal
    cereal_desig = move_and_detect("cereal", pick_pose_cereal)

    NavigateAction([Pose([1.7, 1.8, 0], [0, 0, 0, 1])]).resolve().perform()

    PickUpAction.Action(cereal_desig, "left", "front").perform()

    ParkArmsAction.Action(Arms.BOTH).perform()

    NavigateAction([Pose([4.1, 3.8, 0], [0, 0, 0, 1])]).resolve().perform()

    PlaceAction(cereal_desig, ["left"], ["front"], [Pose([4.8, 4, 0.8], [0, 0, 0, 1])]).resolve().perform()

    ParkArmsAction.Action(Arms.BOTH).perform()

    # bowl
    bowl_desig = move_and_detect("bowl", pick_pose_cereal)

    NavigateAction([Pose([1.7, 2.5, 0], [0, 0, 0, 1])]).resolve().perform()

    PickUpAction.Action(bowl_desig, "right", "top").perform()

    ParkArmsAction.Action(Arms.BOTH).perform()

    NavigateAction([Pose([4.1, 3.8, 0], [0, 0, 0, 1])]).resolve().perform()

    PlaceAction(bowl_desig, ["right"], ["top"], [Pose([4.8, 3.7, 0.8], [0, 0, 0, 1])]).resolve().perform()

    ParkArmsAction.Action(Arms.BOTH).perform()
