from ..robot_description import RobotDescription, KinematicChainDescription, EndEffectorDescription, \
    RobotDescriptionManager, CameraDescription
from ..datastructures.enums import Arms, Grasp, GripperState, GripperType, TorsoState
import rospkg

rospack = rospkg.RosPack()
filename = rospack.get_path('pycram') + '/resources/robots/' + 'fetch' + '.urdf'

fetch_description = RobotDescription("fetch", "base_link", "torso_lift_link", "torso_lift_joint",
                                               filename)

################################## Left Arm ##################################
left_arm = KinematicChainDescription("left", "base_link", "wrist_roll_link",
                                     fetch_description.urdf_object, arm_type=Arms.LEFT)

left_arm.add_static_joint_states("park", {'wrist_roll_joint': -0.0015926535897929917, 'shoulder_pan_joint': 1.6056, 'shoulder_lift_joint': 1.518, 'upperarm_roll_joint': 3.141592653589793, 'elbow_flex_joint': -1.6909999999999998, 'forearm_roll_joint': -0.0015926535897929917, 'wrist_flex_joint': -1.5100000000000002})

fetch_description.add_kinematic_chain_description(left_arm)

################################## Left Gripper ##################################
left_gripper = EndEffectorDescription("left_gripper", "gripper_link", "gripper_link",
                                      fetch_description.urdf_object)
left_gripper.add_static_joint_states(GripperState.OPEN, {'r_gripper_finger_joint': 0.05, 'l_gripper_finger_joint': 0.05})
left_gripper.add_static_joint_states(GripperState.CLOSE, {'r_gripper_finger_joint': 0.0, 'l_gripper_finger_joint': 0.0})

left_gripper.end_effector_type = GripperType.PARALLEL
left_gripper.opening_distance = 0.0
left_arm.end_effector = left_gripper

################################## Right Arm ##################################
right_arm = KinematicChainDescription("right", "base_link", "",
                                      fetch_description.urdf_object, arm_type=Arms.RIGHT)

right_arm.add_static_joint_states("park", {})

fetch_description.add_kinematic_chain_description(right_arm)

################################## Right Gripper ##################################
right_gripper = EndEffectorDescription("right_gripper", "", "",
                                       fetch_description.urdf_object)
right_gripper.add_static_joint_states(GripperState.OPEN, {})
right_gripper.add_static_joint_states(GripperState.CLOSE, {})

right_gripper.end_effector_type = GripperType.PARALLEL
right_gripper.opening_distance = 0.0
right_arm.end_effector = right_gripper

################################## Torso ##################################
torso = KinematicChainDescription("torso", "base_link", "torso_lift_link",
                                  fetch_description.urdf_object)

torso.add_static_joint_states(TorsoState.HIGH, {'torso_lift_joint': 0.38615})
torso.add_static_joint_states(TorsoState.MID, {'torso_lift_joint': 0.19})
torso.add_static_joint_states(TorsoState.LOW, {'torso_lift_joint': 0.0})

fetch_description.add_kinematic_chain_description(torso)

################################## Camera ##################################
camera = CameraDescription("head_camera_depth_optical_frame", "head_camera_depth_optical_frame", 1.0609303712844849,
                           1.447080373764038, 0.99483, 0.75049,
                           [0, 0, 1])
fetch_description.add_camera_description(camera)

################################## Neck ##################################
fetch_description.add_kinematic_chain("neck", "torso_lift_link", "head_tilt_link")
fetch_description.set_neck("head_pan_joint", "head_tilt_joint")

################################# Grasps ##################################
right_orientation = [0, 0, 0, 0]
right_gripper.generate_grasp_descriptions(right_orientation)
right_gripper.set_palm_axis([0, 0, 0])

left_orientation = [0.0, 0.0, 0.0, 1.0]
left_gripper.generate_grasp_descriptions(left_orientation)
left_gripper.set_palm_axis([1.0, 0.0, 0.0])


################################# Additionals ##################################
# fetch_description.set_costmap_offset(0.0)
fetch_description.set_max_reach("torso_lift_link", "gripper_link")

# Add to RobotDescriptionManager
rdm = RobotDescriptionManager()
rdm.register_description(fetch_description)