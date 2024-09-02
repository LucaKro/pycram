from ..robot_description import *


class ICubDescription(RobotDescription):

    def __init__(self):
        super().__init__("iCub", "base_footprint", "base_link", "torso_1", "torso_pitch")
        # Camera
        depth_camera = CameraDescription("DepthCamera",
                                         horizontal_angle=0.99483, vertical_angle=0.75049)
        flea_cameras_center = CameraDescription("FleaCamerasCenter",
                                                horizontal_angle=0.99483, vertical_angle=0.75049)
        roboception = CameraDescription("Roboception",
                                        horizontal_angle=0.99483, vertical_angle=0.75049)
        self.add_cameras({"depth_camera": depth_camera, "flea_cameras_center": flea_cameras_center,
                          "roboception": roboception})
        # The axis which points away from the camera and along which the picture of the camera is created
        self.front_facing_axis = [0, 0, 1]
        # Neck
        neck_joints = ["neck_fixed_joint", "neck_pitch", "neck_roll", "neck_yaw"]
        neck_links = ["neck_1", "neck_2", "neck_3", "head"]
        neck_forward = {"forward": [0.0, 0.0], "down": [0.0, 0, 0]}
        neck_chain = ChainDescription("neck", neck_joints, neck_links, static_joint_states=neck_forward)
        self.add_chain("neck", neck_chain)

        # Define the left arm chain
        arm_l_joints = ["l_shoulder_pitch", "l_shoulder_roll", "l_arm_ft_sensor", "l_shoulder_yaw",
                        "l_elbow", "l_wrist_prosup", "l_wrist_pitch", "l_wrist_yaw"]
        arm_l_links = ["l_shoulder_1", "l_shoulder_2", "l_shoulder_3", "l_upper_arm", "l_elbow_1", "l_forearm",
                       "l_wrist_1", "l_hand"]
        arm_l_chain = ChainDescription("left_arm", arm_l_joints, arm_l_links)
        self.add_chain("left_arm", arm_l_chain)

        # Define the right arm chain
        arm_r_joints = ["r_shoulder_pitch", "r_shoulder_roll", "r_arm_ft_sensor", "r_shoulder_yaw",
                        "r_elbow", "r_wrist_prosup", "r_wrist_pitch", "r_wrist_yaw"]
        arm_r_links = ["r_shoulder_1", "r_shoulder_2", "r_shoulder_3", "r_upper_arm", "r_elbow_1", "r_forearm",
                       "r_wrist_1", "r_hand"]
        arm_r_chain = ChainDescription("right_arm", arm_r_joints, arm_r_links)
        self.add_chain("right_arm", arm_r_chain)

        # Define the left leg chain
        leg_l_joints = ["l_hip_pitch", "l_hip_roll", "l_leg_ft_sensor", "l_hip_yaw", "l_knee", "l_ankle_pitch",
                        "l_ankle_roll"]
        leg_l_links = ["l_hip_1", "l_hip_2", "l_hip_3", "l_upper_leg", "l_lower_leg", "l_ankle_1", "l_ankle_2"]
        leg_l_chain = ChainDescription("left_leg", leg_l_joints, leg_l_links)
        self.add_chain("left_leg", leg_l_chain)

        # Define the right leg chain
        leg_r_joints = ["r_hip_pitch", "r_hip_roll", "r_leg_ft_sensor", "r_hip_yaw", "r_knee", "r_ankle_pitch",
                        "r_ankle_roll"]
        leg_r_links = ["r_hip_1", "r_hip_2", "r_hip_3", "r_upper_leg", "r_lower_leg", "r_ankle_1", "r_ankle_2"]
        leg_r_chain = ChainDescription("right_leg", leg_r_joints, leg_r_links)
        self.add_chain("right_leg", leg_r_chain)

        # Define the left gripper
        gripper_l_joints = ["l_hand_thumb_0_joint", "l_hand_thumb_1_joint", "l_hand_thumb_2_joint",
                            "l_hand_thumb_3_joint",
                            "l_hand_index_0_joint", "l_hand_index_1_joint", "l_hand_index_2_joint",
                            "l_hand_index_3_joint",
                            "l_hand_middle_0_joint", "l_hand_middle_1_joint", "l_hand_middle_2_joint",
                            "l_hand_middle_3_joint",
                            "l_hand_ring_0_joint", "l_hand_ring_1_joint", "l_hand_ring_2_joint", "l_hand_ring_3_joint",
                            "l_hand_little_0_joint", "l_hand_little_1_joint", "l_hand_little_2_joint",
                            "l_hand_little_3_joint"]
        gripper_l_links = ["l_hand_thumb_0", "l_hand_thumb_1", "l_hand_thumb_2", "l_hand_thumb_3", "l_hand_thumb_tip",
                           "l_hand_index_0", "l_hand_index_1", "l_hand_index_2", "l_hand_index_3", "l_hand_index_tip",
                           "l_hand_middle_0", "l_hand_middle_1", "l_hand_middle_2", "l_hand_middle_3",
                           "l_hand_middle_tip",
                           "l_hand_ring_0", "l_hand_ring_1", "l_hand_ring_2", "l_hand_ring_3", "l_hand_ring_tip",
                           "l_hand_little_0", "l_hand_little_1", "l_hand_little_2", "l_hand_little_3",
                           "l_hand_little_tip"]

        # Define the right gripper
        gripper_r_joints = ["r_hand_thumb_0_joint", "r_hand_thumb_1_joint", "r_hand_thumb_2_joint",
                            "r_hand_thumb_3_joint",
                            "r_hand_index_0_joint", "r_hand_index_1_joint", "r_hand_index_2_joint",
                            "r_hand_index_3_joint",
                            "r_hand_middle_0_joint", "r_hand_middle_1_joint", "r_hand_middle_2_joint",
                            "r_hand_middle_3_joint",
                            "r_hand_ring_0_joint", "r_hand_ring_1_joint", "r_hand_ring_2_joint", "r_hand_ring_3_joint",
                            "r_hand_little_0_joint", "r_hand_little_1_joint", "r_hand_little_2_joint",
                            "r_hand_little_3_joint"]
        gripper_r_links = ["r_hand_thumb_0", "r_hand_thumb_1", "r_hand_thumb_2", "r_hand_thumb_3", "r_hand_thumb_tip",
                           "r_hand_index_0", "r_hand_index_1", "r_hand_index_2", "r_hand_index_3", "r_hand_index_tip",
                           "r_hand_middle_0", "r_hand_middle_1", "r_hand_middle_2", "r_hand_middle_3",
                           "r_hand_middle_tip",
                           "r_hand_ring_0", "r_hand_ring_1", "r_hand_ring_2", "r_hand_ring_3", "r_hand_ring_tip",
                           "r_hand_little_0", "r_hand_little_1", "r_hand_little_2", "r_hand_little_3",
                           "r_hand_little_tip"]

        arm_l_park = {"park": [0, 0, 1.5, 0.5, 2.0, 1.5, 0, 0]}
        arm_r_park = {"park": [0, 0, 1.5, 2.64, 2.0, 1.6415, 0, 0]}
        gripper_l = GripperDescription("gripper_l", gripper_links=gripper_l_links, gripper_joints=gripper_l_joints,
                                       gripper_meter_to_jnt_multiplier=16.0, gripper_minimal_position=1.57,
                                       gripper_convergence_delta=0.005)
        arm_l_chain = ChainDescription("left", arm_l_joints, arm_l_links, static_joint_states=arm_l_park)
        arm_l_inter = InteractionDescription(arm_l_chain, "arm_t8_r0")
        arm_l_manip = ManipulatorDescription(arm_l_inter, tool_frame="left_tool_frame",
                                             gripper_description=gripper_l)
        gripper_r = GripperDescription("gripper_r", gripper_links=gripper_r_links, gripper_joints=gripper_r_joints,
                                       gripper_meter_to_jnt_multiplier=16.0, gripper_minimal_position=1.57,
                                       gripper_convergence_delta=0.005)
        arm_r_chain = ChainDescription("right", arm_r_joints, arm_r_links, static_joint_states=arm_r_park)
        arm_r_inter = InteractionDescription(arm_r_chain, "arm_t8_r1")
        arm_r_manip = ManipulatorDescription(arm_r_inter, tool_frame="right_tool_frame",
                                             gripper_description=gripper_r)
        self.add_chains({"left": arm_l_manip, "right": arm_r_manip})
        self.add_static_gripper_chains("left", {"open": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                                "close": [1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57,
                                                          1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, ]})

        self.add_static_gripper_chains("right", {"open": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                                "close": [1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57,
                                                          1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, ]})

        self.grasps = GraspingDescription(
            {"front": [0.707, 0.707, 0.707, 0.707],
             "left": [1, 0, 0, 1],
             "right": [0, 1, 1, 0],
             "top": [-1, 0, 0, 0]})

    def get_camera_frame(self, name="roboception"):
        # TODO: Hacky since only one optical camera frame from pr2 is used
        return super().get_camera_frame(name)
