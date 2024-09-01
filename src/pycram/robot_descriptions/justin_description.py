from ..robot_description import *


class JUSTINDescription(RobotDescription):

    def __init__(self):
        super().__init__("rollin-justin", "base_footprint", "base_link", "torso2", "torso2_joint")
        # Camera
        # fake camera, currently ill just try to use the whole head instead, sicne there is no camera in the urdf
        headcam = CameraDescription("head2",
                                        horizontal_angle=0.99483, vertical_angle=0.75049)
        self.add_cameras({"headcam": headcam})
        # The axis which points away from the camera and along which the picture of the camera is created
        self.front_facing_axis = [1, 0, 0]
        # Neck
        neck_links = ["head1", "head2"]
        neck_joints = ["head1_joint", "head2_joint"]
        neck_forward = {"forward": [0.0, 0.0], "down": [0.0, 0, 0]}
        neck_chain = ChainDescription("neck", neck_joints, neck_links, static_joint_states=neck_forward)
        self.add_chain("neck", neck_chain)
        # Arm
        arm_l_joints = ["torso1_joint", "torso2_joint", "torso3_joint", "torso4_joint", "left_arm1_joint", "left_arm2_joint", "left_arm3_joint", "left_arm4_joint",
                        "left_arm5_joint", "left_arm6_joint", "left_arm7_joint"]
        arm_l_links = ["torso1", "torso2", "torso3", "torso4", "left_arm1", "left_arm2", "left_arm3", "left_arm4", "left_arm5", "left_arm6", "left_arm7"]
        arm_r_joints = ["torso1_joint", "torso2_joint", "torso3_joint", "torso4_joint", "right_arm1_joint", "right_arm2_joint", "right_arm3_joint", "right_arm4_joint",
                        "right_arm5_joint", "right_arm6_joint", "right_arm7_joint"]
        arm_r_links = ["torso1", "torso2", "torso3", "torso4", "right_arm1", "right_arm2", "right_arm3", "right_arm4", "right_arm5", "right_arm6", "right_arm7"]
        gripper_l_joints = ["left_palm_joint", "left_1thumb_base_joint", "left_1thumb1_joint", "left_1thumb2_joint",
                            "left_1thumb3_joint", "left_1thumb4_joint", "left_2tip_base_joint", "left_2tip1_joint",
                            "left_2tip2_joint", "left_2tip3_joint", "left_2tip4_joint", "left_3middle_base_joint",
                            "left_3middle1_joint", "left_3middle2_joint", "left_3middle3_joint", "left_3middle4_joint",
                            "left_4ring_base_joint", "left_4ring1_joint", "left_4ring2_joint", "left_4ring3_joint",
                            "left_4ring4_joint"]
        gripper_l_links = ["left_palm", "left_1thumb_base", "left_1thumb1", "left_1thumb2", "left_1thumb3",
                           "left_1thumb4", "left_2tip_base", "left_2tip1", "left_2tip2", "left_2tip3", "left_2tip4",
                           "left_3middle_base", "left_3middle1", "left_3middle2", "left_3middle3", "left_3middle4",
                           "left_4ring_base", "left_4ring1", "left_4ring2", "left_4ring3", "left_4ring4"]
        gripper_r_joints = ["right_palm_joint", "right_1thumb_base_joint", "right_1thumb1_joint", "right_1thumb2_joint",
                            "right_1thumb3_joint", "right_1thumb4_joint", "right_2tip_base_joint", "right_2tip1_joint",
                            "right_2tip2_joint", "right_2tip3_joint", "right_2tip4_joint", "right_3middle_base_joint",
                            "right_3middle1_joint", "right_3middle2_joint", "right_3middle3_joint",
                            "right_3middle4_joint", "right_4ring_base_joint", "right_4ring1_joint",
                            "right_4ring2_joint", "right_4ring3_joint", "right_4ring4_joint"]
        gripper_r_links = ["right_palm", "right_1thumb_base", "right_1thumb1", "right_1thumb2", "right_1thumb3",
                           "right_1thumb4", "right_2tip_base", "right_2tip1", "right_2tip2", "right_2tip3",
                           "right_2tip4", "right_3middle_base", "right_3middle1", "right_3middle2", "right_3middle3",
                           "right_3middle4", "right_4ring_base", "right_4ring1", "right_4ring2", "right_4ring3",
                           "right_4ring4"]
        arm_l_park = {"park": [0, 0, 0.174533, 0, 0, -1.9, 0, 1, 0, -1, 0]}
        arm_r_park = {"park": [0, 0, 0.174533, 0, 0, -1.9, 0, 1, 0, -1, 0]}
        gripper_l = GripperDescription("left_gripper", gripper_links=gripper_l_links, gripper_joints=gripper_l_joints,
                                       gripper_meter_to_jnt_multiplier=16.0, gripper_minimal_position=1.57,
                                       gripper_convergence_delta=0.005)
        arm_l_chain = ChainDescription("left", arm_l_joints, arm_l_links, static_joint_states=arm_l_park)
        arm_l_inter = InteractionDescription(arm_l_chain, "left_palm")
        arm_l_manip = ManipulatorDescription(arm_l_inter, tool_frame="l_gripper_tool_frame",
                                             gripper_description=gripper_l)

        gripper_r = GripperDescription("right_gripper", gripper_links=gripper_r_links, gripper_joints=gripper_r_joints,
                                       gripper_meter_to_jnt_multiplier=16.0, gripper_minimal_position=1.57,
                                       gripper_convergence_delta=0.005)
        arm_r_chain = ChainDescription("right", arm_r_joints, arm_r_links, static_joint_states=arm_r_park)
        arm_r_inter = InteractionDescription(arm_r_chain, "right_palm")
        arm_r_manip = ManipulatorDescription(arm_r_inter, tool_frame="r_gripper_tool_frame",
                                             gripper_description=gripper_r)

        self.add_chains({"left": arm_l_manip, "right": arm_r_manip})
        # if for some reason this chain is not added, do breakpoint here and click through until the check for chain
        # length = state length. if these are not the same, it doesnt add but also doesnt print a error
        self.add_static_gripper_chains("left", {"open": [0, 0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0],
                                                "close": [0, 0, 0.523599, 1.50098, 1.76278, 1.76278,
                                                          0, 0.523599, 1.50098, 1.76278, 1.76278,
                                                          0, 0.523599, 1.50098, 1.76278, 1.76278,
                                                          0, 0.523599, 1.50098, 1.76278, 1.76278]})

        self.add_chain("torso", ChainDescription("torso",
                                                  ["torso2_joint", "torso3_joint", "torso4_joint"],
                                                  ["torso2", "torso3", "torso4"]))
        self.add_static_joint_chain("torso", "up", [0., 0.174533, 0])
        self.add_static_joint_chain("torso", "down", [-1.22173, 2.33874, -1.57])

        self.add_static_gripper_chains("right", {"open": [0, 0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0,
                                                         0, 0, 0, 0, 0],
                                                "close": [0, 0, 0.523599, 1.50098, 1.76278, 1.76278,
                                                          0, 0.523599, 1.50098, 1.76278, 1.76278,
                                                          0, 0.523599, 1.50098, 1.76278, 1.76278,
                                                          0, 0.523599, 1.50098, 1.76278, 1.76278]})

        # doublecheck navigation. for some reason if i look at the same pose that i navigate to (just moving the look pose on one axis) it doesnt line up as if the robot doesnt navigate correctly


        # the left and right hand have different grasping orientations. noticed during front grasping. need to figure out how to handle this nicely
        self.grasps = GraspingDescription(
            {"front": [0.707, 0.707, 0.707, 0.707],
             "left": [1, 0, 0, 1],
             "right": [0, 1, 1, 0],
             "top": [1, 1, 0, 0]})
             # "top": [-0.707, 0.707, 0, 0]})

    def get_camera_frame(self, name="headcam"):
        # TODO: Hacky since only one optical camera frame from pr2 is used
        return super().get_camera_frame(name)
