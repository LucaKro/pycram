from .default_process_modules import *
from ..external_interfaces.ik import _apply_ik
from .default_process_modules import _move_arm_tcp


class DonbotMoveHead(DefaultMoveHead):
    """
    Moves the head of the iai_donbot robot to look at a specified point in the world coordinate frame.
    This point can be a position or an object, and the orientation is calculated based on the
    robot's base and camera alignment.
    """
    def _execute(self, desig):
        target = desig.target.copy()
        robot = World.robot

        base_frame_pose: Pose = robot.get_link_pose("ur5_base_link").copy()
        base_frame_pose.position.z += 0.4
        base_position = np.array(base_frame_pose.position_as_list())

        target_position = np.array(target.position_as_list())
        direction_vector = target_position - base_position
        direction_vector /= np.linalg.norm(direction_vector)

        current_x = np.array([1, 0, 0])
        rotation_axis = np.cross(current_x, direction_vector)
        rotation_axis /= np.linalg.norm(rotation_axis)
        rotation_angle = np.arccos(np.clip(np.dot(current_x, direction_vector), -1.0, 1.0))

        orientation_quat = R.from_rotvec(rotation_axis * rotation_angle).as_quat()

        adjusted_pose = Pose(base_position.tolist(), orientation_quat.tolist())
        side_grasp, top_grasp, horizontal = (Grasp.FRONT, None, False)
        grasp_orientation = RobotDescription.current_robot_description.get_arm_chain(Arms.LEFT).end_effector.get_grasp(
            side_grasp, top_grasp, horizontal)
        adjusted_pose = adjust_grasp_for_object_rotation(adjusted_pose, grasp_orientation)

        _move_arm_tcp(adjusted_pose, robot, Arms.LEFT)

# TODO: Also need to do DonbotMoveHeadReal


class DonbotManager(DefaultManager):

    def __init__(self):
        super().__init__()
        self.robot_name = "iai_donbot"

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DonbotMoveHead(self._looking_lock)
