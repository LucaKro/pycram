import time

from pycram.robot_description import ViewManager

from pycram.robot_plans import PickUpActionDescription

from pycram.language import CodePlan, ParallelPlan


def run_environmental_monitoring_demo():
    import os
    import threading

    import numpy as np
    import rclpy

    from pycram.datastructures.enums import (
        Arms,
        GripperState,
        VerticalAlignment,
        ApproachDirection,
        MovementType,
    )
    from pycram.datastructures.grasp import GraspDescription
    from pycram.datastructures.pose import PoseStamped
    from pycram.language import SequentialPlan
    from pycram.process_module import simulated_robot
    from pycram.robot_plans import (
        ParkArmsActionDescription,
        SetGripperActionDescription,
        OpenActionDescription,
        MoveTCPMotion,
        PullOutActionDescription,
        CloseActionDescription,
        PlaceActionDescription,
    )
    from semantic_world.adapters.urdf import URDFParser
    from semantic_world.adapters.viz_marker import VizMarkerPublisher
    from semantic_world.pipeline.pipeline import (
        Pipeline,
        FillCollisionWithVisual,
        ReplaceFixedConnectionWithActiveConnectionIfCondition,
    )
    from semantic_world.robots import Tracy
    from semantic_world.spatial_types.spatial_types import TransformationMatrix
    from semantic_world.world_description.connections import (
        FixedConnection,
        RevoluteConnection,
    )
    import pycram.robot_descriptions.tracy_states  # type: ignore
    from semantic_world.world_description.world_entity import Body

    tracy_world = URDFParser.from_file(
        os.path.join(
            os.path.dirname(__file__), "..", "..", "resources", "robots", "tracy.urdf"
        )
    ).parse()
    apartment_world = URDFParser.from_file(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "resources",
            "worlds",
            "chemical_laboratory.urdf",
        )
    ).parse()

    incubator_world = URDFParser.from_file(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "resources",
            "objects",
            "incubator",
            "incubator_correct_rotation.urdf",
        )
    ).parse()


    def condition(body: Body) -> bool:
        if (
            body.parent_connection
            and "door" in body.parent_connection.name.name
            and "handle" not in body.parent_connection.name.name
        ):
            return True
        return False


    incubator_pipeline = Pipeline(
        steps=[
            ReplaceFixedConnectionWithActiveConnectionIfCondition(
                condition=lambda b: condition(b),
                connection_type=RevoluteConnection,
            )
        ]
    )

    incubator_pipeline.apply(incubator_world)

    petri_dish_world = URDFParser.from_file(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "resources",
            "objects",
            "petri_dish",
            "petri_dish.urdf",
        )
    ).parse()

    incubator_world.merge_world(petri_dish_world,
                                FixedConnection(
                                    tray := incubator_world.get_body_by_name("tray"),
                                    petri_dish_world.root,
                                    TransformationMatrix.from_xyz_rpy(
                                        # Petri dish dimensions are ordered incorrectly, i think its a incorrect export from blender
                                        z=min(petri_dish_world.get_body_by_name("petri_dish").visual[0].local_frame_bounding_box.dimensions) / 2,
                                        reference_frame=tray
                                    ),
                                ),
                                handle_duplicates=True,)

    apartment_world.merge_world(
        incubator_world,
        FixedConnection(
            apartment_world.root,
            incubator_world.root,
            TransformationMatrix.from_xyz_rpy(
                0.5, 0.75, 1.25, 0, 0, np.pi, reference_frame=apartment_world.root
            ),
        ),
        handle_duplicates=True,
    )

    apartment_world.merge_world(
        tracy_world,
        FixedConnection(
            apartment_world.root,
            tracy_world.root,
            TransformationMatrix.from_xyz_quat(
                2, 0.75, 0.855, 0, 0, 0, 1, reference_frame=apartment_world.root
            ),
        ),
    )

    pipeline = Pipeline(
        steps=[
            FillCollisionWithVisual(),
        ]
    )

    pipeline.apply(apartment_world)

    tracy = Tracy.from_world(apartment_world)

    node = rclpy.create_node("laboratory_demo")
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    viz = VizMarkerPublisher(world=apartment_world, node=node, visuals_if_available=True)

    print(len(apartment_world.bodies))
    print(len(apartment_world.bodies_with_enabled_collision))
    park_arms = ParkArmsActionDescription([Arms.BOTH])
    park_right_arm = ParkArmsActionDescription([Arms.RIGHT])
    park_left_arm = ParkArmsActionDescription([Arms.LEFT])
    open_left_gripper = SetGripperActionDescription([Arms.LEFT], [GripperState.OPEN])
    open_right_gripper = SetGripperActionDescription([Arms.RIGHT], [GripperState.OPEN])
    close_left_gripper = SetGripperActionDescription([Arms.LEFT], [GripperState.CLOSE])
    close_right_gripper = SetGripperActionDescription([Arms.RIGHT], [GripperState.CLOSE])

    grasp_description = GraspDescription(
        ApproachDirection.FRONT, VerticalAlignment.NoAlignment, True
    )

    original_tray_pose = apartment_world.get_body_by_name("tray").parent_connection.origin_expression
    pullout = PullOutActionDescription(
        apartment_world.get_body_by_name("tray"), [Arms.LEFT], [grasp_description]
    )

    inspect_tray = MoveTCPMotion(
        PoseStamped.from_spatial_type(
            TransformationMatrix.from_xyz_rpy(
                x=0.6,
                y=0.15,
                z=0.7,
                roll=np.pi / 2,
                yaw=-np.pi / 2,
                reference_frame=apartment_world.get_body_by_name("table"),
            ),
        ),
        Arms.LEFT,
        allow_gripper_collision=False,
        movement_type=MovementType.CARTESIAN,
    )

    open_incubator = OpenActionDescription(
        apartment_world.get_body_by_name("Incubator_door_NakedSingularity_handle"),
        [Arms.RIGHT],
    )

    close_incubator = CloseActionDescription(
        apartment_world.get_body_by_name("Incubator_door_NakedSingularity_handle"),
        [Arms.RIGHT],
    )

    place_tray = PlaceActionDescription(
        apartment_world.get_body_by_name("tray"),
        PoseStamped.from_spatial_type(
            original_tray_pose
        ),
        [Arms.LEFT],
    )

    petri_lid_grasp_description = GraspDescription(ApproachDirection.BACK, VerticalAlignment.TOP, False)

    dish_T_lid = apartment_world.get_body_by_name("petri_dish_lid").parent_connection.origin_expression

    lid_pickup = PickUpActionDescription(apartment_world.get_body_by_name("petri_dish_lid"), [Arms.RIGHT], [petri_lid_grasp_description])

    place_lid = PlaceActionDescription(
        apartment_world.get_body_by_name("petri_dish_lid"),
        PoseStamped.from_spatial_type(
            dish_T_lid
        ),
        [Arms.RIGHT],
    )

    plan = SequentialPlan(
        (apartment_world, None),
        tracy,
        park_arms,
        open_left_gripper,
        open_right_gripper,
        open_incubator,
        pullout,
        inspect_tray,
        lid_pickup,
        place_lid,
        place_tray,
        park_left_arm,
        close_incubator,
        park_arms,
    )
    with simulated_robot:
        plan.perform()

    viz._stop_publishing()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    run_environmental_monitoring_demo()
