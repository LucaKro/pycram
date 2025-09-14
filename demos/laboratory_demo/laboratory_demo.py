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
        PickUpActionDescription,
        OpenActionDescription,
        ReachToPickUpActionDescription,
        MoveTCPMotion,
        PullOutAction,
        PullOutActionDescription,
        CloseActionDescription,
        PlaceActionDescription,
    )
    from semantic_world.adapters.mesh import STLParser
    from semantic_world.adapters.urdf import URDFParser
    from semantic_world.adapters.viz_marker import VizMarkerPublisher
    from semantic_world.datastructures.prefixed_name import PrefixedName
    from semantic_world.pipeline.pipeline import (
        Pipeline,
        FillCollisionWithVisual,
        ReplaceFixedConnectionWithActiveConnectionIfCondition,
    )
    from semantic_world.robots import Tracy
    from semantic_world.spatial_types.spatial_types import TransformationMatrix
    from semantic_world.views.views import Dresser
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
    open_left_gripper = SetGripperActionDescription([Arms.LEFT], [GripperState.OPEN])
    open_right_gripper = SetGripperActionDescription([Arms.RIGHT], [GripperState.OPEN])
    close_left_gripper = SetGripperActionDescription([Arms.LEFT], [GripperState.CLOSE])
    close_right_gripper = SetGripperActionDescription([Arms.RIGHT], [GripperState.CLOSE])

    grasp_description = GraspDescription(
        ApproachDirection.FRONT, VerticalAlignment.NoAlignment, True
    )
    pullout = PullOutActionDescription(
        apartment_world.get_body_by_name("tray"), [Arms.LEFT], [grasp_description]
    )

    inspect_tray = MoveTCPMotion(
        PoseStamped.from_spatial_type(
            TransformationMatrix.from_xyz_rpy(
                x=0.75,
                z=0.2,
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
            TransformationMatrix.from_xyz_rpy(
                x=1.2,
                y=0.5,
                z=0.17,
                roll=np.pi / 2,
                reference_frame=apartment_world.get_body_by_name("table"),
            ),
        ),
        [Arms.LEFT],
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
        place_tray,
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
