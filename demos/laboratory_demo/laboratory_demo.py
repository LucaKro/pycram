import os

import numpy as np
import rclpy
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
from semantic_world.world_description.world_entity import Body

from pycram.ros import set_logger_level
import pycram.robot_descriptions.tracy_states  # type: ignore
from pycram.datastructures.enums import (
    Arms,
    GripperState,
    VerticalAlignment,
    ApproachDirection,
    MovementType,
    LoggerLevel,
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
from pycram.robot_plans import PickUpActionDescription


def load_laboratory_world():
    """
    Loads the laboratory world with tracy and an incubator containing a petri dish.
    The incubator door is replaced with a revolute joint to allow opening it.
    The petri dish is placed on the incubator tray.
    Tracy is placed in front of the table in the laboratory.
    """
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
            "incubator.urdf",
        )
    ).parse()

    def joint_replace_condition(body: Body) -> bool:
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
                condition=lambda b: joint_replace_condition(b),
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

    incubator_world.merge_world(
        petri_dish_world,
        FixedConnection(
            tray := incubator_world.get_body_by_name("tray"),
            petri_dish_world.root,
            TransformationMatrix.from_xyz_rpy(
                # Petri dish dimensions are ordered incorrectly, i think its a incorrect export from blender
                z=min(
                    petri_dish_world.get_body_by_name("petri_dish")
                    .visual[0]
                    .local_frame_bounding_box.dimensions
                )
                / 2,
                reference_frame=tray,
            ),
        ),
        handle_duplicates=True,
    )

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

    return apartment_world


def run_environmental_monitoring_demo():
    set_logger_level(LoggerLevel.FATAL)
    node = rclpy.create_node("laboratory_demo")

    viz = VizMarkerPublisher(
        world=(apartment_world := load_laboratory_world()), node=node, visuals_if_available=True
    )

    plan = SequentialPlan(
        (apartment_world, None),
        Tracy.from_world(apartment_world),
        park_both_arms := ParkArmsActionDescription([Arms.BOTH]),
        SetGripperActionDescription([Arms.BOTH], [GripperState.OPEN]),
        OpenActionDescription(
            apartment_world.get_body_by_name("Incubator_door_NakedSingularity_handle"),
            [Arms.RIGHT],
        ),
        PullOutActionDescription(
            apartment_world.get_body_by_name("tray"),
            [Arms.LEFT],
            [
                GraspDescription(
                    ApproachDirection.FRONT, VerticalAlignment.NoAlignment, True
                )
            ],
        ),
        # Hold tray in front of camera to inspect it
        MoveTCPMotion(
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
        ),
        PickUpActionDescription(
            apartment_world.get_body_by_name("petri_dish_lid"),
            [Arms.RIGHT],
            [GraspDescription(ApproachDirection.BACK, VerticalAlignment.TOP, False)],
        ),
        PlaceActionDescription(
            apartment_world.get_body_by_name("petri_dish_lid"),
            PoseStamped.from_spatial_type(
                apartment_world.get_body_by_name(
                    "petri_dish_lid"
                ).parent_connection.origin_expression
            ),
            [Arms.RIGHT],
        ),
        PlaceActionDescription(
            apartment_world.get_body_by_name("tray"),
            PoseStamped.from_spatial_type(
                apartment_world.get_body_by_name(
                    "tray"
                ).parent_connection.origin_expression
            ),
            [Arms.LEFT],
        ),
        ParkArmsActionDescription([Arms.LEFT]),
        CloseActionDescription(
            apartment_world.get_body_by_name("Incubator_door_NakedSingularity_handle"),
            [Arms.RIGHT],
        ),
        park_both_arms,
    )
    with simulated_robot:
        plan.perform()

    viz._stop_publishing()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    run_environmental_monitoring_demo()
