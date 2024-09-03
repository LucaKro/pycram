from pycram.process_module import simulated_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.resolver.action.SPARQL import SPARQL
from IPython.display import display, HTML, clear_output

def start_cutting(obj="obo:FOODON_00003415", technique="cut:Slicing"):
    display(HTML('<img src="https://i.gifer.com/XVo6.gif" alt="Hourglass animation" width="50">'))



    _technique = technique.split(":", 1)[1]
    #######################################################################################
    #rospy.loginfo("Querying the ontology for the cutting task")
    #query_resolver = SPARQL()

    #repititions_ont = query_resolver.repetitions(technique, obj)
    #rospy.loginfo("The repetition for the task is: " + repititions_ont)

    #position_ont, position_name = query_resolver.position(technique, obj)
    #rospy.loginfo("The start position is: " + position_ont)

    # cutting_tool_ont, cutting_tool = query_resolver.get_cutting_tool(technique, obj)
    # rospy.loginfo("The tool to cut with is:  " + cutting_tool_ont)
    # rospy.loginfo("Queried all necessary information from the ontology")

    #######################################################################################

    drawer_island_surface = Pose([2.4, 2, 0.95], [0, 0, -1, -1])
    right_to_sink_surface = Pose([2.4, 2.3, 0.95], [0, 0, -1, -1])
    drawer_island_surface_board = Pose([2.45, 2, 0.95], [0, 0, -1, -1])
    pose = drawer_island_surface

    board = Object("board", "board", "board.stl", drawer_island_surface_board)
    board.set_color([0.4, 0.2, 0.06, 1])
    objects = [(None, None), ('apple', "obo:FOODON_03301710"), ('avocado', "obo:FOODON_00003600"),
               ('banana', "obo:FOODON_00004183"), ('citron', "obo:FOODON_03306596"),
               ('cucumber', "obo:FOODON_00003415"),
               ('kiwi', "obo:FOODON_00004387"), ('lemon', "obo:FOODON_03301441"), ('lime', "obo:FOODON_00003661"),
               ('orange', "obo:FOODON_03309832"), ('peach', "obo:FOODON_03315502"), ('tomato', "obo:FOODON_03309927")]

    for id_, name in objects:
        if name == obj:
            obj_path = id_ + ".stl"
            obj = id_

    obj_to_cut = Object(obj, "object_to_be_cut", obj_path, pose)
    colors = {
        "orange": (1, 0.75, 0, 1),
        "cucumber": (0, 1, 0, 1),
        "banana": (1, 1, 0, 1),
        "lemon": (1, 1, 0, 1),
        "citron": (1, 1, 0, 1),
        "lime": (0.75, 1.0, 0.0, 1),
        "apple": (1, 0, 0, 1),
        "tomato": (1, 0, 0, 1),
        "peach": (1.0, 0.8, 0.64, 1),
        "kiwi": (0.76, 0.88, 0.52, 1),
        "avocado": (0.0, 0.5, 0.0, 1),
    }

    obj_to_cut.set_color(colors[obj])
    length, width, height = obj_to_cut.get_object_dimensions()
    obj_to_cut.set_pose(Pose([pose.position.x, pose.position.y, pose.position.z + height / 1.5], pose.orientation))

    with simulated_robot:
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        location_pose = Pose([1.7, 2, 0])
        looking_pose = Pose([2.5, 2, 0.97])
        NavigateAction([location_pose]).resolve().perform()


        if robot_description.name == "Armar6":
            knife_pose = Pose([2.2049586673391935, 1.40084467778416917, 1.0229705326966067],
                              [0, 0, 1, 1])
        elif robot_description.name == "tiago_dual":
            # knife_pose = Pose([2.1, 1.715, 0.9],
            #                   [0, 0, 0, 1])
            knife_pose = Pose([2.0, 1.73, 0.785],
                              [0.05, 0.999, 0.05, 0.999])
        else:
            knife_pose = Pose([2.0449586673391935, 1.5384467778416917, 1.2229705326966067],
                              [0.14010099565491793, -0.7025332835765593, 0.15537176280408957, 0.6802046102510538])

        cutting_tool = Object("knife", "cutting_tool", "butter_knife.stl", knife_pose)

        tool_frame = robot_description.get_tool_frame("right")
        BulletWorld.current_bullet_world.robot.attach(object=cutting_tool, link=tool_frame)

        if robot_description.name == "Armar6":
            MoveGripperMotion(motion="close", gripper="right").resolve().perform()
            location_pose = Pose([1.6, 2.5, 0])
            NavigateAction([location_pose]).resolve().perform()
        elif robot_description.name == "tiago_dual":
            MoveGripperMotion(motion="close", gripper="right").resolve().perform()
            MoveTorsoAction([0.33]).resolve().perform()
            location_pose = Pose([1.5, 2.25, 0])
            NavigateAction([location_pose]).resolve().perform()

        LookAtAction([looking_pose]).resolve().perform()
        status, object_dict = DetectAction(technique='specific', object_type="object_to_be_cut").resolve().perform()
        if status:
            for key, value in object_dict.items():
                detected_object = object_dict[key]
                bigknife_BO = BelieveObject(names=["knife"]).resolve()
                CuttingAction(detected_object, bigknife_BO, ["right"], _technique).resolve().perform()
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        clear_output(wait=True)
        rospy.loginfo("Cutting task completed!")
        obj_to_cut.remove()
        BulletWorld.current_bullet_world.remove_vis_axis()
#
# start_cutting("obo:FOODON_03301710", "soma:Cutting")
# start_cutting("obo:FOODON_03301710", "soma:Slicing")
# start_cutting("obo:FOODON_03301710", "cut:Halving")
#
# #avocado
# start_cutting("obo:FOODON_00003600", "soma:Cutting")
# start_cutting("obo:FOODON_00003600", "soma:Slicing")
# start_cutting("obo:FOODON_00003600", "cut:Halving")
#
# #banana
# start_cutting("obo:FOODON_00004183", "soma:Cutting")
# start_cutting("obo:FOODON_00004183", "soma:Slicing")
# start_cutting("obo:FOODON_00004183", "cut:Halving")
#
#
# #citron
# start_cutting("obo:FOODON_03306596", "soma:Cutting")
# start_cutting("obo:FOODON_03306596", "soma:Slicing")
# start_cutting("obo:FOODON_03306596", "cut:Halving")


# print("cut")
# start_cutting("obo:FOODON_00003415", "soma:Cutting")
# print("slice")
# start_cutting("obo:FOODON_00003415", "soma:Slicing")
# print("halving")
# start_cutting("obo:FOODON_00003415", "cut:Halving")

# #kiwi
# start_cutting("obo:FOODON_00004387", "soma:Cutting")
# start_cutting("obo:FOODON_00004387", "soma:Slicing")
# start_cutting("obo:FOODON_00004387", "cut:Halving")

#
# #lemon
# start_cutting("obo:FOODON_03301441", "soma:Cutting")
# start_cutting("obo:FOODON_03301441", "soma:Slicing")
# start_cutting("obo:FOODON_03301441", "cut:Halving")
#
# #lime
# start_cutting("obo:FOODON_00003661", "soma:Cutting")
# start_cutting("obo:FOODON_00003661", "soma:Slicing")
# start_cutting("obo:FOODON_00003661", "cut:Halving")
#
# #orange
# start_cutting("obo:FOODON_03309832", "soma:Cutting")
# start_cutting("obo:FOODON_03309832", "soma:Slicing")
# start_cutting("obo:FOODON_03309832", "cut:Halving")
#
#
# #peach
# start_cutting("obo:FOODON_03315502", "soma:Cutting")
# start_cutting("obo:FOODON_03315502", "soma:Slicing")
# start_cutting("obo:FOODON_03315502", "cut:Halving")


# #peach
# start_cutting("obo:FOODON_03309927", "soma:Cutting")
# start_cutting("obo:FOODON_03309927", "soma:Slicing")
# start_cutting("obo:FOODON_03309927", "cut:Halving")


# [INFO] [1710184677.654084]: Querying the ontology for the cutting task
# [INFO] [1710184680.265410]: The repetition for the task is: 0.05
# [INFO] [1710184680.405835]: The start position is: http://www.ease-crc.org/ont/food_cutting#halving_position
# [INFO] [1710184680.543266]: The tool to cut with is:  http://www.ease-crc.org/ont/food_cutting#ParingKnife
# [INFO] [1710184680.547092]: Queried all necessary information from the ontology
# [INFO] [1710184688.615853]: Cutting task completed!