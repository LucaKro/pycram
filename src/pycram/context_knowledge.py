from pycram.designators.location_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose

breakfast_objects_apart = {"milk": {"type": "milk", "model": "milk.stl", "pose": [2.5, 2, 1.02], "color": [1, 0, 0, 1],
                                    "default_location": "island_countertop", "typeloc": "No"},
                           "breakfast_cereal": {"type": "breakfast_cereal", "model": "breakfast_cereal.stl",
                                                "pose": [2.4, 2.5, 1.05], "color": [0, 1, 0, 1],
                                                "default_location": "island_countertop", "typeloc": "No"},
                           "jeroen_cup": {"type": "jeroen_cup", "model": "jeroen_cup.stl", "pose": [2.5, 2.2, 0.95],
                                          "color": [1, 0.4, 0.4, 1], "default_location": "island_countertop",
                                          "typeloc": "No"},
                           "spoon": {"type": "spoon", "model": "spoon.stl", "pose": [2.5, 2.2, 0.85],
                                     "color": [0, 0, 1, 1], "default_location": "cabinet10_drawer_top",
                                     "typeloc": "acces"},
                           }
handles = {"cabinet10_drawer_top": "handle_cab10_t", "cabinet3_door_top_left": "handle_cab3_door_top",
           "cabinet7_door_bottom_left": "handle_cab7"}
# breakfast_objects_kitchen = {
# # "milk": {"type": "milk", "model": "milk.stl", "pose": [2.5, 2, 1.02], "color": [1, 0, 0, 1],
# #                               "default_location": "iai_fridge_door_handle"},
# #                      "breakfast_cereal": {"type": "breakfast_cereal", "model": "breakfast_cereal.stl",
# #                                           "pose": [2.5, 2.5, 1.05], "color": [0, 1, 0, 1],
# #                                           "default_location": "oven_area_area_right_drawer_handle"},
#                      "spoon": {"type": "spoon", "model": "spoon.stl", "pose": [1.5, 0.9, 0.750], "color": [0, 0, 1, 1],
#                                "default_location": "sink_area_left_middle_drawer_handle"},
#                       # "bowl": {"type": "bowl", "model": "bowl.stl", "pose": [2.38, 2.2, 1.02], "color": [1, 1, 0, 1],
#                       #          "default_location": "kitchen_island_surface"}
#                                }
clean_up_objects = {
    "milk": {"type": "milk", "model": "milk.stl", "pose": [4.81671219471512, 3.2802720926294127, 0.8029613686135095],
             "color": [1, 0, 0, 1], "default_location": "table_area_main"},
    "breakfast_cereal": {"type": "breakfast_cereal", "model": "breakfast_cereal.stl",
                         "pose": [4.824226348321504, 4.6595481306892985, 0.821496474677939], "color": [0, 1, 0, 1],
                         "default_location": "table_area_main"},
    "jeroen_cup": {"type": "jeroen_cup", "model": "jeroen_cup.stl",
                   "pose": [4.659999806835296, 4.139999841440455, 0.8119486117362978], "color": [0, 0, 1, 1],
                   "default_location": "table_area_main"},
    "bowl": {"type": "bowl", "model": "bowl.stl", "pose": [4.6599997387605585, 4.339999616031064, 0.8166916942596436],
             "color": [1, 1, 0, 1], "default_location": "table_area_main"}}

cutting_big = {
    "knife": {"type": "cutting_tool", "model": "big-knife.stl", "pose": [2.55, 2.15, 0.85],
                              "color": [0.75, 0.75, 0.75, 1], "default_location": "cabinet10_drawer_top",
                              "typeloc": "acces"},

                 }
cutting_small = {
    "knife": {"type": "cutting_tool", "model": "/objects/butter_knife.stl", "pose": [2.55, 2.25, 0.85],
                 "color": [0.75, 0.75, 0.75, 1], "default_location": "cabinet10_drawer_top", "typeloc": "acces"},
                 }


class ContextConfig:
    environment_object = None
    spoon_ = None

    def __init__(self, context_name, enviornment_name, objects_info):
        self.object_to_be_cut = None
        self.cutting_tool = None
        self.context_name = context_name
        self.objects_info = objects_info  # A dictionary of object names and their types
        self.environment_name = enviornment_name  # Name of the environment model
        self.spawn_objects()
        self.handles = handles


    def spawn_objects(self):
        apart = Object("environment", ObjectType.ENVIRONMENT, self.environment_name)
        apart.set_color([0.5, 0.5, 0.5, 0.7])
        self.environment_object = apart
        # Function to spawn objects in the simulation environment
        for obj_name, obj_info in self.objects_info.items():
            if isinstance(obj_info['pose'], Pose):
                obj = Object(obj_name, obj_info['type'], obj_info['model'], pose=obj_info['pose'])
            else:
                obj = Object(obj_name, obj_info['type'], obj_info['model'], pose=Pose(obj_info['pose']))
            if obj_info['type'] == "spoon" or obj_info['type'] == "cutting_tool":
                if self.environment_name == "apartment-small.urdf":
                    self.environment_object.attach(obj, 'cabinet10_drawer_top')
                else:
                    self.environment_object.attach(obj, 'sink_area_left_middle_drawer_handle')
                self.spoon_ = obj
            if obj_info['type'] == "object_to_be_cut":
                self.object_to_be_cut = obj
            if obj_info['type'] == "cutting_tool":
                self.cutting_tool = obj
            obj.set_color(obj_info['color'])

    def search_locations(self, object_name):
        # Function to provide likely locations of objects within this context
        if object_name in self.objects_info:
            return (self.objects_info[object_name].get('default_location', 'Unknown'),
                    self.objects_info[object_name].get('typeloc', 'Unknown'))
        return 'Unknown'

    def get_all_objects(self):
        return self.objects_info.keys()

    def get_object_type(self, object_name):
        return self.objects_info[object_name].get('type', 'Unknown')

    def get_cutting_objects(self):
        return self.object_to_be_cut

    def get_cutting_tool(self):
        return self.cutting_tool

    def get_handle(self, location):
        try:
            self.handles[location]
        except KeyError:
            pass
            return location
        return self.handles[location]


def generate_context(context_name, enviornment_name, robot_name):
    if context_name == "breakfast" and enviornment_name == "apartment-small.urdf":
        Object(robot_name, ObjectType.ROBOT, robot_name + ".urdf", pose=Pose([1, 2, 0]))
        return ContextConfig(context_name, enviornment_name, breakfast_objects_apart)
    if context_name == "cutting-big" and enviornment_name == "apartment-small.urdf":
        Object(robot_name, ObjectType.ROBOT, robot_name + ".urdf", pose=Pose([1, 2, 0]))
        return ContextConfig(context_name, enviornment_name, cutting_big)
    elif context_name == "cutting-small" and enviornment_name == "apartment-small.urdf":
        Object(robot_name, ObjectType.ROBOT, robot_name + ".urdf", pose=Pose([1, 2, 0]))
        return ContextConfig(context_name, enviornment_name, cutting_small)
    elif context_name == "breakfast" and enviornment_name == "kitchen-small.urdf":
        Object(robot_name, ObjectType.ROBOT, robot_name + ".urdf", pose=Pose([0, 0, 0]))
        return ContextConfig(context_name, enviornment_name, breakfast_objects_kitchen)
    elif context_name == "clean_up":
        Object(robot_name, ObjectType.ROBOT, robot_name + ".urdf", pose=Pose([0, 0, 0]))
        return ContextConfig(context_name, enviornment_name, clean_up_objects)
