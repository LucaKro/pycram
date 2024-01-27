import rospy
from speech_processing.msg import message_to_robot, message_from_robot, message_objects_in_use, dict_object
from time import time

from pycram.fluent import Fluent


class InterruptClient:
    def __init__(self):
        self.minor_interrupt = Fluent()

        # rospy.init_node('nlp_node')
        self.nlp_frequency = 10.0
        self.nlp_timestamp = 0.0
        self.command_queue = Fluent(value=[])
        self.objects_in_use = []

        # Initialize subscribers
        self.nlp_major_sub = rospy.Subscriber("/robot_major_interruption", message_to_robot, self.nlp_major_sub_cb)
        self.nlp_minor_sub = rospy.Subscriber("/robot_minor_interruption", message_to_robot, self.nlp_minor_sub_cb)
        self.from_robot_pub = rospy.Publisher("/from_robot", message_from_robot, queue_size=10)
        self.objects_in_use_pub = rospy.Publisher("/objects_in_use", message_objects_in_use, queue_size=10)

    def nlp_major_sub_cb(self, major_interruption_msg):
        self.handle_interruption_msg(major_interruption_msg, "major")

    def nlp_minor_sub_cb(self, minor_interruption_msg):
        self.handle_interruption_msg(minor_interruption_msg, "minor")

    def handle_interruption_msg(self, interruption_msg, category):
        update_every_secs = 1.0 / self.nlp_frequency
        current_time = time()

        if current_time - self.nlp_timestamp > update_every_secs:
            command_data = {
                category: {
                    "command": interruption_msg.command,
                    "age": interruption_msg.age,
                    "confidence": interruption_msg.confidence,
                    "add_object": interruption_msg.add_object,
                    "del_object": interruption_msg.del_object
                }
            }
            self.modify_objects_in_use(interruption_msg.add_object, interruption_msg.del_object)
            self.command_queue.get_value().append(command_data)
            self.command_queue.pulse()
            self.minor_interrupt.set_value(True)
            print(self.command_queue.get_value())
            self.nlp_timestamp = current_time

    def next_command(self):
        if self.command_queue:
            return self.command_queue.get_value().pop(0)
        else:
            return None

    def get_objects_in_use(self):
        return self.objects_in_use

    def publish_from_robot(self, step, interruptable, object_info, move_arm, move_base, current_location,
                           destination_location):
        obj_msg = dict_object(
            type=object_info.type,
            color=object_info.color,
            name=object_info.name,
            location="not supported yet",  # object_info.location,
            size=object_info.size
        )

        from_robot_msg = message_from_robot(
            step=step,
            interruptable=interruptable,
            object=obj_msg,
            move_arm=move_arm,
            move_base=move_base,
            current_location=current_location,
            destination_location=destination_location
        )

        self.from_robot_pub.publish(from_robot_msg)

    def modify_objects_in_use(self, add_object_info, remove_object_info):

        [self.objects_in_use.append(obj) for obj in add_object_info if obj not in self.objects_in_use]
        [self.objects_in_use.remove(obj) for obj in remove_object_info if obj in self.objects_in_use]
        print("minor interrupt")

        self.objects_in_use_pub.publish(message_objects_in_use(objects=self.objects_in_use))


def test(test):
    print(test)


if __name__ == "__main__":
    nlp_node = InterruptClient()
    nlp_node.command_queue.whenever(test)
    rospy.spin()
