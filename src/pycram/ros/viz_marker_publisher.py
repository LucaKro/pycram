import atexit
import logging
import threading
import time
from enum import Enum

import numpy as np
import tf
from geometry_msgs.msg import Vector3, Quaternion, Point
from std_msgs.msg import ColorRGBA

from pycram.bullet_world import BulletWorld, Object
from visualization_msgs.msg import MarkerArray, Marker
import rospy
import urdf_parser_py
from tf.transformations import quaternion_from_euler

from pycram.pose import Transform, Pose


class VizMarkerPublisher:
    """
    Publishes an Array of visualization marker which represent the situation in the Bullet World
    """

    def __init__(self, topic_name="/pycram/viz_marker", interval=0.1):
        """
        The Publisher creates an Array of Visualization marker with a Marker for each link of each Object in the Bullet
        World. This Array is published with a rate of interval.

        :param topic_name: The name of the topic to which the Visualization Marker should be published.
        :param interval: The interval at which the visualization marker should be published, in seconds.
        """
        self.topic_name = topic_name
        self.interval = interval

        self.pub = rospy.Publisher(self.topic_name, MarkerArray, queue_size=10)

        self.thread = threading.Thread(target=self._publish)
        self.kill_event = threading.Event()
        self.main_world = BulletWorld.current_bullet_world if not BulletWorld.current_bullet_world.is_shadow_world else BulletWorld.current_bullet_world.world_sync.world

        self.thread.start()
        atexit.register(self._stop_publishing)

    def _publish(self) -> None:
        """
        Constantly publishes the Marker Array. To the given topic name at a fixed rate.
        """
        while not self.kill_event.is_set():
            marker_array = self._make_marker_array()

            self.pub.publish(marker_array)
            time.sleep(self.interval)


    def _make_marker_array(self) -> MarkerArray:
        """
        Creates the Marker Array to be published. There is one Marker for link for each object in the Array, each Object
        creates a name space in the visualization Marker. The type of Visualization Marker is decided by the collision
        tag of the URDF.

        :return: An Array of Visualization Marker
        """
        marker_array = MarkerArray()
        objnew = False
        for obj in BulletWorld.current_bullet_world.objects:
            if obj.name in ["floor"]:
                continue
            if obj.name == "hsrb":
                continue
            if not obj.name in ["environment", "pr2"]:
                objnew = True
            for link in obj.links.keys():
                geom = obj.link_to_geometry[link]
                if not geom:
                    continue
                msg = Marker()
                msg.header.frame_id = "simulated/map"
                msg.ns = obj.name
                msg.id = obj.links[link]
                msg.type = Marker.MESH_RESOURCE
                msg.action = Marker.ADD
                link_pose = obj.get_link_pose(link).to_transform(link)
                if hasattr(obj, "urdf_object"):
                    if obj.urdf_object.link_map[link].collision.origin:
                        link_origin = Transform(obj.urdf_object.link_map[link].collision.origin.xyz,
                                                list(quaternion_from_euler(
                                                    *obj.urdf_object.link_map[link].collision.origin.rpy)))
                    else:
                        link_origin = Transform()
                else:
                    link_origin = Transform()
                link_pose_with_origin = link_pose * link_origin
                msg.pose = link_pose_with_origin.to_pose().pose
                color = [1, 1, 1, 1]
                if obj.name == "board":
                    color = [0.4, 0.2, 0.06, 1]
                elif obj.type == "object_to_be_cut":
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
                    color = colors[obj.name]
                else:
                    color = [1, 1, 1, 1] if obj.links[link] == -1 else obj.get_color(link)

                msg.color = ColorRGBA(*color)
                msg.lifetime = rospy.Duration(1)

                if type(geom) == urdf_parser_py.urdf.Mesh:
                    msg.type = Marker.MESH_RESOURCE
                    msg.mesh_resource = "file://" + geom.filename
                    if hasattr(geom, "scale") and geom.scale:
                        msg.scale = Vector3(geom.scale[0], geom.scale[1], geom.scale[2])
                    else:
                        msg.scale = Vector3(1, 1, 1)
                    msg.mesh_use_embedded_materials = True
                elif type(geom) == urdf_parser_py.urdf.Cylinder:
                    msg.type = Marker.CYLINDER
                    msg.scale = Vector3(geom.radius * 2, geom.radius * 2, geom.length)
                elif type(geom) == urdf_parser_py.urdf.Box:
                    msg.type = Marker.CUBE
                    msg.scale = Vector3(*geom.size)
                elif type(geom) == urdf_parser_py.urdf.Sphere:
                    msg.type == Marker.SPHERE
                    msg.scale = Vector3(geom.radius * 2, geom.radius * 2, geom.radius * 2)
                elif obj.customGeom:
                    msg.type = Marker.CUBE
                    x = geom["size"][0]
                    y = geom["size"][1]
                    z = geom["size"][2]
                    msg.scale = Vector3(x, y, z)
                if objnew:
                    #color = obj.get_color()
                    #msg.color = ColorRGBA(*color)
                    objnew = False
                marker_array.markers.append(msg)
        return marker_array

    def _stop_publishing(self) -> None:
        """
        Stops the publishing of the Visualization Marker update by setting the kill event and collecting the thread.
        """
        self.kill_event.set()
        self.thread.join()


class ManualMarkerPublisher:
    def __init__(self, topic_name='/pycram/manual_marker'):
        """
        Class to manually add and remove marker of objects and poses.

        :param topic_name: Name of the marker topic
        """
        self.marker_array_pub = rospy.Publisher(topic_name, MarkerArray, queue_size=10)

        self.marker_array = MarkerArray()
        self.marker_overview = {}
        self.current_id = 0

        self.bulletworld_object = None
        self.color = None
        self.pose = None
        self.name = None

        self.thread = threading.Thread(target=self._publish)

    def publish(self, pose: Pose, color=None, bulletworld_object=None, name=None):
        """
        Publish a pose or an object into the MarkerArray.
        Priorities to add an object if possible

        :param pose: Pose of the marker
        :param color: Color of the marker if no object is given
        :param bulletworld_object: Object to add as a marker
        :param name: Name of the marker
        """
        if color is None:
            color = 'pink'

        self.name = name
        self.pose = pose
        self.color = color
        self.bulletworld_object = bulletworld_object

        self.thread.start()
        rospy.loginfo("Publishing pose visualization")
        self.thread.join()
        rospy.logdebug("Stopped")

    def _publish(self):
        stop_thread = False
        duration = 1
        frequency = 0.2
        start_time = time.time()

        while not stop_thread:
            if time.time() - start_time > duration:
                stop_thread = True

            if self.bulletworld_object is None:
                self._publish_pose(name=self.name, pose=self.pose, color=self.color)
            else:
                self._publish_object(name=self.name, pose=self.pose, bw_object=self.bulletworld_object)

            rospy.sleep(frequency)

    def _publish_pose(self, name, pose, color):
        """
        Publish a Pose as a marker

        :param name: Name of the marker
        :param pose: Pose of the marker
        :param color: Color of the marker
        """

        if name is None:
            name = 'pose_marker'

        if name in self.marker_overview.keys():
            self._update_marker(self.marker_overview[name], new_pose=pose)
            return

        color_rgba = Colors.get_color(color)

        self.create_marker(name=name, marker_type=Marker.ARROW, marker_pose=pose,
                           marker_scales=(0.1, 0.01, 0.01), color_rgba=color_rgba)
        pose = Pose([1,1,1], [0,0,1,1])
        self.create_marker(name="name", marker_type=Marker.ARROW, marker_pose=pose,
                           marker_scales=(0.1, 0.01, 0.01), color_rgba=color_rgba)
        self.marker_array_pub.publish(self.marker_array)

    def _publish_object(self, name, pose, bw_object):
        """
        Publish an Object as a marker

        :param name: Name of the marker
        :param pose: Pose of the marker
        :param bw_object: Bulletworld object for the marker
        """

        bw_real = bw_object.resolve()

        if name is None:
            name = bw_real.name

        if name in self.marker_overview.keys():
            self._update_marker(self.marker_overview[name], new_pose=pose)
            return

        path = bw_real.bullet_world_object.urdf_object.links[0].visual.geometry.filename

        self.create_marker(name=name, marker_type=Marker.MESH_RESOURCE, marker_pose=pose,
                           path_to_resource=path)

        self.marker_array_pub.publish(self.marker_array)
        rospy.loginfo("Marker array published")

    def create_marker(self, name, marker_type, marker_pose: Pose, marker_scales=(1.0, 1.0, 1.0),
                      color_rgba=ColorRGBA(1.0, 1.0, 1.0, 1.0), path_to_resource=None):
        """
        Create a Marker and add it to the MarkerArray

        :param name: Name of the Marker
        :param marker_type: Type of the marker to create
        :param marker_pose: Pose of the marker
        :param marker_scales: individual scaling of the markers axes
        :param color_rgba: Color of the marker as RGBA
        :param path_to_resource: Path to the resource of a Bulletworld object
        """

        frame_id = "simulated/map"

        new_marker = Marker()
        new_marker.id = self.current_id
        new_marker.header.frame_id = frame_id
        new_marker.header.stamp = rospy.Time.now()
        new_marker.type = marker_type
        new_marker.action = Marker.ADD
        new_marker.pose.position.x = marker_pose.pose.position.x
        new_marker.pose.position.y = marker_pose.pose.position.y
        new_marker.pose.position.z = marker_pose.pose.position.z
        new_marker.pose.orientation = marker_pose.pose.orientation #Quaternion(0.0, 0.0, 0.0, 1.0)
        new_marker.scale.x = marker_scales[0]
        new_marker.scale.y = marker_scales[1]
        new_marker.scale.z = marker_scales[2]
        new_marker.color.a = color_rgba.a
        new_marker.color.r = color_rgba.r
        new_marker.color.g = color_rgba.g
        new_marker.color.b = color_rgba.b

        if path_to_resource is not None:
            new_marker.mesh_resource = 'file://' + path_to_resource

        self.marker_array.markers.append(new_marker)
        self.marker_overview[name] = new_marker.id
        self.current_id += 1

    def _update_marker(self, marker_id, new_pose):
        """
        Update an existing marker to a new pose

        :param marker_id: id of the marker that should be updated
        :param new_pose: Pose where the updated marker is set
        """

        # Find the marker with the specified ID
        for marker in self.marker_array.markers:
            if marker.id == marker_id:
                # Update successful
                marker.pose = new_pose
                rospy.logdebug(f"Marker {marker_id} updated")
                self.marker_array_pub.publish(self.marker_array)
                return True

        # Update was not successful
        rospy.logwarn(f"Marker {marker_id} not found for update")
        return False

    def remove_marker(self, bw_object=None, name=None):
        """
        Remove a marker by object or name

        :param bw_object: Object which marker should be removed
        :param name: Name of object that should be removed
        """

        if bw_object is not None:
            bw_real = bw_object.resolve()
            name = bw_real.name

        if name is None:
            logging.error('No name for object')
            return

        marker_id = self.marker_overview.pop(name)

        for marker in self.marker_array.markers:
            if marker.id == marker_id:
                marker.action = Marker.DELETE

        self.marker_array_pub.publish(self.marker_array)
        self.marker_array.markers.pop(marker_id)
        self.marker_array_pub.publish(self.marker_array)

    def clear_all_marker(self):
        """
        Clear all existing markers
        """
        for marker in self.marker_array.markers:
            marker.action = Marker.DELETE

        self.marker_overview = {}
        self.marker_array_pub.publish(self.marker_array)


class AxisMarkerPublisher:
    def __init__(self, topic='/pycram/axis_marker', frame_id='simulated/map'):

        self.marker_pub = rospy.Publisher(topic, MarkerArray, queue_size=10)

        self.marker_array = MarkerArray()
        self.marker_overview = {}
        self.current_id = 0
        self.frame_id = frame_id

        self.length = None
        self.duration = None
        self.pose = None
        self.axis = None
        self.color = None

        self.thread = threading.Thread(target=self._publish)


    def publish(self, pose, duration=15.0, length=0.1, name=None):
        """
        Publish a MarkerArray with given pose and axis.
        Duration, length and color of the line are optional.

        :param pose: Starting position of the Line
        :param axis: Orientation for the Line
        :param duration: Duration of the marker
        :param length: Length of the line
        :param color: Color of the line if it should be personalized
        """

        self.name = name
        self.pose = pose
        self.duration = duration
        self.length = length

        self._create_line(self.pose, AxisIdentifier.Y.value, self.duration, self.length, Colors.get_color('green'))
        self._create_line(self.pose, AxisIdentifier.X.value, self.duration, self.length, Colors.get_color('red'))
        self._create_line(self.pose, AxisIdentifier.Z.value, self.duration, self.length, Colors.get_color('blue'))


        self.thread.start()
        # rospy.loginfo("Publishing axis visualization")
        self.thread.join()
        # rospy.logdebug("Stopped Axis visualization")

    def _publish(self):
        if self.name in self.marker_overview.keys():
            self._update_marker(self.marker_overview[self.name], new_pose=self.pose)
            return

        stop_thread = False
        duration = 1
        frequency = 0.2
        start_time = time.time()

        while not stop_thread:
            if time.time() - start_time > duration:
                stop_thread = True

            # Publish the MarkerArray
            self.marker_pub.publish(self.marker_array)

            rospy.sleep(frequency)
    def _get_color(self, axis):
        """
        Get the color of the given Axis

        :param axis: Used axis
        """
        if axis == AxisIdentifier.X.value:
            axis_color = Colors.get_color('red')
        elif axis == AxisIdentifier.Y.value:
            axis_color = Colors.get_color('green')
        elif axis == AxisIdentifier.Z.value:
            axis_color = Colors.get_color('blue')
        else:
            logging.error(f'Axis {str(axis)} is not valid')
            axis_color = Colors.get_color('red')

        return axis_color

    def _create_line(self, pose, axis, duration, length, color):
        """
        Create a line marker to add to the marker array.

        :param pose: Starting pose of the line
        :param axis: Axis along which the line is set
        :param duration: Duration of the line marker
        :param length: Length of the line
        :param color: Optional color for the Line
        """

        def normalize_quaternion(q):
            norm = np.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
            if norm > 0:
                return q.x / norm, q.y / norm, q.z / norm, q.w / norm
            return q.x, q.y, q.z, q.w

        def quaternion_multiply(q1, q2):
            x1, y1, z1, w1 = q1
            x2, y2, z2, w2 = q2
            return (
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2,
                w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2,
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            )

        def rotate_axis_by_quaternion(axis, quaternion):
            # Normalize the quaternion to avoid distortions
            qx, qy, qz, qw = normalize_quaternion(quaternion)

            # Represent axis as quaternion (x, y, z, 0)
            axis_quat = (*axis, 0)

            # Quaternion components
            q = (qx, qy, qz, qw)

            # Compute the inverse (conjugate for unit quaternion)
            q_conjugate = (-qx, -qy, -qz, qw)

            # Rotate the vector
            rotated_quat = quaternion_multiply(quaternion_multiply(q, axis_quat), q_conjugate)

            # The rotated vector is the vector part of the resulting quaternion
            return rotated_quat[:3]

        # Create a line marker for the axis
        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = f'axis_visualization_{self.current_id}'
        line_marker.id = self.current_id
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.01  # Line width
        line_marker.color = color
        line_marker.lifetime = rospy.Duration(duration)

        # Start point at the position specified by the pose (translation part)
        start_point = Point()
        start_point.x = pose.position.x
        start_point.y = pose.position.y
        start_point.z = pose.position.z

        quaternion = pose.orientation
        rotated_axis = rotate_axis_by_quaternion(axis, quaternion)

        # Calculate the end point by adding the rotated axis vector (scaled by length)
        end_point = Point()
        end_point.x = pose.position.x + (rotated_axis[0] * length)
        end_point.y = pose.position.y + (rotated_axis[1] * length)
        end_point.z = pose.position.z + (rotated_axis[2] * length)

        line_marker.points.append(start_point)
        line_marker.points.append(end_point)

        # Add the line marker to the MarkerArray
        self.marker_array.markers.append(line_marker)
        self.marker_overview[f"{self.name}_{self.current_id}"] = line_marker.id
        self.current_id += 1

    def _update_marker(self, marker_id, new_pose):
        """
        Update an existing marker to a new pose

        :param marker_id: id of the marker that should be updated
        :param new_pose: Pose where the updated marker is set
        """

        # Find the marker with the specified ID
        for marker in self.marker_array.markers:
            if marker.id == marker_id:
                # Update successful
                marker.pose = new_pose
                rospy.logdebug(f"Marker {marker_id} updated")
                self.marker_pub.publish(self.marker_array)
                return True

        # Update was not successful
        rospy.logwarn(f"Marker {marker_id} not found for update")
        return False


class AxisIdentifier(Enum):
    X = (1, 0, 0)
    Y = (0, 1, 0)
    Z = (0, 0, 1)


class Colors:
    @staticmethod
    def get_color(color):
        """
        Retrieve a color based on name

        :param color: Name of a color
        """

        converted_color = ColorRGBA()
        value = lambda x: x / 255

        if color == 'pink':
            return ColorRGBA(r=value(255), g=value(0), b=value(255), a=1.0)
        elif color == 'black':
            return ColorRGBA(r=value(0), g=value(0), b=value(0), a=1.0)
        elif color == 'white':
            return ColorRGBA(r=value(255), g=value(255), b=value(255), a=1.0)
        elif color == 'red':
            return ColorRGBA(r=value(255), g=value(0), b=value(0), a=1.0)
        elif color == 'green':
            return ColorRGBA(r=value(0), g=value(255), b=value(0), a=1.0)
        elif color == 'blue':
            return ColorRGBA(r=value(0), g=value(0), b=value(255), a=1.0)
        elif color == 'yellow':
            return ColorRGBA(r=value(255), g=value(255), b=value(0), a=1.0)
        elif color == 'cyan':
            return ColorRGBA(r=value(0), g=value(255), b=value(255), a=1.0)

        return converted_color
