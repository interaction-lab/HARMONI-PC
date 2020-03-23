import numpy as np
import rospy
import tf
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

n_cols = 848

class DepthRgbCoordinator:
    def __init__(self, height=480, width=848):
        self._height = height
        self._width = width
        rospy.init_node('image_coordinator', anonymous=True)
        rospy.Subscriber('/camera/depth_cloud', PointCloud2, self.handle_depth_image)
        rospy.Subscriber("qtpc/face_frame", Pose2D, self.handle_coordinate)
        rospy.Timer(rospy.Duration(1), self.talker)
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(10.0)
        self._depth_image = None




    def handle_depth_image(self, data):
        self._depth_image = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))

    def handle_coordinate(self, data):
        point = self.get_point(int(data.x), int(data.y))
        self.rotation_matrix(point)
        return point
        
    def rotation_matrix(self, camera_point):
        br = tf.TransformBroadcaster()
        target_frame = "base_link"
        point = PointStamped()
        point.header.frame_id = "Camera"
        point.header.stamp = rospy.Time(0)
        point.point.x = camera_point[0]
        point.point.y = camera_point[1]
        point.point.z = camera_point[2]
        point_target_frame = self.listener.transformPoint(target_frame, point)
        print(str(point_target_frame))
        
        br.sendTransform((point_target_frame.point.x, point_target_frame.point.y, point_target_frame.point.z),
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                           "TargetFrame",
                           "base_link")
        self.rate.sleep()

    def get_point(self,
            pixel_col,
            pixel_row,
    ):
        idx = pixel_row * self._width + pixel_col
        if idx > self._width*self._height:
            raise IndexError("Outside of image index: row '%d', col '%d'" % (pixel_row, pixel_col))
        return self._depth_image[idx]

    def talker(self, timer):
        pass


if __name__ == '__main__':
    try:
        DepthRgbCoordinator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
