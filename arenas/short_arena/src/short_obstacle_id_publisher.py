import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64

class BoundingBoxDetector:
    def __init__(self):
        self.bounding_boxes = [
            #{'id': 0, 'x_min': -7.0, 'x_max': -5, 'y_min': -5.0, 'y_max': -3.0},
            {'id': 1, 'x_min': -5.0, 'x_max': -2.0, 'y_min': -5.0, 'y_max': -3.0},
            {'id': 2, 'x_min': -2.0, 'x_max': 1.0, 'y_min': -5.0, 'y_max': -3.0},
            {'id': 3, 'x_min': 1.0, 'x_max': 5.0, 'y_min': -5.0, 'y_max': -3.0},
            #{'id': 4, 'x_min': 5.0, 'x_max': 8.0, 'y_min': -5.0, 'y_max': 4.0}
        ]

        self.pub_bounding_box_id = rospy.Publisher('/obstacle_id', Int64, queue_size=10)
        self.sub_odom = rospy.Subscriber('/ground_truth_odom', Odometry, self.odom_cb)

    def odom_cb(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        known = False
        for box in self.bounding_boxes:
            if (box['x_min'] <= x <= box['x_max']) and (box['y_min'] <= y <= box['y_max']):
                self.pub_bounding_box_id.publish(box['id'])
                known = True
                break

        if not known:
            self.pub_bounding_box_id.publish(-1)

if __name__ == '__main__':
    rospy.init_node('obstacle_id_publisher')
    detector = BoundingBoxDetector()
    rospy.spin()
