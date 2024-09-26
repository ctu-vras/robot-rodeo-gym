import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64

class BoundingBoxDetector:
    def __init__(self):
        self.bounding_boxes = [
            {'id': 0, 'x_min': -7.0, 'x_max': -5, 'y_min': -5.0, 'y_max': -3.0},
            {'id': 1, 'x_min': -5.0, 'x_max': -2.0, 'y_min': -5.0, 'y_max': -3.0},
            {'id': 2, 'x_min': -2.0, 'x_max': 1.0, 'y_min': -5.0, 'y_max': -3.0},
            {'id': 3, 'x_min': 1.0, 'x_max': 5.0, 'y_min': -5.0, 'y_max': -3.0},
            {'id': 4, 'x_min': 5.0, 'x_max': 8.0, 'y_min': -5.0, 'y_max': 4.0},
            {'id': 5, 'x_min': 1.0, 'x_max': 5.0, 'y_min': 2.0, 'y_max': 4.0},
            {'id': 6, 'x_min': -2.0, 'x_max': 1.0, 'y_min': 2.0, 'y_max': 4.0},
            {'id': 7, 'x_min': -5.0, 'x_max': -2.0, 'y_min': 2.0, 'y_max': 4.0},
            {'id': 8, 'x_min': -9.0, 'x_max': -5.0, 'y_min': 2.0, 'y_max': 4.0},
            {'id': 9, 'x_min': -9.0, 'x_max': -7.0, 'y_min': -1.0, 'y_max': 2.0},
            {'id':10, 'x_min': -9.0, 'x_max': -7.0, 'y_min': -4.0, 'y_max': -1.0},
            {'id':11, 'x_min': -9.0, 'x_max': -7.0, 'y_min': -8.0, 'y_max': -4.0},
            {'id':12, 'x_min': -7.0, 'x_max': -4.0, 'y_min': -8.0, 'y_max': -6.0},
            {'id': 13, 'x_min': -4.0, 'x_max': 0.0, 'y_min': -8.0, 'y_max': -6.0},
            {'id': 14, 'x_min': 0.0, 'x_max': 2.0, 'y_min': -8.0, 'y_max': -6.0},
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
    rospy.init_node('area_id_publisher')
    detector = BoundingBoxDetector()
    rospy.spin()
