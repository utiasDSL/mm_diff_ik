import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

# class Logger2(object):
#     def __init__(self):
#         self.items = []
#         pass
#
#     def _add_pub(self, name, topic, typ):
#         pub = rospy.Publisher(topic, typ, queue_size=1)
#         self.items.append({
#             'data': typ(),
#             'pub':  pub,
#         })
#
#     def publish(self):
#         for item in self.items:
#             item[pub].publish(item['data'])
#         pass


class Logger(object):
    def __init__(self):
        self.force_raw = Vector3()
        self.force = Vector3()
        self.pos_offset = Vector3()

        self.ee_world_des = Vector3()
        self.ee_world_act = Vector3()

        self.ee_arm_des = Vector3()
        self.ee_arm_act = Vector3()

        self.base_des = Vector3()
        self.base_act = Vector3()

        self.time_pub = rospy.Publisher('/log/time', Float64, queue_size=1)

        self.pub_force_raw = rospy.Publisher('/log/force_raw', Vector3, queue_size=1)
        self.pub_force = rospy.Publisher('/log/force', Vector3, queue_size=1)
        self.pub_pos_offset = rospy.Publisher('/log/pos_offset', Vector3, queue_size=1)

        self.pub_ee_world_des = rospy.Publisher('/log/ee_world_des', Vector3, queue_size=1)
        self.pub_ee_world_act = rospy.Publisher('/log/ee_world_act', Vector3, queue_size=1)

        self.pub_ee_arm_des = rospy.Publisher('/log/ee_arm_des', Vector3, queue_size=1)
        self.pub_ee_arm_act = rospy.Publisher('/log/ee_arm_act', Vector3, queue_size=1)

        self.pub_base_des = rospy.Publisher('/log/base_des', Vector3, queue_size=1)
        self.pub_base_act = rospy.Publisher('/log/base_act', Vector3, queue_size=1)

    def publish(self):
        self.time_pub.publish(rospy.get_time())

        self.pub_force_raw.publish(self.force_raw)
        self.pub_force.publish(self.force)
        self.pub_pos_offset.publish(self.pos_offset)

        self.pub_ee_world_des.publish(self.ee_world_des)
        self.pub_ee_world_act.publish(self.ee_world_act)

        self.pub_ee_arm_des.publish(self.ee_arm_des)
        self.pub_ee_arm_act.publish(self.ee_arm_act)

        self.pub_base_des.publish(self.base_des)
        self.pub_base_act.publish(self.base_act)
