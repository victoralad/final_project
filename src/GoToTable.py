import rospy

from geometry_msgs.msg import Twist

pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
rospy.init_node('go_forward')
print "Node done"

speed = 0.5#rospy.get_param("~speed", 0.5)
turn = 1.0#rospy.get_param("~turn", 1.0)
x = 1
y = 0
z = 0
th = 0
status = 0

for i in range(32):
    twist = Twist()
    twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
    pub.publish(twist)
    rospy.sleep(0.1)