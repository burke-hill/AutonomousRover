import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import json
from rclpy.node import Node
import time

'''
	This file subscribes to /scan to check if lidar is functional. While /scan is functional, its data is being published to /laser.
	Once /scan stops, /laser is filled with lidar-replicated data provided by the OAK-D.
'''

qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, \
    history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

last_msg = time.time()

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = fill_msg()
        self.publisher_.publish(msg)
        self.i += 1
        self.get_logger().info(f'Publishing: {self.i}')

class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
	    
	
        # Lidar subscriber
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile=qos_policy)
        self.lidar_sub
    
    def lidar_callback(self, msg):
        print(msg)
        if not msg:
            minimal_publisher = MinimalPublisher()
            rclpy.spin(minimal_publisher)

def fill_msg():
    obj = LaserScan()
    file = open('lidarstuff.json')
    data = json.load(file)
    file.close()
    node = MinimalPublisher()
    curr_time = node.get_clock().now().to_msg()
    obj.header.stamp = curr_time
    obj.header.frame_id = 'laser'
    obj.angle_min = float(data['angle_min'])
    obj.angle_max = float(data['angle_max'])
    obj.angle_increment = float(data['angle_increment'])
    obj.time_increment = float(data['time_increment'])
    obj.scan_time = float(data['scan_time'])
    obj.range_min = float(data['range_min'])
    obj.range_max = float(data['range_max'])
    obj.ranges = [float(i) for i in data['ranges']]
    return obj

def get_topics():
    rclpy.init()
    node = rclpy.create_node('list_all_topics_example')
    print(node.get_topic_names_and_types(no_demangle=True))

def sub_callback(msg):
    global last_msg
    print('sub_callback')
    last_msg = time.time()
    publisher.publish(msg)

def no_callback():
    if time.time() - last_msg > 4:
        print('lidar failed :(')
        print('switching to oak-d')
        laser.destroy_timer(check_scan)
        laser.destroy_subscription(subscriber)
        timer = laser.create_timer(0.5, timer_callback)

def timer_callback():
    print('timer_callback')
    msg = fill_msg()
    publisher.publish(msg)

def laser_sub_callback(msg):
    print('laser_sub_callback')

rclpy.init()
laser = rclpy.create_node('lidar_stuff')

publisher = laser.create_publisher(LaserScan, 'laser', qos_policy)
laser_subscriber = laser.create_subscription(LaserScan, 'laser', laser_sub_callback, qos_policy)

subscriber = laser.create_subscription(LaserScan, 'scan', sub_callback, qos_policy)
check_scan = laser.create_timer(0.5, no_callback)
timer = None
rclpy.spin(laser)
laser.destroy_node()
rclpy.shutdown()
