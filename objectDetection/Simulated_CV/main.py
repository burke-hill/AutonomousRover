import  cv2
import  math
import  rclpy
from    rclpy.node          import Node
from    sensor_msgs.msg     import Image, LaserScan
from    cv_bridge           import CvBridge

'''
	This file performs all object detection work for the rover's simulation. Due to hardware performance limitations, our team could not use
	popular object detection methods like YOLO, resulting in our independently-authored detection algorithm.
'''


bridge = CvBridge()

distance_info = []

# working image dimensions: 640, 640; 480, 480; 320, 320
img_size = 640

# Camera can see lidar 152-207
left_fov, right_fov = 152, 207

def suppress_bbox_intersection(arr):
    # Given collection of xy min and xy max tuples, merge to get collections of absolute min and max xy tuples
    # in -> [[0, 0, 50, 50], [10, 10, 30, 30]]
    # out -> ans = [[0, 0, 50, 50]]

    '''
        Add each array to dict. At the end of suppression, create new final array. loop through original array. if item in original array not in dict,
        dont add to final array.

    '''
    d = {}
    for i in arr:
        d[str(i)] = i
    new_arr = sorted(arr, key=lambda x: (x[2] - x[0]) * (x[3] - x[1]), reverse=True)
    ans = []

    for i in new_arr:
        if not ans:
            # Add the largest bounding box
            ans.append(i)
            continue
        ixmin, iymin, ixmax, iymax = i
        flag = False
        for j in ans:
            # Check if current box is inside any of the larger bounding boxes
            jxmin, jymin, jxmax, jymax = j
            if ixmin >= jxmin and ixmax <= jxmax and iymin >= jymin and iymax <= jymax:
                # if the current bounding box is entirely consumed by a larger box, turn flag to True.
                flag = True
        if flag:
            # If bounding box is not the largest, delete from dict
            del d[str(i)]
        if not flag:
            # If bounding box is the largest, add to final list
            ans.append(i)
    new_arr = []
    for i in ans:
        if str(i) in d:
            new_arr.append(i)
    return new_arr

class Camera_subscriber(Node):

    def __init__(self):
        # All subscribers

        super().__init__('camera_subscriber')
	    
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, \
		    history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
	
        # Lidar subscriber
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile=qos_policy)
        self.lidar_sub

        # Camera subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/depth_camera/image_raw',
            self.camera_callback,
            qos_profile=qos_policy)
        self.image_sub
    
    def lidar_callback(self, msg):
        # Collects positional data of objects around the rover

        global distance_info

        obstacles = {'open': [], 'blocked': []}
        inf = True if math.isinf(msg.ranges[0]) else False
        small, large = 0, None

        # Construct intervals of which a path is open or blocked in the 360 degrees around the rover
        for i in range(len(msg.ranges)):
            if math.isinf(msg.ranges[i]) != inf:
                large = i - 1
                status = 'open' if inf == True else 'blocked'
                obstacles[status].append((small, large))
                small = i
                inf = not inf

        status = 'open' if inf == True else 'blocked'
        obstacles[status].append((small, len(msg.ranges) - 1))

        # (min distance, min degree, min degree distance, max degree, max degree distance)
        distances = []
        for i in obstacles['blocked']:
            ranges = [msg.ranges[j] for j in range(i[0], i[1])]
            if not ranges: 
                ranges = [100, -1]
            distances.append((min(ranges),                                      # min distance
                            i[0],                                               # min degree
                            msg.ranges[i[0]],                                   # min degree distance
                            i[1],                                               # max degree 
                            msg.ranges[i[1]]))                                  # max degree distance

        distance_info = distances
        
        #print(distances)
        #print(obstacles)

    def camera_callback(self, data):
        # Uses a combination of Lidar positional data and Computer Vision to draw bounding boxes around obstacles as well as
        # label their width and distance relative to the position of the rover

        global distance_info, img_size

        # Create grey scale image to find contours
        img = cv2.resize(bridge.imgmsg_to_cv2(data, "mono8"), (img_size, img_size))

        # THRESH_BINARY_INV rather THRESH_BINARY as the inverse removes the camera detecting the entire frame as one large object
        binary = cv2.threshold(img, 153, 255, cv2.THRESH_BINARY_INV)[1]

        # Retrieve all contours using RETR_LIST
        contours = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contours = contours[0] if len(contours) == 2 else contours[1]

        # Create BGR image to display to the user in color
        img = cv2.resize(bridge.imgmsg_to_cv2(data, "bgr8"), (img_size, img_size))

        # Sort the contours in order as they will be display to the image (left to right)
        # cv2.contourArea

        sorted_contours = filter_contours(sorted(contours, key = get_x, reverse = True))
        
        arr = []

        for contour in sorted_contours:
            x, y, w, h = cv2.boundingRect(contour)
            arr.append([x, y, x + w, y + h])
        arr = suppress_bbox_intersection(arr)
        
        for contour in arr:
            x,y,w,h = contour
            cv2.rectangle(img, (x, y), (w, h), (0, 255, 0), 2)
        
        # Display the contours created in the grey scale image over the color image
        #cv2.drawContours(img, sorted_contours, -1, (255,255,0), 2)
        
        # Fill with -1 if no detected objects
        if not distance_info:
            distance_info = []
        else:
            # Only keep the objects detected by lidar that are displayed within the camera frame
            dis = []
            for obj in distance_info:
                if obj[3] < left_fov or obj[1] > right_fov:
                    continue
                else:
                    dis.append(obj)
            distance_info = sorted(dis, key= lambda x: x[1])

        while len(distance_info) < len(sorted_contours):
            distance_info.append([-1,-1,-1,-1,-1])

        for (i,c) in enumerate(sorted_contours):

            x,y,w,h = cv2.boundingRect(c)
            # Given an angle and two sides of a triangle using lidar (farthest left and right distances of the detected object), calculate the third side (width)
            a, c, B = distance_info[i][2], distance_info[i][4], distance_info[i][3] - distance_info[i][1]

            width = (a**2+c**2-2*a*c*math.cos(abs(B)))**0.5

            # Add label
            '''
            cv2.putText(img, text = f'w={width:.2F} d={distance_info[i][0]:.2f}', org = (x,y-10),
                    fontFace = cv2.FONT_HERSHEY_DUPLEX, fontScale = 0.75, color = (232,48,166),
                    thickness = 2, lineType = cv2.LINE_AA)
            '''
        cv2.imshow("IMAGE", img)
        cv2.waitKey(1)

def filter_contours(contours):
    # Filter out unncessarily detected objects such as pebbles
    approved_contours = []
    for (i,c) in enumerate(contours):
        x,y,w,h = cv2.boundingRect(c)
        if w * h >= 1000:
            approved_contours.append(c)
    return approved_contours

def get_x(contour):
    x, y, w, h = cv2.boundingRect(contour)
    return x

rclpy.init(args=None)
camera_subscriber = Camera_subscriber()
rclpy.spin(camera_subscriber)
rclpy.shutdown()
