#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 2

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.use_simulator = rospy.get_param('~simulator_used')
        self.light_classifier = TLClassifier(simulator = self.use_simulator)
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.prev_state = TrafficLight.UNKNOWN
        self.prev_waypoint = -1
        self.state_count = 0
        self.car_position_prev = 0
        self.waypoint_traffic_light_previous = []

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

        light_wp, state = self.process_traffic_lights()
        
        if state == TrafficLight.RED:
            rospy.loginfo("Detected RED light")
        elif state == TrafficLight.YELLOW:
            rospy.loginfo("Detected YELLOW light")
        elif state == TrafficLight.GREEN:
            rospy.loginfo("Detected GREEN light")
        else:
            rospy.loginfo("No traffic light detected")
        
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if(state != TrafficLight.UNKNOWN):
            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.prev_state = self.state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.prev_waypoint = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))           
            else:
                self.upcoming_red_light_pub.publish(Int32(self.prev_waypoint))
            self.state_count += 1
    

    def l2_dist_tl(self, tl_pose, wp_pose):
        """Calculate euqlidian distance between two points
        """
        dist = math.sqrt( pow(tl_pose[0] - wp_pose.x, 2 ) + pow(tl_pose[1] - wp_pose.y, 2 ))
        return dist 


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        # format from: waypoint_loader.py
        waypoints_List = [(Pt.pose.pose.position.x, Pt.pose.pose.position.y) for Pt in self.waypoints.waypoints]
        waypoint_idx = -1

        x1 = pose.position.x
        y1 = pose.position.y

        min_Dist = float('inf')

        for idx in range(len(waypoints_List)):

            x2 = waypoints_List[idx][0]
            y2 = waypoints_List[idx][1]

            delta_x = x1-x2
            delta_y = y1-y2

            distance = math.sqrt(delta_x*delta_x + delta_y*delta_y)

            if(distance < min_Dist):
                min_Dist = distance
                waypoint_idx = idx


        return waypoint_idx


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if(not self.has_image):
            self.prev_light_loc = None
            return False
        
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        return self.light_classifier.get_classification(cv_image)


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        light = None
        stop_line_positions = self.config['stop_line_positions']                
                
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            if self.use_simulator == True:
                if car_position is not None:
                    self.car_position_prev = car_position
        else:
            return -1, TrafficLight.UNKNOWN 

        waypoints_traffic_light = []
        if self.waypoints is not None:
            waypoints_position = self.waypoints
            for i in range(len(stop_line_positions)):                
                waypoint_closest_idx = None
                waypoints = waypoints_position.waypoints
                min_Dist = self.l2_dist_tl(stop_line_positions[i], waypoints[0].pose.pose.position)
                for j, point in enumerate(waypoints):
            	    dist = self.l2_dist_tl(stop_line_positions[i], point.pose.pose.position)
                    if dist < min_Dist:
                        waypoint_closest_idx = j
                        min_Dist = dist
                waypoints_traffic_light.append(waypoint_closest_idx)		
            self.waypoint_traffic_light_previous = waypoints_traffic_light
        else:
            waypoints_traffic_light = self.waypoint_traffic_light_previous

        if(self.car_position_prev > max(waypoints_traffic_light)):
            traffic_light_waypoint_id = min(waypoints_traffic_light)
        else:
            light_car_dist = [None]*len(waypoints_traffic_light)
            traffic_light_waypoint_id = 99999
            for i in range(len(light_car_dist)):
                light_car_dist[i] = waypoints_traffic_light[i] - self.car_position_prev
                if light_car_dist[i] > 0 and light_car_dist[i] < traffic_light_waypoint_id:
                    traffic_light_waypoint_id = light_car_dist[i]
            traffic_light_waypoint_id += self.car_position_prev

        light = stop_line_positions[waypoints_traffic_light.index(traffic_light_waypoint_id)]

        traffic_light_dist = self.l2_dist_tl(light, self.waypoints.waypoints[self.car_position_prev].pose.pose.position)
        
        if light:
            if (traffic_light_dist < 100.0):
                state = self.get_light_state(light)
                return traffic_light_waypoint_id, state
            else:
                return -1, TrafficLight.UNKNOWN
        else:            
            self.waypoints = None
            return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
