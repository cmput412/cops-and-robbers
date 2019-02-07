#!/usr/bin/env python

import rospy
import smach
import smach_ros
import numpy
import time
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Joy
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import Led, BumperEvent

g_obstacle_stopping = None


class DrivingForward(smach.State):
    def __init__(self):
        global g_obstacle_stopping
        smach.State.__init__(self, outcomes=['Obstacle', 'Done', 'Sleep'])
        self.g_range_ahead = 1
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 1 )
        self.bumper = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        self.rate = rospy.Rate(10)
        self.button = rospy.Subscriber('/joy', Joy, self.button_callback)
        self.stop = 0
        self.hit = 0

    def bumper_callback(self,msg):
        if msg.state == 1:
            self.hit = 1

    def button_callback(self,msg):
        if msg.buttons[1] == 1:
            self.stop =1

    def scan_callback(self, msg):
        self.g_range_ahead = msg.ranges[len(msg.ranges)/2]
        if math.isnan(self.g_range_ahead):
            self.g_range_ahead = 0

    def execute(self, userdata):
        g_obstacle_stopping = 1
        rospy.loginfo('Executing state DrivingForward')
        state_change_time = rospy.Time.now() + rospy.Duration(30)


        while not rospy.is_shutdown():
            if self.stop:
                self.stop = 0
                return 'Sleep'
                
            if self.hit:
                twist = Twist()
                twist.linear.x = -0.2
                self.cmd_vel_pub.publish(twist)
                time.sleep(1)
                twist = Twist(0)
                self.cmd_vel_pub.publiah(twist)
                return 'Obstacle'


            if (self.g_range_ahead < g_obstacle_stopping):
                print(self.g_range_ahead, " ", g_obstacle_stopping)
                return 'Obstacle'

            twist = Twist()
            twist.linear.x = 0.6

            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        return 'Done'


class Turning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['BacktoForwardDriving', 'Done','Sleep'])
        self.g_range_ahead = 1
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 1 )
        self.rate = rospy.Rate(10)
        self.button = rospy.Subscriber('/joy', Joy, self.button_callback)
        self.stop = 0

    def button_callback(self,msg):
        if msg.buttons[1] == 1:
            self.stop =1

    def scan_callback(self, msg):
        self.g_range_ahead =  msg.ranges[len(msg.ranges)/2]

    def execute(self, userdata):
        direction = numpy.random.randint(0,2) # 0 turn left, 1 turn right
        rospy.loginfo('Executing state Turning')
        state_change_time = rospy.Time.now() + rospy.Duration(5)

        while not rospy.is_shutdown():

            if self.stop:
                self.stop = 0
                return 'Sleep'

            twist = Twist()
            if direction == 0:
                twist.angular.z = 2
            else:
                twist.angular.z = -2 
                
            rospy.loginfo(self.g_range_ahead)
            rospy.loginfo(g_obstacle_stopping)
            if (self.g_range_ahead > g_obstacle_stopping ):
                return 'BacktoForwardDriving'

            
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        return 'Done'


class StartStateRobber(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['BacktoForwardDriving','Done', 'Sleep'])
        self.cmd_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 1 )
        self.rate = rospy.Rate(10)
        self.button = rospy.Subscriber('/joy', Joy, self.button_callback)
        self.stop = 0

    def button_callback(self,msg):
        if msg.buttons[1] == 1:
            self.stop =1

    def execute(self, userdata):
        rospy.loginfo('Executing robber start state')
        state_change_time = rospy.Time.now() + rospy.Duration(1)
        pi = 3.14

        while not rospy.is_shutdown():
            twist = Twist()
            while state_change_time > rospy.Time.now():
                if self.stop:
                    self.stop = 0
                    return 'Sleep'
                twist.linear.x = -0.1
                self.cmd_vel_pub.publish(twist)
            twist.linear.x = 0
            self.cmd_vel_pub.publish(twist)
            state_change_time = rospy.Time.now() + rospy.Duration(5)
            twist = Twist()
            while state_change_time > rospy.Time.now():
                twist.angular.z = pi/7
                self.cmd_vel_pub.publish(twist)
                if self.stop:
                    self.stop = 0
                    return 'Sleep'

            twist = Twist()
            self.cmd_vel_pub.publish(twist)


            return 'BacktoForwardDriving'
        return 'Done'

class SleepState(smach.State):
    def __init__(self):
        self.START = 0
        self.MODE = 1
        smach.State.__init__(self, outcomes=['Robber','Cop','Done'])
        self.led = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size = 1 )
        self.rate = rospy.Rate(10)
        self.button = rospy.Subscriber('/joy', Joy, self.button_callback)
        self.end = 0


    def button_callback(self,msg):
        rospy.loginfo('in callback')
        if msg.buttons[0] == 1:
            self.START = 1
        elif msg.buttons[3] == 1:
            if self.MODE == 1:
                self.MODE = 0
                rospy.loginfo('Cop mode')
                self.led.publish(1)
            else:
                self.MODE = 1 
                rospy.loginfo('Robber mode')
                self.led.publish(3)

    def execute(self, userdata):
        rospy.loginfo('Executing sleep state')

        while not rospy.is_shutdown():
            if self.end:
                return 'Done'
            if self.START == 1:
                rospy.loginfo('should run')
                if self.MODE ==1:
                    self.START = 0
                    return 'Robber'
                elif self.MODE == 0:
                    self.START = 0
                    return 'Cop'

        return 'Done'



class StartStateCop(smach.State):
    def __init__(self):
        self.START = 1
        smach.State.__init__(self, outcomes=['Done', 'Sleep'])
        self.rate = rospy.Rate(10)

        ### Subscribers/ Publishers
        self.button = rospy.Subscriber('joy', Joy, self.button_callback)
        self.vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size = 5 )
        self.laser_sub = rospy.Subscriber('scan', LaserScan, self.laser_callback, queue_size=1)

        ### Non-paramter attributes
        self.followed_angle = None #angle of object we are tracking
        self.twist = Twist() #Twist msg
        self.minimum_range = None #minimum dist to object we are tracking 
        self.scan_init = False
        self.timer = rospy.get_time()
        self.stop = 0
        self.end = 0

        ### Parameters (may be fine tuned)
        self.max_turn_speed = 1
        self.follow_distance = 0.6
        self.follow_distance_err = 0.05 #thresholding paramter
        self.lin_x_scale = 2
        self.angular_scale = 4
        self.image_recog_offset = 0.01

    def odom_callback(self,msg):
        self.orientation = msg.pose.pose.orientation
        quaternion = (self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        self.yaw = euler[2]

    def laser_callback(self,msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment

        self.process_scan(msg.ranges, angle_min, angle_max, angle_increment)

    def button_callback(self,msg):
        if msg.buttons[1] == 1:
            self.stop = 1

    def execute(self, userdata):
        rospy.loginfo('Executing cop state')

        while not rospy.is_shutdown():
            if self.stop:
                self.stop = 0
                return 'Sleep'
            # Check movement thresholds
            if self.minimum_range != None:
                
                if ( self.follow_distance - self.follow_distance_err ) > self.minimum_range:
                    #Too close  

                    # Compute the linear component of the movement
                    self.twist.linear.x = (self.minimum_range - self.follow_distance) * self.lin_x_scale #negative

                elif ( self.follow_distance + self.follow_distance_err ) < self.minimum_range:
                    self.twist.linear.x = (self.minimum_range - self.follow_distance) * self.lin_x_scale #positive

                # #angular speed
                self.twist.angular.z =  self.followed_angle * self.angular_scale 
                
                
            # # Publish the movement command
            self.vel_pub.publish(self.twist)
        return 'Done'

    def process_scan(self, scan, angle_min, angle_max, angle_increment):
        """Sets angle of tracked object, as well as range of followed object"""

        minimum_range = 10000 # arbitray big number
        scan = list(scan)
        counter = 0 #position of minimum point within array

        #find minimum range point
        for i in range(len(scan)):
            if scan[i] < minimum_range and (not math.isnan(scan[i])) :
                minimum_range = scan[i]
                counter = i

        #next check if this is the robot to be followed
        size, objStart_ind, objEnd_ind  =  self.obj_point_range(minimum_range, counter, scan, angle_min, angle_max, angle_increment)
        
        if size < 0.3 and size > 0.1  and (objStart_ind > 40) and (objEnd_ind < 595) : # of object is of correct size
            self.followed_angle = angle_min + (counter * angle_increment) #gives the position of the object in radians
            self.minimum_range = minimum_range

    def obj_point_range(self, minimum_range, scan_coord, scan, angle_min, angle_max, angle_increment):
        #Return the coord range 
        #look right
        objStart = scan_coord
        objStart_dist = minimum_range
        last_seen = minimum_range

        for i in range(scan_coord, 1, -1):
            if not math.isnan(scan[i]) and not math.isnan(scan[i-1]):
                if abs(last_seen - scan[i - 1]) < self.image_recog_offset:
                        objEnd_dist = scan[i - 1]
                        objStart = i - 1
                        last_seen = scan[i-1]
                else:
                    break
            
        objEnd = scan_coord
        objEnd_dist = minimum_range
        last_seen = minimum_range

        for i in range(scan_coord, len(scan)-1, 1):
            if not math.isnan(scan[i]) and not math.isnan(scan[i+1]):
                if abs(last_seen - scan[i + 1]) < self.image_recog_offset:
                    objEnd_dist = scan[i + 1]
                    objEnd = i + 1
                    last_seen = scan[i + 1]

                else:
                    break

        #convert to radians

        objStart_ind = objStart
        objEnd_ind = objEnd

        objStart = angle_min + objStart * angle_increment
        objEnd = angle_min + objEnd * angle_increment

        dist = ( objStart_dist + objEnd_dist) / 2 #avg dist away
        size = math.sin( abs(objStart - objEnd) / 2 ) * 2 * dist

        return size, objStart_ind, objEnd_ind

def main():
    rospy.init_node('robber')
    global g_obstacle_stopping
    g_obstacle_stopping = 1 #fetch_param('~obstacle_stopping',1.5)
    sm = smach.StateMachine(outcomes = ['DoneProgram'])
    sm.set_initial_state(['SleepState'])

    with sm:
        
        smach.StateMachine.add('DrivingForward', DrivingForward(),
                                        transitions = {'Obstacle': 'Turning',
                                                        'Done': 'DoneProgram',
                                                        'Sleep':'SleepState'})

        smach.StateMachine.add('Turning', Turning(),
                                        transitions = {'BacktoForwardDriving': 'DrivingForward',
                                                        'Done' : 'DoneProgram',
                                                        'Sleep':'SleepState'})

        smach.StateMachine.add('StartStateRobber', StartStateRobber(),
                                        transitions = {'BacktoForwardDriving': 'DrivingForward',
                                                        'Done' : 'DoneProgram',
                                                        'Sleep': 'SleepState'})

        smach.StateMachine.add('SleepState', SleepState(),
                                        transitions = {'Robber': 'StartStateRobber',
                                                        'Cop': 'StartStateCop',
                                                        'Done' : 'DoneProgram'})
        smach.StateMachine.add('StartStateCop', StartStateCop(),
                                        transitions = {'Done' : 'DoneProgram',
                                                        'Sleep':'SleepState'})
 
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    

    sis.start()
    
    outcome = sm.execute() 
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

'''def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print " parameter [%s] not defined, Defaulting to %.3f" % (name,default)
        return default'''



if __name__ == '__main__':
    main()
