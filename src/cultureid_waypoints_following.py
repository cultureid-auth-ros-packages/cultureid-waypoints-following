#!/usr/bin/env python

import threading
import rospy
import actionlib
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, PointStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from tf import TransformListener
import tf
import math
import rospkg
import csv
import time

### Global variables

# All waypoints
waypoints = []

# 0 if this waypoint is not an intermediate point, 1 otherwise;
# The robot does not stop at intermediate waypoints, and waits at the others
waypoints_types = []

# Read waypoints from this file
input_file_path = ""

# Write waypoints to this file
output_file_path = ""

################################################################################
################################################################################
class FollowPath(State):

    ############################################################################
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])

        self.frame_id = rospy.get_param('~goal_frame_id','map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id','odom')
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint')
        self.duration = rospy.get_param('~wait_duration', 1.0)
        self.go_to_next_waypoint_trigger_topic = this_node_name + "/" + rospy.get_param('~go_to_next_waypoint_trigger_topic', 'go_to_next_waypoint_trigger')
        self.stopped_at_waypoint_topic = this_node_name + "/" + rospy.get_param('~stopped_at_waypoint_topic', 'stopped_at_waypoint')
        self.stopped_at_waypoint_alert_pub = rospy.Publisher(self.stopped_at_waypoint_topic, Empty, queue_size=1)


        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listener.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.distance_tolerance = rospy.get_param('waypoint_distance_tolerance', 0.5)

    ############################################################################
    def execute(self, userdata):
        global waypoints
        global waypoints_types

        waypoint_counter = 0

        # Execute waypoints each in sequence
        for waypoint in waypoints.poses:

            # Break if preempted
            if waypoints == []:
                rospy.loginfo('The waypoint queue has been reset.')
                break

            waypoint_counter = waypoint_counter + 1

            # Otherwise publish next waypoint as goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                    (waypoint.pose.position.x, waypoint.pose.position.y))
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal(goal)

            #This is the loop for the robot near a certain GOAL point.
            distance = 10
            while(distance > self.distance_tolerance):
                time.sleep(self.duration) # To prevent from flooding the cpu
                now = rospy.Time.now()
                self.listener.waitForTransform(self.frame_id, self.base_frame_id, now, rospy.Duration(4.0))
                trans,rot = self.listener.lookupTransform(self.frame_id,self.base_frame_id, now)
                distance = math.sqrt(pow(waypoint.pose.position.x-trans[0],2)+pow(waypoint.pose.position.y-trans[1],2))

            # Wait for the robot to navigate to the waypoint
            #self.client.wait_for_result()

            # Wait for a message publication in
            # 'go_to_next_waypoint_trigger_topic'
            # before moving on to the next waypoint.
            # The last waypoint signifies a de facto path_complete state on its own
            #if waypoint_counter < len(waypoints.poses) and waypoints_types[waypoint_counter-1] == 1:
            if waypoints_types[waypoint_counter-1] == 1:
                empty_msg = Empty()
                self.stopped_at_waypoint_alert_pub.publish(empty_msg)
                rospy.loginfo("Reached target waypoint")
                rospy.loginfo("Currently waiting for msg in '%s' to go to next waypoint" % self.go_to_next_waypoint_trigger_topic)
                rospy.loginfo("To do so issue 'rostopic pub %s std_msgs/Empty -1'" % self.go_to_next_waypoint_trigger_topic)
                rospy.wait_for_message(self.go_to_next_waypoint_trigger_topic, Empty)
                rospy.loginfo("msg received in '%s'; next waypoint underway" % self.go_to_next_waypoint_trigger_topic)

        return 'success'



################################################################################
################################################################################
class GetPath(State):

    ############################################################################
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])

        # Topics
        self.rviz_waypoints_ready_topic = this_node_name + "/" + rospy.get_param('~rviz_waypoints_ready_topic', 'rviz_waypoints_ready');
        self.file_waypoints_ready_topic = this_node_name + "/" + rospy.get_param('~file_waypoints_ready_topic', 'file_waypoints_ready');
        self.waypoints_reset_topic = this_node_name + "/" + rospy.get_param('~waypoints_reset_topic', 'waypoints_reset');
        self.waypoints_vis_topic = this_node_name + "/" + rospy.get_param('~waypoints_visualisation_topic', 'waypoints_viz');

        # Create publisher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.poseArray_publisher = rospy.Publisher(self.waypoints_vis_topic, PoseArray, queue_size=1)

        # For self-publishing the required message; added 01/10/2020
        #self.empty_msg_pub = rospy.Publisher(self.rviz_waypoints_ready_topic, Empty, queue_size=1)

        # Start thread to listen for reset messages to clear the waypoint queue
        def wait_for_path_reset():
            """thread worker function"""
            global waypoints
            while not rospy.is_shutdown():
                data = rospy.wait_for_message(self.waypoints_reset_topic, Empty)
                rospy.loginfo('Received path RESET message')
                self.initialize_path_queue()
                rospy.sleep(3) # Wait 3 seconds because `rostopic echo` latches
                               # for three seconds and wait_for_message() in a
                               # loop will see it again.
        reset_thread = threading.Thread(target=wait_for_path_reset)
        reset_thread.start()

    ############################################################################
    def convert_Path_to_PoseArray(self, waypoints):
        """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
        poses = PoseArray()
        poses.header.frame_id = rospy.get_param('~goal_frame_id','map')
        poses.poses = [pose.pose for pose in waypoints.poses]
        return poses

    ############################################################################
    def initialize_path_queue(self):
        global waypoints
        waypoints = [] # the waypoint queue
        # publish empty waypoint queue as pose array so that you can see them the change in rviz, etc.
        #self.poseArray_publisher.publish(self.convert_Path_to_PoseArray(waypoints))

    ############################################################################
    def execute(self, userdata):
        global waypoints
        global output_file_path
        self.initialize_path_queue()
        self.path_ready = False

        # Start thread to listen for when the path is ready (this function will end then)
        # Also will save the clicked path to pose.csv file
        def wait_for_path_ready():
            """thread worker function"""
            data = rospy.wait_for_message(self.rviz_waypoints_ready_topic, Empty)
            rospy.loginfo('Received path READY from rviz waypoints')
            self.path_ready = True
#            with open(output_file_path, 'w') as file:
                #for current_pose in waypoints.poses:
                    #file.write(str(current_pose.pose.position.x) + ',' + str(current_pose.pose.position.y) + ',' + str(current_pose.pose.position.z) + ',' + str(current_pose.pose.orientation.x) + ',' + str(current_pose.pose.orientation.y) + ',' + str(current_pose.pose.orientation.z) + ',' + str(current_pose.pose.orientation.w)+ '\n')
                #file.close()
					#rospy.loginfo('rviz waypoints written to file ' + output_file_path)
        ready_thread = threading.Thread(target=wait_for_path_ready)
        ready_thread.start()

        self.start_journey_bool = False

        ########################################################################
        # Start thread to listen self.file_waypoints_ready_topic
        # for loading the saved poses from saved_path/poses.csv
        def wait_for_start_journey():
            global waypoints
            global waypoints_types
            global input_file_path

            """thread worker function"""
            data_from_start_journey = rospy.wait_for_message(self.file_waypoints_ready_topic, Empty)
            rospy.loginfo('Received path READY from file waypoints')
            rospy.loginfo('Reading poses from %s' % input_file_path)
            waypoints = Path()
            with open(input_file_path, 'r') as file:
                reader = csv.reader(file, delimiter = ',')
                for row in reader:
                    #print row
                    current_pose = PoseStamped()
                    current_pose.pose.position.x    = float(row[0])
                    current_pose.pose.position.y    = float(row[1])
                    current_pose.pose.position.z    = float(row[2])
                    current_pose.pose.orientation.x = float(row[3])
                    current_pose.pose.orientation.y = float(row[4])
                    current_pose.pose.orientation.z = float(row[5])
                    current_pose.pose.orientation.w = float(row[6])
                    waypoint_type                   = int(row[7])
                    waypoints_types.append(waypoint_type)
                    waypoints.poses.append(current_pose)
            self.poseArray_publisher.publish(self.convert_Path_to_PoseArray(waypoints))
            self.start_journey_bool = True


        start_journey_thread = threading.Thread(target=wait_for_start_journey)
        start_journey_thread.start()

        # The topic where the waypoints provided via rviz are handed to this node for following
        topic = this_node_name + "/" + rospy.get_param('~rviz_waypoints_topic', 'rviz_waypoints')
        rospy.loginfo("Waiting to receive waypoints via rviz on topic '%s'" % topic)
        rospy.loginfo("To start following rviz waypoints: 'rostopic pub %s std_msgs/Empty -1'" % self.rviz_waypoints_ready_topic)
        rospy.loginfo("OR")
        rospy.loginfo("To start following saved waypoints: 'rostopic pub %s std_msgs/Empty -1'" % self.file_waypoints_ready_topic)


        # Wait for published waypoints or saved path loaded
        while (not self.path_ready and not self.start_journey_bool):
            try:
                waypoints = rospy.wait_for_message(topic, Path, timeout=1)
            except rospy.ROSException as e:
                if 'timeout exceeded' in e.message:
                    continue  # no new waypoint within timeout, looping...
                else:
                    raise e
            rospy.loginfo("Received waypoints from rviz")

            with open(output_file_path, 'w') as file:
                for current_pose in waypoints.poses:
                    file.write(str(current_pose.pose.position.x) + ',' + str(current_pose.pose.position.y) + ',' + str(current_pose.pose.position.z) + ',' + str(current_pose.pose.orientation.x) + ',' + str(current_pose.pose.orientation.y) + ',' + str(current_pose.pose.orientation.z) + ',' + str(current_pose.pose.orientation.w)+ '\n')
                file.close()

            rospy.loginfo('rviz waypoints written to file ' + output_file_path)
            rospy.logwarn('To follow these waypoints in the future you should \
                rename this file by removing the word "latest" and adding a \
                last column where 1 signifies a full stop at that waypoint \
                and 0 a plain passing over it')

            # publish waypoint queue as pose array so that you can see them in rviz, etc.
            self.poseArray_publisher.publish(self.convert_Path_to_PoseArray(waypoints))

            # Message by li9i, 1/10/2020:
            # Once the waypoints message is received, this package by default
            # awaits for the publication of an empty message to commence
            # waypoint following. But the waypoints are published all at once
            # from the waypoint_navigation_plugin, contrary to the intended
            # logic of the default package. Therefore after receiving the
            # waypoints self-publish the empty message; following will then
            # commence at once
            #empty_msg = Empty()
            #self.empty_msg_pub.publish(empty_msg)

        # Path is ready! return success and move on to the next state (FOLLOW_PATH)
        return 'success'


################################################################################
################################################################################
class PathComplete(State):

    ############################################################################
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    ############################################################################
    def execute(self, userdata):
        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        rospy.loginfo('###############################')
        return 'success'


################################################################################
def main():

    # Init node
    rospy.init_node('cultureid_waypoints_following')

    global this_node_name
    this_node_name = rospy.get_name().lstrip('/')
    print("%s" % this_node_name)

    # The name of the map of the environment running waypoints' following
    map_name = rospy.get_param('~map_name', '')

    # Essentially which waypoints to follow
    game_id = rospy.get_param('~game_id', 0)

    # Paths for saving and retrieving the poses to be followed
    global input_file_path
    input_file_path = rospkg.RosPack().get_path(this_node_name)+"/saved_path/" + map_name + "_pose_" + str(game_id) + ".csv"


    global output_file_path
    output_file_path = rospkg.RosPack().get_path(this_node_name)+"/saved_path/" + map_name + "_pose_latest_" + str(game_id) + ".csv"

    sm = StateMachine(outcomes=['success'])

    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                           transitions={'success':'FOLLOW_PATH'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                           transitions={'success':'PATH_COMPLETE'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('PATH_COMPLETE', PathComplete(),
                           transitions={'success':'GET_PATH'})

    outcome = sm.execute()



################################################################################
if __name__ == '__main__':
    main()
