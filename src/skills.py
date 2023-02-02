#!/usr/bin/env python3
import rospy
from turtlebot3_gazebo.srv import navigation, navigationResponse, objects_location,objects_locationResponse, pick, pickResponse, place, placeResponse, robot_location, robot_locationResponse
import rospy
import actionlib
import time
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def spawn_model(name, file_location='/home/niv/.gazebo/models/objects/red_ball.sdf', spawn_location=[0.0,0.0,1.0]):
    pose = Pose()
    pose.position.x = spawn_location[0]
    pose.position.y = spawn_location[1]
    pose.position.z = spawn_location[2]
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(model_name=name,
                       model_xml=open(file_location, 'r').read(),
                       robot_namespace='/stuff', initial_pose=pose, reference_frame='world')

def delete_model(name):
    # delete model
    srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    req = DeleteModelRequest()
    req.model_name = name
    resp = srv(req)

def distance(x1, y1, x2, y2):
    dist = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** .5
    return dist


def movebase_client(x,y,w=1.0):
    #moves the robot collision free to a x,y,theta pose (must be valid/reachable in the map)
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.w = w
    print("goal sent:")
    print(goal)
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def gps_location():
    # request a GPS like pose information from the Gazebo server
    rospy.loginfo("Requesting Global Robot Pose from Gazebo")
    model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
    me_pose = Pose()
    me_pose = model_state.pose[2]
    me_pose_angles = euler_from_quaternion([me_pose.orientation.x, me_pose.orientation.y, me_pose.orientation.z, me_pose.orientation.w])
    print('My pose is (x,y,theta): ')
    print(me_pose.position.x, me_pose.position.y, me_pose_angles[2])
    return me_pose.position.x, me_pose.position.y, me_pose_angles[2]


def find_objects():
    # request from Gazebo the global pose of all objects
    rospy.loginfo("Requesting Global Object Poses from Gazebo")
    model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
    number_of_objects = len(model_state.pose)  - 3 # ignore: [ground_plane, room1, turtlebot3_burger]    	  	   	
    print('I found ' +str(number_of_objects) +' Objects')
    print(model_state.name[3:])
    print(" Objects data :")

    for name, pos in zip(model_state.name[3:],  model_state.pose[3:]):
        print("name: " + name)
        print(pos)
    return model_state.name[3:], model_state.pose[3:]

def is_empty_knapsack(): 
	names, positions = find_objects()
	for pos in positions: 
		if pos == [0.0, -10.0, 1.0]: 
			return False
	return True

def pick_object(object_name, object_position):
	print('Trying to pick up: ' + object_name)
	me_pose = gps_location()
	object_x = object_position[0]
	object_y = object_position[1]	
	dist = distance(me_pose[0],me_pose[1],object_x,object_y)
	isEmpty = is_empty_knapsack() 
	if dist <.35 and isEmpty:
		delete_model(object_name)
		time.sleep(1)
		spawn_model(name=object_name, spawn_location=[0.0, -10.0, 1.0]) #put in knapsack
		time.sleep(1)
		print('...successfully.')
		
	else: 
		print('...unsuccessfully. Need to be closer to the object to pick it')

def is_picked(ball_name):
	names, positions = find_objects()
	for name, pos in zip(names, positions):
		if name == ball_name:
			print(pos)
			return pos.position.x == 0.0 and pos.position.y == -10.0
	return False 	


def place_object(object_name, place_location):
	# delete selected object from bag and place it in gazebo
	me_pose = gps_location()
	dist2 = distance(me_pose[0], me_pose[1], place_location[0], place_location[1])
	isPicked = is_picked(object_name)
	if not isPicked: 
		print('Object is not with me...')
		return False
	if dist2<.35:
		delete_model(object_name)
		spawn_model(name=object_name, spawn_location=place_location)
		print('Placed the object')
		return True
	else: 
		print('Need to be closer to the location to place the object (and NOT on it!)') 
		return False


def navigation_callback(request):
    rospy.loginfo("navigation service is on ...")
    movebase_client(request.x, request.y, request.theta)
    return navigationResponse("done")


def objects_location_callback(request):
    rospy.loginfo("object location service is on..")
    finds = find_objects()
    return objects_locationResponse(finds[0],finds[1]) 


def pick_callback(request):
    rospy.loginfo(" pick service is on...")
    pick_object(request.name, [request.x, request.y, 0.0])
    return pickResponse("done")


def place_callback(request):
    rospy.loginfo("place service is on...")
    place_object(request.name, [request.x, request.y, 0.2])
    return placeResponse("done")


def robot_location_callback(request):
    rospy.loginfo("robot location service is on...")
    loc = gps_location()
    return robot_locationResponse(loc[0],loc[1],loc[2]) 


if __name__ == "__main__":

    rospy.init_node('skills', anonymous=True)

    navigation_service  = rospy.Service("navigation",navigation, navigation_callback)
    objects_location_service  = rospy.Service("objects_location",objects_location, objects_location_callback)
    pick_service  = rospy.Service("pick",pick, pick_callback)
    place_service  = rospy.Service("place",place, place_callback)
    robot_location_service  = rospy.Service("robot_location",robot_location, robot_location_callback)
     
    rospy.spin()