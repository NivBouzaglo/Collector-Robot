#!/usr/bin/env python3
import rospy
from turtlebot3_gazebo.srv import create_scene, create_sceneResponse, goal_checker, goal_checkerResponse
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates, ModelState
import time
import random




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


def random_room1():
    x = 2.0
    y = random.randint(0, 1)
    return [x, y, 0.2]

def random_room2():
    x = random.randint(0,2)
    y = random.randint(2, 3)
    return [x, y, 0.2] 

def random_room3():
    x = random.randint(-3,-1)
    y = random.randint(2,3)
    return [x, y, 0.2]

def random_room4():
    x = random.randint(-3,-2)
    y = random.randint(0, 1)
    return [x, y, 0.2] 

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


def create_scene_function():  
    delete_model('blue_cube') 
    time.sleep(1)
    spawn_locations =[random_room1(),random_room2(),random_room3(),random_room4()]
    for n in range (len(spawn_locations)):
        delete_model('red_ball'+str(n)) 
        time.sleep(.5)
        spawn_model('red_ball'+str(n), '/home/niv/.gazebo/models/objects/red_ball.sdf', spawn_locations[n])
    spawn_model('blue_cube', '/home/niv/.gazebo/models/objects/blue_cube.sdf', [0.0,-0.5,0.2])

def goal_checker_function():
    names, positions = find_objects()
    index = names.index('blue_cube')
    cube_pos = positions[index]
    for pos in positions : 
        if pos != cube_pos : 
            return False 
    return True 

def goal_checker_callback(request): 
    return goal_checkerResponse(goal_checker_function())

def create_scene_callback(request):
    create_scene_function()
    return create_sceneResponse("done")



if __name__ == "__main__": 
    rospy.init_node('enviroment',anonymous = True)
    create_scene_service  = rospy.Service("create_scene",create_scene, create_scene_callback)

    goal_checker_service = rospy.Service("goal_checker", goal_checker, goal_checker_callback)
    rospy.spin() 