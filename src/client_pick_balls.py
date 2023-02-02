import rospy
from turtlebot3_gazebo.srv import goal_checker,goal_checkerResponse,navigation, objects_location, pick, place, placeResponse, robot_location, robot_locationResponse,create_scene

if __name__ == "__main__":
    try:
        rospy.init_node('talker', anonymous=True)
        rospy.wait_for_service('/create_scene')
        scene = rospy.ServiceProxy('create_scene', create_scene)
        scene()

        rospy.wait_for_service('/objects_location')
        find_func = rospy.ServiceProxy('objects_location', objects_location)
        result = find_func()

        for name,pos in zip(result.name,result.pose):
            if name != 'blue_cube':
                rospy.wait_for_service('/robot_location')
                rob_loc = rospy.ServiceProxy('robot_location', robot_location)
                rob_loc()

                rospy.wait_for_service('/navigation')
                navigate = rospy.ServiceProxy('navigation', navigation)
                navigate(pos.position.x - 0.15 ,pos.position.y - 0.15 ,0.8)

                rospy.wait_for_service('/pick')
                pick_ball= rospy.ServiceProxy('pick', pick)
                pick_ball(name , pos.position.x ,pos.position.y)

                rospy.wait_for_service('/navigation')
                navigate_to_box = rospy.ServiceProxy('navigation', navigation)
                navigate_to_box(0.0, -0.35 ,1.0)

                rospy.wait_for_service('/place')
                place_cube = rospy.ServiceProxy('place', place)
                place_cube(name , 0.0, -0.5)
                print("place red ball in the box")

                #rospy.wait_for_service('/navigation')
                #navigate_to_box = rospy.ServiceProxy('navigation', navigation)
                #navigate_to_box(0.0 ,0.5 ,1.0)
          
        rospy.wait_for_service('/goal_checker')
        final_check = rospy.ServiceProxy('goal_checker', goal_checker)
        final_check()
        
    except rospy.ROSInterruptException:
            rospy.loginfo("node terminated...")
    
