import time
from subprocess import Popen



server = Popen(["rosrun", "turtlebot3_gazebo", "environment.py"])
time.sleep(2)
skills = Popen(["rosrun", "turtlebot3_gazebo", "skills.py"])
time.sleep(2)
client = Popen("""rosrun turtlebot3_gazebo client_pick_balls.py""",shell =True)
client.wait()