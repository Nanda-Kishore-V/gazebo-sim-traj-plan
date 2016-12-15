#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('diff_wheeled_robot_control')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import heapq

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("rgbd_camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(1)
    # global pp
    # pp.path_generation((0,0),(2,2),[])
    # talker(x_val,z_val

class plan_path:

    def initialization(self,start,goal,obstacles):
        self.path = [];              self.open_list = []
        self.size = 5;               self.came_from = {}
        self.cost = {}
        self.start = start;      self.goal = goal
        self.obstacles = obstacles
        heapq.heappush(self.open_list,(self.heuristic(self.start,self.goal),self.start))
        self.came_from[self.start] = None;    self.cost[self.start] = 0

    def __init__(self):
        self.directions = [(-1,0),(0,-1),(0,1),(1,0)]
        self.dir_in_words = ["UP","LEFT","RIGHT","DOWN"]

    def heuristic(self,start,goal):
        return abs(start[0]-goal[0])+abs(start[1]-goal[1])

    def path_generation(self,start,goal,obstacles):
        self.initialization(start,goal,obstacles)
        while len(self.open_list) != 0:
            current = heapq.heappop(self.open_list)[1]
            if(current == self.goal):
                while current in self.came_from:
                    self.path.append(current)
                    current = self.came_from[current]
                self.path.reverse()
                for i in range(0,len(self.path)-1):
                    print(self.dir_in_words[self.directions.index((self.path[i+1][0]-self.path[i][0],self.path[i+1][1]-self.path[i][1]))])
                return
            for i,j in self.directions:
                neighbor = (current[0]+i,current[1]+j)
                if neighbor[0] < 0 or neighbor[0] >= self.size or neighbor[1] < 0 or neighbor[1] >= self.size:
                    continue
                if neighbor in self.obstacles:
                    continue
                if neighbor not in self.cost or self.cost.get(neighbor,0) > self.cost[current]+1:
                    self.came_from[neighbor] = current
                    self.cost[neighbor] = self.cost[current]+1
                    priority = self.cost[neighbor] + self.heuristic(neighbor,self.goal)
                    heapq.heappush(self.open_list,(priority,neighbor))

def talker(x,z):
    pub = rospy.Publisher('/first_robot/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    msg.linear.x = x_val
    msg.angular.z = z_val
    pub.publish(msg)

def main(args):
    ic = image_converter()
    global pp
    pp = plan_path()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
