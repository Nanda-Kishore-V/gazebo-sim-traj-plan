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
import cv2
import numpy as np
import math

class image_converter:

  def __init__(self):
    self.cX = 0; self.cY = 0; self.yawAngle = 0; self.yY = 0; self.yX = 0;
    self.grid_x_pxls = 10;      self.grid_y_pxls = 10;
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("rgbd_camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(1)

    imgGray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(imgGray,200,255,cv2.THRESH_BINARY)

    cv2.imshow("Thresh",thresh)
    cv2.waitKey(1)

    _,contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, 2)

    j = 0
    for i in range(len(contours)):
        c = contours[i]
        area = cv2.contourArea(c)
        # print(area)
        if(area > 200 and area < 400):
            # print("Area" , area)
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)

            cnt = contours[hierarchy[0][i][2]]
            areaSmall = cv2.contourArea(cnt)
            # print("Area small", areaSmall)
            Msmall = cv2.moments(cnt)
            if Msmall["m00"] == 0:
                continue
            yX = int(Msmall["m10"] / Msmall["m00"])
            yY = int(Msmall["m01"] / Msmall["m00"])
            cv2.drawContours(cv_image, [cnt], -1, (0, 255, 0), 2)

            cv2.imshow("Image",cv_image)
            cv2.waitKey(1)

            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            yawAngle = math.atan2(cY-yY,cX-yX)*180/math.pi

            print("cX",cX)
            print("cY",cY)
            print("yaw",yawAngle)

    cX = math.floor(cX/10);    cY = math.floor(cY/10);
    print(cX,cY)
    global pp
    pp.path_generation((cX,cY),(12,18),[])
    # talker(x_val,z_val



class plan_path:

    def initialization(self,start,goal,obstacles):
        self.path = [];              self.open_list = []
        self.size_x = 32;            self.size_y = 24;
        self.came_from = {}
        self.cost = {}
        self.start = start;         self.goal = goal
        self.obstacles = obstacles
        heapq.heappush(self.open_list,(self.heuristic(self.start,self.goal),self.start))
        self.came_from[self.start] = None;    self.cost[self.start] = 0

    def __init__(self):
        self.directions = [(-1,0),(0,-1),(0,1),(1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        self.dir_in_words = ["UP","LEFT","RIGHT","DOWN","RIGHT-DOWN","LEFT-DOWN","RIGHT-UP","LEFT-UP"]

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
                return self.path
            for i,j in self.directions:
                neighbor = (current[0]+i,current[1]+j)
                if neighbor[0] < 0 or neighbor[0] >= self.size_x or neighbor[1] < 0 or neighbor[1] >= self.size_y:
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
