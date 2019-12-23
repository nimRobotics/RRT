"""

Path planning with Rapidly-Exploring Random Trees (RRT)

author: Aakash(@nimrobotics)
web: nimrobotics.github.io

"""

import cv2
import numpy as np
import math
import random

class Nodes:
    """Class to store the RRT node coordinates and node path"""
    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []

# check collision
def collision(x1,y1,x2,y2):
    color=[]
    x = list(np.arange(x1,x2,(x2-x1)/100))
    y = list(((y2-y1)/(x2-x1))*(x-x1) + y1)
    print("collision",x,y)
    for i in range(len(x)):
        print(int(x[i]),int(y[i]))
        color.append(img[int(y[i]),int(x[i])])
    if (0 in color):
        return True #collision
    else:
        return False #no-collision

# check the  collision with obstacle and trim
def check_collision(x1,y1,x2,y2):
    _,theta = dist_and_angle(x2,y2,x1,y1)
    x=x2 + stepSize*np.cos(theta)
    y=y2 + stepSize*np.sin(theta)
    print(x2,y2,x1,y1)
    print("theta",theta)
    print("check_collision",x,y)

    # TODO: trim the branch if its going out

    # check direct connection
    if collision(x,y,end[0],end[1]):
        directCon = False
    else:
        directCon=True

    # check connection between two nodes
    if collision(x,y,x2,y2):
        nodeCon = False
    else:
        nodeCon = True

    return(x,y,directCon,nodeCon)

# return dist and angle b/w new point and nearest node
def dist_and_angle(x1,y1,x2,y2):
    dist = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
    angle = math.atan2(y2-y1, x2-x1)
    return(dist,angle)

# return the neaerst node index
def nearest_node(x,y):
    temp_dist=[]
    for i in range(len(node_list)):
        dist,_ = dist_and_angle(x,y,node_list[i].x,node_list[i].y)
        temp_dist.append(dist)
    return temp_dist.index(min(temp_dist))

# generate a random point in the image space
def rnd_point():
    new_y = random.randint(0, h)
    new_x = random.randint(0, l)
    return (new_x,new_y)



# loading the maze
img = cv2.imread('world.png',0) # load grayscale image
img2 = cv2.imread('world.png',1)
start = (20,20) # starting coordinate
end = (450,250) # target coordinate
stepSize = 20 # stepsize for RRT
h,l= img.shape # dim of the loaded image
# print(img.shape) # (384, 683)
# print(h,l)

node_list = [0] # list to store all the node points

# insert the starting point in the node class
node_list[0] = Nodes(start[0],start[1])
node_list[0].path_x.append(start[0])
node_list[0].path_y.append(start[1])

# display start and end
cv2.circle(img2, (start[0],start[1]), 5,(0,0,255),thickness=3, lineType=8)
cv2.circle(img2, (end[0],end[1]), 5,(0,0,255),thickness=3, lineType=8)

i=1
pathFound = False
while pathFound==False:
    nx,ny = rnd_point()
    print("Random points:",nx,ny)

    nearest_ind = nearest_node(nx,ny)
    nearest_x = node_list[nearest_ind].x
    nearest_y = node_list[nearest_ind].y
    print("Nearest node coordinates:",nearest_x,nearest_y)

    #check direct connection
    tx,ty,directCon,nodeCon = check_collision(nx,ny,nearest_x,nearest_y)
    print("Check collision:",tx,ty,directCon,nodeCon)

    if directCon and nodeCon:
        node_list.append(i)
        node_list[i] = Nodes(tx,ty)
        node_list[i].path_x = node_list[nearest_ind].path_x.copy()
        node_list[i].path_y = node_list[nearest_ind].path_y.copy()
        node_list[i].path_x.append(tx)
        node_list[i].path_y.append(ty)

        cv2.circle(img2, (int(tx),int(ty)), 2,(0,0,255),thickness=3, lineType=8)
        cv2.line(img2, (int(tx),int(ty)), (int(node_list[nearest_ind].x),int(node_list[nearest_ind].y)), (0,255,0), thickness=1, lineType=8)
        cv2.line(img2, (int(tx),int(ty)), (end[0],end[1]), (255,0,0), thickness=2, lineType=8)

        print("Path has been found")
        print("path_x",node_list[i].path_x)
        for j in range(len(node_list[i].path_x)-1):
            cv2.line(img2, (int(node_list[i].path_x[j]),int(node_list[i].path_y[j])), (int(node_list[i].path_x[j+1]),int(node_list[i].path_y[j+1])), (255,0,0), thickness=2, lineType=8)
        cv2.waitKey(1)
        cv2.imwrite("out.jpg",img2)
        break

    elif nodeCon:
        print("Nodes connected")
        node_list.append(i)
        node_list[i] = Nodes(tx,ty)
        node_list[i].path_x = node_list[nearest_ind].path_x.copy()
        node_list[i].path_y = node_list[nearest_ind].path_y.copy()
        # print(i)
        # print(node_list[nearest_ind].path_y)
        node_list[i].path_x.append(tx)
        node_list[i].path_y.append(ty)
        i=i+1
        # display
        cv2.circle(img2, (int(tx),int(ty)), 2,(0,0,255),thickness=3, lineType=8)
        cv2.line(img2, (int(tx),int(ty)), (int(node_list[nearest_ind].x),int(node_list[nearest_ind].y)), (0,255,0), thickness=1, lineType=8)
        cv2.imshow("sdc",img2)
        cv2.waitKey(1)
        continue

    else:
        print("No direct con. and no node con. :( Generating new rnd numbers")
        continue
