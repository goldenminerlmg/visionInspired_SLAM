
import numpy as np
import numpy.linalg as ln
import sys
import rospy
import roslib
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import string
from math import *
import random
import tf.transformations as tft
import copy
import time
# importing functional scripts
import grabframes
import ExtractLandmarks
import reproj_worldcordinates
import reobserved


Observations=[]
world_size= 20.0
N = 20
Q_t = np.array(([0.01,0,0],[0,0.01,0],[0,0,0.01]),dtype=float)
########################################################################################################################
def calJacobian(pos_r,pos_l):
    dx = pos_l[0]-pos_r[0]
    dy = pos_l[1]-pos_r[1]
    q =  (dx**2) + (dy**2)
    z_pr = sqrt(q)
    z_p_phi = atan2(dy,dx)-pos_r[2]
    H = np.array(([-dx/z_pr,-dy/z_pr,0],[dy/q,-dx/q,-1],[0,0,1]),dtype=float)
    return H
########################################################################################################################
class Landmark(object):

        def __init__(self,pos_p, pos_l, des, kp):

            distance_x =  sqrt((pos_p.x-pos_l[0]) ** 2)
            distance_y =  sqrt((pos_p.y-pos_l[1])** 2)
            # dist = np.array([distance_x, distance_y])
            # bearing = atan2(distance_y, distance_x)
            #
            # land_x = np.linalg.norm(dist) * cos(bearing + pos_p.orientation) + pos_p.x
            # land_y = np.linalg.norm(dist) * sin(bearing + pos_p.orientation) + pos_p.y

            self.mean = [distance_x,distance_y,0]

            par_p=np.array(([pos_p.x,pos_p.y,pos_p.orientation]),dtype=float)
            H = calJacobian(par_p,pos_l)
            H_inv = ln.inv(H)
            H_t = np.transpose(H_inv)
            temp_array = np.dot(H_inv,Q_t)

            self.covarince = np.dot(temp_array,H_t)
            self.descriptor = np.asarray(des, dtype=float)
            self.kp = kp





########################################################################################################################
class Particles(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.movement_noise = np.array(([0.01,0],[0,0.05]),dtype=float)
        self.weight = 1/float (N)
        self.l=np.array(([]),dtype=float)

    def add_landmarks(self,Landmark):
        self.l=np.append(self.l,Landmark)

    def move(self,movement):
        self.orientation = self.orientation + float(movement[1]) + random.gauss(0.0,self.movement_noise[1,1])
        self.orientation %= 2 * pi

        randomness = movement[0] + random.gauss(0.0,self.movement_noise[0,0])

        self.x = self.x + cos(self.orientation)*randomness
        self.y = self.y + sin(self.orientation)*randomness

        # x %=world_size
        # y %=world_size
        # self.x = x
        # self.y = y


    def Measuement_particles(self,land_pos):

            distx = land_pos.mean[0] - self.x
            disty = land_pos.mean[1] - self.y
            q = sqrt((distx**2) + (disty**2))
            dist =  q + random.gauss(0.0, 0.01)

            angle = atan2(land_pos.mean[1] - self.y,land_pos.mean[0] - self.x)
            angle = angle + random.gauss(0.0, 0.01)
            M = [dist, angle,0]
            return M

    def update(self,index,Z,M,H):
        z = np.asarray(Z)
        m = np.asarray(M)
        temp1 = np.dot(H,self.l[index].covarince)
        temp2 = np.transpose(H)
        Q = np.dot(temp1,temp2) + Q_t
        temp3 = np.dot(self.l[index].covarince,temp2)
        temp4 = ln.inv(Q)
        K = np.dot(temp3,temp4)
        diff = z - m
        temp5 = np.dot(K,H)

        self.l[index].mean = self.l[index].mean + np.dot(K,np.transpose(diff))
        self.l[index].covarince = np.dot((np.identity(3)-temp5),self.l[index].covarince)

        temp6 = ln.det(2*pi*Q)
        temp7 = np.transpose(diff)
        temp8 = np.dot(temp4,diff)
        temp9 = np.dot(temp7,temp8)
        temp10 = exp(-1/2*temp9)
        self.weight = np.dot(temp6,temp10)





def sense(pos,landmarks):
     Z=[]

     for i in range (len(landmarks)):
         distx = landmarks[i][0]-pos[0]
         disty = landmarks[i][1]-pos[1]
         q = ((distx**2) + (disty**2))
         q = sqrt(q)
         dist = q + random.gauss(0.0, 0.01)
         angle = atan2(landmarks[i][1]-pos[1],landmarks[i][1]-pos[1])
         angle = angle+random.gauss(0.0,0.01)

         Z.append([distx,angle,0])

     return Z





def evall( r, p):
    sum = 0.0;
    for i in range(len(p)):  # calculate mean error
        dx = (p[i].x - r[0])
        dy = (p[i].y - r[1])
        err = sqrt(dx * dx + dy * dy )
        sum += err
    return (sum / float(len(p)))

########################################################################################################################
p = np.array(([]),dtype=float)
for i in range (N):
    par = Particles()
    p=np.append(p,par)

descriptors1 = np.empty((0, 32), dtype=float)
keypoints1 = []

time_stamp = list(range(1,8))
for t in time_stamp:
    print "#" ,t
    pos_r, rot_r = grabframes.main(t)
    pos_r = np.asarray(pos_r)
    o_r = tft.quaternion_matrix(rot_r)
    orient_r = o_r[0:3, 0:3]
    r = acos(orient_r[0, 0])
    pos_r[2] = r
    pos_r[0] = -pos_r[0]
    pos_r[1] = -pos_r[1]

    t0=time.time()
    kp1, des1 = ExtractLandmarks.main(t)
    des = np.array((des1), dtype=float)
    newlandmarks = reproj_worldcordinates.main(t, kp1,orient_r,pos_r)
    t1 = time.time()
    print "time for extracting Landmarks and reprojecting them to world" ,t1-t0
    t4=time.time()
    if t==1:
        Update = []
        Init = newlandmarks
        for k in range(len(newlandmarks)):
            Observations.append(newlandmarks[k] )
    else :

        Update = []
        Init = []
        descriptors2 = des
        keypoints2 = kp1

        Update, Init = reobserved.main(descriptors1, descriptors2, keypoints1, keypoints2)
        # Update = list(set(Update))
        # Init = list(set(Init))
        if len(Init) !=0:
            remove = np.array(Init)
            rmlandmarks = np.asarray(newlandmarks)
            newlm = np.delete(rmlandmarks, remove, 0)
            des = np.delete(des,remove,0)
            kp = np.array(kp1)
            kp1 = np.delete(kp,remove,0)
        else:
            continue
        newlm = list(newlm)
        Init = newlm
        kp1 = list(kp1)
        for i in range(len(newlm)):
            Observations.append(newlm[i] )

    Z = sense( pos_r , Observations )



    for i in range(N):
        p[i].move([0.5, 0])

        if len (Init) is not 0:
            for j in range (len(Init)):
                p[i].add_landmarks(Landmark(p[i],Init[j],des[j],kp1[j]))

        if len(Update) is not 0:
            for k in Update:
                M = p[i].Measuement_particles(p[i].l[k])
                par_p = np.array(([p[i].x, p[i].y, p[i].orientation]), dtype=float)
                l_p = np.array(([p[i].l[k].mean[0], p[i].l[k].mean[1], p[i].l[k].mean[2]]), dtype=float)
                h = calJacobian(par_p, l_p)
                p[i].update(j, Z[k], M, h)



    w=[]
    for i in range (N):
        w.append(p[i].weight)
    p3 = np.array(([]),dtype=float)

    if t ==0:
        minerror_x = (pos_r[0] - p[0].x)**2
        minerror_y = (pos_r[1] - p[0].y)**2
        min = sqrt(minerror_x + minerror_y)
        min_index = 0

        for i in range (1,N):
            dx = (pos_r[0] - p[i].x)**2
            dy = (pos_r[1] - p[i].y)**2
            error = sqrt(dx + dy)
            if error<min:
                min_index = i
                min = error
                dup = copy.deepcopy(p[min_index])
                p3=np.append(p3,dup)
            else:
                dup = copy.deepcopy(p[min_index])
                p3=np.append(p3,dup)
        p = p3
    else:
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            dup = copy.deepcopy(p[index])
            p3=np.append(p3,dup)

        ## most important deep copy in python if not used will replicate memeory address of instances of class.
        p = p3

    descriptors1 = np.append(descriptors1, des, 0)
    for i in range(len(kp1)):
        keypoints1.append(kp1[i])
    t5 = time.time()
    print "Accuray of system",evall(pos_r,p)
    print "time taken for the entire particle filter backend to run" ,t5-t4
    t0=0
    t1=0

    t4=0
    t5=0

