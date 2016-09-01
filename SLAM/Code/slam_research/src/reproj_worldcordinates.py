import numpy as np
import math
import tf.transformations as tft
import numpy.linalg as la
import trans


# camera intrinsic parameters

# RGB camera
C =np.array(([5.3009194943536181e+02, 0., 3.2821930715948992e+02],
             [ 0.,5.2635860167133876e+02, 2.6872781351282777e+02],
             [ 0., 0., 1.]),dtype = float)

# IR camera
Cd = np.array(([5.9425464969100040e+02, 0., 3.3978729959351779e+02],
               [0.,5.9248479436384002e+02, 2.4250301427866111e+02],
               [ 0., 0., 1.]),dtype=float)

# Camera extrinsic parameters projecting IR camera on RGB camera

R1 = [9.9977321644139494e-01, 1.7292658422779497e-03,-2.1225581878346968e-02]
R2 = [ -2.0032487074002391e-03,9.9991486643051353e-01, -1.2893676196675344e-02]
R3 = [2.1201478274968936e-02, 1.2933272242365573e-02,9.9969156632836553e-01]

R = np.array([R1,R2,R3],dtype = float)
T = np.array(([ 2.1354778990792557e-02, 2.5073334719943473e-03,-1.2922411623995907e-02 ]),dtype=float)


Trans= np.array(([-0.02200007297369995, 0.2649999270260862, -0.05500032241106303]),dtype=float)
Rot=tft.quaternion_matrix([0.4999999999995599, -0.4999993366025517, 0.4999999999995599, 0.5000006633974483])
rot= Rot[0:3,0:3]


# # Extrinsic parameters projecting camera coordinates to world coordinates
#
# Rw= np.array(([0, 0, 1],[-1,0, 0],[0, -1, 0]),dtype= float)
# Tw = np.array((0,0,0),dtype = float)

def main(argv,keypoint,orient_r,pos_r):
    # print orient_r
    # print pos_r

    nub=str(argv)
    str2= nub + '.npy'
    worldCoordinates=[]
    depth = np.load(str2)

   

    for i in keypoint:
        x = i.pt[0]
        y = i.pt[1]
        dep= depth[y,x]/ float (1000)

        Xir = ((x-3.3978729959351779e+02)*dep)/5.9425464969100040e+02
        Yir = ((y-2.4250301427866111e+02)*dep)/5.9248479436384002e+02
        Zir = dep
        IR = np.array(([Xir,Yir,Zir]),dtype = float)
        RGB = np.dot(R,IR) + T
        # print RGB.shape

        base_coordiante = np.dot(RGB,rot)+Trans
        # print base_coordiante.shape

        gl_coordinate = np.dot(orient_r,base_coordiante)+pos_r
        # print gl_coordinate.shape
        worldCoordinates.append(gl_coordinate)

    return worldCoordinates


if __name__ == '__main__':
    main()