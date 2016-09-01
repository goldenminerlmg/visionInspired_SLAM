import numpy as np
import cv2
from matplotlib import pyplot as plt


def main(argv):

    nub=str(argv)
    str1= nub + '.png'
    str2= nub + '.npy'

    # loading depth and rgb image for views
    img1= cv2.imread(str1,0)
    img1d=np.load(str2)




    # defining  the ORB feature generator with 50 keypoints and corresponding Descriptors
    ob = cv2.ORB(300)
    img1 = cv2.medianBlur(img1, 3, img1)
    
    kp1 = ob.detect(img1, None)
    keypoints = np.asarray(kp1)

    #  Removing the kepoints with no associated depth
    depth=[]
    for i in kp1:
        pix = i.pt
        x = pix[0]
        y = pix[1]
        depth.append(img1d[y,x])

    dep= np.asarray(depth)
    index = np.where(dep == 0.0)
    ind = index[0]
    new_kp = np.delete(keypoints,ind)
    kp=new_kp.tolist()

    kp1, des1 = ob.compute(img1, kp)

    # img5 = cv2.drawKeypoints(img1, kp, color=(0, 255, 0), flags=0)
    # plt.imshow(img5, ), plt.show()
    return kp,des1

