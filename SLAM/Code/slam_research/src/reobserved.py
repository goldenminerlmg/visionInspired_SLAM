import numpy as np
import cv2
from matplotlib import pyplot as plt
import sys

# # loading depth and rgb image for views
# img1= cv2.imread("1.png",0)
# img1d=np.load("1.npy")
# img3 = cv2.imread("2.png",0)
# img3d=np.load("2.npy")
#
# # defining  the ORB feature generator with 50 keypoints and corresponding Descriptors
# ob=cv2.ORB(50)
# kp1=ob.detect(img1,None)
# kp2=ob.detect(img3,None)
# index=0
# count=0
# for i in kp1:
#
#     pix= i.pt
#     print img1d[pix[1],pix[0]]
#     if (img1d[pix[1],pix[0]]==0):
#         count = count + 1
#         del kp1[index]
#     index = index + 1
#
#
#
#
# kp1, des1 = ob.compute(img1,kp1)
# kp2, des2 = ob.compute(img3,kp2)
#
#
# # drawing the keypoints on the images
# img5 = cv2.drawKeypoints(img1,kp1,color=(0,255,0),flags=0)
# img6 = cv2.drawKeypoints(img3,kp2,color=(0,255,0),flags=0)


def main(d1,d2,kp1,kp2):

    # print len(des1)
    # print len(des2)
    des1=d1.astype(np.uint8)
    des2=d2.astype(np.uint8)
# comparing the descriptors using a Hashing Table to find possible correspondance
    FLANN_INDEX_LSH = 6
    index_params= dict(algorithm = FLANN_INDEX_LSH,
                       table_number = 6, # 12
                       key_size = 12,     # 20
                       multi_probe_level = 1) #2
    search_params = dict(checks=100)
    flann = cv2.FlannBasedMatcher(index_params,search_params)
    matches = flann.knnMatch(des2,des1,k=2)


    # checking for possible matches
    if len(matches)>0:
        print "%d total matches found" % (len(matches))
    else:
        print "No matches were found "
        sys.exit()

    # computing the good matches based upon threshold for minimum distance
    good = []
    nUpdate=[]
    nInit=[]

    for m_n in matches:
      if len(m_n) != 2:
        continue
      (m,n) = m_n
      if m.distance < 0.6*n.distance:
        good.append(m)
        nInit.append(m.queryIdx)
        nUpdate.append(m.trainIdx)

    if len(good) > 10:
        src_pts = np.float32([ kp2[m.queryIdx].pt for m in good ])
        dst_pts = np.float32([ kp1[n.trainIdx].pt for n in good ])




        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()
        print "total number of good correspondance made : %d" %len(mask)
        index=[]
        for i in range(len(mask)):
            if mask[i]==0:
                index.append(i)
        if index is not []:
            index=np.asarray(index,dtype=float)
            nUpdate=np.delete(nUpdate,index,0)
            nInit=np.delete(nInit,index,0)
        return nUpdate,nInit



        # h,w = img1.shape
        # pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        # dst = cv2.perspectiveTransform(pts,M)
        # img7 = cv2.polylines(img3,[np.int32(dst)],True,255,3, cv2.CV_AA)

    else:
        # print "Not enough matches are found - %d/%d" % (len(good),10)
        matchesMask = None
    return [],[]

# result = cv2.warpPerspective(img1, M,(img1.shape[1] + img3.shape[1], img1.shape[0]))
# result[0:img3.shape[0], 0:img3.shape[1]] = img3
# im_out = cv2.cvtColor(result,cv2.COLOR_GRAY2BGR)
# cv2.imwrite("g.png",im_out,None)
#
# plt.imshow( im_out), plt.show()
# plt.imshow(img5,), plt.show()
# plt.imshow(img6), plt.show()
# cv2.waitKey(0)

#
if __name__ =='__main__':
    main()



