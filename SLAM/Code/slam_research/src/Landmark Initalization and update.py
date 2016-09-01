import numpy.linalg as lin
import numpy as np
import math
def Lankmark_Init(l):
    var = 0.0035
    #  Mean position
    mu = l
    sigma =
    # Measurment Noise
    R = np.array(([lin.norm(l)*var,0,0],[0,lin.norm(l)*var,0],[0,0,math.pow(lin.norm(l),2)*var]),dtype = float )
