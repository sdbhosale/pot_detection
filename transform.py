
import numpy as np
import cv2
'''
Projective Matrix, P = KR[I|-C]

[x]              [sX]     
[y] = KR[I|-C]   [sY]
[1]              [0] 
                 [1]

solve for X and Y
'''


def get_R(theta):
    R = np.array([[1,0,0],
                 [0,np.sin(theta),-np.cos(theta)],
                 [0,np.cos(theta),np.sin(theta)]])
    return R

def ground_project(pix_x,pix_y,K,R,C,s):
    pix = np.array([[pix_x],[pix_y],[1]])
    vector= np.matmul(np.linalg.inv(np.matmul(K,R)),pix)
    vector = vector/vector[2]
    X=(C[0]*-vector[0]*C[2])/s
    Y=(C[1]*-vector[1]*C[2])/s
    return X,Y

if __name__ == "__main__":
    H=480
    # read the image
    img = cv2.imread('img.jpg')
    
    theta= np.pi/4 #from IMU
    
    K = np.zeros((3,3))  #from realsense

    C = np.array([0,0,H])
    
    s = 12

    R = get_R(theta)
    X,Y=ground_project(0,0,K,R,C,s)

