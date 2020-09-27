import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D\

## 从CSV文件加载深度图数据img并显示
img=np.genfromtxt('img_dep_640x480.csv', delimiter=',').astype(np.float32)
plt.imshow(np.clip(img,0.55,0.7),cmap='jet')    # 显示加载的深度图
plt.title('depth image')
plt.show()

#####################################
## 在下面补充你的代码，从深度图img生成点云数据pc，并保存为CSV文件
## 生成点云使用的相机参数如下：
CAM_WID,CAM_HGT = 640,480           # 深度图img的图像尺寸
CAM_FX,CAM_FY   = 795.209,793.957   # 相机的fx/fy参数
CAM_CX,CAM_CY   = 332.031,231.308   # 相机的cx/cy参数
CAM_DVEC = np.array([-0.33354, 0.00924849, -0.000457208, -0.00215353, 0.0]) # 相机镜头的矫正参数，用于cv2.undistort()的输入之一
##
##
## 在这里填写你的代码，其中在磁盘上以CSV文件格式存储点云以及显示点云的例子见本下面的例子
##
##
#####################################

## 下面是保存CSV代码的例子以及显示点云的例子
# 例子：生成并保存随机点云，保存为CSV文件
pc=np.random.rand(5000,3)*2.0-1.0 # 生成随机点云
np.savetxt('pc.csv', pc, fmt='%.18e', delimiter=',', newline='\n')

## 从CSV文件加载点云并显示
pc=np.genfromtxt('pc.csv', delimiter=',').astype(np.float32)
ax = plt.figure(1).gca(projection='3d')
ax.plot(pc[:,0],pc[:,1],pc[:,2],'b.',markersize=0.5)
plt.title('point cloud')
plt.show()    

