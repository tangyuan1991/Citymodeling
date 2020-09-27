#!/usr/bin/python3.5
# coding=utf-8

import numpy as np
import cv2

import matplotlib.pyplot as plt

####################
# SAD双目匹配算法
####################

# 从CSV读取灰度图imgL和imgR
print('loading image from CSV file')
imgL = np.genfromtxt('aL_gray.csv',delimiter=',').astype(np.float32)
imgR = np.genfromtxt('aR_gray.csv',delimiter=',').astype(np.float32) 
print(imgL.shape)

plt.clf()
plt.subplot(1,2,1);plt.imshow(imgL,cmap='gray')
plt.subplot(1,2,2);plt.imshow(imgR,cmap='gray')
plt.show()


#####################################
## 在下面补充你的代码，对imgL中的每个像素，找到imgR中匹配的像素，
## 并将匹配像素的水平偏移量（取绝对值）保存在文件math.csv中
##
## 在这里填写你的代码
##
#####################################

## 下面是保存CSV代码的例子
data=np.random.randint(0,10,(427,370))  # 生成尺寸为427x370的随机整数矩阵
np.savetxt('match.csv', data, fmt='%d', delimiter=',', newline='\n') # 保存为csv文件


