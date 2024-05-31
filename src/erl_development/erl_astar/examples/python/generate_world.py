#from euclid import Circle, Point2, Vector2, LineSegment2
#import math
#import matplotlib.pyplot as plt
#import numpy as np
#import random

#size = (100, 100)
#res = (100, 100)
#walls = [LineSegment2(Point2(0,0),              Point2(0, size[1])),
#         LineSegment2(Point2(0,size[1]),        Point2(size[0], size[1])),
#         LineSegment2(Point2(size[0], size[1]), Point2(size[0], 0))]


#objects = []
#num_obj = 10
#obj_rad = 3
#for _ in range(num_obj):
#  position = np.random.uniform([obj_rad, obj_rad], np.array(size) - obj_rad)
#  position = Point2(float(position[0]), float(position[1]))
#  objects.append(position)

# http://www.scipy-lectures.org/
import numpy as np
import matplotlib.pyplot as plt

# Create empty map with 3 walls
size = np.array([1001, 1001])
m = np.zeros(size)
m[size[0]-1,:] = 1
m[:,0] = 1
m[:,size[1]-1] = 1

# Add obstacles
obst_num = 30
obst_rad = 30
pos = np.vstack((np.random.randint(0,size[0],obst_num), np.random.randint(0,size[1],obst_num))) # 2 x obst_num

# Ugly python bs:
for k in range(obst_num):
  for ii in xrange(max(0,pos[0,k]-obst_rad),min(size[0]-1,pos[0,k]+obst_rad)+1):
    for jj in xrange(max(0,pos[1,k]-obst_rad),min(size[1]-1,pos[1,k]+obst_rad)+1):
      if (ii-pos[0,k])**2 + (jj-pos[1,k])**2 <= obst_rad**2:
        m[ii,jj] = 1



np.savetxt('test.cfg', m, fmt='%d')

plt.imshow(m, interpolation='none')
plt.gca().invert_yaxis()
plt.show()










