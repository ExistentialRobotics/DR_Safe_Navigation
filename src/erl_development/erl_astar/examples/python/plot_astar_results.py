"""
This script plots the A* output yaml
"""
# import numpy, scipy, pylab, random
import numpy as np
import os, sys
import matplotlib.pyplot as plt; #plt.ion()
import yaml


def import_matrix(path_to_cfg, dlm=' '):
  """
  This function imports a dlm separated matrix from file path_to_cfg and
  returns it as a np array with num_line elements, each of size lengh_row
  """
  with open(path_to_cfg,'r') as f:
    A = np.asarray([ map(int,line.split(dlm)) for line in f ])
  return A


if __name__ == '__main__':
  stream = open(sys.argv[1], "r")

  #stream = open("/media/natanaso/Data/Stuff/Research/git/libnx_ws/src/nx_planning/astar_nx/data/init_circles.yaml", "r")
  #stream = open("/media/natanaso/Data/Stuff/Research/git/libnx_ws/src/nx_planning/astar_nx/data/init_small_map.yaml", "r")
  #stream = open("/media/natanaso/Data/Stuff/Research/git/libnx_ws/src/nx_planning/astar_nx/data/init_large_map_2d.yaml", "r")
  #stream = open("/media/natanaso/Data/Stuff/Research/git/libnx_ws/src/nx_planning/astar_nx/data/init_large_map.yaml", "r")
  #stream = open("/media/natanaso/Data/Stuff/Research/git/libnx_ws/src/nx_planning/astar_nx/data/init_small_map_2d.yaml", "r")
  #stream = open("/media/natanaso/Data/Stuff/Research/git/libnx_ws/src/nx_planning/astar_nx/data/init_circles_2d.yaml", "r")
  #stream = open("/media/natanaso/Data/Stuff/Research/git/libnx_ws/src/nx_planning/astar_nx/data/init_empty_2d.yaml", "r")
  #stream = open("/media/natanaso/Data/Stuff/Research/git/libnx_ws/src/nx_planning/astar_nx/data/init_large_empty_2d.yaml", "r")

  #docs = yaml.load_all(stream)
  #for doc in docs:
  #    for k,v in doc.items():
  #        print k, "->", v
  #    print "\n",
  doc = yaml.load(stream)
  #MAP = import_matrix(doc["mappath"])
  MAP = import_matrix(os.path.join(os.path.dirname(sys.argv[1]),doc["mappath"]))
  
  fig = plt.figure(1)
  ax = fig.add_subplot(111)
  ax.imshow(MAP, interpolation='none') # plots dimension 0 as y-axis!
  ax.set_xlim([-0.5, MAP.shape[1]-0.5])
  ax.set_ylim([-0.5, MAP.shape[0]-0.5])

  # Display path
  #stream = open("/media/natanaso/Data/Stuff/Research/git/libnx_ws/src/nx_planning/astar_nx/bin/test_path.yaml","r")
#  stream = open(sys.argv[2],"r")
#  doc = yaml.load(stream)
#  path = doc["path (discrete)"]
#  ax.plot([x[0] for x in path],[x[1] for x in path],'r-')
#  print(path)
  
  plt.show()

