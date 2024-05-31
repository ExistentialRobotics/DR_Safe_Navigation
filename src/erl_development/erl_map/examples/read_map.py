from pyErlMap import GridMap

if __name__ == "__main__":
    input_yaml = '/home/brent/arl-unity-ros/src/arl-unity-ros/environments/meshes/lejeune/Lejeune.yaml'
    gridmap = GridMap()
    gridmap.load(input_yaml)
    print("Shape {}".format(gridmap.size()))
