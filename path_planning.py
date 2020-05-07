import rospy
import time
import argparse
import numpy as np
import copy 
from mapCreation import addPadding 
from geometry_msgs.msg import PoseStamped 

poses = PoseStamped
ZMAP = 8 #map depth
YMAP = 8 #map height
XMAP = 8 #map width

def pathPlanning(args):
    global worldList_index

    # source and destination vertex
   	source = ijk_to_cell_index(float(args.spose[0]),float(args.spose[1]),float(args.spose[2]))
   	dest = ijk_to_cell_index(float(args.gpose[0]),float(args.gpose[1]),float(args.gpose[2]))
   	
   	# world map
   	representation = np.zeros(shape=(ZMAP, YMAP, XMAP))
	representation[3][5][5:8] = 1
	representation[1][0][1] = 1
	representation = addPadding(representation)

	# assigned index to world map
    worldList_index = np.zeros(shape=(ZMAP, YMAP, XMAP))
    for k in range(0,ZMAP):
        for j in range(0,YMAP):
            for i in range(0,XMAP):
                worldList_index[k][j][i] = ijk_to_cell_index(i,j,k)
    prev = dijkstra(source)
    path = traceback(prev,source,dest)
#   publisher = rospy.Publisher('',PoseStamped,queue_size=10)
#	rospy.init_node('drone', anonymous=True)
#	while not is_shutdown():
		#poses.pose.position.x = float(args.gpose[0])
		#poses.pose.position.y = float(args.gpose[1])
		#poses.pose.position.z = float(args.gpose[2])
#		publisher.publish(poses)
#		rospy.sleep(1)
#		rospy.loginfo(poses)
   # print(worldList_index[0][:][:])

# Convert from i,j coordinates to a single integer that identifies a grid cell
def ijk_to_cell_index(i,j,k):
    return j * XMAP + i + k * XMAP * YMAP

# Determine the cost
def travel_cost(source,dest):
    tmp = copy.copy(representation)
    tmp.resize(1,XMAP * YMAP * ZMAP)
    if dest < 0 or dest >= XMAP * YMAP * ZMAP:
        return 100
    if tmp[0][dest] == 1:
        return 100
    return 1

# Using Dijkstra's algorithm for path planning
def dijkstra(source):
    global XMAP, YMAP, ZMAP, worldList_index
    neighbors = [-1,1,-XMAP,XMAP,-(XMAP * YMAP),(XMAP * YMAP)] # neighbors in x,y,z directions
    dist = [0] * XMAP * YMAP * ZMAP
    Q_cost = []
    prev = [-1] * XMAP * YMAP * ZMAP
    if source < 0 or source >= XMAP * YMAP * ZMAP:
        print("Out of range")
    Q_cost.append((source,0))
    while Q_cost:
        tmp = Q_cost.pop(0)
        for n in neighbors:
            cost = travel_cost(tmp[0], tmp[0] + n)
            if cost == 1 and (dist[tmp[0] + n] == 0 or (dist[tmp[0] + n] > (tmp[1] + cost))):
                prev[tmp[0] + n] = tmp[0]
                dist[tmp[0] + n] = cost + tmp[1]
                Q_cost.append((tmp[0] + n, dist[tmp[0] + n]))
                Q_cost.sort()
    prev[source] = -1
    dist[source] = 0
    return prev

def traceback(prev,source,dest):
    finalPath = []
    finalPath.append(dest)
    tmp = dest
    while prev[tmp] != -1:
        tmp = prev[tmp]
        finalPath.insert(0,tmp)
    if tmp != source:
        finalPath = []
    return finalPath

if __name__ == "__main__":
	default_x, default_y, default_z = 0.0
	position = argparse.ArgumentParser(description="Map on Gazebo")
	position.add_argument('-s', '--spose', nargs=3, default=[default_x, default_y, default_z], help='starting x, y, z')
	position.add_argument('-g', '--gpose', nargs=3, default=[default_x, default_y, default_z], help='goal x, y, z')
	args = pose.parse_args()
