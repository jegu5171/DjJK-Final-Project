import numpy as np

#these can all be set as needed depending on size of drone used and how large you want the map representation to be
ZDRONE = 1 #drone depth
YDRONE = 1 #drone height
XDRONE = 1 #drone width

#values were set to 8 when testing addPadding, and 2 when testing createRepresentation
#just have to make sure they are divisible into the dimensions of the world list evenly
ZMAP = 2 #map depth
YMAP = 2 #map height
XMAP = 2 #map width

representation = np.zeros(shape=(ZMAP, YMAP, XMAP)) #add size of drone onto each edge so its easier to pad, this can be removed later

#adds obstacles to the representation of the size of the drone for all locations adjacent to a current obstacle
def addPadding(representation):
    paddedRep = np.zeros(shape=(ZMAP+2*ZDRONE, YMAP+2*YDRONE, XMAP+2*XDRONE)) #add size of drone onto each edge so its easier to pad, this can be removed later
    for z in range(representation.shape[0]):
        for y in range(representation.shape[1]):
            for x in range(representation.shape[2]):
                if representation[z, y, x] == 1:
                    #print(paddedRep[w:w+2*ZDRONE+1, h:h+2*YDRONE+1, d:d+2*XDRONE+1])
                    paddedRep[z:z+2*ZDRONE+1, y:y+2*YDRONE+1, x:x+2*XDRONE+1] = 1
    return paddedRep[ZDRONE:ZDRONE+ZMAP, YDRONE:YDRONE+YMAP, XDRONE:XDRONE+XMAP] #removes the added values on the edges of map that allowed paddedunit to be applied unconditionally

#since im not sure how the gazebo sim will be accessed atm, im creating this as if it was set up in a 3d numpy array, this can be left alone by creating a different function to turn that world into a compatible array
#so this will take a current 3d array of undetermined size and fit it into the desired representation map size
def createRepresentation(worldList, representation):
    #assumption made that the worldList is larger than the desired map size otherwise there is no point in creating a representation of a given granularity
    #additionall requirement is that the three dimensions of the desired representation will divide evenly into the worldList
    zfactor = int(worldList.shape[0]/ZMAP)
    yfactor = int(worldList.shape[1]/YMAP)
    xfactor = int(worldList.shape[2]/XMAP)
    for z in range(0,worldList.shape[0], zfactor):
        for y in range(0, worldList.shape[1], yfactor):
            for x in range(0, worldList.shape[2], xfactor):
                mean = np.mean(worldList[z:z+zfactor, y:y+yfactor, x:x+xfactor])
                #print(mean)
                if mean > .5: #more than half of the original cells were full we will call it an obstacle, can get away with this since the way padding works it makes technically perfect amount of room as an obstacle so there is slight wiggle room
                    representation[int(z/zfactor)][int(y/yfactor)][int(x/xfactor)] = 1
    return representation



#example of createRepresentation working (have map dimensions all set to 2)
worldList = np.array([[[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]],
             [[0, 0, 0, 0], [0, 0, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
             [[0, 0, 0, 0], [0, 1, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]],
             [[1, 1, 0, 0], [1, 1, 0, 0], [1, 1, 0, 0], [1, 1, 1, 1]]])
print(worldList.shape)
print(createRepresentation(worldList, representation))


#example of addPadding working (set map dimensions to 8)
representation[3][5][5:8] = 1
representation[1][0][1] = 1
print(representation)
print(addPadding(representation))