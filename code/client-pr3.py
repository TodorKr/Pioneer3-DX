#!/usr/bin/python

print('### Script:', __file__)

import math
from math import pi, cos, sin
import sys
import time
from random import randrange, random, seed
from matplotlib import pyplot as plt

import numpy as np
import vrep

# --------------------------------------------------------------------------

sensOri = [pi/2, 5*pi/18, pi/6, pi/18, -pi/18, -pi/6, -5*pi/18, -pi/2, -pi/2, -13*pi/18, -5*pi/6, -17*pi/18, 17*pi/18, 5*pi/6, 13*pi/18, pi/2]
sensDist = 0.2
squareLength = 4 #SCENE'S EDGE LENGTH
resolution = 0.2 #RESOLUTION FOR THE GRID
dim = int(squareLength/resolution) #GRID DIMENSION
grid = np.zeros((dim,dim))
seed(1)


def getRobotHandles(clientID):
    # Robot handle
    _,rbh = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx',
                                     vrep.simx_opmode_blocking)
    
    # Motor handles
    _,lmh = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                     vrep.simx_opmode_blocking)
    _,rmh = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                     vrep.simx_opmode_blocking)

    # Sonar handles
    str = 'Pioneer_p3dx_ultrasonicSensor%d'
    sonar = [0] * 16
    for i in range(16):
        _,h = vrep.simxGetObjectHandle(clientID, str % (i+1),
                                       vrep.simx_opmode_blocking)
        sonar[i] = h
        vrep.simxReadProximitySensor(clientID, h, vrep.simx_opmode_streaming)

    return [lmh, rmh], sonar, rbh

# --------------------------------------------------------------------------

def setSpeed(clientID, hRobot, lspeed, rspeed):
    vrep.simxSetJointTargetVelocity(clientID, hRobot[0][0], lspeed,
                                    vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID, hRobot[0][1], rspeed,
                                    vrep.simx_opmode_oneshot)

# --------------------------------------------------------------------------

def getSonar(clientID, hRobot):
    r = [1.0] * 16
    for i in range(16):
        handle = hRobot[1][i]
        e,s,p,_,_ = vrep.simxReadProximitySensor(clientID, handle,
                                                 vrep.simx_opmode_buffer)
        if e == vrep.simx_return_ok and s:
            r[i] = math.sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])

    return r

# --------------------------------------------------------------------------

def getRobotPosition(clientID, hRobot):
    _,rpos = vrep.simxGetObjectPosition(clientID, hRobot[2], -1,
                                           vrep.simx_opmode_streaming)
    return rpos[0:2]

# --------------------------------------------------------------------------

def getRobotHeading(clientID, hRobot):
    _,reul = vrep.simxGetObjectOrientation(clientID, hRobot[2], -1,
                                           vrep.simx_opmode_streaming)
    return reul[2]

# --------------------------------------------------------------------------
# --------------------------------------------------------------------------

def avoid(sonar):
    left = sonar[1]
    frontLeft = sonar[3]
    frontRight = sonar[4]
    right = sonar[6]
    #Adicionalmente, se utilizan los sensores 0, 2, 5 y 7: TODOS LOS FRONTALES

    #SI HAY UNA PARED EN FERNTE
    if ((sonar[2] < 0.2 and sonar[5] < 0.2)):
        lspeed, rspeed = -(sonar[2]+0.1), -(sonar[5]+0.1)
    
    elif (frontLeft < 0.2 and frontRight < 0.2):
        lspeed,rspeed = -(frontLeft+0.1), -(frontRight+0.1)

    elif ((frontLeft < 0.3) or (frontRight < 0.3)):
        
        if (abs(left - right) < 0.05): #SI TENEMOS LA MISMA DIST A AMBOS LADOS: ELECCION ALEATORIA
            if (random() > 0.5):
                lspeed, rspeed = -0.3, +0.5
            else:
                lspeed, rspeed = +0.5, -0.3
        else: #IR HACIA EL LADO CON MAYOR DISTANCIA SIN OBSTACULO
            if (left > right):
                lspeed, rspeed = -0.3, +0.5
            else:
                lspeed, rspeed = +0.5, -0.3
    


    elif (sonar[2] < 0.2):
        lspeed, rspeed = 0.5-sonar[2], -sonar[2]

    elif (sonar[5] < 0.2):
        lspeed, rspeed = -sonar[5], 0.5-sonar[5]

    else: #SI NO HAY NINGUNA PARED EN FRENTE:

        #SI RECORRE UN PASILLO
        if ( ( (left < 0.25) and (right < 0.25) ) ):
            #ENDEREZAR LA DIRECCION A TRAVES DEL PASILLO
            lspeed, rspeed = right/(right+left), left/(right+left)

        elif ((sonar[0] < 0.1) and (sonar[7] < 0.1)):
            lspeed, rspeed = sonar[7]/(sonar[7]+sonar[0]), sonar[0]/(sonar[7]+sonar[0])

        #SI NO ESTA EN UN PASILLO, MANTENER UNA DISTANCIA DE 0.15 CON LA PARED LATERAL
        elif (left < 0.15):
            lspeed, rspeed = 0.5-left, left
        elif (right < 0.15):
            lspeed, rspeed = right, 0.5-right
        elif (sonar[0] < 0.1):
            lspeed, rspeed = 0.5-sonar[0], sonar[0]
        elif (sonar[7] < 0.1):
            lspeed, rspeed = sonar[7], 0.5-sonar[7]

        #SI LA POSICION NO ES CRITICA, REALIZAR GIROS ALEATORIOS PARA INCENTIVAR LA EXPLORACION
        elif(random() < 0.1):
            if (left > right):
                lspeed, rspeed = -0.5, +0.5
            else:
                lspeed, rspeed = 0.5, -0.5
        else:
            lspeed, rspeed = +0.5, +0.5

    return lspeed*3, rspeed*3

# --------------------------------------------------------------------------
 

def updateGrid(pos, robotOri, sonar):
    for i in range(len(sonar)):
        offset = 0.08
        if (sonar[i] < 0.95):
            if (i > 7):
                offset = -0.08
            x = pos[0] + (sonar[i]+sensDist) * cos (robotOri + sensOri[i]) + offset * cos (robotOri)
            y = pos[1] + (sonar[i]+sensDist) * sin (robotOri + sensOri[i]) + offset * sin (robotOri)
            
            x = int(x / resolution)
            y = int(y / resolution)
            if (x >= dim):
                x = dim-1
            if (y >= dim):
                y = dim-1
            grid[x][y] += 1

# --------------------------------------------------------------------------

def main():
    print('### Program started')

    print('### Number of arguments:', len(sys.argv), 'arguments.')
    print('### Argument List:', str(sys.argv))

    vrep.simxFinish(-1) # just in case, close all opened connections

    port = int(sys.argv[1])
    clientID = vrep.simxStart('127.0.0.1', port, True, True, 2000, 5)

    if clientID == -1:
        print('### Failed connecting to remote API server')

    else:
        print('### Connected to remote API server')
        hRobot = getRobotHandles(clientID)

        #gridPrinter()
        counter = 0
        while vrep.simxGetConnectionId(clientID) != -1:
            counter += 1

            # Perception
            robotOri = getRobotHeading(clientID, hRobot)
            pos = getRobotPosition(clientID, hRobot)
            sonar = getSonar(clientID, hRobot)

            # Rescale of the coordinates' system
            pos[0] += 2.0
            pos[1] += 2.0

            #print(f"P: {pos}; Th: {math.degrees(getRobotHeading(clientID, hRobot))}")
            updateGrid(pos, robotOri, sonar)
            # Planning
            lspeed, rspeed = avoid(sonar)
            # Action
            setSpeed(clientID, hRobot, lspeed, rspeed)

            if (counter == 1500):
                #print(grid)
                setSpeed(clientID, hRobot, 0, 0)
                np.savetxt('grid.txt', grid, fmt = '%d', delimiter='\t')
                plt.imshow(grid, interpolation='nearest')
                plt.show()
                #img = Image.new('L',(dim,dim))
                #img.putdata(grid)
                #img.save('obstaculos.png')
                counter = 0
                break
            time.sleep(0.1)

        print('### Finishing...')
        vrep.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
