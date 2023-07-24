
import math
from math import pi, cos, sin, atan2
import sys
import time
#import threading
from random import randrange, random, seed
from matplotlib import pyplot as plt
#from PIL import Image

import numpy as np
import vrep

# --------------------------------------------------------------------------

robotPosition = [3.0, 0.5] #POSICION EXPLICITA INICIAL DEL ROBOT
points = [[0.5, 0.5] , [2.5,3.4], [2.5, 1.5]] #PUNTOS DESTINO
resolution = 0.2 #RESOLUCION DEL GRID


grid = np.loadtxt("grid.txt", dtype = np.int16)

verticesSet = []
paths = []


# --------------------------------------------------------------------------

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

#FUNCION QUE ENCUENTRA EL CAMINO MAS CORTO DADO UN GRAFO Y VERTICES INICIO Y FIN
def find_shortest_path(graph, start, end, path=[]):

    path = path + [start]

    if start == end:
        return path

    shortest = []

    for i in range(len(graph[start])):
        if ((i not in path) and graph[start][i] != -10):
            newpath = find_shortest_path(graph, i, end, path)
            if newpath:
                if not shortest or len(newpath) < len(shortest):
                    shortest = newpath
    return shortest

# --------------------------------------------------------------------------

#PROCEDIMIENTO QUE, DADO EL GRID DE UNA ESCENA, SE ENCARGA DE CONSTRUIR LOS CAMINOS MEDIANTE UNA MÁSCARA DE 2x2
def pathConstruction():

    thresh = 8

    for j in range(1, len(grid)-1):
        for i in range (1, len(grid)-1):

            if (grid[i][j] > thresh):

                if (grid [i-1, j] < thresh): #ESQUINA SUPERIOR DE OBSTACULO
                    if (grid[i, j-1] < thresh):
                        coordinates = [(i-2), (j-2)]
                        verticesSet.append(coordinates) if not(coordinates in verticesSet) else None #COORDENADAS DEL VERTICE ESQUINA SUPERIOR-IZQUIERDA
                    elif (grid[i, j+1] < thresh):
                        coordinates = [(i-2), (j+2)]
                        verticesSet.append(coordinates) if not(coordinates in verticesSet) else None #COORDENADAS DEL VERTICE ESQUINA SUPERIOR-DERECHA

                elif (grid[i+1, j] < thresh): #ESQUINA INFERIOR DE OBSTACULO
                    if (grid[i, j-1] < thresh):
                        coordinates = [(i+2), (j-2)]
                        verticesSet.append(coordinates) if not(coordinates in verticesSet) else None #COORDENADAS DEL VERTICE ESQUINA INFERIOR-IZQUIERDA
                    elif (grid[i, j+1] < thresh):
                        coordinates = [(i+2), (j+2)]
                        verticesSet.append(coordinates) if not(coordinates in verticesSet) else None #COORDENADAS DEL VERTICE ESQUINA INFERIOR-DERECHA
    
    #NO EXISTE ARISTA SI EL VALOR ES -10
    paths = np.full( (len(verticesSet),len(verticesSet)), -10.0)

    thresh = 30

    for i in range(len(verticesSet)):
        for j in range(len(verticesSet)):
            if i != j:
                #CREACION DEL ARRAY FORMADO ENTRE UN VERTICE Y OTRO PARA DETERMINAR SI EXISTE OBSTACULO ENTRE ELLOS (SUMA DE ELEMENTOS DEL SUBARRAY)
                #EN CASO DE QUE NO EXISTA OBSTACULO, ESTABLECER ARISTA CON LA ORIENTACION QUE SE HA DE SEGUIR PARA IR DEL VERTICE i AL j (DEL j al i => SUMAR pi)

                #CON ESTOS CONDICIONALES SE ESTABLECEN CORRECTAMENTE LOS LIMITES DEL SUBARRAY
                if verticesSet[i][0] < verticesSet[j][0]:
                    if verticesSet[i][1] < verticesSet[j][1]:
                        if (np.sum( grid [verticesSet[i][0]:verticesSet[j][0]+1, verticesSet[i][1]:verticesSet[j][1]+1] ) < thresh):
                            paths[i,j] = atan2( (verticesSet[i][1]-verticesSet[j][1]), (verticesSet[i][0]-verticesSet[j][0]))
                            paths[j,i] = pi + paths[i,j]
                    else:
                        if (np.sum( grid [verticesSet[i][0]:verticesSet[j][0]+1, verticesSet[j][1]:verticesSet[i][1]+1] ) < thresh):
                            paths[i,j] = atan2( (verticesSet[i][1]-verticesSet[j][1]), (verticesSet[i][0]-verticesSet[j][0]))
                            paths[j,i] = pi + paths[i,j]
                else:
                    if verticesSet[i][1] < verticesSet[j][1]:
                        if (np.sum( grid [verticesSet[j][0]:verticesSet[i][0]+1, verticesSet[i][1]:verticesSet[j][1]+1] ) < thresh):
                            paths[i,j] = atan2( (verticesSet[i][1]-verticesSet[j][1]), (verticesSet[i][0]-verticesSet[j][0]))
                            paths[j,i] = pi + paths[i,j]
                    else:
                        if (np.sum( grid [verticesSet[j][0]:verticesSet[i][0]+1, verticesSet[j][1]:verticesSet[i][1]+1] ) < thresh):
                            paths[i,j] = atan2( (verticesSet[i][1]-verticesSet[j][1]), (verticesSet[i][0]-verticesSet[j][0]))
                            paths[j,i] = pi + paths[i,j]

    #np.savetxt('grafo.txt', paths, fmt = '%f', delimiter='\t')
    return paths

# --------------------------------------------------------------------------

#FUNCION QUE CALCULA LA TRAYECTORIA QUE SE HA DE HACER PARA QUE EL ROBOT VAYA DE UN PUNTO ORIGEN A OTRO DESTINO
def robotTrajectory(origin, destination): #origin, destination = [x,y]

    #DISCRETIZAR LAS COORDENADAS EN VALORES PARA EL GRID
    orig = [int(origin[0]/resolution),int(origin[1]/resolution)]
    dest = [int(destination[0]/resolution),int(destination[1]/resolution)]

    #VALOR UMBRAL DE EXISTENCIA DE OBSTACULO EN UN AREA
    thresh = 30

    #ENCONTRAR UN VERTICE DEL GRAFO ACCESIBLE DESDE EL ORIGEN
    initialPoint = 0
    for i in verticesSet:
        if orig[0] < i[0]:
            if orig[1] < i[1]:
                if (np.sum( grid [orig[0]:i[0]+1, orig[1]:i[1]+1] ) < thresh):
                    initialPoint = i
                    break
            else:
                if (np.sum( grid [orig[0]:i[0]+1, i[1]:orig[1]+1] ) < thresh):
                    initialPoint = i
                    break                    
        else:
            if orig[1] < i[1]:
                if (np.sum( grid [i[0]:orig[0]+1, orig[1]:i[1]+1] ) < thresh):
                    initialPoint = i
                    break                    
            else:
                if (np.sum( grid [i[0]:orig[0]+1, i[1]:orig[1]+1] ) < thresh):
                    initialPoint = i
                    break


    #ENCONTRAR UN VERTICE DEL GRAFO ACCESIBLE DESDE EL DESTINO
    finalPoint = 0
    for i in verticesSet:
        if dest[0] < i[0]:
            if dest[1] < i[1]:
                if (np.sum( grid [dest[0]:i[0]+1, dest[1]:i[1]+1] ) < thresh):
                    finalPoint = i
                    break
            else:
                if (np.sum( grid [dest[0]:i[0]+1, i[1]:dest[1]+1] ) < thresh):
                    finalPoint = i
                    break                    
        else:
            if dest[1] < i[1]:
                if (np.sum( grid [i[0]:dest[0]+1, dest[1]:i[1]+1] ) < thresh):
                    finalPoint = i
                    break                    
            else:
                if (np.sum( grid [i[0]:dest[0]+1, i[1]:dest[1]+1] ) < thresh):
                    finalPoint = i
                    break
    

    #ESTABLECER EL CAMINO A SEGUIR PARA LLEGAR DEL ORIGEN AL DESTINO
    trajectory = find_shortest_path(paths,verticesSet.index(initialPoint),verticesSet.index(finalPoint))


    #ESTABLECER ADECUADAMENTE LA SECUENCIA DE PUNTOS A SEGUIR
    sequence = []
    for i in range(len(trajectory)):
        sequence.append(verticesSet[trajectory[i]].copy())
        sequence[i][0] *= resolution
        sequence[i][1] *= resolution
    sequence.append(destination.copy())

    #ESTABLECER LA ORIENTACION QUE HA DE LLEVAR EL ROBOT DESDE UN PUNTO PARA LLEGAR AL SIGUIENTE
    angles = []
    angles.append(atan2( (verticesSet[trajectory[0]][1]-orig[1]), (verticesSet[trajectory[0]][0]-orig[0])))

    
    if(angles[0] < 0):
        angles[0] += 2*pi #AJUSTE (-pi, pi) => (0, 2pi)

    for i in range(len(trajectory)-1):
        angles.append(paths[trajectory[i+1],trajectory[i]])
        if (angles[i+1] < 0):
            angles[i+1] += 2*pi

    angles.append(atan2( (dest[1] - verticesSet[trajectory[len(trajectory)-1]][1]), (dest[0] - verticesSet[trajectory[len(trajectory)-1]][0])))

    if(angles[len(angles)-1] < 0):
        angles[len(angles)-1] += 2*pi
        

    #DEVOLVER LA SECUENCIA DE PUNTOS Y ORIENTACIONES PARA LLEGAR DEL ORIGEN AL DESTINO DADOS
    return sequence, angles

# --------------------------------------------------------------------------

#DEFINE LA VELOCIDAD PARA FIJAR UNA ORIENTACION DETERMINADA
def setOrientation(robotOri, desired):
        
    robotOri = round(robotOri,3)
    desired = round(desired,3)

    if (robotOri > desired):
        lspeed, rspeed = abs(robotOri-desired+0.001), -abs(robotOri-desired+0.001)
    else:
        lspeed, rspeed = -abs(robotOri-desired+0.001), abs(robotOri-desired+0.001)

    return lspeed, rspeed

# --------------------------------------------------------------------------

paths = pathConstruction()

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

        
        setSpeed(clientID, hRobot, 0.0, 0.0)
        time.sleep(3)

        pos = robotPosition #POSICION INICIAL EXPLICITA DEL ROBOT (PRIMERA LECTURA INADECUADA)
        count = 0
        
        for i in points:
            print(f"SIGUIENTE DESTINO: {i}")
            trajectory, orientation = robotTrajectory(pos, i)

            while vrep.simxGetConnectionId(clientID) != -1:

                # PERCEPCION
                robotOri = getRobotHeading(clientID, hRobot)
                pos = getRobotPosition(clientID, hRobot)

                # Reescalado de las coordenadas y de la orientacion
                if robotOri < 0:
                    robotOri += 2*pi
                    
                pos[0] += 2.0
                pos[1] += 2.0


                # PLANIFICACION
                if (round(robotOri,2) != round(orientation[count],2)):
                    lspeed, rspeed = setOrientation(robotOri, orientation[count])
                else:
                    if (abs(pos[0] - (trajectory[count][0])) > 0.15 or
                        abs(pos[1] - (trajectory[count][1])) > 0.15):
                        lspeed, rspeed = 1.0, 1.0
                    else:
                        count += 1

                if count == (len(trajectory)):
                    print("DESTINO ALCANZADO!!\n")
                    lspeed, rspeed = 0.0 , 0.0
                    setSpeed(clientID, hRobot, lspeed, rspeed)
                    count = 0
                    time.sleep(1)
                    break

                # ACCION
                setSpeed(clientID, hRobot, lspeed, rspeed)

                time.sleep(0.1)

            pos = getRobotPosition(clientID, hRobot)
            pos[0] += 2.0
            pos[1] += 2.0

        print("FIN DE TRAYECTORIA.")
        print('### Finishing...')
        vrep.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()

