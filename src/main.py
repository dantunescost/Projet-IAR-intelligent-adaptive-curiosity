# -*- coding: utf-8 -*-
"""
@author: Daniel Antunes & Cédric Hubert

"""

import vrep
import sys

# declaration des variables
DELAY = 150
NS = 250


vrep.simxFinish(-1) # fermeture de toutes les connexions ouvertes
clientID = vrep.simxStart('127.0.0.1',19999,True,True,5000,5) #etablissement de la connexion avec V-REP

if clientID == -1:
    print("La connexion a échouée")
    sys.exit("Connexion échouée")
else:
    print("Connexion au seveur remote API établie")
    

# recuperation des "handlers" dans la scene
returnCode, robotHandle = vrep.simxGetObjectHandle(clientID,"Robot", vrep.simx_opmode_oneshot_wait)
returnCode, leftMotor = vrep.simxGetObjectHandle(clientID,"leftMotor", vrep.simx_opmode_oneshot_wait)
returnCode, rightMotor = vrep.simxGetObjectHandle(clientID,"rightMotor", vrep.simx_opmode_oneshot_wait)
returnCode, balle = vrep.simxGetObjectHandle(clientID,"balle", vrep.simx_opmode_oneshot_wait)

print("echec? : " + str(returnCode) + ", robotHandle : " + str(robotHandle))

returnCode, positionRobot = vrep.simxGetObjectPosition(clientID,robotHandle,-1,vrep.simx_opmode_oneshot_wait)

print("echec? : " + str(returnCode) + ", position du Robot : " + str(positionRobot))

#vrep.simxSet(clientID,robotHandle,-1,[-0.38424891233444214, -0.7967437505722046, 0.1386629045009613],vrep.simx_opmode_oneshot_wait)

returnVal = vrep.simxCallScriptFunction(clientID,"",)

def execute_action(cID, leftHandle, rightHandle, (speed_left,speed_right,frequency)):
    vrep.simxSetJointTargetVelocity(cID,leftHandle,-1,vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(cID,rightHandle,-1,vrep.simx_opmode_oneshot)
    
    
    
    
    
    
    
    
    
    
    