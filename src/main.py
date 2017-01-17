# -*- coding: utf-8 -*-
"""
@author: Daniel Antunes & Cédric Hubert

"""

import vrep
import sys
import numpy as np
import random

# declaration des variables
DELAY = 150
NB_ITERATIONS = 1500
NS = 250
LE = []	#liste erreur à chaque pas de temps
LEm= [] #liste erreur moyenne à chaque pas de temps

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
    
    
    
    
def bouclePrincipale():
	t=0
	possibleActions = []	#liste d'actions possibles à ce step
    LPActions= []			#learning progress calculé pour chaque action

	while t < NB_ITERATIONS:
		'''
			Selection de l'action
		'''
		for i in range(len(LPActions)):
		
			Ep = MetaPredictionMP(possibleActions[i]) #calcul de la prediction de l'erreur
			tempLE = liste(LE) #on clone LE
			tempLE.append(Ep) #on rajoute à la liste clonée l'erreur prédite
			Emp= np.mean(tempLE[-DELAY:])
			LP= -(-Emp-LEm[t+1-DELAY])
			LPActions.append(LP)
		
		indiceActionChoisie = 0
		if(random.random() > 0.1):			#exploitation
			indiceActionChoisie = np.argmax(LPactions)
		else:								#exploration
			indiceActionChoisie = 0

		'''
			Prediction de la machine P
		'''
			S = PredictionP(possibleActions[indiceActionChoisie])
		'''
			Réalisation de l'action dans le simulateur
		'''
		#TODO

		#On stoppe le robot
		vrep.simxSetJointTargetVelocity(clientID,leftMotor,0,vrep.simx_opmode_oneshot)
		vrep.simxSetJointTargetVelocity(clientID,rightMotor,0,vrep.simx_opmode_oneshot)
		t += 1
		'''
			Vérification résultat action
		'''
		#TODO calcul de la distance, ce serait bien de faire avec senseurs, sinon directement avec les fonctions de VREPs
		Sa=0
		#calcul de l'erreur
		E= abs(S-Sa)
		#maj listes
		LE.append(E)
		Em= np.mean(LE[-DELAY:])
		LEm.append(Em)
    
def MetaPredictionMP(action):
	#TODO

def PredictionP(action):
	#TODO
    
