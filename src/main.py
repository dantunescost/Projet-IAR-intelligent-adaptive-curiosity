# -*- coding: utf-8 -*-
"""
@author: Daniel Antunes & Cédric Hubert

"""

import vrep
import sys
import numpy as np
import random

# declaration des constantes
DELAY = 150
NB_ITERATIONS = 1500
NB_ACTIONS_ECHANTILLONAGE = 20
NS = 250
K = 5
# declaration des variables
LE = []	#liste erreur à chaque pas de temps
LEm= [] #liste erreur moyenne à chaque pas de temps
data_MP = []	#de la forme [param1,param2,param3,err]



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
			Génération liste d'actions possibles
		'''
		for i in range(NB_ACTIONS_ECHANTILLONAGE):
			possibleActions.append( [random.uniform(-1,1) , random.uniform(-1,1) , random.uniform(0,1)] )
		'''
			Selection de l'action
		'''
		for i in range(len(possibleActions)):
		
			Ep = MetaPredictionMP(possibleActions[i]) #calcul de la prediction de l'erreur
			tempLE = list(LE) #on clone LE
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
		vrep.simxSetJointTargetVelocity(clientID,leftMotor,0,vrep.simx_opmode_oneshot_wait)
		vrep.simxSetJointTargetVelocity(clientID,rightMotor,0,vrep.simx_opmode_oneshot_wait)
		t += 1
		'''
			Vérification résultat action
		'''
		#TODO calcul de la distance, ce serait bien de faire avec senseurs, sinon directement avec les fonctions de VREP
		Sa=0
		#calcul de l'erreur
		E= abs(S-Sa)
		#sauvegarde dans data_MP
		ajoutData=list(possibleActions[indiceActionChoisie])
		ajoutData.append(E)
		data_MP.append(ajoutData)
		#maj listes
		LE.append(E)
		Em= np.mean(LE[-DELAY:])
		LEm.append(Em)
	return 0

def MetaPredictionMP(action):
	d=[]	#on va ranger dans cette liste l'écart entre notre action et chaque example de la bdd
	res=0	#valeur moyenne des K plus proches voisins, à retourner
	for i in range(len(data_MP)):
		d1=abs(data_MP[i][0]-action[0])
		d2=abs(data_MP[i][1]-action[1])
		d3=abs(data_MP[i][2]-action[2])
		dtot=d1+d2+d3
		d.append([dtot,i])
	d.sort()	#on trie dans l'ordre croissant des écarts
	for i in range(K):
		res += data_MP[i][3]
	res = res / K
	return res

def PredictionP(action):
	#TODO
	return 0
    
