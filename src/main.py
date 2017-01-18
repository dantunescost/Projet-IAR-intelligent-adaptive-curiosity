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
NB_ACTIONS_ECHANTILLONAGE = 50
NS = 250
K = 5
# declaration des variables
LE = []         #liste erreur à chaque pas de temps
LEm= []         #liste erreur moyenne à chaque pas de temps
data_MP = []	#de la forme [param1,param2,param3,ecart]
data_P = []	#de la forme [param1,param2,param3,distance]


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

returnVal = vrep.simxCallScriptFunction(clientID,"",)

def execute_action(cID, leftHandle, rightHandle, (speed_left,speed_right,frequency), botHandle, ballHandle):
    vrep.simxSetJointTargetVelocity(cID,leftHandle,speed_left,vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(cID,rightHandle,speed_right,vrep.simx_opmode_oneshot)
    if frequency > 0.66 and frequency <= 1:
        # ball jumps to robot
        returnCode1, pos1 = vrep.simxGetObjectPosition(cID,botHandle,-1,vrep.simx_opmode_oneshot_wait)
        returnCode2, pos2 = vrep.simxGetObjectPosition(cID,ballHandle,-1,vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectPosition(cID,ballHandle,-1,[pos1[0],pos1[1],pos2[2]],vrep.simx_opmode_oneshot)
#        signeX = -1 if (pos2[0]-pos1[0] < 0) else 1
#        signeY = -1 if (pos2[1]-pos1[1] < 0) else 1
#        if np.abs(pos1[0]-pos2[0]) < np.abs(pos1[1]-pos2[1]): # balle plus loin dans l'axe y
#            ratio = np.abs(pos1[0]-pos2[0])/np.abs(pos1[1]-pos2[1])
#            vrep.simxSetObjectPosition(cID,ballHandle,botHandle,[largeurRobot*signeX*ratio,largeurRobot*signeY,0],vrep.simx_opmode_oneshot)
#        else: # balle plus loin sur l'axe x
#            ratio = np.abs(pos1[1]-pos2[1])/np.abs(pos1[0]-pos2[0]) if np.abs(pos1[0]-pos2[0]) != 0 else 1
#            vrep.simxSetObjectPosition(cID,ballHandle,botHandle,[largeurRobot*signeX,largeurRobot*signeY*ratio,0],vrep.simx_opmode_oneshot)
    if frequency >= 0 and frequency <= 0.33:
        # ball goes to random position
        returnCode1, pos1 = vrep.simxGetObjectPosition(cID,botHandle,-1,vrep.simx_opmode_oneshot_wait)
        returnCode2, pos2 = vrep.simxGetObjectPosition(cID,ballHandle,-1,vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectPosition(cID,ballHandle,-1,[random.uniform(-2.3,2.3),random.uniform(-2.3,2.3),pos2[2]],vrep.simx_opmode_oneshot)
       
def distance(cID):
#    returnCode1, position1 = vrep.simxGetObjectPosition(cID,handle1,-1,vrep.simx_opmode_oneshot_wait)
#    returnCode2, position2 = vrep.simxGetObjectPosition(cID,handle2,-1,vrep.simx_opmode_oneshot_wait)
#    if(returnCode1 or returnCode2):
#        print("Erreur dans requête de position")
#        sys.exit("simxGetObjectionPosition error")
#    else:
#        return np.sqrt((position1[0]-position2[0])**2+(position1[1]-position2[1])**2)
    retCode, dist = vrep.simxGetDistanceHandle(cID,"distance",vrep.simx_opmode_oneshot)
    retCode, distance = vrep.simxReadDistance(cID,dist,vrep.simx_opmode_oneshot)
    return distance
    
    
    
    
def bouclePrincipale():
	t=0
	possibleActions = []	#liste d'actions possibles à ce step
	LPActions= []		#learning progress calculé pour chaque action

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
			Emp= np.mean(tempLE[-DELAY:]) #TODO calculer la moyenne des exemples existants s'il n'y en a pas encore DELAY
			LP= -(-Emp-LEm[t+1-DELAY])
			LPActions.append(LP)
		
		if(random.random() > 0.1):			#exploitation
			indiceActionChoisie = np.argmax(LPActions)
		else:								#exploration
			indiceActionChoisie = 0

		'''
			Prediction de la machine P
		'''
		S = PredictionP(possibleActions[indiceActionChoisie])
		'''
			Réalisation de l'action dans le simulateur
		'''
		#TODO executer l'action dans le simulateur

		#On stoppe le robot
#		vrep.simxSetJointTargetVelocity(clientID,leftMotor,0,vrep.simx_opmode_oneshot_wait)
#		vrep.simxSetJointTargetVelocity(clientID,rightMotor,0,vrep.simx_opmode_oneshot_wait)
		t += 1
		'''
			Vérification résultat action
		'''
		# calcul de la distance, ce serait bien de faire avec senseurs, sinon directement avec les fonctions de VREP
		Sa = distance(clientID)
		#sauvegarde dans data_P
		ajoutData=list(possibleActions[indiceActionChoisie])
		ajoutData.append(Sa)
		data_P.append(ajoutData)
		#calcul de l'erreur
		E = abs(S-Sa)
		#sauvegarde dans data_MP
		ajoutData=list(possibleActions[indiceActionChoisie])
		ajoutData.append(E) #a verfifier
		data_MP.append(ajoutData)
		#maj listes
		LE.append(E) 
		Em = np.mean(LE[-DELAY:]) #TODO que si t > DELAY
		LEm.append(Em)
	return 0

#TODO pour les deux algos k-moyenne, renvoyer la moyenne des existants au lieu de 0 dans le cas ou len(data) < K ???
def MetaPredictionMP(action):
	d=[]	#on va ranger dans cette liste l'écart entre notre action et chaque exemple de la bdd
	res=0	#valeur moyenne des K plus proches voisins, à retourner
	for i in range(len(data_MP)):
		d1=abs(data_MP[i][0]-action[0])
		d2=abs(data_MP[i][1]-action[1])
		d3=abs(data_MP[i][2]-action[2])
		dtot=d1+d2+d3
		d.append([dtot,i])
	d.sort()	#on trie dans l'ordre croissant des écarts
	if K <= len(data_MP):
		for i in range(K):
			res += data_MP[i][3]
		res = res / K
	return res

def PredictionP(action):
	d=[]	#on va ranger dans cette liste l'écart entre notre action et chaque exemple de la bdd
	res=0	#valeur moyenne des K plus proches voisins, à retourner
	for i in range(len(data_P)):
		d1=abs(data_P[i][0]-action[0])
		d2=abs(data_P[i][1]-action[1])
		d3=abs(data_P[i][2]-action[2])
		dtot=d1+d2+d3
		d.append([dtot,i])
	d.sort()	#on trie dans l'ordre croissant des écarts
	if K <= len(data_P):
		for i in range(K):
			res += data_P[i][3]
		res = res / K
	
	return res
    
