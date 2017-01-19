# -*- coding: utf-8 -*-
"""
@author: Daniel Antunes & Cédric Hubert

"""

import vrep, copy, sys, random, operator#,time
import numpy as np
import matplotlib.pyplot as plt

# declaration des constantes
DELAY = 150
NB_ITERATIONS = 1500
NB_ACTIONS_ECHANTILLONAGE = 40
NS = 250
K = 10
# declaration des variables
DATA = []       #base contenant les experts
LE = []         #liste erreur à chaque pas de temps
LEm= []         #liste erreur moyenne à chaque pas de temps
data_MP = []    #de la forme [speedLeft,speedRight,frequence,erreur de prediction]
data_P = []     #de la forme [speedLeft,speedRight,frequence,distance avant action ,distance après action de la balle]
Cut = []
Cut.append("")

dict = {'Name': 'Zara', 'Age': 7, 'Class': 'First'}
# Creation du premier expert (unique), child1 contient les valeurs en-dessous de cutVal
DATA.append({"LE" : LE, "LEm" : LEm, "data_P" : data_P, "indexDim" : -1, "cutVal" : -99, "child1" : -1, "child2" : -1})


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


def execute_action(cID, leftHandle, rightHandle, action, botHandle, ballHandle):
    vrep.simxSetJointTargetVelocity(cID,leftHandle,action[0]*5,vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(cID,rightHandle,action[1]*5,vrep.simx_opmode_oneshot)
    if action[2] > 0.66 and action[2] <= 1:
        # ball jumps to robot
        returnCode1, pos1 = vrep.simxGetObjectPosition(cID,botHandle,-1,vrep.simx_opmode_oneshot_wait)
        returnCode2, pos2 = vrep.simxGetObjectPosition(cID,ballHandle,-1,vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectPosition(cID,ballHandle,-1,[pos1[0],pos1[1],pos2[2]],vrep.simx_opmode_oneshot)
    if action[2] >= 0 and action[2] <= 0.33:
        # ball goes to random position
        returnCode1, pos1 = vrep.simxGetObjectPosition(cID,botHandle,-1,vrep.simx_opmode_oneshot_wait)
        returnCode2, pos2 = vrep.simxGetObjectPosition(cID,ballHandle,-1,vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectPosition(cID,ballHandle,-1,[random.uniform(-2.3,2.3),random.uniform(-2.3,2.3),pos2[2]],vrep.simx_opmode_oneshot)
       
def distance(cID):
    retCode, dist = vrep.simxGetDistanceHandle(cID,"distance",vrep.simx_opmode_oneshot)
    retCode, distance = vrep.simxReadDistance(cID,dist,vrep.simx_opmode_oneshot)
    return distance
    
    
    
    
def bouclePrincipale():
    t=0

    while t < NB_ITERATIONS:
        possibleActions = []    #liste d'actions possibles à ce step
        LPActions= []           #learning progress calculé pour chaque action
        '''
            Génération liste d'actions possibles
        '''
        for i in range(NB_ACTIONS_ECHANTILLONAGE):
            possibleActions.append( [random.uniform(-1,1) , random.uniform(-1,1) , random.uniform(0,1)] )
        '''
            Selection de l'action
        '''
        distanceInitiale=distance(clientID)

        for i in range(len(possibleActions)):
        
            Ep = MetaPredictionMP(possibleActions[i]) #calcul de la prediction de l'erreur
            expert = expertAUtiliser(possibleActions[i],distanceInitiale)
            tempLE = list(expert['LE']) #on clone LE
            tempLE.append(Ep) #on rajoute à la liste clonée l'erreur prédite
            if t == 0:
                LP = Ep
            else:
                if t < DELAY: 
                    Emp = np.mean(tempLE)
                    LP = -(Emp-expert['LEm'][0])
                else:
                    Emp= np.mean(tempLE[-DELAY:]) 
                    LP = -(Emp-expert['LEm'][len(expert['LE'])-DELAY]) #len(expert['LE]) contient le nombre de fois que cet expert a ete utilise
            LPActions.append(LP)

        if(random.random() > 0.1):          #exploitation
            indiceActionChoisie = np.argmax(LPActions)
        else:                               #exploration
            indiceActionChoisie = 0

        # on calcule l'expert a utiliser pour cette action
        expert = expertAUtiliser(possibleActions[indiceActionChoisie],distanceInitiale)
        '''
            Prediction de la machine P
        '''
        S = PredictionP(possibleActions[indiceActionChoisie],expert,distanceInitiale)
        
        
        '''
            Réalisation de l'action dans le simulateur
        '''
        execute_action(clientID,leftMotor,rightMotor,possibleActions[indiceActionChoisie],robotHandle,balle)
        #time.sleep(0.500)
        
        '''
            Vérification résultat action
        '''
        # calcul de la distance, ce serait bien de faire avec senseurs, sinon directement avec les fonctions de VREP
        Sa = distance(clientID)
        #sauvegarde dans data_P
        ajoutData=list(possibleActions[indiceActionChoisie])
        ajoutData.append(distanceInitiale)
        ajoutData.append(Sa)
        expert['data_P'].append(ajoutData)
        #calcul de l'erreur
        E = abs(S-Sa)
        #sauvegarde dans data_MP
        ajoutData=list(possibleActions[indiceActionChoisie])
        ajoutData.append(E) 
        data_MP.append(ajoutData)
        #maj listes
        expert['LE'].append(E) 
        if len(LE) < DELAY:
            Em = 0
        else: 
            Em = np.mean(expert['LE'][-DELAY:]) 
        expert['LEm'].append(Em)
        
        #on verifie si on doit ajouter un expert
        if len(expert['data_P']) == NS:
            Cut[0] += cutExpert(expert,len(DATA))
        print(t)
        t += 1
    
    for i in range(len(DATA)):
        plt.plot(DATA[i]['LEm'])
    plt.show()
    print Cut[0]
    return 0

def MetaPredictionMP(action):
    d=[]    #on va ranger dans cette liste l'écart entre notre action et chaque exemple de la bdd
    res=0   #valeur moyenne des K plus proches voisins, à retourner
    if len(data_MP) == 0:
        return res
    if len(data_MP) < K:
        for i in range(len(data_MP)):
            res += data_MP[i][3]
        res = res / len(data_MP)
    else:
        for i in range(len(data_MP)):
            d1=abs(data_MP[i][0]-action[0])
            d2=abs(data_MP[i][1]-action[1])
            d3=abs(data_MP[i][2]-action[2])
            dtot=d1+d2+d3
            d.append([dtot,i])
        d.sort()    #on trie dans l'ordre croissant des écarts
        for i in range(K):
            res += data_MP[d[i][1]][3]
        res = res / K
    return res

def PredictionP(action,expert,distanceInitiale):
    d=[]    #on va ranger dans cette liste l'écart entre notre action et chaque exemple de la bdd
    res=0   #valeur moyenne des K plus proches voisins, à retourner
    data_P = expert['data_P']
    if len(data_P) == 0:
        return res
    if len(data_P) < K:
        for i in range(len(data_P)):
            res += data_P[i][4]
        res = res / len(data_P)
    else:
        dtot=0
        for i in range(len(data_P)):
            d1=abs(data_P[i][0]-action[0])
            d2=abs(data_P[i][1]-action[1])
            d3=abs(data_P[i][2]-action[2])
            d4=abs(data_P[i][3]-distanceInitiale)   #On ajoute la distance initiale parmis les parametres sensori-moteurs
            dtot=d1+d2+d3+d4*(1/2)  #somme pondérée
            
            d.append([dtot,i])
        d.sort()    #on trie dans l'ordre croissant des écarts
        if K <= len(data_P):
            for i in range(K):
                res += data_P[d[i][1]][4]
            res = res / K
    return res
    
    
def cutExpert(expert,taille):       #Fonction qui permet de générer 2 experts à partir d'un expert parent
    #print("CUUUUUUUUUUUUUUUUUUUUUUUT")
    dataCopy = copy.copy(expert['data_P'])  #On copie la base de donnée de l'expert original
    stdMin = 1000
    dim = -1
    indexCutVal = -1
    cutVal = -99
    for i in range(len(expert['data_P'][0])-1):     #on parcoure les dimensions speedLeft, speedRight, Frequence
        dataCopy.sort(key=operator.itemgetter(i))
        for j in range((len(dataCopy)//2)-(len(dataCopy)//20),(len(dataCopy)//2)+(len(dataCopy)//20)):  #on cherche à minimiser la somme des variances de chaque sous-liste
            var = np.std(dataCopy[:j][4]) + np.std(dataCopy[j:][4])
            if var < stdMin:
                stdMin = var
                dim = i
                indexCutVal = j
                cutVal = dataCopy[j][i]
    expert['child1'] = taille
    expert['child2'] = taille + 1
    expert['indexDim'] = dim
    expert['cutVal'] = cutVal
    dataCopy.sort(key=operator.itemgetter(dim))
    DATA.append({"LE" : list(expert['LE']), "LEm" : list(expert['LEm']), "data_P" : list(dataCopy[:indexCutVal]), "indexDim" : -1, "cutVal" : -99, "child1" : -1, "child2" : -1})
    DATA.append({"LE" : list(expert['LE']), "LEm" : list(expert['LEm']), "data_P" : list(dataCopy[indexCutVal:]), "indexDim" : -1, "cutVal" : -99, "child1" : -1, "child2" : -1})
    return ("on coupe a la dimension "+ str(dim) + ", a la valeur " + str(cutVal) + ", à l'index "+str(indexCutVal) + ".\n")    
    
    
def expertAUtiliser(action,distanceInitiale):       #Cette fonction permet de trouver le meilleur expert pour l'action en parcourant le pseudo-arbre
    expert = DATA[0]
    tempAction=list(action)
    tempAction.append(distanceInitiale) 
    
    while expert['child1'] != -1:
        if tempAction[expert['indexDim']] <= expert['cutVal']:
            expert = DATA[expert['child1']]
        else:
            expert = DATA[expert['child2']]
    return expert
    
bouclePrincipale()
    

            