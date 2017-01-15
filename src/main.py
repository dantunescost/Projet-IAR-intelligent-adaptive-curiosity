# -*- coding: utf-8 -*-
"""
@author: Daniel Antunes & Cédric Hubert

"""

import vrep.py

vrep.simxFinish(-1) # fermeture de toutes les connexions ouvertes
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) #etablissement de la connexion avec V-REP