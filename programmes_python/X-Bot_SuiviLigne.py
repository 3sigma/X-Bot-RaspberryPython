#!/usr/bin/python
# -*- coding: utf-8 -*-

# Import WiringPi2
import wiringpi2

# Imports pour l'i2c
from a_star import AStar
a_star = AStar()

import time, sched
import os
import threading
import signal
import json
import sys

# Pour la détection d'adresse IP
import socket
import fcntl
import struct

# Pour le serveur de socket
import tornado.httpserver
import tornado.ioloop
from tornado.ioloop import PeriodicCallback
import tornado.web
import tornado.websocket
import tornado.template

Nmoy = 10

directionMoteurDroit = 4
pwmMoteurDroit = 5

directionMoteurGauche = 7
pwmMoteurGauche = 6

omegaDroit = 0
codeurDroitDeltaPos = 0
codeurDroitDeltaPosPrec = 0
omegaGauche = 0
codeurGaucheDeltaPos = 0
codeurGaucheDeltaPosPrec = 0

mesuremV1 = 0
mesuremV2 = 0
mesuremV3 = 0
mesuremV1Prec = 0
mesuremV2Prec = 0
mesuremV3Prec = 0

# Les moteurs sont asservis en vitesse grâce à un régulateur de type PID
# On déclare ci-dessous les variables et paramètres nécessaires à l'asservissement et au régulateur
vref = 0. # consigne de vitesse
vrefDroit = 0. # consigne vitesse de rotation du moteur droit
vrefGauche = 0. # consigne vitesse de rotation du moteur gauche
omegaDroit = 0. # vitesse de rotation du moteur droit
omegaGauche = 0. # vitesse de rotation du moteur gauche
commandeDroit = 0. # commande en tension calculée par le PID pour le moteur droit
commandeGauche = 0. # commande en tension calculée par le PID pour le moteur gauche
commande_avant_sat_Droit = 0. # valeur de la commande avant la saturation (voir ci-dessous) pour le moteur droit
commande_avant_sat_Gauche = 0. # valeur de la commande avant la saturation (voir ci-dessous) pour le moteur gauche
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur

# Déclarations pour les consignes de mouvement
vref = 3

T0 = time.time()
dt = 0.02
i = 0
tdebut = 0
# Création d'un scheduler pour exécuter des opérations à cadence fixe
s = sched.scheduler(time.time, time.sleep)

tensionBatterie = 7.4

#--- setup --- 
def setup():
    wiringpi2.wiringPiSetupGpio() # For GPIO pin numbering
    
    CommandeMoteurs(0, 0, tensionBatterie)

    
# -- fin setup -- 
 
# -- loop -- 
def loop():
    global i
    i = i+1
    s.enterabs( T0 + (i * dt), 1, CalculVitesse, ())
    s.run()
# -- fin loop --


def CalculVitesse():
    global ticksCodeurDroit, ticksCodeurGauche, indiceTicksCodeurDroit, indiceTicksCodeurGauche, started, vref, \
        omegaDroit, omegaGauche, ticksCodeurDroitTab, ticksCodeurGaucheTab, codeurDroitDeltaPosPrec, codeurGaucheDeltaPosPrec, \
        codeurDroitDeltaPos, codeurGaucheDeltaPos, commandeDroit, commandeGauche, interruptKO, vrefDroit, vrefGauche, \
        codeurGaucheDeltaPosPrec, codeurDroitDeltaPosPrec, mesuremV1, mesuremV2, mesuremV3, mesuremV1Prec, mesuremV2Prec, mesuremV3Prec
    
    debut = time.time()
    # Mesure de la vitesse des moteurs grâce aux codeurs incrémentaux
    try:
        #codeursDeltaPos = a_star.read_codeursDeltaPos()
        #print bin(codeursDeltaPos)
        #codeurGaucheDeltaPos = u2s16(long(codeursDeltaPos) >> 16)
        #codeurDroitDeltaPos = u2s16(long(codeursDeltaPos) & 0b00000000000000001111111111111111)
        codeurGaucheDeltaPos = a_star.read_codeurGaucheDeltaPos()
        codeurDroitDeltaPos = a_star.read_codeurDroitDeltaPos()
        codeurGaucheDeltaPosPrec = codeurGaucheDeltaPos
        codeurDroitDeltaPosPrec = codeurDroitDeltaPos
        
        # Lecture des entrées analogiques sur lesquelles sont branchés les capteurs de suivi de ligne
        mesuremV1 = a_star.analog_read(3)
        mesuremV2 = a_star.analog_read(4)
        mesuremV3 = a_star.analog_read(5)
        mesuremV1Prec = mesuremV1
        mesuremV2Prec = mesuremV2
        mesuremV3Prec = mesuremV3
    except:
        print "Error getting data"
        codeurGaucheDeltaPos = codeurGaucheDeltaPosPrec
        codeurDroitDeltaPos = codeurDroitDeltaPosPrec
        mesuremV1 = mesuremV1Prec
        mesuremV2 = mesuremV2Prec
        mesuremV3 = mesuremV3Prec
    
    # (dt / 2) car l'A* mesure les codeurXDeltaPos toutes les 10 ms, or dt=20 ms
    omegaDroit = -2 * ((2 * 3.141592 * codeurDroitDeltaPos) / 1632) / (Nmoy * (dt / 2))  # en rad/s
    omegaGauche = 2 * ((2 * 3.141592 * codeurGaucheDeltaPos) / 1632) / (Nmoy * (dt / 2))  # en rad/s

    
    # On compare par rapport à un seuil pour savoir si le capteur voit la ligne ou non
    seuil = 700
    surLigne1 = False
    surLigne2 = False
    surLigne3 = False
    if mesuremV1 > seuil:
        surLigne1 = True
    if mesuremV2 > seuil:
        surLigne2 = True
    if mesuremV3 > seuil:
        surLigne3 = True
    
    # Si le robot est centré sur la ligne, on va tout droit
    if ((surLigne1 == False) and (surLigne2 == True) and (surLigne3 == False)) or ((surLigne1 == True) and (surLigne2 == True) and (surLigne3 == True)):
        vrefDroit = vref
        vrefGauche = vref
    # Si seul le capteur de droite est sur la ligne on tourne à droite fort
    elif (surLigne1 == True) and (surLigne2 == False) and (surLigne3 == False):
        vrefDroit = -vref * 2 / 3
        vrefGauche = vref
    # Si seul le capteur de gauche est sur la ligne on tourne à gauche fort
    elif (surLigne1 == False) and (surLigne2 == False) and (surLigne3 == True):
        vrefDroit = vref
        vrefGauche = -vref * 2 / 3
    # Si les deux capteurs de droite sont sur la ligne on tourne à droite normalement
    elif (surLigne1 == True) and (surLigne2 == True) and (surLigne3 == False):
        vrefDroit = -vref / 2
        vrefGauche = vref
    # Si les deux capteurs de gauche sont sur la ligne on tourne à gauche normalement
    elif (surLigne1 == False) and (surLigne2 == True) and (surLigne3 == True):
        vrefDroit = vref
        vrefGauche = -vref / 2

    
    # Calcul de la commande avant saturation
    commande_avant_sat_Droit = vrefDroit
    commande_avant_sat_Gauche = vrefGauche

    # Application de la saturation sur la commande
    if (commande_avant_sat_Droit > umax):
        commandeDroit = umax
    elif (commande_avant_sat_Droit < umin):
        commandeDroit = umin
    else:
        commandeDroit = commande_avant_sat_Droit
    
    if (commande_avant_sat_Gauche > umax) :
        commandeGauche = umax
    elif (commande_avant_sat_Gauche < umin):
        commandeGauche = umin
    else:
        commandeGauche = commande_avant_sat_Gauche


    CommandeMoteurs(commandeDroit, commandeGauche, tensionBatterie)
    #print time.time() - debut
        
        
def CommandeMoteurs(commandeDroit, commandeGauche, tensionAlim):
    # Cette fonction calcule et envoi les signaux PWM au pont en H
    # en fonction des tensions de commande et d'alimentation

    # L'ensemble pont en H + moteur pourrait ne pas être linéaire
    tensionDroit = commandeDroit
    tensionGauche = commandeGauche

    # Normalisation de la tension d'alimentation par
    # rapport à la tension d'alimentation
    tension_int_droit = int(400 * tensionDroit / tensionAlim)
    tension_int_gauche = -int(400 * tensionGauche / tensionAlim)

    # Saturation par sécurité
    if (tension_int_droit > 400):
        tension_int_droit = 400

    if (tension_int_droit < -400):
        tension_int_droit = -400

    if (tension_int_gauche > 400):
        tension_int_gauche = 400

    if (tension_int_gauche < -400):
        tension_int_gauche = -400

    # Commande PWM
    try:
        a_star.motors(tension_int_gauche, tension_int_droit)    
    except:
        print "Erreur moteurs"

    
def emitData():
    # Délai nécessaire pour que le serveur ait le temps de démarrer
    wiringpi2.delay(5000)
    while True: loop() # appelle fonction loop sans fin

    
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global socketOK
        print 'connection opened...'
        socketOK = True
        self.callback = PeriodicCallback(self.sendToSocket, 100)
        self.callback.start()
    

    def on_message(self, message):
        global vref

        jsonMessage = json.loads(message)
        
        if jsonMessage.get('vref') != None:
            vref = float(jsonMessage.get('vref'))
          

    def on_close(self):
        global socketOK, vrefDroit, vrefGauche
        print 'connection closed...'
        socketOK = False
        vrefDroit = 0.
        vrefGauche = 0.

    def sendToSocket(self):
        global codeurDroitDeltaPos, codeurGaucheDeltaPos, socketOK, commandeDroit, commandeGauche, interruptKO
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({'Temps':("%.2f" % tcourant), 'Consigne':("%.2f" % vref), 'omegaDroit':("%.2f" % omegaDroit), 'omegaGauche':("%.2f" % omegaGauche), 'Raw':("%.2f" % tcourant) + "," + ("%.2f" % vref) + "," + ("%.2f" % omegaDroit) + "," + ("%.2f" % omegaGauche)})
        if socketOK:
            try:
                self.write_message(aEnvoyer)
            except:
                pass
            
    def check_origin(self, origin):
        # Voir http://www.tornadoweb.org/en/stable/websocket.html#tornado.websocket.WebSocketHandler.check_origin
        # et http://www.arundhaj.com/blog/tornado-error-during-websocket-handshake.html
        return True        

    
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])
    
application = tornado.web.Application([
    (r'/ws', WSHandler)
])

def startTornado():
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(9090)
    tornado.ioloop.IOLoop.instance().start()


# Gestion du CTRL-C
def signal_handler(signal, frame):
    global vrefDroit, vrefGauche
    print 'You pressed Ctrl+C!'
    vrefDroit = 0.
    vrefGauche = 0.
    CommandeMoteurs(0, 0, 5)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#--- obligatoire pour lancement du code -- 
if __name__=="__main__": # pour rendre le code executable 
    started = False
    startedDroit = False
    startedGauche = False
    setup() # appelle la fonction setup
    print "Setup done."
    
    th = threading.Thread(None, emitData, None, (), {})
    th.daemon = True
    th.start()
    
    print "Starting Tornado."
    try:
        print "Connect to ws://" + get_ip_address('eth0') + ":9090/ws with Ethernet."
    except:
        pass
        
    try:
        print "Connect to ws://" + get_ip_address('wlan0') + ":9090/ws with Wifi."
    except:
        pass
    socketOK = False
    startTornado()


