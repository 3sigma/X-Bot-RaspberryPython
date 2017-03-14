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

# Déclaration des variables pour la réception des données
typeSignal = 0
offset = 0.
amplitude = 0.
frequence = 0.
moteurint = 0 # Moteur droit par défaut

# Timeout de réception des données
timeout = 2
timeLastReceived = 0
timedOut = False

T0 = time.time()
dt = 0.01
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

def u2s16(x):
    return x - 65536 * (x >= 32768)

def CalculVitesse():
    global started, omegaDroit, omegaGauche, timeLastReceived, timeout, timedOut, \
        tdebut, imu, ad, P_x_Droit, I_x_Droit, D_x_Droit, P_x_Gauche, I_x_Gauche, D_x_Gauche, bd, Td, Tt, \
        codeurDroitDeltaPos, codeurGaucheDeltaPos, commandeDroit, commandeGauche, vrefDroit, vrefGauche, \
        codeurGaucheDeltaPosPrec, codeurDroitDeltaPosPrec
    
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
    except:
        print "Error getting data"
        codeurGaucheDeltaPos = codeurGaucheDeltaPosPrec
        codeurDroitDeltaPos = codeurDroitDeltaPosPrec
    
    omegaDroit = -2 * ((2 * 3.141592 * codeurDroitDeltaPos) / 1632) / (Nmoy * dt)  # en rad/s
    omegaGauche = 2 * ((2 * 3.141592 * codeurGaucheDeltaPos) / 1632) / (Nmoy * dt)  # en rad/s
    
    # Calcul de la commande avant saturation
    # Si on n'a pas reçu de données depuis un certain temps, celles-ci sont annulées
    if (time.time()-timeLastReceived) > timeout and not timedOut:
        timedOut = True
        vrefDroit = 0
        vrefGauche = 0
        
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
        self.callback = PeriodicCallback(self.sendToSocket, 20)
        self.callback.start()
    

    def on_message(self, message):
        global vref, vrefDroit, vrefGauche, typeSignal, offset, amplitude, frequence, Kp, Ki, Kd, moteurint, timeLastReceived, timedOut
        jsonMessage = json.loads(message)
        
        # Annulation du timeout de réception des données
        timeLastReceived = time.time()
        timedOut = False;
        
        if jsonMessage.get('typeSignal') != None:
            typeSignal = int(jsonMessage.get('typeSignal'))
        if jsonMessage.get('offset') != None:
            offset = float(jsonMessage.get('offset'))
        if jsonMessage.get('amplitude') != None:
            amplitude = float(jsonMessage.get('amplitude'))
        if jsonMessage.get('frequence') != None:
            frequence = float(jsonMessage.get('frequence'))
        if jsonMessage.get('moteurint') != None:
            moteurint = int(jsonMessage.get('moteurint'))
        
        tcourant = time.time() - T0
        # Calcul de la consigne en fonction des données reçues sur la liaison série
        if typeSignal == 0: # signal carré
            if frequence > 0:
                if (tcourant - (float(int(tcourant*frequence)))/frequence < 1/(2*frequence)):
                    vref = offset + amplitude
                else:
                    vref = offset
            else:
                vref = offset + amplitude
        else: # sinus
            if frequence > 0:
                vref = offset + amplitude * sin(2*3.141592*frequence*tcourant)
            else:
                vref = offset + amplitude

        # Application de la consigne sur chaque moteur
        if moteurint == 0:
            vrefDroit = vref
            vrefGauche = 0.
        elif moteurint == 1:
            vrefDroit = 0.
            vrefGauche = vref
        elif moteurint == 2:
            vrefDroit = vref
            vrefGauche = vref
        else:
            vrefDroit = 0.
            vrefGauche = 0.
            
        if not socketOK:
            vrefDroit = 0.
            vrefGauche = 0.


    def on_close(self):
        global socketOK, vrefDroit, vrefGauche
        print 'connection closed...'
        socketOK = False
        vrefDroit = 0.
        vrefGauche = 0.

    def sendToSocket(self):
        global codeurDroitDeltaPos, codeurGaucheDeltaPos, socketOK, commandeDroit, commandeGauche
        
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


