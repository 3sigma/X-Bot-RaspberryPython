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
import math

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

# Déclarations pour le capteur de distance
trig = 22
echo = 23
pulse_start = 0
pulse_end = 0
pulse_duration = 0
last_pulse_duration = 0
distance = 0
idecim = 0

# Les moteurs sont asservis en vitesse grâce à un régulateur de type PID
# On déclare ci-dessous les variables et paramètres nécessaires à l'asservissement et au régulateur
R = 0.045 # Rayon d'une roue
W = 0.14 # Largeur du robot
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur
vxmes = 0. # vitesse longitudinale mesurée
xidotmes = 0. # vitesse de rotation mesurée
Tf = 0.02 # constante de temps de filtrage de l'action dérivée du PID
Kpvx = 1 # gain proportionnel pour l'asservissement de vitesse longitudinale
Kivx = 10 # gain intégral pour l'asservissement de vitesse longitudinale
Kdvx = 0.00 # gain dérivé pour l'asservissement de vitesse longitudinale
Kpxidot = 0.1 # gain proportionnel pour l'asservissement de rotation
Kixidot = 1 # gain intégral pour l'asservissement de rotation
Kdxidot = 0.000 # gain dérivé pour l'asservissement de rotation
commande_avant_sat_vx = 0. # commande avant la saturation pour l'asservissement de vitesse longitudinale
commande_vx = 0. # commande pour l'asservissement de vitesse longitudinale
commande_avant_sat_xidot = 0. # commande avant la saturation pour l'asservissement de rotation
commande_xidot = 0. # commande pour l'asservissement de rotation
P_vx = 0. # action proportionnelle pour l'asservissement de vitesse longitudinale
I_vx = 0. # action intégrale pour l'asservissement de vitesse longitudinale
D_vx = 0. # action dérivée pour l'asservissement de vitesse longitudinale
P_xidot = 0. # action proportionnelle pour l'asservissement de rotation
I_xidot = 0. # action intégrale pour l'asservissement de rotation
D_xidot = 0. # action dérivée pour l'asservissement de rotation
commandeDroit = 0. # commande en tension calculée par le PID pour le moteur droit
commandeGauche = 0. # commande en tension calculée par le PID pour le moteur gauche
yprecvx = 0. # Mesure de la vitesse longitudinale au calcul précédent
yprecxidot = 0. # Mesure de la vitesse de rotation au calcul précédent
# Variables intermédiaires
Ti = 0
ad = 0
bd = 0

# Variables utilisées pour les données reçues
x1 = 0.
x2 = 0.
Kp2 = 1.
Ki2 = 1.
Kd2 = 1.
Kpxi2 = 1.
Kixi2 = 1.
Kdxi2 = 1.

# Déclarations pour les consignes de mouvement
vxref = 0.
xidotref = 0.

T0 = time.time()
dt = 0.01
i = 0
tprec = 0
tdebut = 0
# Création d'un scheduler pour exécuter des opérations à cadence fixe
s = sched.scheduler(time.time, time.sleep)

tensionBatterie = 7.4


#--- setup --- 
def setup():
    wiringpi2.wiringPiSetupGpio() # Numérotation des pins en mode GPIO
    CommandeMoteurs(0, 0, tensionBatterie)
    
    # Capteur de distance: trig en sortie, echo en entrée
    wiringpi2.pinMode(trig, 1)
    wiringpi2.pinMode(echo, 0)
    
    # Initialisation du capteur de distance
    wiringpi2.digitalWrite(trig, 0)
    print "Attente du capteur de distance"
    time.sleep(2)
    
    wiringpi2.digitalWrite(trig, 1)
    time.sleep(0.00001)
    wiringpi2.digitalWrite(trig, 0)
    
# -- fin setup -- 
 
# -- loop -- 
def loop():
    global i
    i = i+1
    s.enterabs( T0 + (i * dt), 1, CalculVitesse, ())
    s.run()
# -- fin loop --


def CalculVitesse():
    global ticksCodeurDroit, ticksCodeurGauche, indiceTicksCodeurDroit, indiceTicksCodeurGauche, started, \
        omegaDroit, omegaGauche, ticksCodeurDroitTab, ticksCodeurGaucheTab, codeurDroitDeltaPosPrec, codeurGaucheDeltaPosPrec, \
        tdebut, ad, P_vx, I_vx, D_vx, P_xidot, I_xidot, D_xidot, bd, Ti, yprecvx, yprecxidot, interruptKO, \
        codeurDroitDeltaPos, codeurGaucheDeltaPos, commandeDroit, commandeGauche, vxmes, xidotmes, vxref, xidotref, \
        codeurGaucheDeltaPosPrec, codeurDroitDeltaPosPrec, \
        pulse_start, pulse_end, pulse_duration, last_pulse_duration, distance, idecim
        
        
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

    # Application de la consigne lue
    vxref = x1
    xidotref = x2

    # Définition des entrées de la fonction d'asservissement
    vxmes = (omegaDroit + omegaGauche)*R/2
    xidotmes = -(omegaDroit - omegaGauche)*R/W

    # Calcul du PID sur vx
    # Paramètres intermédiaires
    Ti = Ki2 * Kivx/(Kp2 * Kpvx + 0.01)
    ad = Tf/(Tf+dt)
    bd = Kd2 * Kdvx/(Tf+dt)
    
    # Terme proportionnel
    P_vx = Kpvx * Kp2 * (vxref - vxmes)

    # Terme dérivé
    D_vx = ad * D_vx - bd * (vxmes - yprecvx)
    
    # Calcul de la commande
    commande_vx = P_vx + I_vx


    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_vx = I_vx + Kivx * Ki2 * dt * (vxref - vxmes)

    # Stockage de la mesure courante pour utilisation lors du pas d'échantillonnage suivant
    yprecvx = vxmes
    
    # Fin Calcul du PID sur vx

    # Calcul du PID sur xidot
    # Paramètres intermédiaires
    Ti = Kixi2 * Kixidot/(Kpxi2 * Kpxidot + 0.01)
    ad = Tf/(Tf+dt)
    bd = Kdxi2 * Kdxidot/(Tf+dt)
    
    # Terme proportionnel
    P_xidot = Kpxidot * Kpxi2 * (xidotref - xidotmes)

    # Terme dérivé
    D_xidot = ad * D_xidot - bd * (xidotmes - yprecxidot)
    
    # Calcul de la commande
    commande_xidot = P_xidot + I_xidot + D_xidot


    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_xidot = I_xidot + Kixidot * Kixi2 * dt * (xidotref - xidotmes)

    # Stockage de la mesure courante pour utilisation lors du pas d'échantillonnage suivant
    yprecxidot = xidotmes
    
    # Fin Calcul du PID sur xidot


    # Calcul des commandes des moteurs
    commandeDroit = (commande_vx - commande_xidot);
    commandeGauche = (commande_vx + commande_xidot);
      
    CommandeMoteurs(commandeDroit, commandeGauche, tensionBatterie)
    #print time.time() - debut
    # Calcul de la distance mesurée par le capteur ultrason
    if idecim >= 10:
        idecim = 0
        wiringpi2.digitalWrite(trig, 1)
        time.sleep(0.00001)
        wiringpi2.digitalWrite(trig, 0)
        
        pulse_duration = 0
        while wiringpi2.digitalRead(echo) == 0:
            pulse_start = time.time()

        while wiringpi2.digitalRead(echo) == 1 and pulse_duration < 0.01166:
            pulse_end = time.time()
            last_pulse_duration = pulse_duration
            pulse_duration = pulse_end - pulse_start
        
        distance = last_pulse_duration * 17150
        distance = round(distance, 2)
        print "Distance: ", distance, " cm"
    else:
        idecim = idecim + 1
        
        
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
    global tprec
    # Délai nécessaire pour que le serveur ait le temps de démarrer
    tprec = time.time()
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
        global x1, x2, Kp2, Ki2, Kd2, Kpxi2, Kixi2, Kdxi2

        jsonMessage = json.loads(message)
        
        if jsonMessage.get('vref') != None:
            x1 = float(jsonMessage.get('vref'))
            #print ("x1: %.2f" % x1)
        if jsonMessage.get('psidotref') != None:
            x2 = (float(jsonMessage.get('psidotref'))) * 3.141592 / 180
            #print ("x2: %.2f" % x2)
        if jsonMessage.get('Kp2ref') != None:
            Kp2 = float(jsonMessage.get('Kp2ref'))
            #print ("Kp2: %.2f" % Kp2)
        if jsonMessage.get('Ki2ref') != None:
            Ki2 = float(jsonMessage.get('Ki2ref'))
            #print ("Ki2: %.2f" % Ki2)
        if jsonMessage.get('Kd2ref') != None:
            Kd2 = float(jsonMessage.get('Kd2ref'))
            #print ("Kd2: %.2f" % Kd2)
        if jsonMessage.get('Kpxi2ref') != None:
            Kpxi2 = float(jsonMessage.get('Kpxi2ref'))
            #print ("Kpxi2: %.2f" % Kpxi2)
        if jsonMessage.get('Kixi2ref') != None:
            Kixi2 = float(jsonMessage.get('Kixi2ref'))
            #print ("Kixi2: %.2f" % Kixi2)
        if jsonMessage.get('Kdxi2ref') != None:
            Kdxi2 = float(jsonMessage.get('Kdxi2ref'))
            #print ("Kdxi2: %.2f" % Kdxi2)
        

    def on_close(self):
        global socketOK, commandeDroit, commandeGauche
        print 'connection closed...'
        socketOK = False
        commandeDroit = 0.
        commandeGauche = 0.

    def sendToSocket(self):
        global started, codeurDroitDeltaPos, codeurGaucheDeltaPos, socketOK, commandeDroit, commandeGauche, vxref, xidotref, vxmes, xidotmes, interruptKO

        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({'Temps':("%.2f" % tcourant), 'Consigne vitesse longitudinale':("%.2f" % x1), 'Consigne vitesse de rotation':("%.2f" % x2), 'Vitesse longitudinale':("%.2f" % vxmes), 'Vitesse de rotation':("%.2f" % (180 * xidotmes/3.141592)), 'omegaDroit':("%.2f" % omegaDroit), 'omegaGauche':("%.2f" % omegaGauche), 'commandeDroit':("%.2f" % commandeDroit), 'commandeGauche':("%.2f" % commandeGauche), 'Raw':("%.2f" % tcourant) + "," + ("%.2f" % x1) + "," + ("%.2f" % x2) + "," + ("%.2f" % vxmes) + "," + ("%.2f" % (180 * xidotmes/3.141592)) + "," + ("%.2f" % omegaDroit) + "," + ("%.2f" % omegaGauche) + "," + ("%.2f" % commandeDroit) + "," + ("%.2f" % commandeGauche)})
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
    global commandeDroit, commandeGauche
    print 'You pressed Ctrl+C!'
    commandeDroit = 0.
    commandeGauche = 0.
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


