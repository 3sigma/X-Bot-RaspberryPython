/******************************************************************************
Pilotage en tension des moteurs à courant continu de X-Bot Raspberry Python
Ce programme est destiné à fonctionner avec la famille de robots gyropodes
Geeros, disponible à l'adresse suivante:
http://boutique.3sigma.fr/12-robots

Auteur: 3Sigma
Version 1.1 - 28/04/2017
*******************************************************************************/

// Inclusion d'une bibliothèque permettant l'exécution à cadence fixe
// d'une partie du programme. Télécharger à l'adresse http://www.3sigma.fr/telechargements/FlexiTimer2.zip
// et décompresser dans le sous-répertoire « libraries » de votre installation Arduino
// Pour plus de détails, voir les pages (en anglais): 
// http://www.arduino.cc/playground/Main/FlexiTimer2
// https://github.com/wimleers/flexitimer2
#include <FlexiTimer2.h>

// Inclusion de la bibliothèque de gestion de la carte AStar32U4 de contrôle moteur
// Télécharger à l'adresse http://www.3sigma.fr/telechargements/AStar32U4.zip
// Pour plus de détails, voir la page (en anglais): 
// https://pololu.github.io/a-star-32u4-arduino-library/
#include <AStar32U4.h>
AStar32U4Motors motors;

// Définitions et déclarations pour le codeur incrémental du moteur droit
#define codeurDroitPinA 0
#define codeurDroitPinB 7
volatile long ticksCodeurDroit = 0;
int codeurDroitDeltaPos;

// Définitions et déclarations pour le codeur incrémental du moteur gauche
#define codeurGauchePinA 1
#define codeurGauchePinB 8
volatile long ticksCodeurGauche = 0;
int codeurGaucheDeltaPos;

// ATTENTION: donner la bonne valeur de la tension d'alimentation sur la ligne ci-dessous. Ici, 7.4 V
double tensionAlim = 7.4;

// Ce programme reçoit par liaison série des données issues d'un programme qui est exécuté sur
// l'ordinateur connecté à la carte. Si le programme ne reçoit rien pendant un temps donné,
// défini ci-dessous, un traitement particulier sera réalisé.
// Timeout de réception des données en s
#define TIMEOUT 2
unsigned long timeLastReceived = 0;
int timedOut = 0;

// Ce programme envoie par liaison série des données à l'ordinateur connecté à la carte.
// Cadence d'envoi des données en ms
#define TSDATA 20
unsigned long tempsDernierEnvoi = 0;
unsigned long tempsCourant = 0;

// Certaines parties du programme sont exécutées à cadence fixe grâce à la bibliothèque FlexiTimer2.
// Cadence d'échantillonnage en ms
#define CADENCE_MS 20
double dt = CADENCE_MS/1000.;
double temps = -CADENCE_MS/1000.;

// Déclaration des variables concernant la commande et la mesure de vitesse des moteurs
double commande = 0.;
double omegaDroit = 0.;
double omegaGauche = 0.;

// Déclaration des variables pour la réception des données sur la ligne série
const int NOMBRE_DE_CHAMPS = 6; // nombre de champs de données reçues sur la liaison série
int champIndex = 0; // champ courant reçu
int valeursRecues[NOMBRE_DE_CHAMPS]; // tableau contenant les champs de données reçues
// Variables utilisées pour les données reçues
int typeSignal = 0;
double offset = 0.;
double amplitude = 0.;
double frequence = 0.;
int moteurint = 0; // Moteur droit par défaut
// Variables pour la vérification de cohérence des données
double crc = 0.;
double crc_check = 0.;

// Initialisations
void setup(void) {
  
  // Codeur incrémental moteur droit
  FastGPIO::Pin<codeurDroitPinA>::setInputPulledUp();
  FastGPIO::Pin<codeurDroitPinB>::setInputPulledUp();
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurDroitPinA (définie à la fin du programme)
  attachInterrupt(digitalPinToInterrupt(codeurDroitPinA), GestionInterruptionCodeurDroitPinA, CHANGE);

  // Codeur incrémental moteur gauche
  FastGPIO::Pin<codeurGauchePinA>::setInputPulledUp();
  FastGPIO::Pin<codeurGauchePinB>::setInputPulledUp();
  // A chaque changement de niveau de tension sur le pin A du codeur,
  // on exécute la fonction GestionInterruptionCodeurGauchePinA (définie à la fin du programme)
  attachInterrupt(digitalPinToInterrupt(codeurGauchePinA), GestionInterruptionCodeurGauchePinA, CHANGE);

  // Liaison série
  // ATTENTION: ne pas modifier la vitesse de transmission de la liaison série,
  // sinon le programme de pilotage exécuté sur le PC connecté à la carte ne fonctionnera plus
  Serial.begin(115200);
  while (!Serial) {
    ; // Attente de la connexion du port série. Requis pour l'USB natif
  }
  
  /* Le programme principal est constitué:
     - de la traditionnelle boucle "loop" des programmes Arduino, dans laquelle on lit 
       en permanence la liaison série pour récupérer les nouvelles consignes de tension
       de commande du moteur
     - de la fonction "isrt", dont l'exécution est définie à cadence fixe par les deux instructions
       ci-dessous. La bonne méthode pour exécuter une fonction à cadence fixe, c'est de l'exécuter sur interruption
       (la boucle "loop" est alors interrompue régulièrement, à la cadence voulue, par un timer).
       Une mauvaise méthode serait d'utiliser la fonction Arduino "delay". En effet, la fonction "delay" définit
       un délai entre la fin des instructions qui la précèdent et le début des instructions qui la suivent, mais si
       on ne connait pas le temps d'exécution de ces instructions (ou si celui-ci est inconnu), on n'obtiendra
       jamais une exécution à cadence fixe.
       Il est nécessaire d'exécuter la fonction "isrt" à cadence fixe car cette fonction:
       - doit renvoyer des données à cadence fixe au programme exécuté sur l'ordinateur connecté à la carte
       - calcule la vitesse de rotation du moteur en comptant le nombre d'impulsions du codeur pendant un temps fixe
     
  */
  FlexiTimer2::set(CADENCE_MS, 1/1000., isrt); // résolution timer = 1 ms
  FlexiTimer2::start();

}

// Boucle principale
void loop() {

  // Réception des données sur la liaison série
  readData();
  
}

// Fonction excutée sur interruption
void isrt(){

  // Nombre de ticks codeur depuis la dernière fois: correspond à l'angle de rotation du moteur pendant la période d'échantillonnage
  // Ce nombre est mis à jour par les fonctions GestionInterruptionCodeurPinA et GestionInterruptionCodeurPinB,
  // exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
  codeurDroitDeltaPos = ticksCodeurDroit;
  // Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
  ticksCodeurDroit = 0;
  
  // Nombre de ticks codeur depuis la dernière fois: correspond à l'angle de rotation du moteur pendant la période d'échantillonnage
  // Ce nombre est mis à jour par les fonctions GestionInterruptionCodeurPinA et GestionInterruptionCodeurPinB,
  // exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
  codeurGaucheDeltaPos = ticksCodeurGauche;
  // Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
  ticksCodeurGauche = 0;
  
  // Calcul de la vitesse de rotation. C'est le nombre d'impulsions converti en radian, divisé par la période d'échantillonnage
  omegaDroit = -2*((2.*3.141592*((double)codeurDroitDeltaPos))/1632.)/dt;  // en rad/s
  omegaGauche = 2*((2.*3.141592*((double)codeurGaucheDeltaPos))/1632.)/dt;  // en rad/s
  
  // Calcul de la consigne en fonction des données reçues sur la liaison série
  if (typeSignal==0) { // signal carré
    if (frequence > 0) {
      if (temps - ((double)((int)(temps*frequence)))/frequence < 1/(2*frequence)) {
        commande = offset + amplitude;
      }
      else {
        commande = offset;
      }
    }
    else {
      commande = offset + amplitude;
    }
  }
  else { //sinus
    if (frequence > 0) {
      commande = offset + amplitude * sin(2*3.141592*frequence*temps);
    }
    else {
      commande = offset + amplitude;
    }
  }
        
  // Envoi de la commande aux moteurs
  if (moteurint == 0) {
    CommandeMoteurDroit(commande, tensionAlim);
    CommandeMoteurGauche(0, tensionAlim);
  }
  else if (moteurint == 1) {
    CommandeMoteurDroit(0, tensionAlim);
    CommandeMoteurGauche(-commande, tensionAlim);
  }
  else if (moteurint == 2) {
    CommandeMoteurDroit(commande, tensionAlim);
    CommandeMoteurGauche(-commande, tensionAlim);
  }
  else {
    CommandeMoteurDroit(0, tensionAlim);
    CommandeMoteurGauche(0, tensionAlim);
  }

  // Incrémentation du temps courant
  temps += dt;
}

void ecritureData(void) {

  // Ecriture des données en sortie tous les TSDATA millisecondes
  
  // On envoie les données s'il s'est écoulé plus de TSDATA secondes
  // entre l'instant courant et le dernier envoi
  tempsCourant = millis();
  if (tempsCourant-tempsDernierEnvoi > TSDATA) {
    Serial.print(temps);
    
    Serial.print(",");
    Serial.print(commande);
    Serial.print(",");
    Serial.print(omegaDroit);
    Serial.print(",");
    Serial.print(omegaGauche);
   
    Serial.print("\r");
    Serial.print("\n");
    
    tempsDernierEnvoi = tempsCourant;
  }  
}

void readData(void) {
  // Lecture des données sur la liaison série
  // On attend une série de NOMBRE_DE_CHAMPS valeurs entières séparées par des virgules
  
  // Initialisations
  champIndex = 0;
  for(int i=0; i < NOMBRE_DE_CHAMPS; i++) {
    valeursRecues[i] = 0;
  }

  while(true) {

    // Ecriture des données sur la liaison série
    ecritureData();
  
    if (Serial.available()>0) { // si une donnée est reçue
      // Réinitialisation du timeout (voir plus loin)
      timeLastReceived = millis();
      timedOut = 0;
      // Lecture de la donnée
      char ch = Serial.read();
      if (ch >= '0' && ch <= '9') { // caractère ascii entre 0 et 9 ?
        // Si oui, nous stockons la valeur
        valeursRecues[champIndex] = (valeursRecues[champIndex] * 10) + (ch - '0');
      }
      else if (ch == ',') { // séparateur virgule détecté, on passe au champ suivant
        if(champIndex < NOMBRE_DE_CHAMPS-1) {
          champIndex++; // incrémentation de l'index de champ
        }
      }
      else {
        // tout caractère autre qu'un chiffre ou une virgule (le retour chariot de fin de trame par exemple)
        // stoppe l'acquisition des champs
        break; // on sort de la boucle
      } 
    }
    // Timeout
    // Si aucune donnée n'a été reçue pendant plus de TIMEOUT ms, on met à zéro toutes les
    // consignes par sécurité
    else { 
      if ((millis()-timeLastReceived > TIMEOUT*1000) && (timedOut==0)) {
        for(int i=0; i < NOMBRE_DE_CHAMPS; i++) {
          offset = 0.;
          amplitude = 0.;
          frequence = 0.;
        }
        timedOut = 1;
      }
    }    
  }
  
  // Traitement des données reçues (on extrait chaque valeur de la trame complète)
  if (champIndex == NOMBRE_DE_CHAMPS-1) { // si le bon nombre de champs a été reçu
      
    // Récupération de la dernière valeur (somme de contrôle, envoyée par l'émetteur)
    crc = valeursRecues[NOMBRE_DE_CHAMPS-1];
    // Calcul de la somme de contrôle (somme de toutes les autres données) côté récepteur
    crc_check = 0;
    for(int i=0; i < NOMBRE_DE_CHAMPS-1; i++) {
      crc_check += valeursRecues[i];
    }
    
    // Si les sommes de contrôle sont identiques, on récupère les données
    if (crc == crc_check) { 
      typeSignal = valeursRecues[0];
      // Conversion des données d'entiers en flottants
      offset = ((double)valeursRecues[1]-60)/10.;
      amplitude = ((double)valeursRecues[2])/10.;
      frequence = ((double)valeursRecues[3])/100.;
      moteurint = valeursRecues[4];
    }
    // Si les sommes de contrôle sont différentes, on fait un traitement particulier
    else {
      // Par exemple, envoi d'un message, désactivé ici
      //Serial.print("\r\r\rErreur CRC !!!");
    }
  }
  else { // on n'a pas reçu le bon nombre de champs
    // On fait un traitement particuliern pas exemple, envoi d'un message (désactivé ici)
    //Serial.print("\r\r\rErreur reception !!!\r\r\r");
  }
}

void CommandeMoteurDroit(double commande, double tensionAlim) {
  // Cette fonction calcule et envoi les signaux PWM au pont en H
  // en fonction des tensions de commande et d'alimentation
  int16_t speed;
  double tension;
  
  // L'ensemble pont en H + moteur n'est pas linéaire
  // On redresse la caractéristique avec cette fonction
  tension = redresse_commandeDroit(commande);
  
  // Normalisation de la tension d'alimentation par
  // rapport à la tension d'alimentation
  speed = (int16_t)(400*(tension/tensionAlim));
  
  // Saturation par sécurité
  if (speed>400) {
    speed = 400;
  }
  if (speed<-400) {
    speed = -400;
  }
  
  // Commande PWM
  motors.setM2Speed(speed);
}

double redresse_commandeDroit(double commande) {
  // Cette fonction redresse la non-linéarité de l'ensemble pont en H + moteur
  if (commande>=0) {
    return(commande + 0.); 
  }
  else if (commande<0) {
    return(commande - 0.); 
  }
  else {
    return(0); 
  }
}

void CommandeMoteurGauche(double commande, double tensionAlim) {
  // Cette fonction calcule et envoi les signaux PWM au pont en H
  // en fonction des tensions de commande et d'alimentation
  int16_t speed;
  double tension;
  
  // L'ensemble pont en H + moteur n'est pas linéaire
  // On redresse la caractéristique avec cette fonction
  tension = redresse_commandeGauche(commande);
  
  // Normalisation de la tension d'alimentation par
  // rapport à la tension d'alimentation
  speed = (int16_t)(400*(tension/tensionAlim));
  
  // Saturation par sécurité
  if (speed>400) {
    speed = 400;
  }
  if (speed<-400) {
    speed = -400;
  }
  
  // Commande PWM
  motors.setM1Speed(speed);
}

double redresse_commandeGauche(double commande) {
  // Cette fonction redresse la non-linéarité de l'ensemble pont en H + moteur
  if (commande>0) {
    return(commande+0.); 
  }
  else if (commande<0) {
    return(commande-0.); 
  }
  else {
    return(0); 
  }
}


void GestionInterruptionCodeurDroitPinA() {  
  // Routine de service d'interruption attachée à la voie A du codeur incrémental droit
  // On utilise la fonction isInputHigh de la librairie FastGPIO
  // car les lectures doivent être très rapides pour passer le moins de temps possible
  // dans cette fonction (appelée de nombreuses fois, à chaque impulsion de codeur)
  if (FastGPIO::Pin<codeurDroitPinA>::isInputHigh() == FastGPIO::Pin<codeurDroitPinB>::isInputHigh()) {
    ticksCodeurDroit--;
  }
  else {
    ticksCodeurDroit++;
  }
}


void GestionInterruptionCodeurGauchePinA() {  
  // Routine de service d'interruption attachée à la voie A du codeur incrémental gauche
  // On utilise la fonction isInputHigh de la librairie FastGPIO
  // car les lectures doivent être très rapides pour passer le moins de temps possible
  // dans cette fonction (appelée de nombreuses fois, à chaque impulsion de codeur)
  if (FastGPIO::Pin<codeurGauchePinA>::isInputHigh() == FastGPIO::Pin<codeurGauchePinB>::isInputHigh()) {
    ticksCodeurGauche--;
  }
  else {
    ticksCodeurGauche++;
  }
}



