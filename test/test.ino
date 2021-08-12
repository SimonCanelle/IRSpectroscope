/*
   Créer par Simon Janelle-Bombardier
   le 2 février 2021

   1. création du pwm de 10kHz 15% avec 2
   2. fonction retournant la température lue par la thermistance
   3. calcul des tension à appliquer sur le DigAnConv pour le nombre de point sur la plage voulue
   4. prise du spectre de référence
   5. prise du spectre de l'item
   6. Normalise pour préparer les données pour l'IA
   7. commande à partir du port série (voir Serial Port Commands)
   8. impression des spectre
   9. création de la fonction mapFromVectorArray
   10. contrôle de la lumière

   TODO :augmenter la résolution des vecteur de tensions

   Serial Port Commands :
    r : prise d'un spectre de référence
    m : prise de mesure du spectre d'un item
    p : (p minuscule) imprime le spectre de l'item
    P : (p majuscule) imprime le spectre de reference
    i : passe le spectre dans le réseau de neuronne pour la reconnaissance
    s : imprime le spectre normalisé
    t : test de la comm serielle


*/

//librairies
#include <Arduino.h>
#include <Math.h>
#include <Adafruit_MCP4725.h>
//AI
#include "AI.h"
#include <ADS1115_WE.h>
#include <mbed.h>

//Pins
#define pin_PWM 9 //D9
#define pin_Lum 8 //D8

//ADC Channels
#define chan_LowMir ADS1115_COMP_2_GND
#define chan_Ther ADS1115_COMP_1_GND
#define chan_Photo ADS1115_COMP_0_GND

//objects
Adafruit_MCP4725 DigAnConv;
//AI
AI reconnaisseur;
ADS1115_WE adc(0x48);
mbed::PwmOut CustomPWM(digitalPinToPinName(pin_PWM));

//defined constantes
#define dutyCycle 0.15f//15%
#define periodeus 100 //us
#define nombreMesures 100
#define limMinLambda 1750.0  //nm
#define limMaxLambda 2150.0  //nm
#define VmaxFiltre 48.5 //V
#define DigAnConvRange 4096
#define boostStableTime 75 //ms


//constantes de température
//thermistor
const float refTemp = 25.0;
const float A = 0.00109316;
const float B = 0.000240113;
const float C = 0.0000000514556;
//filtre, à calculer pour chaque capteur //[A-F][0-2]
const float constFiltre[6][3] = {
  { -1.596574071466200E+33, 1.271046040795440E+30, -7.084867962098990E+28},
  { 1.176291614374700E+28, -7.721088842478010E+24, 4.788276672479380E+23},
  { -3.491342594303010E+22, 1.605995742705430E+19, -1.205177912795680E+18},
  { 4.811425902115860E+16, -1.343284145247720E+13, 1.370414460760080E+12},
  { -2.696092023432810E+10, 4.868461100144200E+06, -6.777698446387000E+05},
  { 5.133430626689090E+03, -6.405447419656160E-01, 1.192616387325150E-01}
};

//tableau de calibration du boost, tension nécessaire sur le AC pour avoir la tension voulue sur le boost
//TODO : augmenter la résolution des vecteurs (25 points min pour être certain de la précision) (le code est ok pour juste changer les arrays
const double vectorDigAnConv[26] = { 0, 164, 328, 491, 655, 819, 983, 1147, 1310, 1474, 1638, 1802, 1966, 2129, 2293, 2457, 2621, 2785, 2948, 3112, 3276, 3440, 3604, 3767, 3931, 4095};
//valeur temporaire venant de la sim
const double vectorBoost[26] = {11.82, 11.82, 11.82, 11.82, 11.82, 14.39, 17.20, 19.97, 22.84, 25.75, 28.55, 31.24, 34.18, 36.95, 39.68, 42.5, 45.3, 48.1, 50.9, 53.8, 110.0, 110.0, 110.0, 110.0, 110.0, 110.0};

//arrays
float spectreRef[nombreMesures]; //spectre de référence
float spectreAbsorbance[nombreMesures]; //spectre de l'item
int pasArray[nombreMesures];//tableau des tension pour les mesures en valeur 0-4096 pour le DigAnConv
float spectreNormalise[nombreMesures];//Normalise toutes les mesures entre 0 et 1
double lambdaMesure[nombreMesures];//tableau des longueurs d'ondes auxquelles les mesure on été prises

//états
bool refPris = false;
bool serialMode = true;//pour test

//timers
unsigned int tempsPrec = 0;
unsigned int tempsNow;

void setup() {

  //initialisation de la communication sérielle
  Serial.begin(9600);
  Serial.println("depart de l'initialisation");
 
  //pinModes
  pinMode(pin_Lum, OUTPUT);


  //setup de l'ADC
  Wire.begin();
  if (!adc.init()) {
    Serial.println("ADS1115 not connected!");
  }
  adc.setVoltageRange_mV(ADS1115_RANGE_0256);
  adc.setMeasureMode(ADS1115_CONTINUOUS);

  //setup du model tensorflow
  //AI 
  reconnaisseur.setup();

  //setup du pwm
  CustomPWM.period_us(periodeus);
  CustomPWM.write(dutyCycle);
  CustomPWM.suspend();

  // For Adafruit MCP4725 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  DigAnConv.begin(0x62);
  DigAnConv.setVoltage(0, false); //pour s'assurer que le DigAnConv est a 0 au départ

  //delai de stabilisation du boost
  delay(5000);
  //message init 
  Serial.println("initialisation terminé");

}
void loop() {
  while(!Serial.available());
  Serial.read();
  Serial.println("test");
}
