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
//#include <Wire.h>
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
  Serial.begin(57600);
  Serial.println("depart de l'initialisation");
  
  //pinModes
  pinMode(pin_Lum, OUTPUT);

  //setup de l'ADC
  Wire.begin();
  if (!adc.init()) {
    Serial.println("ADS1115 not connected!");
  }
  adc.setVoltageRange_mV(ADS1115_RANGE_0256);

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
  //timer pour limiter
  tempsNow = millis();
  if (Serial.available() > 0 && tempsNow - tempsPrec > 250) {
    char a = Serial.read();

    switch (a)
    {
      case 'r': //r pour référence
        reference();
        break;

      case 'm': //pour mesure

        //si il n'y a pas de référence
        if (!refPris)
        {
          reference();
        }
        //prise de mesure
        mesureSpectre();

        break;

      case 'p'://print le spectre de l'item
        printspectreAbsorbance();
        break;

      case 'P'://print le spectre de reference
        printSpectreRef();
        break;

      case 'i'://a pour intelligence artificielle
        //AI
        Serial.println(reconnaissance());
        break;

      case 's':
        printSpectreNormalise();
        break;

      case 't':
        calculPas();
        for(int i= 0; i <= nombreMesures; i++ ){ 
         Serial.println(pasArray[i]);
        }
        Serial.println("test terminé");
        break;

      default:
        Serial.println("commande incorecte");
        break;
    }

  }

}

//calcul de la température selon la thermistance du capteur
float temperature() {
  float R = 10000.0 * (5.0 / readChannel(chan_Ther) - 1.0);
  float temp = (1 / (A + B * (log(R)) + C * (pow(log(R), 3.0)))) - 273.0; //température en C
  return temp;
}

//prise d'un spectre de référence
void reference() {
  Serial.println("prise de reference");


  //prédéfini les différente tension à appliquer pour la prise de mesure
  calculPas();

  //PWM start
  CustomPWM.resume();

  //allume juste avant pour ne pas surchauffer
  allumerLumiere();

  //step-up primaire pour ne pas avoir un trop gros step à faire lors de la première mesure
  DigAnConv.setVoltage(DigAnConvRange / 10, false);

  //max lambda est Vmin
  for (int i = 0; i++; i <= nombreMesures) {
    DigAnConv.setVoltage(pasArray[i], false);
    delay(boostStableTime); //temps de stabilisation du boost
    //lecture de la photodiode avec moyenne mobile
    spectreRef[i] = readChannel(chan_Photo);
    for (size_t i = 0; i < 100; i++)
    {
      spectreRef[i] = spectreRef[i] * 0.95 + readChannel(chan_Photo) * 0.05;
    }
  }

  //arrête le boost
  DigAnConv.setVoltage(0, false);
  //arrête le PWM
  CustomPWM.suspend();

  //ferme la lumière pour éviter la surchauffe
  fermerLumiere();



  refPris = true;
}

//prise de mesure du spectre d'un item
void mesureSpectre() {
  Serial.println("prise de mesure");
  
  //PWM start
  CustomPWM.resume();

  //allume juste avant pour ne pas surchauffer
  allumerLumiere();

  //step-up primaire pour ne pas avoir un trop gros step à faire lors de la première mesure
  DigAnConv.setVoltage(DigAnConvRange / 4, false);

  //variable temporaire pour le calcul d'absorbance
  int mesureTemp;
  //max lambda est Vmin
  for (int i = 0; i++; i <= nombreMesures) {
    DigAnConv.setVoltage(pasArray[i], false);
    delay(boostStableTime); //temps de stabilisation du boost


    //lecture de la photodiode avec moyenne mobile et ajustement selon la référence
    mesureTemp = readChannel(chan_Photo);
    for (size_t i = 0; i < 100; i++)
    {
      mesureTemp  = mesureTemp  * 0.95 + readChannel(chan_Photo) * 0.05;
    }

    //calcul d'absorbance
    spectreAbsorbance[i] = -log(mesureTemp / spectreRef[i]);

  }
  //arrête le boost
  DigAnConv.setVoltage(0, false);
  //arrête le PWM
  CustomPWM.suspend();

  //ferme la lumière pour éviter la surchauffe
  fermerLumiere();

  normaliseArray(spectreAbsorbance, getMax(spectreAbsorbance));
}

//calcul les valeur de tenion pour le DigAnConv
void calculPas() {
  Serial.println("départ du calcul de pas");
  //lecture de la temperature
  float T = temperature() - refTemp;
  Serial.print("température prise : ");
  Serial.println(T);
  //pas (m)
  float pas = ((limMaxLambda - limMinLambda) / nombreMesures) * 1E-09; //nm à m pour la formule
  //lambda de départ
  float lambdaVoulue = limMaxLambda * 1E-09;

  //float temporaire pour les calculs (pas nécessaire mais améliore la lecture du code
  float tempFloat;

  //calcul des tension ajusté selon la température avec le tableau de constante constFiltre[x][y] x:lettre(A-F) y:chiffre(0-2)
  for (int i = 0; i < nombreMesures ; i++) {
    //enregistre la longueru d'onde mesuré
    lambdaMesure[i] = lambdaVoulue / 1E-09;
    //calcul de V^2
    tempFloat = constFiltre[0][0] * pow(lambdaVoulue, 5) + constFiltre[1][0] * pow(lambdaVoulue, 4) + constFiltre[2][0] * pow(lambdaVoulue, 3) + constFiltre[3][0] * pow(lambdaVoulue, 2) + constFiltre[4][0] * lambdaVoulue + constFiltre[5][0] + T * (constFiltre[0][1] * pow(lambdaVoulue, 5) + constFiltre[1][1] * pow(lambdaVoulue, 4) + constFiltre[2][1] * pow(lambdaVoulue, 3) + constFiltre[3][1] * pow(lambdaVoulue, 2) + constFiltre[4][1] * lambdaVoulue + constFiltre[5][1]) + pow(T, 2) * (constFiltre[0][2] * pow(lambdaVoulue, 5) + constFiltre[1][2] * pow(lambdaVoulue, 4) + constFiltre[2][2] * pow(lambdaVoulue, 3) + constFiltre[3][2] * pow(lambdaVoulue, 2) + constFiltre[4][2] * lambdaVoulue + constFiltre[5][2]);

    //map pour transformer en valeur pour le DigAnConv
    pasArray[i] = mapFromVectorArray(sqrt(tempFloat), vectorBoost, vectorDigAnConv);

    //limite pour ne pas dépasser la tension mximale
    if (pasArray[i] > (mapFromVectorArray(VmaxFiltre - 0.25, vectorBoost, vectorDigAnConv))) {
      pasArray[i] = (mapFromVectorArray(VmaxFiltre - 0.25, vectorBoost, vectorDigAnConv));
    }

    //prochaine longueur d'onde à mesurer
    lambdaVoulue -= pas;
  }
}

//permet de map une valeur à partir de 2 tableau de vecteur (const double) arr1 : vecteur source  arr2 : vecteur sortie
double mapFromVectorArray(float value, const double arr1[], const double arr2[]) {

  //trouve les bons vecteurs pour map
  for (int i = 0; i < sizeof(arr1); i++) {

    if (value < arr1[i]) {
      //si plus petit que le premier element
      if (i == 0) {
        return arr2[0];
      }

      //map à partir des vecteurs
      else {
        return round(map(value, arr1[i - 1], arr1[i - 1], arr2[i - 1], arr2[i - 1]));
      }
    }
  }
  //si plus grand que le dernier element
  return arr2[sizeof(arr2) - 1];
}

//allume la lumiere
void allumerLumiere() {
  digitalWrite(pin_Lum, HIGH);
}

//ferme la lumiere
void fermerLumiere() {
  digitalWrite(pin_Lum, LOW);
}

//ramene les valeur du spectre entre 0 et 1 pour l'IA
void normaliseArray(float arr1[], float maxVal) {
  for (int i = 0; i < nombreMesures; i++)
  {
    spectreNormalise[i] = arr1[i] / maxVal;
  }
}

//AI
//utilisation de l'IA
const char* reconnaissance() {
  return reconnaisseur.triage(spectreNormalise);
}

//imprime les valeur du spectre de l'item dans le port série
void printspectreAbsorbance() {
  //start of text ASCII code : 02
  Serial.write(2);
  for (int i = 0; i < nombreMesures; i++)
  {
    Serial.print(lambdaMesure[i]);
    Serial.print(',');
    Serial.println(spectreAbsorbance[i]);
  }
  //end of text ASCII code : 03
  Serial.write(3);
}

//imprime les valeur du spectre de référence dans le port série
void printSpectreRef() {
  //start of text ASCII code : 02
  Serial.write(2);
  for (int i = 0; i < nombreMesures; i++)
  {
    Serial.print(lambdaMesure[i]);
    Serial.print(',');
    Serial.println(spectreRef[i]);
  }
  //end of text ASCII code : 03
  Serial.write(3);
}

//imprime les valeur du spectre de référence dans le port série
void printSpectreNormalise() {
  //start of text ASCII code : 02
  Serial.write(2);
  for (int i = 0; i < nombreMesures; i++)
  {
    Serial.print(lambdaMesure[i]);
    Serial.print(',');
    Serial.println(spectreNormalise[i], 6);
  }
  //end of text ASCII code : 03
  Serial.write(3);

}

//fonction de test
void Test() {
  serialMode = !serialMode;

  if (serialMode){
     CustomPWM.resume();
  }
  else{
    CustomPWM.suspend();
  }
}

//return max value in float array
float getMax(float arr1[]) {
  float maxVal = 0;
  for (int i = 1; i < sizeof(arr1); i++) {
    if (arr1[i] > maxVal) {
      maxVal = arr1[i];
    }
  }
  return maxVal;
}

//fonction de lecture de l'ADC
float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while(adc.isBusy()){Serial.println("adc busy");}
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}
