/**
 * Hier alle bronnen : 
 * https://www.instructables.com/How-to-use-DHT-22-sensor-Arduino-Tutorial/ 
 * https://elektronicavoorjou.nl/project/arduino-temp-luchtvochtigheid-dht22-iot-cloud/ 
 * erna ergens : 
 * Ook, waarom accelerometer (bv als plant wordt omver geduwd sat watertoevoer stopt)
 * Waarom DHT 22 gekozen
 * Hoe bereken je de samengestelde categorieën (2 bodemvochtigheidssensoren : bv 1 priotair nemen (DROOG bv altijd eerste prioritair))
 */

#include "Arduino.h"
#include "config.h"
#include <DHT.h>
#include <Wire.h>
#define ARDUINOTRACE_ENABLE 0
 // zodat we de debugging kunnen aanzetten (1) of afzetten (0)
#include <ArduinoTrace.h>

// TODO: Definieer juiste pinnummers voor sensoren
#define DHTPIN 14 // Pas aan naar de juiste pin voor de DHT sensor
#define DHTTYPE DHT22 // Of DHT11, afhankelijk van je sensor
DHT dht(DHTPIN, DHTTYPE);
#define RESISTIEVE_BVH_PIN 34
#define CAPACITIEVE_BVH_PIN 39

String categorieNaarString(VochtCategorie categorie) {
  switch (categorie) {
    case UITGEDROOGD: return "UITGEDROOGD";
    case LICHT_VOCHTIG: return "LICHT_VOCHTIG";
    case DOORNAT: return "DOORNAT";
    default: return "ONBEKEND";
  }
}


float tempGlobal = 0; // Globale temperatuur variabele

// TODO: Variabelen om wachttijd tussen inlezen sensoren te kunnen regelen

// TODO: Variabelen om duurtijd van water geven te kunnen regelen

// TODO: Variabele om status van de waterpomp aan te geven, dit is nodig om te kunnen controlleren of de waterpomp gestopt moet worden

/**
 * Bepaal de temperatuur, op basis van de gekozen temperatuursensor.
 * Voor een digitale sensor zal dit anders zijn dan voor een analoge.
 * Geeft de temperatuur in °C terug.
 */
float leesTemperatuur() {
  // TODO: Implementeer zodat de temperatuur op de juiste manier wordt ingelezen
  float temp = dht.readTemperature();

  return temp;
}

/**
 * Bepaal de juiste sensorwaarde voor de capacitieve bodemvochtigheidssensor.
 */
VochtCategorie berekenCategorieCapactieveBHV(int capacitieve_bvh_waarde) {
  // TODO: Implementeer inlezen met correcte pinnen
  if (capacitieve_bvh_waarde <= CAPACITIEVE_SENSOR_DROOG_INTERVAL_MIN && capacitieve_bvh_waarde >= CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX) {
    DUMP(UITGEDROOGD);
    return VochtCategorie::UITGEDROOGD;
  } else if (capacitieve_bvh_waarde <= CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MIN && capacitieve_bvh_waarde >= CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MAX) {
    DUMP(LICHT_VOCHTIG);
    return VochtCategorie::LICHT_VOCHTIG;
  } else {
    DUMP(DOORNAT);
    return VochtCategorie::DOORNAT;
  }
}

/**
 * Bepaal de juiste sensorwaarde voor de resistieve bodemvochtigheidssensor.
 */
VochtCategorie berekenCategorieResistieveBVH(int resistieve_bvh_waarde) {

  // TODO: Implementeer inlezen met correcte pinnen
  if (resistieve_bvh_waarde >= RESISTIEVE_SENSOR_DROOG_INTERVAL_MIN && resistieve_bvh_waarde <= RESISTIEVE_SENSOR_DROOG_INTERVAL_MAX) {
      DUMP(UITGEDROOGD);
      return VochtCategorie::UITGEDROOGD;
    } else if (resistieve_bvh_waarde >= RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MIN && resistieve_bvh_waarde <= RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MAX) {
      DUMP(LICHT_VOCHTIG);
      return VochtCategorie::LICHT_VOCHTIG;
    } else {
      DUMP(DOORNAT);
      return VochtCategorie::DOORNAT;
    }
}

/* MOCK functies om sensoren te bypassen */
int leesTemperatuur_MOCK() {
  // Return random waarde tussen 0 en 30 °C
  return random(0, 30);
}

int leesCapacitieveBVHSensor_MOCK() {
  // Return random waarde tussen 0 en 4095
  return random(0, 4095);
}

int leesResistieveBVHSensor_MOCK() {
  // Return random waarde tussen 0 en 4095
  return random(0, 4095);
}

/**
 * Bepaal de categorie van de capacitieve bodemvochtigheidssensor voor de gemeten sensorwaarde.
 * We gebruiken hierbij de configuratie uit onze calibratie.  Per categorie checken we of de waarde
 * tussen de MIN en de MAX valt.
 * Opgelet!!  Gebruik enkel de categoriën uit je configuratiebestand!
 * Pas het type van de return value in het functievoorschrift aan op basis van je configuratiebestand.
 */

/**
 * Bepaal de categorie van de resistieve bodemvochtigheidssensor voor de gemeten sensorwaarde.
 * We gebruiken hierbij de configuratie uit onze calibratie.  Per categorie checken we of de waarde
 * tussen de MIN en de MAX valt.
 * Opgelet!!  Gebruik enkel de categoriën uit je configuratiebestand!
 * Pas het type van de return value in het functievoorschrift aan op basis van je configuratiebestand.
 */


/**
 * Bereken de samengestelde categorie voor beide bodemvochtigheidssensoren.
 * Mogelijke strategiën: droogste wint altijd / één wint altijd / geen mogelijke categorie bij verschil
 * Opgelet!!  Gebruik enkel de categoriën uit je configuratiebestand!
 * Pas het type van de return value in het functievoorschrift aan op basis van je configuratiebestand.
 */
VochtCategorie berekenSamengesteldeCategorie(VochtCategorie categorieResistieveBVH, VochtCategorie categorieCapacitieveBVH) {
  // Todo: Implementeer zodat een samengstelde categorie wordt berekend.  Documenteer de strategie!
  if (categorieResistieveBVH == VochtCategorie::UITGEDROOGD || categorieCapacitieveBVH == VochtCategorie::UITGEDROOGD) {
    return VochtCategorie::UITGEDROOGD;
  } else if (categorieResistieveBVH == VochtCategorie::LICHT_VOCHTIG || categorieCapacitieveBVH == VochtCategorie::LICHT_VOCHTIG) {
    return VochtCategorie::LICHT_VOCHTIG;
  } else {
    return VochtCategorie::DOORNAT;
  }
}

/**
 * Zet de waterpomp aan voor een bepaalde tijd.   
 * Opgelet!!  Deze functie mag GEEN DELAY bevatten.  De duurtijd zal dus via een variabele moeten bijgehouden worden.
 *            Het hoofdprogramma moet telkens controlleren of de duurtijd reeds verstreken is, via millis().
 *            Gebruik een status om aan te geven dat de waterpomp aan het water geven is.
 */
void zetWaterpompAan(int duurtijd) {
  // TODO: Implementeer code om de pomp aan te zetten

  // TODO: Initialiseer de variabelen om de starttijd en duurtijd van het water geven te regelen
 
}

/**
 * Zet de waterpomp uit. 
 * Opgelet!! Aangezien de zetWaterpompAan() functie geen delay bevat, zullen de variabelen die daar gebruikt worden
 *           opnieuw geïnitialiseerd moeten worden bij het uitzetten van de pomp.
 */
void zetWaterpompUit() {
  // TODO: Implementeer code om de pomp uit te zetten
  
  // TODO: Initialiseer de variabelen om de starrtijd en duurtijd van het water geven te regelen

}

/**
 * Deze functie bevat alle code voor het uitlezen van de sensoren en om de waterpomp indien nodig aan te zetten.
 * Het uitzetten van de waterpomp gebeurt niet hier maar in de loop() functie na controle of er voldoende tijd verstreken is.
 */
void leesSensorenEnGeefWaterIndienNodig() {
  // TODO: Implementeer inlezen met correcte pinnen
  int capacitieve_bvh_waarde = analogRead(CAPACITIEVE_BVH_PIN);
  int resistieve_bvh_waarde = analogRead(RESISTIEVE_BVH_PIN);
  int temperatuur = leesTemperatuur();

  // Bepaal individuele categoriën en samengestelde categorie
  VochtCategorie categorieCapacitieveBVH = berekenCategorieCapactieveBHV(capacitieve_bvh_waarde);
  VochtCategorie categorieResistieveBVH = berekenCategorieResistieveBVH(resistieve_bvh_waarde);
  VochtCategorie categorie = berekenSamengesteldeCategorie(categorieCapacitieveBVH, categorieResistieveBVH);

Serial.print("Capacitieve BVH: ");
Serial.println(capacitieve_bvh_waarde);
Serial.print("Resistieve BVH: ");
Serial.println(resistieve_bvh_waarde);

  // TODO: Beslis over water geven en pas de controles toe uit de flowchart.  
  // !! Gebruik enkel de constanten uit de configuratie om met een categorie te vergelijken!
  // !! Gebruik enkel de constanten uit de configuratie om de duurtijd van het water geven mee te geven
  // !! Gebruik verder enkel de functies zetWaterpompAan() aan te zetten

}

void setup() {
  // TODO: Implementeer de nodig code voor lezen sensoren (indien nodig)
  Serial.begin(115200); // 9600 als baudrate lukt niet, want dan krijg ik rare tekens
  dht.begin();
}

void loop() {
  // We hebben huidige millis nodig om de verschillende processen te controleren (water geven / stoppen)
  long huidigeMillis = millis();
  tempGlobal = leesTemperatuur();
  //DUMP(tempGlobal);
  leesSensorenEnGeefWaterIndienNodig();
  delay(1000); // Wacht even voor nieuwe meting
  // TODO: Controleer of de waterpomp uitgezet moet worden en roep functie zetWaterpompUit() aan indien nodig

  // TODO: Controleer of sensoren ingelezen moeten worden en roep functie leesSensorenEnGeefWaterIndienNodig() aan indien nodig

}