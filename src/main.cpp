/**
 * Hier alle bronnen : 
 * https://www.instructables.com/How-to-use-DHT-22-sensor-Arduino-Tutorial/ 
 * https://elektronicavoorjou.nl/project/arduino-temp-luchtvochtigheid-dht22-iot-cloud/ 
 * erna ergens : 
 * Ook, waarom accelerometer (bv als plant wordt omver geduwd sat watertoevoer stopt)
 * Waarom DHT 22 gekozen
 * Hoe bereken je de samengestelde categorieën (2 bodemvochtigheidssensoren : bv 1 priotair nemen (DROOG bv altijd eerste prioritair))
 */

 // Opmerking voor mij : Ctrl + Alt + U kan ook om te herstarten

#include "Arduino.h"
#include "config.h"
#include <DHT.h>
#include <Wire.h>
#define ARDUINOTRACE_ENABLE 1 // Zet deze op 1 om debugging aan te zetten, op 0 om debugging uit te zetten
#include <ArduinoTrace.h>


// Enum voor status van de waterpomp (blijft hier gedefinieerd)
enum StatusWaterpomp { STATUS_POMP_UIT, STATUS_POMP_AAN };
StatusWaterpomp statusWaterpomp = StatusWaterpomp::STATUS_POMP_UIT;

// Globale variabelen
unsigned long vorigeInleesMillis = 0;

// TODO: Definieer juiste pinnummers voor sensoren
#define DHTPIN 14 // Pas aan naar de juiste pin voor de DHT sensor
#define DHTTYPE DHT22 // Of DHT11, afhankelijk van je sensor
DHT dht(DHTPIN, DHTTYPE);

#define RESISTIEVE_BVH_PIN 34  // A0 is GPIO36, maar GPIO34 is betrouwbaarder voor analogRead
#define CAPACITIEVE_BVH_PIN 39

#define pompPin 25 // Pin voor de waterpomp relais


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

// Variabelen voor het bijhouden van de starttijd en duurtijd van het water geven
unsigned long startTijdWaterGeven = 0;
unsigned long duurTijdWaterGeven = 0;

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
VochtCategorie berekenCategorieCapacitieveBVH(int capacitieve_bvh_waarde) {
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
 * OPMERKING : DE GROND AANDRUKKEN
 */
VochtCategorie berekenCategorieResistieveBVH(int resistieve_bvh_waarde) {

  // TODO: Implementeer inlezen met correcte pinnen
  if (resistieve_bvh_waarde >= RESISTIEVE_SENSOR_UITGEDROOGD_INTERVAL_MIN && resistieve_bvh_waarde <= RESISTIEVE_SENSOR_UITGEDROOGD_INTERVAL_MAX) {
      DUMP(UITGEDROOGD);
      return VochtCategorie::UITGEDROOGD;
    } else if (resistieve_bvh_waarde >= RESISTIEVE_SENSOR_LICHT_VOCHTIG_INTERVAL_MIN && resistieve_bvh_waarde <= RESISTIEVE_SENSOR_LICHT_VOCHTIG_INTERVAL_MAX) {
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
  startTijdWaterGeven = millis();
  duurTijdWaterGeven = duurtijd;
  digitalWrite(pompPin, HIGH); // Zet de pomp aan
  Serial.println("Waterpomp is aangezet.");
  statusWaterpomp = StatusWaterpomp::STATUS_POMP_AAN; // Zet de status van de waterpomp op AAN


  DUMP("Waterpomp aan, starttijd: ");
  DUMP(startTijdWaterGeven);
  DUMP(" Duurtijd: ");
  DUMP(duurTijdWaterGeven);
  DUMP(" Waterpomp status: ");
  DUMP(statusWaterpomp);
}

/**
 * Zet de waterpomp uit. 
 * Opgelet!! Aangezien de zetWaterpompAan() functie geen delay bevat, zullen de variabelen die daar gebruikt worden
 *           opnieuw geïnitialiseerd moeten worden bij het uitzetten van de pomp.
 */
void zetWaterpompUit() {
  digitalWrite(pompPin, LOW); // Zet de pomp uit
  Serial.println("Waterpomp is uitgezet.");
  statusWaterpomp = StatusWaterpomp::STATUS_POMP_UIT; // Zet de status van de waterpomp op UIT
  startTijdWaterGeven = 0;
  duurTijdWaterGeven = 0;

  DUMP("Waterpomp uit, starttijd: ");
  DUMP(startTijdWaterGeven);
  DUMP(" Duurtijd: ");
  DUMP(duurTijdWaterGeven);
  DUMP(" Waterpomp status: ");
  DUMP(statusWaterpomp);
}

/**
 * Deze functie bevat alle code voor het uitlezen van de sensoren en om de waterpomp indien nodig aan te zetten.
 * Het uitzetten van de waterpomp gebeurt niet hier maar in de loop() functie na controle of er voldoende tijd verstreken is.
 */
void leesSensorenEnGeefWaterIndienNodig() {
    Serial.println("Sensoren worden ingelezen..."); 
  // TODO: Implementeer inlezen met correcte pinnen
  int capacitieve_bvh_waarde = analogRead(CAPACITIEVE_BVH_PIN);
  int resistieve_bvh_waarde = analogRead(RESISTIEVE_BVH_PIN);
  int temperatuur = leesTemperatuur();

  Serial.print("Temperatuur: ");
  Serial.println(temperatuur);

  // Bepaal individuele categoriën en samengestelde categorie
  VochtCategorie categorieCapacitieveBVH = berekenCategorieCapacitieveBVH(capacitieve_bvh_waarde);
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

  if (categorie == VochtCategorie::UITGEDROOGD) {
    if (temperatuur < TEMPERATUUR_LAAG) {
      Serial.println("Temperatuur is te laag, water geven wordt overgeslagen.");
    } else if (temperatuur > TEMPERATUUR_HOOG) {
      Serial.println("Temperatuur is hoog, extra water wordt gegeven.");
      zetWaterpompAan(WATER_GEEF_DUUR_LANG);
    } else {
      zetWaterpompAan(WATER_GEEF_DUUR_NORMAAL);
      Serial.print("Temperatuur is normaal, water wordt gegeven.");
    }
  }
  
}

void setup() {
  // TODO: Implementeer de nodig code voor lezen sensoren (indien nodig)

  pinMode(pompPin, OUTPUT);
  digitalWrite(pompPin, LOW); // Zet de waterpomp uit bij opstart

  Serial.begin(115200); // 9600 als baudrate lukt niet, want dan krijg ik rare tekens
  dht.begin();
}

void loop() {
  // Gebruik unsigned long en lees de tijd aan het begin.
  unsigned long huidigeMillis = millis(); 

  // Als de pomp UIT is én het tijd is om sensoren te lezen
  if (statusWaterpomp == StatusWaterpomp::STATUS_POMP_UIT && (huidigeMillis - vorigeInleesMillis >= SENSOR_INLEES_INTERVAL)) {
    vorigeInleesMillis = huidigeMillis;
    leesSensorenEnGeefWaterIndienNodig();
  }

  // --- Oplossing: Haal millis() opnieuw op voor de pomp check ---
  huidigeMillis = millis(); 

  // Controleer of de waterpomp uitgezet moet worden.
  // We printen eerst de waarden om te debuggen (optioneel).
  if (statusWaterpomp == StatusWaterpomp::STATUS_POMP_AAN) {

    // Deze berekening zal nu meestal een klein positief getal geven.
    // De berekening (unsigned long - unsigned long) werkt correct,
    // ook bij rollover, zolang de duur korter is dan ~49 dagen.
    Serial.println(huidigeMillis - startTijdWaterGeven);

    // De eigenlijke check om de pomp uit te zetten.
    if ((huidigeMillis - startTijdWaterGeven) >= duurTijdWaterGeven) {
      zetWaterpompUit(); // Zet de waterpomp uit als de tijd verstreken is
    }
  }
}