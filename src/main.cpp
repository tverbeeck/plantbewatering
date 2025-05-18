#include "Arduino.h"
#include "config.h"

// TODO: Definieer juiste pinnummers voor sensoren

// TODO: Variabelen om wachttijd tussen inlezen sensoren te kunnen regelen

// TODO: Variabelen om duurtijd van water geven te kunnen regelen

// TODO: Variabele om status van de waterpomp aan te geven, dit is nodig om te kunnen controlleren of de waterpomp gestopt moet worden

/**
 * Bepaal de temperatuur, op basis van de gekozen temperatuursensor.
 * Voor een digitale sensor zal dit anders zijn dan voor een analoge.
 * Geeft de temperatuur in °C terug.
 */
int leesTemperatuur() {
  // TODO: Implementeer zodat de temperatuur op de juiste manier wordt ingelezen
  return 0;
}

/**
 * Bepaal de juiste sensorwaarde voor de capacitieve bodemvochtigheidssensor.
 */
int leesCapacitieveBVHSensor() {
  // TODO: Implementeer inlezen met correcte pinnen
  return 0;
}

/**
 * Bepaal de juiste sensorwaarde voor de resistieve bodemvochtigheidssensor.
 */
int leesResistieveBVHSensor() {
  // TODO: Implementeer inlezen met correcte pinnen
  return 0;
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
String berekenCategorieCapactieveBHV(int sensorwaarde) {
  // TODO: Implementeer zodat de categorie voor de capacitieve BVH sensor wordt berekend.
  return "";
}

/**
 * Bepaal de categorie van de resistieve bodemvochtigheidssensor voor de gemeten sensorwaarde.
 * We gebruiken hierbij de configuratie uit onze calibratie.  Per categorie checken we of de waarde
 * tussen de MIN en de MAX valt.
 * Opgelet!!  Gebruik enkel de categoriën uit je configuratiebestand!
 * Pas het type van de return value in het functievoorschrift aan op basis van je configuratiebestand.
 */
String berekenCategorieResistieveBVH(int sensorwaarde) {
  // TODO: Implementeer zodat de categorie voor de resistieve BVH sensor wordt berekend.
  return "";
}

/**
 * Bereken de samengestelde categorie voor beide bodemvochtigheidssensoren.
 * Mogelijke strategiën: droogste wint altijd / één wint altijd / geen mogelijke categorie bij verschil
 * Opgelet!!  Gebruik enkel de categoriën uit je configuratiebestand!
 * Pas het type van de return value in het functievoorschrift aan op basis van je configuratiebestand.
 */
String berekenSamengesteldeCategorie(String categorieResistieveBVH, String categorieCapacitieveBVH) {
  // Todo: Implementeer zodat een samengstelde categorie wordt berekend.  Documenteer de strategie!
  return "";
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
  int capacitieve_bvh_waarde = leesCapacitieveBVHSensor();
  int resistieve_bvh_waarde = leesResistieveBVHSensor();
  int temperatuur = leesTemperatuur();

  // Bepaal individuele categoriën en samengestelde categorie
  String categorieCapacitieveBVH = berekenCategorieCapactieveBHV(capacitieve_bvh_waarde);
  String categorieResistieveBVH = berekenCategorieResistieveBVH(resistieve_bvh_waarde);
  String categorie = berekenSamengesteldeCategorie(categorieCapacitieveBVH, categorieResistieveBVH);

  // TODO: Beslis over water geven en pas de controles toe uit de flowchart.  
  // !! Gebruik enkel de constanten uit de configuratie om met een categorie te vergelijken!
  // !! Gebruik enkel de constanten uit de configuratie om de duurtijd van het water geven mee te geven
  // !! Gebruik verder enkel de functies zetWaterpompAan() aan te zetten

}

void setup() {
  // TODO: Implementeer de nodig code voor lezen sensoren (indien nodig)

}

void loop() {
  // We hebben huidige millis nodig om de verschillende processen te controleren (water geven / stoppen)
  long huidigeMillis = millis();
  
  // TODO: Controleer of de waterpomp uitgezet moet worden en roep functie zetWaterpompUit() aan indien nodig

  // TODO: Controleer of sensoren ingelezen moeten worden en roep functie leesSensorenEnGeefWaterIndienNodig() aan indien nodig

}