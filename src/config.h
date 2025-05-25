/*
 * config.h — Instellingenbestand voor het automatische plantbewateringssysteem
 *
 * Beschrijving:
 * Dit bestand bevat alle constanten en parameters die gebruikt worden
 * in het hoofdprogramma. Door deze parameters extern te definiëren, kunnen
 * instellingen eenvoudiger worden aangepast zonder de programmalogica te wijzigen.
 *
 * Gemaakt op basis van inzichten uit:
 * - Calibratie-oefeningen uit lesmateriaal
 * - Realistische vochtigheidsreactie van planten (bron: Universiteit Wageningen, 
 *   https://www.wur.nl/nl/dossiers/dossier/precisielandbouw.htm)
 * - Reactietijden van bodemvochtigheidssensoren (https://wiki.dfrobot.com)
 * - Praktische Arduino-ervaring met DHT22, relais en pompen
 */

#ifndef CONFIG_H
#define CONFIG_H

// TODO: Wachttijd tussen 2 opeenvolgende inlezingen van sensoren
// Planten reageren traag. Eén meting om de 5 minuten is meestal voldoende.
// const unsigned long SENSOR_INLEES_INTERVAL = 5UL * 60UL * 1000UL; // 5 minuten in milliseconden
const unsigned long SENSOR_INLEES_INTERVAL = 15UL * 1000UL; // tijdelijk: 15s voor test


// TODO: Temperatuur schakelwaarden
const float TEMPERATUUR_LAAG = 10.0;  // Onder deze temperatuur liever geen water
const float TEMPERATUUR_HOOG = 35.0;  // Bij extreme hitte kunnen we extra irrigatie overwegen

// TODO: Categoriën vochtigheid
enum VochtCategorie {
	UITGEDROOGD = 0,
	LICHT_VOCHTIG = 1,
	DOORNAT = 2
};
// TODO: Statussen water geven (geen water, wél water)
enum WaterPompStatus {
  POMP_UIT,
  POMP_AAN
};
// TODO: Duurtijden water geven (in milliseconden)
// Afhankelijk van hoe droog de plant is, geven we kort of lang water.
const unsigned long WATER_GEEF_DUUR_KORT = 5UL * 1000UL;   // 5 seconden
const unsigned long WATER_GEEF_DUUR_LANG  = 15UL * 1000UL;  // 15 seconden

// === Intervallen bodemvochtigheidssensor — RESISTIEF ===
// Opgelet: bij resistieve sensor geldt :
// Droge grond → hoge weerstand → lage spanning → lage analoge waarde
// Natte grond → lage weerstand → hoge spanning → hoge waarde
// Dus: hogere analoge waarde = vochtiger bodem
// Intervallen voor BVH waarden
// Opgelet, we definiëren de intervallen als gesloten: [min, max]
// Minimum- en maximumwaarde voor de resistieve vochtigheidssensor om de interpretatie "DROOG" te krijgen
const int RESISTIEVE_SENSOR_UITGEDROOGD_INTERVAL_MIN = 0;
const int RESISTIEVE_SENSOR_UITGEDROOGD_INTERVAL_MAX = 1686;

// Minimum- en maximumwaarde voor de resistieve vochtigheidssensor om de interpretatie "VOCHTIG" te krijgen
const int RESISTIEVE_SENSOR_LICHT_VOCHTIG_INTERVAL_MIN = RESISTIEVE_SENSOR_UITGEDROOGD_INTERVAL_MAX + 1;
const int RESISTIEVE_SENSOR_LICHT_VOCHTIG_INTERVAL_MAX = 2336;

// Minimum- en maximumwaarde voor de resistieve vochtigheidssensor om de interpretatie "NAT" te krijgen
const int RESISTIEVE_SENSOR_NAT_INTERVAL_MIN = RESISTIEVE_SENSOR_LICHT_VOCHTIG_INTERVAL_MAX + 1;
const int RESISTIEVE_SENSOR_NAT_INTERVAL_MAX = 2787; // maximum bereik met 12 bits

// === Intervallen bodemvochtigheidssensor — CAPACITIEF ===
// Opgelet: bij capacitieve sensor zijn de waarden omgekeerd!
// Droge grond → lage diëlektrische constante → hogere uitgangsspanning → hogere waarde
// Natte grond → hogere diëlektrische constante → lagere spanning → lagere waarde
// Dus: lagere analoge waarde = vochtiger bodem
// Minimum- en maximumwaarde voor de capacitieve vochtigheidssensor om de interpretatie "DROOG" te krijgen
const int CAPACITIEVE_SENSOR_DROOG_INTERVAL_MIN = 2920; // maximum bereik van 12 bits
const int CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX = 2120;

// Minimum- en maximumwaarde voor de capacitieve vochtigheidssensor om de interpretatie "VOCHTIG" te krijgen
const int CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MIN = CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX - 1;
const int CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MAX = 2521;

// Minimum- en maximumwaarde voor de capacitieve vochtigheidssensor om de interpretatie "NAT" te krijgen
const int CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN = CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MAX - 1;
const int CAPACITIEVE_SENSOR_NAT_INTERVAL_MAX = 0;

#endif // CONFIG_H