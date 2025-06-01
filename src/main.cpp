/**
 * Hier alle bronnen : 
 * https://www.instructables.com/How-to-use-DHT-22-sensor-Arduino-Tutorial/ 
 * https://elektronicavoorjou.nl/project/arduino-temp-luchtvochtigheid-dht22-iot-cloud/ 
 * https://lastminuteengineers.com/esp32-deep-sleep-wakeup-sources/ (1/6/2025)
 * https://randomnerdtutorials.com/esp32-external-wake-up-deep-sleep/ (1/6/2025)
 * https://github.com/stm32duino/LIS2DW12 (1/6/2025)
 * https://www.st.com/resource/en/datasheet/lis2dw12.pdf (1/6/2025)
 * erna ergens : 
 * Ook, waarom accelerometer (bv als plant wordt omver geduwd sat watertoevoer stopt)
 * Waarom DHT 22 gekozen
 * Hoe bereken je de samengestelde categorieën (2 bodemvochtigheidssensoren : bv 1 priotair nemen (DROOG bv altijd eerste prioritair))
 */

 // Opmerking voor mij : Ctrl + Alt + U kan ook om te herstarten

 // =============== ENUM ivm WAKE UP ===============
enum class WakeupReason {
  BY_PANIC_BUTTON,
  BY_TIMER,             // Voor het geval ik later timer wake-ups toevoeg
  UNKNOWN_OR_POWER_ON   // Of een eerste start óf een onbekende wake-up
};


#include "Arduino.h"
#include "config.h"
#include <DHT.h>
#include <Wire.h>
#define ARDUINOTRACE_ENABLE 1 // Zet deze op 1 om debugging aan te zetten, op 0 om debugging uit te zetten
#include <ArduinoTrace.h>
#include <DFRobot_LIS2DW12.h>


// Device ID for LIS2DW12 according to datasheet (0x44)
#define LIS2DW12_DEVICE_ID 0x44

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

#define PANIC_BUTTON_PIN 26  // dd 31/05 dit gekozen, maar nog aan te passen volgens de opstelling
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// Maak een TwoWire instantie voor I2C communicatie
TwoWire dev_i2c(0); // Gebruik I2C bus 0 op ESP32

// Maak een LIS2DW12Sensor object
DFRobot_LIS2DW12_I2C Accelero(&dev_i2c);

// Drempelwaarde voor valdetectie op Y-as (in mg, experimenteer hiermee)
// Een plotselinge daling naar bijna 0g kan een vrije val indiceren.
// De waarde hangt af van de oriëntatie van de sensor.
// Als Y-as verticaal omhoog wijst, is rust ~+1000mg. Tijdens val ~0mg.
const int16_t FREEFALL_THRESHOLD_MG = 200; // Bijvoorbeeld, als Y-waarde onder 0.2g komt
const unsigned long FREEFALL_DURATION_MS = 100; // Moet minstens X ms onder drempel zijn

// Variabelen voor simpele valdetectie logica
bool potentiallyFalling = false;
unsigned long fallStartTime = 0;


RTC_DATA_ATTR int bootCount = 0; /* Houdt het aantal herstarts bij */

String categorieNaarString(VochtCategorie categorie) {
  switch (categorie) {
    case UITGEDROOGD: return "UITGEDROOGD";
    case LICHT_VOCHTIG: return "LICHT_VOCHTIG";
    case DOORNAT: return "DOORNAT";
    default: return "ONBEKEND";
  }
}


float tempGlobal = 0; // Globale temperatuur variabele
DFRobot_LIS2DW12_I2C acce;

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
  float x, y, z;
  Accelero.readAccX(); // x, y, z zijn nu in m/s²
  Accelero.readAccY();
  Accelero.readAccZ();

    Serial.print("X: "); Serial.print(x); Serial.print(" m/s²\t");
    Serial.print("Y: "); Serial.print(y); Serial.print(" m/s²\t");
    Serial.print("Z: "); Serial.print(z); Serial.println(" m/s²");

    // Je kunt nu je valdetectie direct op y (m/s²) doen, OF omzetten naar mg:
    int y_mg = int(y / 9.81 * 1000); // m/s² -> mg, als je drempel in mg wilt aanhouden
if (abs(y_mg) < FREEFALL_THRESHOLD_MG) {
    if (!potentiallyFalling) {
      potentiallyFalling = true;
      fallStartTime = millis();
      Serial.println("--- Potentiële start van val ---");
    } else {
      if (millis() - fallStartTime > FREEFALL_DURATION_MS) {
        Serial.println("!!!!!!!!!!!!!!!! VAL GEDETECTEERD (Y-as) !!!!!!!!!!!!!!!!");
        // Reset voor volgende detectie (of blokkeer voor een tijdje)
        potentiallyFalling = false;
        // Hier zou je een actie kunnen uitvoeren (alarm, bericht sturen, etc.)
        delay(2000); // Wacht even om spam te voorkomen
      }
    }
  } else {
    if (potentiallyFalling) {
      Serial.println("--- Potentiële val afgebroken ---");
    }
    potentiallyFalling = false;
  }
  // TODO: Implementeer inlezen met correcte pinnen
  int capacitieve_bvh_waarde = analogRead(CAPACITIEVE_BVH_PIN);
  int resistieve_bvh_waarde = analogRead(RESISTIEVE_BVH_PIN);
  float temperatuur = leesTemperatuur();

  DUMP(capacitieve_bvh_waarde);
  DUMP(resistieve_bvh_waarde);
  DUMP(temperatuur);  

  Serial.print("Temperatuur: ");
  Serial.println(temperatuur);

  // Bepaal individuele categoriën en samengestelde categorie
  VochtCategorie categorieCapacitieveBVH = berekenCategorieCapacitieveBVH(capacitieve_bvh_waarde);
  VochtCategorie categorieResistieveBVH = berekenCategorieResistieveBVH(resistieve_bvh_waarde);
  VochtCategorie categorie = berekenSamengesteldeCategorie(categorieCapacitieveBVH, categorieResistieveBVH);

  DUMP(categorieCapacitieveBVH);
  DUMP(categorieResistieveBVH);
  DUMP(categorie);
 
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

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/


// =============== FUNCTIE OM WAKE-UP REDEN TE BEPALEN (RETOURNEERT een ENUM) ===============
WakeupReason print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  // Log de rauwe oorzaak voor debuggen
  Serial.print("Raw wakeup cause: ");
  Serial.println(wakeup_reason);

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: // Wake-up door externe interrupt 0 (onze knop)
      Serial.println("Wakeup identified: BY_PANIC_BUTTON (EXT0)");
      return WakeupReason::BY_PANIC_BUTTON;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup identified: BY_TIMER");
      return WakeupReason::BY_TIMER;
    default:
      Serial.println("Wakeup identified: UNKNOWN_OR_POWER_ON");
      return WakeupReason::UNKNOWN_OR_POWER_ON;
  }
}


void setup() {
  // TODO: Implementeer de nodig code voor lezen sensoren (indien nodig)
  Serial.begin(115200); // 9600 als baudrate lukt niet, want dan krijg ik rare tekens
// Initialiseer I2C
  dev_i2c.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Initialiseer de accelerometer
  Accelero.begin(); // Dit probeert het standaard I2C adres

  // Controleer of de sensor is gevonden
  uint8_t id = Accelero.getID();
  if (id != LIS2DW12_DEVICE_ID) {
    Serial.println("LIS2DW12 niet gevonden! Controleer bedrading en I2C adres.");
    while (1); // Stop uitvoering
  }
  Serial.println("LIS2DW12 gevonden!");

  // Schakel de accelerometer in (de begin() functie activeert de meting)
  // Indien nodig, stel bereik en powermode in:
  // Accelero.setRange(DFRobot_LIS2DW12::RANGE_2G);
  // Accelero.setPowerMode(DFRobot_LIS2DW12::HIGH_PERFORMANCE_MODE);
    ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  esp_sleep_enable_ext0_wakeup((gpio_num_t)PANIC_BUTTON_PIN, 1);  // 1 = High, 0 = Low (zie https://randomnerdtutorials.com/esp32-external-wake-up-deep-sleep/)
  // hierboven de casting (gpio_num_t)omdat hij in deepsleep niet aan de code kan


  pinMode(pompPin, OUTPUT);

  /*
    First we configure the wake up source
    We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");


  
  dht.begin();
}

void loop() {
  print_wakeup_reason(); // Print de reden waarom de ESP32 is wakker geworden
  // Als je de enum reden nodig hebt, kun je get_wakeup_reason() aanroepen
  Serial.println("Loop gestart");
  // PANIC BUTTON Check (active LOW)
  if (digitalRead(PANIC_BUTTON_PIN) == HIGH && statusWaterpomp == STATUS_POMP_UIT) {
    Serial.println("PANIC BUTTON INGEDRUKT!");
    zetWaterpompAan(WATER_GEEF_DUUR_PANIC);
    delay(200); // debounce
  }

  // lees de tijd aan het begin via de standaard constructor millis()
  // Dit is nodig om de tijd te kunnen vergelijken met de vorige inleestijd.
  unsigned long huidigeMillis = millis(); 

  if (statusWaterpomp == StatusWaterpomp::STATUS_POMP_UIT) {
    leesSensorenEnGeefWaterIndienNodig();
    DUMP("Sensoren zijn ingelezen, we gaan in deep sleep  : ");

    Serial.flush();
    esp_deep_sleep_start();
    Serial.println("We zouden nu in deep sleep moeten gaan, maar dat lukt niet altijd. Dit is een bekend probleem bij de ESP32. Als ik deze lijn lees zitten we dus niet in deep sleep");
  }

  // Controleer of de waterpomp uitgezet moet worden.
  // We printen eerst de waarden om te debuggen.
  if (statusWaterpomp == StatusWaterpomp::STATUS_POMP_AAN) {
    // De eigenlijke check om de pomp uit te zetten.
    if ((huidigeMillis - startTijdWaterGeven) >= duurTijdWaterGeven) {
      zetWaterpompUit(); // Zet de waterpomp uit als de tijd (duurtTijdWaterGeven) verstreken is
    }
  }

}