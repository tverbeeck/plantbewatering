// TODO: Wachttijd tussen 2 opeenvolgende inlezingen van sensoren

// TODO: Temperatuur schakelwaarden

// TODO: Categoriën vochtigheid
enum VochtCategorie {
	UITGEDROOGD = 0,
	LICHT_VOCHTIG = 1,
	DOORNAT = 2
};
// TODO: Statussen water geven (geen water, wél water)

// TODO: Duurtijden water geven (in milliseconden)

// Intervallen voor BVH waarden
// Opgelet, we definiëren de intervallen als gesloten: [min, max]
// Minimum- en maximumwaarde voor de resistieve vochtigheidssensor om de interpretatie "DROOG" te krijgen
const int RESISTIEVE_SENSOR_DROOG_INTERVAL_MIN = 0;
const int RESISTIEVE_SENSOR_DROOG_INTERVAL_MAX = 376;

// Minimum- en maximumwaarde voor de resistieve vochtigheidssensor om de interpretatie "VOCHTIG" te krijgen
const int RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MIN = RESISTIEVE_SENSOR_DROOG_INTERVAL_MAX + 1;
const int RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MAX = 743;

// Minimum- en maximumwaarde voor de resistieve vochtigheidssensor om de interpretatie "NAT" te krijgen
const int RESISTIEVE_SENSOR_NAT_INTERVAL_MIN = RESISTIEVE_SENSOR_VOCHTIG_INTERVAL_MAX + 1;
const int RESISTIEVE_SENSOR_NAT_INTERVAL_MAX = 4095; // maximum bereik met 12 bits

// Minimum- en maximumwaarde voor de capacitieve vochtigheidssensor om de interpretatie "DROOG" te krijgen
const int CAPACITIEVE_SENSOR_DROOG_INTERVAL_MIN = 4095; // maximum bereik van 12 bits
const int CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX = 3150;

// Minimum- en maximumwaarde voor de capacitieve vochtigheidssensor om de interpretatie "VOCHTIG" te krijgen
const int CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MIN = CAPACITIEVE_SENSOR_DROOG_INTERVAL_MAX - 1;
const int CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MAX = 1420;

// Minimum- en maximumwaarde voor de capacitieve vochtigheidssensor om de interpretatie "NAT" te krijgen
const int CAPACITIEVE_SENSOR_NAT_INTERVAL_MIN = CAPACITIEVE_SENSOR_VOCHTIG_INTERVAL_MAX - 1;
const int CAPACITIEVE_SENSOR_NAT_INTERVAL_MAX = 0;