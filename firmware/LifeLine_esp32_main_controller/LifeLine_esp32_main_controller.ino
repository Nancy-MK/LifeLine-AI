#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ================= WIFI =================
const char* ssid = "SmartEvacSystem";
const char* password = "evac2025";
WebServer server(80);

// ================= TIME =================
unsigned long lastRead = 0;
const int interval = 500;

// ================= DS18B20 =================
#define ONE_WIRE_BUS 15
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);

// ================= GAS =================
#define MQ2_PIN 34
#define MQ7_PIN 35

// ================= PIR =================
#define PIR_LOUNGE 25
#define PIR_OPEN_TOP 26
#define PIR_OPEN_BOTTOM 27
#define PIR_KITCHEN 32
#define PIR_PASSAGE 33
#define PIR_HALL 14
#define PIR_PRIVATE 19
#define PIR_MEETING_TR 17
#define PIR_MEETING_BR 18

// ================= ALERT =================
#define LED_PIN 13
#define BUZZER_PIN 12

// ================= ROOM IDS =================
#define R_LOUNGE 0
#define R_OPEN_TOP 1
#define R_OPEN_BOTTOM 2
#define R_KITCHEN 3
#define R_PASSAGE 4
#define R_HALL 5
#define R_FEMALE_BATH 6
#define R_MALE_BATH 7
#define R_PRIVATE 8
#define R_MEETING_TR 9
#define R_MEETING_BR 10
#define NUM_ROOMS 11

#define MAX_NEIGHBORS 5

// ================= ADAPTIVE THRESHOLDS =================
// Deviation above baseline to start scoring
#define TEMP_DEVIATION    8.0f    // °C above room's normal
#define GAS_DEVIATION     300     // ADC above room's normal
#define RATE_DEVIATION    1.0f    // °C/sec — rapid rise

// Temperature override — unambiguous fire regardless of other sensors
#define TEMP_OVERRIDE     70.0f

// ================= ADJACENCY =================
const int adjacency[NUM_ROOMS][MAX_NEIGHBORS] = {
  /* Lounge          */ { R_OPEN_TOP, R_OPEN_BOTTOM, R_KITCHEN,     -1, -1 },
  /* OpenOffice_Top  */ { R_LOUNGE,   R_HALL,        R_MEETING_TR,  -1, -1 },
  /* OpenOffice_Bot  */ { R_LOUNGE,   R_HALL,        R_MEETING_BR,  -1, -1 },
  /* Kitchen         */ { R_LOUNGE,   R_PASSAGE,     -1,            -1, -1 },
  /* Passage         */ { R_KITCHEN,  R_HALL,        R_FEMALE_BATH, R_MALE_BATH, -1 },
  /* Hall            */ { R_PASSAGE,  R_OPEN_TOP,    R_OPEN_BOTTOM, R_PRIVATE,   -1 },
  /* FemaleBathroom  */ { R_PASSAGE,  -1,            -1,            -1, -1 },
  /* MaleBathroom    */ { R_PASSAGE,  -1,            -1,            -1, -1 },
  /* PrivateOffice   */ { R_HALL,     -1,            -1,            -1, -1 },
  /* Meeting_TR      */ { R_OPEN_TOP, -1,            -1,            -1, -1 },
  /* Meeting_BR      */ { R_OPEN_BOTTOM, -1,         -1,            -1, -1 }
};

// ================= SENSOR CONFIG PER ROOM =================
struct SensorConfig {
  bool hasTemp;
  bool hasGas;
};

const SensorConfig sensorConfig[NUM_ROOMS] = {
  /* Lounge          */ { false, false },
  /* OpenOffice_Top  */ { true,  true  },  // Temp + MQ2
  /* OpenOffice_Bot  */ { false, false },
  /* Kitchen         */ { true,  true  },  // Temp + MQ7
  /* Passage         */ { false, false },
  /* Hall            */ { true,  false },  // Temp only
  /* FemaleBathroom  */ { false, false },
  /* MaleBathroom    */ { false, false },
  /* PrivateOffice   */ { false, false },
  /* Meeting_TR      */ { false, false },
  /* Meeting_BR      */ { true,  false },  // Temp only
};

// ================= ROOM =================
struct Room {
  String name;
  float temp = 0;
  float prevTemp = 0;
  float tempRate = 0;
  int gas = 0;
  int motion = 0;
  int rawRisk = 0;
  int risk = 0;
  bool blocked = false;

  // Adaptive baselines — learn what's "normal" for each room
  float tempBaseline = 25.0;   // Adapts over time
  float gasBaseline = 0;       // Adapts over time
};

Room rooms[] = {
  {"Lounge"},
  {"OpenOffice_Top"},
  {"OpenOffice_Bottom"},
  {"Kitchen"},
  {"Passage"},
  {"Hall"},
  {"FemaleBathroom"},
  {"MaleBathroom"},
  {"PrivateOffice"},
  {"Meeting_TopRight"},
  {"Meeting_BottomRight"}
};

// ================= FILTER =================
float smooth(float newVal, float oldVal) {
  return 0.7 * oldVal + 0.3 * newVal;
}

// ================= GAS HARDWARE BASELINE =================
int mq2_baseline = 0;
int mq7_baseline = 0;

void calibrateGas() {
  Serial.println("Calibrating gas hardware baseline...");
  for (int i = 0; i < 20; i++) {
    mq2_baseline += analogRead(MQ2_PIN);
    mq7_baseline += analogRead(MQ7_PIN);
    delay(100);
  }
  mq2_baseline /= 20;
  mq7_baseline /= 20;
  Serial.printf("Hardware baseline — MQ2: %d | MQ7: %d\n", mq2_baseline, mq7_baseline);
}

// ================= ADAPTIVE BASELINE UPDATE =================
// Only learns when the room is safe (risk < 20)
// Slowly drifts toward current readings over minutes
void updateBaselines() {
  for (int i = 0; i < NUM_ROOMS; i++) {
    // Only learn when safe — don't adapt to fire as "normal"
    if (rooms[i].risk >= 20) continue;

    SensorConfig cfg = sensorConfig[i];

    // Temperature baseline: 99% old + 1% new → full adaptation in ~5 minutes
    if (cfg.hasTemp && rooms[i].temp > 0) {
      rooms[i].tempBaseline = rooms[i].tempBaseline * 0.99 + rooms[i].temp * 0.01;
    }

    // Gas baseline: same slow learning
    if (cfg.hasGas) {
      rooms[i].gasBaseline = rooms[i].gasBaseline * 0.99 + rooms[i].gas * 0.01;
    }
  }
}

// ================= ADAPTIVE SENSOR FUSION ENGINE =================
// Scores DEVIATION from each room's learned baseline
// Not fixed thresholds — the system adapts to each room's normal
int calculateRisk(int roomId) {
  Room& room = rooms[roomId];
  SensorConfig cfg = sensorConfig[roomId];

  // ---- Step 1: Score deviation from this room's baseline ----

  int tempScore = 0;
  if (cfg.hasTemp && room.temp > (room.tempBaseline + TEMP_DEVIATION)) {
    float deviation = room.temp - room.tempBaseline - TEMP_DEVIATION;
    tempScore = constrain((int)(deviation * 5), 0, 100);
  }

  int rateScore = 0;
  if (cfg.hasTemp && room.tempRate > RATE_DEVIATION) {
    float deviation = room.tempRate - RATE_DEVIATION;
    rateScore = constrain((int)(deviation * 30), 0, 100);
  }

  int gasScore = 0;
  if (cfg.hasGas && room.gas > (room.gasBaseline + GAS_DEVIATION)) {
    float deviation = room.gas - room.gasBaseline - GAS_DEVIATION;
    gasScore = constrain((int)(deviation * 100.0 / 2500.0), 0, 100);
  }

  // ---- Step 2: Adaptive weighted combination ----
  float totalWeight = 0;
  float risk = 0;

  if (cfg.hasTemp) {
    risk += tempScore * 0.35;
    risk += rateScore * 0.25;
    totalWeight += 0.60;
  }

  if (cfg.hasGas) {
    risk += gasScore * 0.40;
    totalWeight += 0.40;
  }

  // Normalize for rooms with fewer sensors
  if (totalWeight > 0) {
    risk = risk / totalWeight;
  }

  // ---- Step 3: Cross-confirmation bonus ----
  int confirming = 0;
  if (cfg.hasTemp && tempScore > 30)  confirming++;
  if (cfg.hasTemp && rateScore > 30)  confirming++;
  if (cfg.hasGas  && gasScore > 30)   confirming++;

  if (confirming >= 2) {
    risk *= 1.5;
  }
  if (confirming >= 3) {
    risk *= 1.3;
  }

  // Single-sensor boost
  if (confirming == 0 && risk > 15) {
    risk *= 1.3;
  }

  return constrain((int)risk, 0, 100);
}

// ================= PROPAGATION =================
void propagateRisk() {
  int tempArr[NUM_ROOMS];

  for (int i = 0; i < NUM_ROOMS; i++) {
    tempArr[i] = rooms[i].rawRisk;

    int maxNeighbor = 0;
    for (int j = 0; j < MAX_NEIGHBORS; j++) {
      int n = adjacency[i][j];
      if (n == -1) break;
      if (rooms[n].rawRisk > maxNeighbor)
        maxNeighbor = rooms[n].rawRisk;
    }

    int boost = min(50, maxNeighbor * 40 / 100);
    if (boost > tempArr[i]) tempArr[i] = boost;
  }

  for (int i = 0; i < NUM_ROOMS; i++) {
    rooms[i].rawRisk = tempArr[i];
  }
}

// ================= SMOOTH + DECAY =================
void smoothRisk() {
  for (int i = 0; i < NUM_ROOMS; i++) {
    if (rooms[i].rawRisk > rooms[i].risk) {
      rooms[i].risk = rooms[i].risk * 0.3 + rooms[i].rawRisk * 0.7;
    } else {
      rooms[i].risk = rooms[i].risk * 0.5 + rooms[i].rawRisk * 0.5;
    }

    rooms[i].risk = constrain(rooms[i].risk, 0, 100);
    rooms[i].blocked = rooms[i].risk >= 60;
  }
}

// ================= TEMP =================
void updateTemp(int i, float t) {
  float old = rooms[i].temp;
  rooms[i].temp = smooth(t, old);
  rooms[i].tempRate = (rooms[i].temp - old) * 2;
}

// ================= UPDATE =================
void updateSystem() {

  tempSensors.requestTemperatures();
  delay(100);

  float t[4];
  for (int i = 0; i < 4; i++) t[i] = tempSensors.getTempCByIndex(i);

  if (t[0] > 0) updateTemp(R_OPEN_TOP, t[0]);
  if (t[1] > 0) updateTemp(R_KITCHEN, t[1]);
  if (t[2] > 0) updateTemp(R_HALL, t[2]);
  if (t[3] > 0) updateTemp(R_MEETING_BR, t[3]);

  // GAS (hardware baseline subtracted)
  int mq2 = max(0, analogRead(MQ2_PIN) - mq2_baseline);
  int mq7 = max(0, analogRead(MQ7_PIN) - mq7_baseline);

  // Dead zone — noise filter
  if (mq2 < 200) mq2 = 0;
  if (mq7 < 200) mq7 = 0;

  rooms[R_OPEN_TOP].gas = mq2;
  rooms[R_KITCHEN].gas = mq7;

  // PIR (occupancy only — never affects fire risk)
  rooms[R_LOUNGE].motion = digitalRead(PIR_LOUNGE);
  rooms[R_OPEN_TOP].motion = digitalRead(PIR_OPEN_TOP);
  rooms[R_OPEN_BOTTOM].motion = digitalRead(PIR_OPEN_BOTTOM);
  rooms[R_KITCHEN].motion = digitalRead(PIR_KITCHEN);
  rooms[R_PASSAGE].motion = digitalRead(PIR_PASSAGE);
  rooms[R_HALL].motion = digitalRead(PIR_HALL);
  rooms[R_PRIVATE].motion = digitalRead(PIR_PRIVATE);
  rooms[R_MEETING_TR].motion = digitalRead(PIR_MEETING_TR);
  rooms[R_MEETING_BR].motion = digitalRead(PIR_MEETING_BR);

  // ADAPTIVE BASELINES (learn what's normal per room)
  updateBaselines();

  // RAW RISK (deviation-based adaptive fusion)
  for (int i = 0; i < NUM_ROOMS; i++) {
    rooms[i].rawRisk = calculateRisk(i);
  }

  propagateRisk();
  smoothRisk();

  // ALERT
  int maxRisk = 0;
  for (int i = 0; i < NUM_ROOMS; i++) {
    if (rooms[i].risk > maxRisk) maxRisk = rooms[i].risk;
  }

  // Temperature override — only unambiguous signal
  // 70°C is definitely fire, no other explanation
  bool tempOverride = false;
  for (int i = 0; i < NUM_ROOMS; i++) {
    if (rooms[i].temp > TEMP_OVERRIDE) {
      tempOverride = true;
      break;
    }
  }

  // Two-stage alert
  digitalWrite(LED_PIN, maxRisk >= 50 || tempOverride);
  digitalWrite(BUZZER_PIN, maxRisk >= 70 || tempOverride);

  // DEBUG
  Serial.println("==== ROOMS ====");
  for (int i = 0; i < NUM_ROOMS; i++) {
    Serial.printf("%s | T:%.1f(b:%.1f) dT:%.2f G:%d(b:%.0f) M:%d | raw:%d risk:%d%s\n",
      rooms[i].name.c_str(),
      rooms[i].temp,
      rooms[i].tempBaseline,
      rooms[i].tempRate,
      rooms[i].gas,
      rooms[i].gasBaseline,
      rooms[i].motion,
      rooms[i].rawRisk,
      rooms[i].risk,
      rooms[i].blocked ? " [BLOCKED]" : ""
    );
  }
}

// ================= API =================
void handleAPI() {
  Serial.println("API HIT");

  DynamicJsonDocument doc(3072); // slightly bigger but safe
  JsonArray arr = doc.to<JsonArray>();

  for (int i = 0; i < NUM_ROOMS; i++) {
    JsonObject r = arr.createNestedObject();
    r["name"] = rooms[i].name;
    r["temp"] = rooms[i].temp;
    r["gas"] = rooms[i].gas;
    r["risk"] = rooms[i].risk;
    r["blocked"] = rooms[i].blocked;
    r["occupied"] = rooms[i].motion;

    JsonArray nb = r.createNestedArray("neighbors");
    for (int j = 0; j < MAX_NEIGHBORS; j++) {
      int n = adjacency[i][j];
      if (n == -1) break;
      nb.add(rooms[n].name);
    }
  }

  String res;
  serializeJson(doc, res);
  server.send(200, "application/json", res);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(PIR_LOUNGE, INPUT);
  pinMode(PIR_OPEN_TOP, INPUT);
  pinMode(PIR_OPEN_BOTTOM, INPUT);
  pinMode(PIR_KITCHEN, INPUT);
  pinMode(PIR_PASSAGE, INPUT);
  pinMode(PIR_HALL, INPUT);
  pinMode(PIR_PRIVATE, INPUT);
  pinMode(PIR_MEETING_TR, INPUT);
  pinMode(PIR_MEETING_BR, INPUT);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  tempSensors.begin();
  tempSensors.setResolution(9);

  int sensorCount = tempSensors.getDeviceCount();
  Serial.printf("[TEMP] Found %d DS18B20 sensor(s) on pin %d\n", sensorCount, ONE_WIRE_BUS);

  calibrateGas();

WiFi.softAP(ssid, password);

// ✅ ADD THIS
server.on("/", []() {
  server.send(200, "text/plain", "ESP32 SERVER WORKING");
});

// ✅ KEEP THIS
server.on("/api/rooms", handleAPI);

server.begin();



  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.softAPIP());
  

  Serial.println("[BOOT] System ready — adaptive baselines learning...");
}

// ================= LOOP =================
void loop() {
  server.handleClient();

  if (millis() - lastRead > interval) {
    lastRead = millis();
    updateSystem();
  }
}