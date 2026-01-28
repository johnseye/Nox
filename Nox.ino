// NOX v4.8 - Added "HEART" Guided Pattern
// 1. "HEART": Runs Green (5s) -> Yellow (3s) -> Blue (5s) with Heartbeat Pulse.
// 2. "OFF": Forces darkness for 8s.
// 3. "BLINK": 4s Rainbow Spinner.
// 4. DYNAMIC: "BREATH" command updates standard rhythm.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_NeoPixel.h>

// -------------------- PIN CONFIG --------------------
#define PIN_NEOPIXEL    13
#define NUM_PIXELS      12
#define PIN_PULSE       25
#define PIN_SDA         23
#define PIN_SCL         22

// -------------------- TUNING PARAMETERS --------------------
#define IDLE_TIMEOUT    5000    
#define MIN_AMPLITUDE   100     
#define DEBUG_SERIAL    0

// -------------------- OBJECTS --------------------
Adafruit_NeoPixel strip(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// -------------------- GLOBAL STATE --------------------
unsigned long lastActivityTime = 0;
bool isActive = false; 

// --- MODES ---
bool isBlinking = false;
unsigned long blinkStartTime = 0;
unsigned long sensorLockoutUntil = 0;

bool isOffMode = false;          
unsigned long offStartTime = 0;  

// NEW: Heart Pattern Mode
bool isHeartMode = false;
unsigned long heartStartTime = 0;

// -------------------- PULSE VARS --------------------
int bpm = 0;
int pulseSignal;
int IBI = 600;
bool pulseDetected = false;
bool QS = false;                
int rate[10];
unsigned long sampleCounter = 0;
unsigned long lastBeatTime = 0;
int P = 2048, T = 2048, thresh = 2048, amp = 0;
bool firstBeat = true, secondBeat = false;

// -------------------- DYNAMIC BREATHING VARS --------------------
unsigned long breathStartTime = 0;
unsigned long timeIn = 7000;    
unsigned long timeHold = 4000;  
unsigned long timeOut = 8000;   
unsigned long cycleTotal = 19000; 

// -------------------- SESSION VARS --------------------
bool sessionRunning = false;
unsigned long sessionStartMs = 0;
unsigned long sessionEndMs = 0;
long sumFirst20 = 0;
int  cntFirst20 = 0;
const int LAST20_BUF = 300; 
int last20Bpm[LAST20_BUF];
unsigned long last20T[LAST20_BUF];
int last20Head = 0;
int last20Count = 0;
int bpmMin = 999;
int bpmMax = 0;
unsigned long bpmMinAt = 0;
unsigned long bpmMaxAt = 0;

// -------------------- HELPERS --------------------
void sendToProtoPie(const char* message) { Serial.println(message); }
void sendToProtoPie(const char* message, long value) {
  Serial.print(message); Serial.print("||"); Serial.println(value);
}
void resetSessionStats() {
  sumFirst20 = 0; cntFirst20 = 0; last20Head = 0; last20Count = 0;
  bpmMin = 999; bpmMax = 0; bpmMinAt = 0; bpmMaxAt = 0;
}
void pushLast20Sample(int sampleBpm, unsigned long tMs) {
  last20Bpm[last20Head] = sampleBpm; last20T[last20Head] = tMs;
  last20Head = (last20Head + 1) % LAST20_BUF;
  if (last20Count < LAST20_BUF) last20Count++;
}
int computeAvgLast20(unsigned long endMs) {
  if (last20Count == 0) return 0;
  const unsigned long windowStart = (endMs >= 20000) ? (endMs - 20000) : 0;
  long sum = 0; int cnt = 0;
  for (int i = 0; i < last20Count; i++) {
    int idx = (last20Head - 1 - i); if (idx < 0) idx += LAST20_BUF;
    unsigned long t = last20T[idx]; if (t < windowStart) break;
    sum += last20Bpm[idx]; cnt++;
  }
  return (cnt == 0) ? 0 : (int)(sum / cnt);
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  strip.begin(); strip.clear(); strip.show(); strip.setBrightness(255);
  Wire.begin(PIN_SDA, PIN_SCL);
  if (!accel.begin()) { /* Accel ignored */ }
  
  cycleTotal = timeIn + timeHold + timeOut;
  breathStartTime = millis();
  
  sendToProtoPie("DEVICE_READY");
}

// -------------------- LOOP --------------------
void loop() {
  unsigned long now = millis();
  pulseSignal = analogRead(PIN_PULSE);
  sampleCounter = now;

  // --- 1. COMMAND LISTENER ---
  if (Serial.available() > 0) {
    String incoming = Serial.readStringUntil('\n');
    incoming.trim(); 

    // COMMAND: OFF
    if (incoming.startsWith("OFF")) {
      isOffMode = true; offStartTime = now; isActive = false;
      isHeartMode = false; // Override Heart
    }
    // COMMAND: HEART (NEW)
    else if (incoming.startsWith("HEART")) {
      isHeartMode = true;
      heartStartTime = now;
      isActive = true;
      lastActivityTime = now;
      isOffMode = false; isBlinking = false; // Override others
    }
    // COMMAND: BLINK
    else if (incoming.startsWith("BLINK")) {
      isBlinking = true; blinkStartTime = now; isActive = true; lastActivityTime = now;
      isOffMode = false; isHeartMode = false;
    }
    // DEBUG H
    else if (incoming == "H" || incoming == "h") {
      runProtoPieTest();
    }
    // DEBUG R
    else if (incoming == "R" || incoming == "r") {
      Serial.print("Breath: IN="); Serial.print(timeIn);
      Serial.print(" HOLD="); Serial.print(timeHold);
      Serial.print(" OUT="); Serial.println(timeOut);
    }
    // UPDATE BREATH
    else if (incoming.startsWith("BREATH")) {
      parseBreathingCommand(incoming);
    }
  }

  // --- 2. SENSOR LOGIC ---
  bool fingerDetected = false;
  // Sensors allowed if NOT Locked, NOT Off, and NOT in Heart Mode (Guided)
  if (now > sensorLockoutUntil && !isHeartMode) {
      fingerDetected = runPulseSensorAlgorithm(pulseSignal, now); 
  } else {
      fingerDetected = false; pulseDetected = false; QS = false;
  }
  
  bool wasActive = isActive;

  if (fingerDetected) {
    if (!isActive) {
      breathStartTime = now;
      thresh = 2048; P = 2048; T = 2048;
      if (DEBUG_SERIAL) Serial.println("WAKE UP");
    }
    lastActivityTime = now;
    isActive = true;
  } else {
    // SLEEP LOGIC
    if (now - lastActivityTime > IDLE_TIMEOUT && !isBlinking && !isOffMode && !isHeartMode) {
      if (isActive) { isActive = false; if (DEBUG_SERIAL) Serial.println("SLEEP"); }
    }
  }

  // --- 3. SESSION MANAGEMENT ---
  if (!sessionRunning && isActive && fingerDetected) {
    sessionRunning = true; sessionStartMs = now; resetSessionStats();
    sendToProtoPie("SESSION_START", (long)sessionStartMs);
  }

  if (sessionRunning && !fingerDetected && !isBlinking && !isOffMode && !isHeartMode) {
    sessionRunning = false; sessionEndMs = now;
    unsigned long duration = sessionEndMs - sessionStartMs;
    int avgFirst20 = (cntFirst20 > 0) ? (int)(sumFirst20 / cntFirst20) : 0;
    int avgLast20  = computeAvgLast20(sessionEndMs);
    if (bpmMin == 999) bpmMin = 0;
    
    sendToProtoPie("BPM_MIN", (long)bpmMin);
    sendToProtoPie("BPM_MIN_AT", (long)bpmMinAt);
    sendToProtoPie("BPM_MAX", (long)bpmMax);
    sendToProtoPie("BPM_MAX_AT", (long)bpmMaxAt);
    sendToProtoPie("AVG_BPM_FIRST20", (long)avgFirst20);
    sendToProtoPie("AVG_BPM_LAST20", (long)avgLast20);
    sendToProtoPie("SESSION_END", (long)duration);
    isActive = false;
  }
  
  // --- 4. DATA STREAMING ---
  if (QS && bpm > 0 && sessionRunning) {
    sendToProtoPie("BPM", (long)bpm);
    unsigned long tRel = now - sessionStartMs;
    if (tRel <= 20000) { sumFirst20 += bpm; cntFirst20++; }
    pushLast20Sample(bpm, now);
    if (bpm < bpmMin) { bpmMin = bpm; bpmMinAt = tRel; }
    if (bpm > bpmMax) { bpmMax = bpm; bpmMaxAt = tRel; }
  }

  // --- 5. ANIMATION CONTROLLER ---
  
  // PRIORITY 1: OFF MODE
  if (isOffMode) {
    if (now - offStartTime > 8000) isOffMode = false; 
    else { strip.clear(); strip.show(); }
  }
  // PRIORITY 2: HEART PATTERN MODE (NEW)
  else if (isHeartMode) {
    unsigned long tHeart = now - heartStartTime;
    
    if (tHeart > 13000) {
      isHeartMode = false; // Sequence finished
      strip.clear(); strip.show();
    } else {
      // Determine Color Phase
      uint8_t r = 0, g = 0, b = 0;
      if (tHeart < 5000) {
         // GREEN (0-5s)
         g = 255; 
      } else if (tHeart < 8000) {
         // YELLOW (5-8s)
         r = 255; g = 255;
      } else {
         // BLUE (8-13s)
         b = 255;
      }
      
      // Simulate Heartbeat Pulse (Sine Wave ~75 BPM)
      // Varies brightness between 50 and 255
      float beat = sin(now * 0.008); 
      int brightness = 50 + (int)(205.0 * abs(beat));
      
      strip.setBrightness(brightness);
      for(int i=0; i<strip.numPixels(); i++) strip.setPixelColor(i, strip.Color(r, g, b));
      strip.show();
      strip.setBrightness(255); // Reset
    }
  }
  // PRIORITY 3: BLINK MODE
  else if (isBlinking) {
    unsigned long elapsed = now - blinkStartTime;
    if (elapsed > 4000) {
      isBlinking = false; isActive = false; sensorLockoutUntil = now + 5000; 
      strip.clear(); strip.show();
    } else {
      int brightness = 255;
      if (elapsed < 500) brightness = map(elapsed, 0, 500, 0, 255);
      else if (elapsed > 3500) brightness = map(elapsed, 3500, 4000, 255, 0);
      
      long firstPixelHue = (now * 20) % 65536; 
      strip.setBrightness(brightness); 
      for(int i=0; i<strip.numPixels(); i++) { 
        int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
        strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
      }
      strip.show();
      strip.setBrightness(255); 
    }
  } 
  // PRIORITY 4: BREATHING
  else if (isActive) {
    runBreathingPattern(now, pulseSignal, thresh, amp);
  } 
  else {
    strip.clear(); strip.show();
  }
  delay(10);
}

// -------------------- PARSE CONFIG FUNCTION --------------------
void parseBreathingCommand(String msg) {
  msg.replace("||", ",");
  int firstComma = msg.indexOf(',');
  int secondComma = msg.indexOf(',', firstComma + 1);
  int thirdComma = msg.indexOf(',', secondComma + 1);

  if (firstComma > 0 && secondComma > 0 && thirdComma > 0) {
    String sIn = msg.substring(firstComma + 1, secondComma);
    String sHold = msg.substring(secondComma + 1, thirdComma);
    String sOut = msg.substring(thirdComma + 1);

    long newIn = sIn.toInt();
    long newHold = sHold.toInt();
    long newOut = sOut.toInt();

    if (newIn < 100) newIn *= 1000;
    if (newHold < 100) newHold *= 1000;
    if (newOut < 100) newOut *= 1000;

    timeIn = newIn;
    timeHold = newHold;
    timeOut = newOut;
    cycleTotal = timeIn + timeHold + timeOut;
    breathStartTime = millis(); 
  }
}

// -------------------- ORIGINAL ALGORITHM --------------------
bool runPulseSensorAlgorithm(int curSignal, unsigned long now) {
  QS = false; 
  if (curSignal < thresh && (now - lastBeatTime > (IBI/5)*3)) { if (curSignal < T) T = curSignal; }
  if (curSignal > thresh && curSignal > P) P = curSignal;
  if ((curSignal > thresh) && (pulseDetected == false) && (now - lastBeatTime > (IBI/5)*3)) {
    if ((P - T) > MIN_AMPLITUDE) {
      pulseDetected = true; IBI = now - lastBeatTime; lastBeatTime = now;
      if (secondBeat) { secondBeat = false; for (int i = 0; i <= 9; i++) rate[i] = IBI; }
      if (firstBeat) { firstBeat = false; secondBeat = true; return true; }
      long runningTotal = 0; for (int i = 0; i <= 8; i++) { rate[i] = rate[i + 1]; runningTotal += rate[i]; }
      rate[9] = IBI; runningTotal += rate[9]; runningTotal /= 10;
      bpm = 60000 / runningTotal; QS = true;
    }
  }
  if (curSignal < thresh && pulseDetected == true) { pulseDetected = false; amp = P - T; thresh = amp / 2 + T; P = thresh; T = thresh; }
  if (now - lastBeatTime > 2500) { thresh = 2048; P = 2048; T = 2048; lastBeatTime = now; firstBeat = true; secondBeat = false; bpm = 0; amp = 0; }
  return (amp > MIN_AMPLITUDE);
}

// -------------------- DYNAMIC BREATHING PATTERN --------------------
void runBreathingPattern(unsigned long now, int currentSignal, int threshold, int amplitude) {
  unsigned long cycleTime = (now - breathStartTime) % cycleTotal;
  uint8_t targetR = 0, targetG = 0, targetB = 0;

  if (cycleTime < timeIn) {
    float ratio = (float)cycleTime / timeIn;
    targetG = (uint8_t)(255 * ratio); 
  } 
  else if (cycleTime < (timeIn + timeHold)) {
    targetR = 255; targetG = 255; 
  } 
  else {
    unsigned long timeInOut = cycleTime - (timeIn + timeHold);
    float ratio = 1.0 - ((float)timeInOut / timeOut);
    targetB = (uint8_t)(255 * ratio); 
  }

  float pulseFactor = 0.2; 
  int minSig = threshold - (amplitude / 2); int maxSig = threshold + (amplitude / 2);
  if (maxSig > minSig) { long mappedVal = map(currentSignal, minSig, maxSig, 50, 255); mappedVal = constrain(mappedVal, 50, 255); pulseFactor = mappedVal / 255.0; }
  uint8_t r = (uint8_t)(targetR * pulseFactor); uint8_t g = (uint8_t)(targetG * pulseFactor); uint8_t b = (uint8_t)(targetB * pulseFactor);
  for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(r, g, b));
  strip.show();
}
