// FUNZIONA TUTTO YAY :D
// BPM ACCURATE
// BPM COLOR PULSE
// COLOR BREATHING 748
// AUTO TURN OFF
// AUTO TURN ON
// PICK UP DETECTION

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_NeoPixel.h>

// --- CONFIGURAZIONE PIN ---
#define PIN_NEOPIXEL    13
#define NUM_PIXELS      12
#define PIN_PULSE       25
#define PIN_SDA         23
#define PIN_SCL         22

// --- PARAMETRI DINAMICI ---
#define MOTION_THRESH   1.5     // Soglia per MANTENERE acceso (movimenti continui)
#define WAKE_THRESH     4.0     // [NUOVO] Soglia per SVEGLIARLO (movimento deciso o cambio inclinazione)
#define IDLE_TIMEOUT    5000    // Spegni led dopo 5 sec di inattività
#define MIN_AMPLITUDE   100     // Ampiezza minima battito

// --- OGGETTI ---
Adafruit_NeoPixel strip(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// --- VARIABILI GLOBALI ---

// Accelerometro
float lastX, lastY, lastZ;
unsigned long lastActivityTime = 0;
bool isActive = false; // Parte spento

// --- VARIABILI ALGORITMO PULSESENSOR ---
int bpm = 0;
int pulseSignal;           
int IBI = 600;             
bool pulseDetected = false;
bool QS = false;           
int rate[10];              
unsigned long sampleCounter = 0; 
unsigned long lastBeatTime = 0;  
int P = 2048;              // Picco
int T = 2048;              // Valle
int thresh = 2048;         // Soglia
int amp = 0;               // Ampiezza
bool firstBeat = true;     
bool secondBeat = false;   

// Respirazione
unsigned long breathStartTime = 0;
const int TIME_IN = 7000;
const int TIME_HOLD = 4000;
const int TIME_OUT = 8000;
const int CYCLE_TOTAL = 19000;

void setup() {
  Serial.begin(115200);

  // Setup NeoPixel
  strip.begin();
  strip.clear(); 
  strip.show();
  strip.setBrightness(255); 

  // Setup I2C & Accelerometro
  Wire.begin(PIN_SDA, PIN_SCL);
  if (accel.begin()) {
    accel.setRange(ADXL345_RANGE_16_G);
    sensors_event_t event;
    accel.getEvent(&event);
    lastX = event.acceleration.x;
    lastY = event.acceleration.y;
    lastZ = event.acceleration.z;
  } else {
    Serial.println("Errore Accelerometro");
  }

  breathStartTime = millis();
}

void loop() {
  unsigned long now = millis();
  pulseSignal = analogRead(PIN_PULSE); 
  sampleCounter = now;

  // 1. GESTIONE BATTITO 
  bool fingerDetected = runPulseSensorAlgorithm(pulseSignal, now);

  // 2. GESTIONE MOVIMENTO (Modificata per WAKE UP intelligente)
  bool motionDetected = checkMotion();

  // 3. LOGICA ATTIVITÀ (Wake Up & Sleep)
  if (fingerDetected || motionDetected) {
    if (!isActive) {
        breathStartTime = now; 
        Serial.println("WAKE UP! (Movimento o Dito rilevato)");
        
        // Reset rapido per feedback immediato
        if (fingerDetected) {
           // Se svegliato dal dito, resetta algoritmo BPM
           thresh = 2048; P = 2048; T = 2048;
        }
    }
    lastActivityTime = now;
    isActive = true;
  } else {
    if (now - lastActivityTime > IDLE_TIMEOUT) {
      if (isActive) {
          Serial.println("SLEEP... (Posizione memorizzata)"); 
          isActive = false;
          // NOTA: Quando isActive diventa false, lastX/Y/Z smettono di aggiornarsi
          // in checkMotion(), congelando l'ultima posizione nota (quella di riposo).
      }
    }
  }

  // 4. ANIMAZIONE
  if (isActive) {
    runBreathingPattern(now, pulseSignal, thresh, amp);
  } else {
    strip.clear();
    strip.show();
  }
  
  delay(10); 
}

// --- ALGORITMO STANDARD PULSESENSOR ---
bool runPulseSensorAlgorithm(int curSignal, unsigned long now) {
  
  if (curSignal < thresh && (now - lastBeatTime > (IBI/5)*3)) {
    if (curSignal < T) T = curSignal;
  }

  if (curSignal > thresh && curSignal > P) P = curSignal;

  if ((curSignal > thresh) && (pulseDetected == false) && (now - lastBeatTime > (IBI/5)*3)) {
    if ((P - T) > MIN_AMPLITUDE) {
      pulseDetected = true;
      IBI = now - lastBeatTime;
      lastBeatTime = now;

      if (secondBeat) {
        secondBeat = false;
        for (int i = 0; i <= 9; i++) rate[i] = IBI;
      }

      if (firstBeat) {
        firstBeat = false;
        secondBeat = true;
        return true; 
      }

      long runningTotal = 0;
      for (int i = 0; i <= 8; i++) {
        rate[i] = rate[i + 1];
        runningTotal += rate[i];
      }

      rate[9] = IBI;
      runningTotal += rate[9];
      runningTotal /= 10;
      bpm = 60000 / runningTotal;
      
      QS = true;
    }
  }

  if (curSignal < thresh && pulseDetected == true) {
    pulseDetected = false;
    amp = P - T;
    thresh = amp / 2 + T; 
    P = thresh;            
    T = thresh;            
  }

  if (now - lastBeatTime > 2500) {
    thresh = 2048; 
    P = 2048;
    T = 2048;
    lastBeatTime = now;
    firstBeat = true;
    secondBeat = false;
    bpm = 0;
    amp = 0; 
  }

  // --- SERIAL OUTPUT PER PLOTTER ---
  Serial.print("Raw:"); Serial.print(curSignal);
  Serial.print(",Thresh:"); Serial.print(thresh);
  Serial.print(",BPM:"); Serial.print(bpm);
  
  // Aggiungiamo i dati accelerometro (Moltiplicati per 100 per visibilità su grafico)
  Serial.print(",AX:"); Serial.print(lastX * 100); 
  Serial.print(",AY:"); Serial.print(lastY * 100); 
  Serial.print(",AZ:"); Serial.println(lastZ * 100); 

  return (amp > MIN_AMPLITUDE);
}

// --- CHECK MOVIMENTO INTELLIGENTE ---
bool checkMotion() {
  sensors_event_t event;
  accel.getEvent(&event);
  
  float deltaX = abs(event.acceleration.x - lastX);
  float deltaY = abs(event.acceleration.y - lastY);
  float deltaZ = abs(event.acceleration.z - lastZ);
  float movement = deltaX + deltaY + deltaZ;
  
  if (!isActive) {
      // --- MODALITÀ SLEEP ---
      // NON aggiorniamo lastX/Y/Z. Li usiamo come riferimento "fisso" di riposo.
      // Se l'accelerazione attuale si discosta molto da quella di riposo (WAKE_THRESH), ci svegliamo.
      // Questo rileva benissimo se il dispositivo viene capovolto (G invertita) o sollevato.
      
      if (movement > WAKE_THRESH) {
          // Movimento rilevato! Aggiorniamo la posizione corrente per iniziare il tracking attivo
          lastX = event.acceleration.x;
          lastY = event.acceleration.y;
          lastZ = event.acceleration.z;
          return true; // SVEGLIA!
      }
      return false; // Continua a dormire
      
  } else {
      // --- MODALITÀ ATTIVA ---
      // Aggiorniamo costantemente il riferimento per tracciare i movimenti relativi (flusso continuo)
      // Qui la soglia è più bassa (MOTION_THRESH) per mantenerlo acceso anche se lo tieni in mano tremando poco.
      
      lastX = event.acceleration.x;
      lastY = event.acceleration.y;
      lastZ = event.acceleration.z;
      return (movement > MOTION_THRESH);
  }
}

// --- ANIMAZIONE RESPIRAZIONE + PULSAZIONE ---
void runBreathingPattern(unsigned long now, int currentSignal, int threshold, int amplitude) {
  
  unsigned long cycleTime = (now - breathStartTime) % CYCLE_TOTAL;
  uint8_t targetR = 0, targetG = 0, targetB = 0;

  if (cycleTime < TIME_IN) {
    float ratio = (float)cycleTime / TIME_IN;
    targetG = (uint8_t)(255 * ratio);
  } else if (cycleTime < (TIME_IN + TIME_HOLD)) {
    targetR = 255; targetG = 255;
  } else {
    unsigned long timeInOut = cycleTime - (TIME_IN + TIME_HOLD);
    float ratio = 1.0 - ((float)timeInOut / TIME_OUT);
    targetB = (uint8_t)(255 * ratio);
  }

  float pulseFactor = 0.2; 
  
  int minSig = threshold - (amplitude / 2);
  int maxSig = threshold + (amplitude / 2);

  if (maxSig > minSig) {
     long mappedVal = map(currentSignal, minSig, maxSig, 50, 255); 
     mappedVal = constrain(mappedVal, 50, 255);
     pulseFactor = mappedVal / 255.0;
  }

  uint8_t r = (uint8_t)(targetR * pulseFactor);
  uint8_t g = (uint8_t)(targetG * pulseFactor);
  uint8_t b = (uint8_t)(targetB * pulseFactor);

  for(int i=0; i<NUM_PIXELS; i++) strip.setPixelColor(i, strip.Color(r, g, b));
  strip.show();
}