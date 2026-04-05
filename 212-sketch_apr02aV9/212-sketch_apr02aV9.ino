// ------------------------------------------------------------
// Engine Tachometer + PWM Control
// Arduino Uno / Nano
//
// Pin 9  -> 25 Hz PWM output for injector
// A1     -> duty cycle trim potentiometer
// A0     -> MAP sensor input
// Pin 3  -> tach input (1 pulse per revolution)
// Pin 4  -> ARM switch
// Pin 5  -> PRIME button
// Pin 6  -> fuel pump relay output
//
// Full rewritten version for port injection system
//
// Features:
// - PRIME button gives manual fuel squirt before start
// - ARM switch enables the system
// - Fuel pump runs for 3 seconds when ARM is switched on
// - Fuel pump turns on again automatically when engine is running
// - Injector takes over automatically above 600 RPM
// - Injector shuts off below 450 RPM
// - Fast stall detection cuts fuel quickly if engine dies
// - Separate RPM display timeout can be slower than stall timeout
// - MAP sampling is adaptive for idle vs higher RPM
// - Fuel table is RPM x MAP with trim knob
// - AE code is still present but disabled for now
// - RPM limiter cuts fuel at a user-set maximum RPM
// ------------------------------------------------------------


// ------------------------------------------------------------
// Pin assignments
// ------------------------------------------------------------
const byte pwmPin      = 9;   // Injector PWM output
const byte potPin      = A1;  // Fine trim potentiometer
const byte mapPin      = A0;  // MAP sensor input
const byte freqPin     = 3;   // Tach input, 1 pulse per revolution
const byte armPin      = 4;   // ARM switch, active low with INPUT_PULLUP
const byte primePin    = 5;   // PRIME button, active low with INPUT_PULLUP
const byte fuelPumpPin = 6;   // Fuel pump relay output


// ------------------------------------------------------------
// PWM settings
// ------------------------------------------------------------
const uint16_t pwmFrequency = 25;   // Injector PWM frequency in Hz


// ------------------------------------------------------------
// Tachometer pulse timing variables
// These are updated inside the interrupt service routine.
// ------------------------------------------------------------
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulsePeriod = 0;
volatile bool newPulse = false;


// ------------------------------------------------------------
// Moving average filter for RPM measurement
// ------------------------------------------------------------
const int AVG_COUNT = 8;
unsigned long periodBuffer[AVG_COUNT];
int periodIndex = 0;
bool bufferFilled = false;


// ------------------------------------------------------------
// Main measured values
// ------------------------------------------------------------
float measuredFrequency = 0.0;
float rpm = 0.0;


// ------------------------------------------------------------
// MAP sampling variables
// Brute-force MAP capture for single-cylinder engine.
// Repeatedly sample the sensor and keep the lowest reading.
// ------------------------------------------------------------
int lowestMapReading = 350;
int currentMapReading = 0;
int MAP = 0;
int mapSampleCounter = 0;


// ------------------------------------------------------------
// General debug counter
// ------------------------------------------------------------
int debugCounter = 0;


// ------------------------------------------------------------
// Switch and state variables
// ------------------------------------------------------------
bool armed = false;             // Current ARM switch state
bool lastArmed = false;         // Previous ARM switch state, used to detect edges
bool primePressed = false;      // PRIME button state
bool fuelEnabled = false;       // Injector automatic control enabled
bool engineStalled = true;      // True when no valid tach pulse has arrived recently


// ------------------------------------------------------------
// Automatic fuel handoff hysteresis
// Fuel turns on above 600 RPM, turns off below 450 RPM.
// ------------------------------------------------------------
const int AUTO_RUN_RPM_ON  = 600; //<---------------------------------auto run above this RPM
const int AUTO_RUN_RPM_OFF = 450; //<---------------------------------Turn injector off below this RPM


// ------------------------------------------------------------
// Stall detection and RPM display timeout
//
// STALL_TIMEOUT_US is used for safety and fuel shutoff.
// RPM_ZERO_TIMEOUT_US is used to force displayed RPM to zero.
// ------------------------------------------------------------
const unsigned long STALL_TIMEOUT_US    = 25000UL;   // 25 ms, fast fuel cutoff <-------------25ms is good for clean tach signal, longer for dirty signal
const unsigned long RPM_ZERO_TIMEOUT_US = 100000UL;  // 100 ms, display / reported RPM zero


// ------------------------------------------------------------
// Fuel pump prime timing
//
// When ARM is switched on, the fuel pump runs for 3 seconds
// to build pressure. After that it shuts off unless the engine
// is actually running.
// ------------------------------------------------------------
const unsigned long FUEL_PUMP_PRIME_MS = 3000UL;
bool fuelPumpPrimeActive = false;
unsigned long fuelPumpPrimeStartMs = 0;
bool fuelPumpOn = false;


// ------------------------------------------------------------
// RPM limiter <--------------------------------------------------------------RPM LIMITER
//
// Fuel is cut when RPM reaches RPM_LIMIT_ON.
// Fuel resumes when RPM falls below RPM_LIMIT_OFF.
// This hysteresis prevents rapid chatter at one exact RPM.
// ------------------------------------------------------------
const int RPM_LIMIT_ON  = 5000;   // Set desired max RPM here
const int RPM_LIMIT_OFF = 4900;   // Fuel resumes below this RPM
bool rpmLimitActive = false;


// ------------------------------------------------------------
// Fuel table axes
// ------------------------------------------------------------
const int RPM_BIN_COUNT = 8;
const int MAP_BIN_COUNT = 8;

const int rpmBins[RPM_BIN_COUNT] = {1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000};
const int mapBins[MAP_BIN_COUNT] = {50, 80, 100, 130, 160, 220, 280, 320};


// ------------------------------------------------------------
// Fuel table
// Rows = RPM bins
// Cols = MAP bins
// ------------------------------------------------------------
const uint16_t fuelTable[RPM_BIN_COUNT][MAP_BIN_COUNT] =
{
  // MAP →      50    80    100   130   160   220   280   320
  /*1500*/   { 377,  428,  430,  1300,  1400, 1500, 1700, 1900 },
  /*2000*/   { 364,  300,  400,  600, 1000, 1100, 1150, 1200 },
  /*2500*/   { 406,  767, 1251, 1400, 1549, 1800, 2250, 2300 },
  /*3000*/   { 600,  800, 1050, 1400, 1665, 2400, 2500, 2600 },
  /*3500*/   { 700, 600, 1200, 1450, 1700, 2400, 2400, 2400 },
  /*4000*/   { 800, 400, 1750, 2000, 2100, 2200, 2300, 2400 },
  /*4500*/   {850, 900, 1200, 1500, 1500, 1500, 1500, 1500 },
  /*5000*/   {875, 900, 1200, 1500, 1500, 1500, 3200, 1500 }
};


// ------------------------------------------------------------
// Adaptive MAP sampling
// ------------------------------------------------------------
const int MAP_SAMPLES_LOW_RPM  = 1000;// was 3500
const int MAP_SAMPLES_HIGH_RPM = 150; // was 500
const int MAP_RPM_SWITCH_LOW   = 1700;
const int MAP_RPM_SWITCH_HIGH  = 1900;

int mapSampleTarget = MAP_SAMPLES_LOW_RPM;
bool fastMapMode = false;


// ------------------------------------------------------------
// Acceleration enrichment
// Present for future use, disabled now with AE_AMOUNT = 0.
// ------------------------------------------------------------
int previousMAP = 0;
int mapDelta = 0;
int aeExtra = 0;
int aeDecayCounter = 0;
int aeLockout = 0;

const int AE_RPM_MIN = 1200;
const int AE_MAP_THRESHOLD = 50;
const int AE_AMOUNT = 200;      // set to 0 to Disable
const int AE_DECAY_LOOPS = 50;
const int AE_LOCKOUT_LOOPS = 106;


// ------------------------------------------------------------
// Helper function: find nearest table index
// ------------------------------------------------------------
int findClosestIndex(int value, const int *array, int size)
{
  int closestIndex = 0;
  int smallestDiff = abs(value - array[0]);

  for (int i = 1; i < size; i++)
  {
    int diff = abs(value - array[i]);
    if (diff < smallestDiff)
    {
      smallestDiff = diff;
      closestIndex = i;
    }
  }

  return closestIndex;
}


// ------------------------------------------------------------
// Helper function: find lower bin index for interpolation
//
// Returns the lower index of the two bins surrounding the value.
// If below range, returns 0.
// If above range, returns size - 2.
// ------------------------------------------------------------
int findLowerIndex(int value, const int *array, int size)
{
  if (value <= array[0])
  {
    return 0;
  }

  for (int i = 0; i < size - 1; i++)
  {
    if (value < array[i + 1])
    {
      return i;
    }
  }

  return size - 2;
}


// ------------------------------------------------------------
// Helper function: linear interpolation
//
// Interpolates between y0 and y1 using x between x0 and x1.
// Uses float math for smooth table blending.
// ------------------------------------------------------------
float lerpFloat(float x, float x0, float x1, float y0, float y1)
{
  if (x1 == x0)
  {
    return y0;
  }

  return y0 + ((x - x0) * (y1 - y0) / (x1 - x0));
}


// ------------------------------------------------------------
// Helper function: bilinear interpolation for the fuel table
//
// This blends between four surrounding table cells:
//
//   q11 ---- q21
//    |        |
//   q12 ---- q22
//
// First interpolate across RPM at two MAP rows,
// then interpolate those two results across MAP.
// ------------------------------------------------------------
float getInterpolatedFuelDuty(int rpmValue, int mapValue)
{
  int rpmLowIndex = findLowerIndex(rpmValue, rpmBins, RPM_BIN_COUNT);
  int mapLowIndex = findLowerIndex(mapValue, mapBins, MAP_BIN_COUNT);

  int rpmHighIndex = rpmLowIndex + 1;
  int mapHighIndex = mapLowIndex + 1;

  int rpmLow = rpmBins[rpmLowIndex];
  int rpmHigh = rpmBins[rpmHighIndex];
  int mapLow = mapBins[mapLowIndex];
  int mapHigh = mapBins[mapHighIndex];

  float q11 = fuelTable[rpmLowIndex][mapLowIndex];
  float q21 = fuelTable[rpmHighIndex][mapLowIndex];
  float q12 = fuelTable[rpmLowIndex][mapHighIndex];
  float q22 = fuelTable[rpmHighIndex][mapHighIndex];

  float r1 = lerpFloat((float)rpmValue, (float)rpmLow, (float)rpmHigh, q11, q21);
  float r2 = lerpFloat((float)rpmValue, (float)rpmLow, (float)rpmHigh, q12, q22);

  return lerpFloat((float)mapValue, (float)mapLow, (float)mapHigh, r1, r2);
}


// ------------------------------------------------------------
// Interrupt Service Routine
//
// Records time between tach pulses.
// Rejects impossible pulses that are too close together.
// ------------------------------------------------------------
void pulseISR()
{
  unsigned long now = micros();
  unsigned long dt = now - lastPulseTime;

  // Reject impossible pulses / noise
  if (dt > 4000)
  {
    pulsePeriod = dt;
    lastPulseTime = now;
    newPulse = true;
  }
}


// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup()
{
  pinMode(pwmPin, OUTPUT);
  pinMode(freqPin, INPUT);
  pinMode(primePin, INPUT_PULLUP);
  pinMode(armPin, INPUT_PULLUP);
  pinMode(fuelPumpPin, OUTPUT);

  digitalWrite(fuelPumpPin, LOW);
  OCR1A = 0;

  Serial.begin(115200);

  // ----- TIMER1 PWM SETUP (Pin 9) -----
  // Fast PWM, TOP = ICR1
  // Prescaler = 64
  // Output on OC1A = pin 9
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);

  ICR1 = (16000000UL / (64UL * pwmFrequency)) - 1;

  OCR1A = 0;

  // ----- interrupt setup -----
  attachInterrupt(digitalPinToInterrupt(freqPin), pulseISR, RISING);
}


// ------------------------------------------------------------
// Main Loop
// ------------------------------------------------------------
void loop()
{
  // ----------------------------------------------------------
  // Read switch inputs
  // ARM and PRIME are active low because of INPUT_PULLUP.
  // ----------------------------------------------------------
  armed = (digitalRead(armPin) == LOW);
  primePressed = (digitalRead(primePin) == LOW);


  // ----------------------------------------------------------
  // ARM switch edge detect for fuel pump prime
  //
  // When ARM is switched from OFF to ON, run the fuel pump for
  // 3 seconds to build fuel pressure.
  // ----------------------------------------------------------
  if (armed && !lastArmed)
  {
    fuelPumpPrimeActive = true;
    fuelPumpPrimeStartMs = millis();
  }

  // If ARM is turned off, clear related states
  if (!armed)
  {
    fuelEnabled = false;
    fuelPumpPrimeActive = false;
    fuelPumpOn = false;
    rpmLimitActive = false;
  }

  lastArmed = armed;


  // ----------------------------------------------------------
  // Adaptive MAP sample window selection
  // ----------------------------------------------------------
  if (fastMapMode)
  {
    if (rpm < MAP_RPM_SWITCH_LOW)
    {
      fastMapMode = false;
    }
  }
  else
  {
    if (rpm > MAP_RPM_SWITCH_HIGH)
    {
      fastMapMode = true;
    }
  }

  if (fastMapMode)
  {
    mapSampleTarget = MAP_SAMPLES_HIGH_RPM;
  }
  else
  {
    mapSampleTarget = MAP_SAMPLES_LOW_RPM;
  }


  // ----------------------------------------------------------
  // Brute force MAP sampling
  // ----------------------------------------------------------
  currentMapReading = analogRead(mapPin);

  if (currentMapReading < lowestMapReading)
  {
    lowestMapReading = currentMapReading;
  }

  mapSampleCounter++;

  if (mapSampleCounter >= mapSampleTarget)
  {
    MAP = lowestMapReading;
    lowestMapReading = 350;
    mapSampleCounter = 0;

    mapDelta = MAP - previousMAP;
    previousMAP = MAP;

    if ((rpm > AE_RPM_MIN) && armed && (mapDelta >= AE_MAP_THRESHOLD) && (aeLockout == 0))
    {
      aeExtra = AE_AMOUNT;
      aeDecayCounter = AE_DECAY_LOOPS;
      aeLockout = AE_LOCKOUT_LOOPS;
    }
  }


  // ----------------------------------------------------------
  // AE decay
  // ----------------------------------------------------------
  if (aeLockout > 0)
  {
    aeLockout--;
  }

  if (aeDecayCounter > 0)
  {
    aeDecayCounter--;
    aeExtra = (AE_AMOUNT * aeDecayCounter) / AE_DECAY_LOOPS;
  }
  else
  {
    aeExtra = 0;
  }


  // ----------------------------------------------------------
  // Safely copy tach data from ISR
  // ----------------------------------------------------------
  unsigned long localPeriod;
  unsigned long localLastPulse;
  bool localNewPulse;

  noInterrupts();
  localPeriod = pulsePeriod;
  localLastPulse = lastPulseTime;
  localNewPulse = newPulse;
  if (newPulse) newPulse = false;
  interrupts();


  // ----------------------------------------------------------
  // RPM update from new pulse
  // ----------------------------------------------------------
  if (localNewPulse && localPeriod > 0)
  {
    periodBuffer[periodIndex] = localPeriod;
    periodIndex++;

    if (periodIndex >= AVG_COUNT)
    {
      periodIndex = 0;
      bufferFilled = true;
    }

    int count = bufferFilled ? AVG_COUNT : periodIndex;

    unsigned long sum = 0;
    for (int i = 0; i < count; i++)
    {
      sum += periodBuffer[i];
    }

    float avgPeriod = (float)sum / count;
    measuredFrequency = 1000000.0 / avgPeriod;
  }


  // ----------------------------------------------------------
  // Stall detection and RPM timeout
  //
  // engineStalled is used for fast fuel cutoff.
  // measuredFrequency is forced to zero on the slower display
  // timeout.
  // ----------------------------------------------------------
  unsigned long timeSinceLastPulse = micros() - localLastPulse;

  engineStalled = (timeSinceLastPulse > STALL_TIMEOUT_US);

  if (engineStalled)
  {
    fuelEnabled = false;
  }

  if (timeSinceLastPulse > RPM_ZERO_TIMEOUT_US)
  {
    measuredFrequency = 0.0;
  }

  rpm = measuredFrequency * 60.0;


  // ----------------------------------------------------------
  // RPM limiter with hysteresis
  //
  // Cut fuel when RPM reaches the upper limit.
  // Restore fuel when RPM drops below the lower limit.
  // ----------------------------------------------------------
  if (!rpmLimitActive && ((int)rpm >= RPM_LIMIT_ON))
  {
    rpmLimitActive = true;
  }

  if (rpmLimitActive && ((int)rpm <= RPM_LIMIT_OFF))
  {
    rpmLimitActive = false;
  }


  // ----------------------------------------------------------
  // Automatic injector fuel enable with hysteresis
  //
  // ARM must be on.
  // Fuel turns on above AUTO_RUN_RPM_ON.
  // Fuel turns off below AUTO_RUN_RPM_OFF.
  // Fuel is also forced off if the engine is stalled or the
  // RPM limiter is active.
  // ----------------------------------------------------------
  if (!armed || engineStalled || rpmLimitActive)
  {
    fuelEnabled = false;
  }
  else
  {
    if (!fuelEnabled && ((int)rpm >= AUTO_RUN_RPM_ON))
    {
      fuelEnabled = true;
    }

    if (fuelEnabled && ((int)rpm <= AUTO_RUN_RPM_OFF))
    {
      fuelEnabled = false;
    }
  }


  // ----------------------------------------------------------
  // Fuel pump control
  //
  // Pump turns on in two cases:
  // 1. During the 3-second ARM prime window
  // 2. Whenever the engine is running / fuel control is active
  //
  // Prime window expires automatically after 3 seconds.
  // ----------------------------------------------------------
  if (fuelPumpPrimeActive)
  {
    if (millis() - fuelPumpPrimeStartMs >= FUEL_PUMP_PRIME_MS)
    {
      fuelPumpPrimeActive = false;
    }
  }

  fuelPumpOn = false;

  if (armed && fuelPumpPrimeActive)
  {
    fuelPumpOn = true;
  }

  if (armed && fuelEnabled)
  {
    fuelPumpOn = true;
  }

  digitalWrite(fuelPumpPin, fuelPumpOn ? HIGH : LOW);


  // ----------------------------------------------------------
  // Fuel calculation
  // Base fuel now comes from interpolated RPM and MAP table
  // position instead of nearest table cell.
  // Potentiometer provides fine trim only.
  // ----------------------------------------------------------
  int potValue = analogRead(potPin);
  int trim = map(potValue, 0, 1023, -635, 635); // fuel trim + or - <-----------------------------FUEL TRIM

  int rpmIndex = findClosestIndex((int)rpm, rpmBins, RPM_BIN_COUNT);
  int mapIndex = findClosestIndex(MAP, mapBins, MAP_BIN_COUNT);

  int selectedRpmBin = rpmBins[rpmIndex];
  int selectedMapBin = mapBins[mapIndex];

  float interpolatedTableDutyFloat = getInterpolatedFuelDuty((int)rpm, MAP);
  int tableDuty = (int)(interpolatedTableDutyFloat + 0.5f);

  int dutyInt = tableDuty + trim + aeExtra;

  if (dutyInt < 0)
  {
    dutyInt = 0;
  }

  if (dutyInt > ICR1)
  {
    dutyInt = ICR1;
  }

  uint16_t duty = (uint16_t)dutyInt;


  // ----------------------------------------------------------
  // Output control
  //
  // PRIME overrides everything and gives a manual fuel squirt.
  // Otherwise injector follows automatic control.
  // ----------------------------------------------------------
  if (primePressed)
  {
    for (int i = 0; i < 30000; i++)
    {
      OCR1A = 5000;   // About 50% duty for manual priming
    }
    OCR1A = 0;
  }
  else
  {
    if (fuelEnabled)
    {
      OCR1A = duty;
    }
    else
    {
      OCR1A = 0;
    }
  }


  // ----------------------------------------------------------
  // Debug serial output
  // ----------------------------------------------------------
  debugCounter++;
  if (debugCounter > 2000)
  {
    debugCounter = 0;

    Serial.print("RPM: ");
    Serial.println((int)rpm);

    Serial.print("MAP: ");
    Serial.println(MAP);

    Serial.print("Duty Cycle: ");
    Serial.println(OCR1A);

    // Serial.print("ARMED: ");
    //Serial.println(armed);

    // Serial.print("PRIME: ");
    // Serial.println(primePressed);

    //Serial.print("Engine Stalled: ");
    //Serial.println(engineStalled);

    //Serial.print("Fuel Enabled: ");
    //Serial.println(fuelEnabled);

    // Serial.print("Fuel Pump On: ");
    // Serial.println(fuelPumpOn);

    // Serial.print("Fuel Pump Prime Active: ");
    // Serial.println(fuelPumpPrimeActive);

    // Serial.print("RPM Limit Active: ");
    //Serial.println(rpmLimitActive);

    // Serial.print("RPM Limit ON: ");
    // Serial.println(RPM_LIMIT_ON);

    // Serial.print("RPM Limit OFF: ");
    // Serial.println(RPM_LIMIT_OFF);

    // Serial.print("Fuel ON RPM: ");
    // Serial.println(AUTO_RUN_RPM_ON);

    // Serial.print("Fuel OFF RPM: ");
    // Serial.println(AUTO_RUN_RPM_OFF);

    Serial.print("RPM Index: ");
    Serial.println(rpmIndex);

    Serial.print("MAP Index: ");
    Serial.println(mapIndex);

    Serial.print("Selected RPM Bin: ");
    Serial.println(selectedRpmBin);

    Serial.print("Selected MAP Bin: ");
    Serial.println(selectedMapBin);

    Serial.print("MAP Sample Target: ");
    Serial.println(mapSampleTarget);

    Serial.print("Fast MAP Mode: ");
    Serial.println(fastMapMode);

    Serial.print("Trim: ");
    Serial.println(trim);

    Serial.print("Table Duty: ");
    Serial.println(tableDuty);

    Serial.print("Interpolated Duty Float: ");
    Serial.println(interpolatedTableDutyFloat);

    // Serial.print("AE Delta MAP: ");
    //Serial.println(mapDelta);

    // Serial.print("AE Extra: ");
    // Serial.println(aeExtra);

    // Serial.print("AE Decay Counter: ");
    // Serial.println(aeDecayCounter);

    // Serial.print("AE Lockout: ");
    // Serial.println(aeLockout);

    Serial.print("Final Duty: ");
    Serial.println(duty);

    Serial.println("--------------------");
  }
}
