// MODES OF OPERATION 
// COUNT - Timer increments each second.  Started by "START" button.
// PAUSED - Timer stops counting, shows current time.
//          "START" button toggles between PAUSED and COUNT
//          TIME blinks in sync with colon when PAUSED
// STOPPED - Timer stops counting.  Mode entered by "STOP" button.
//          Pressing "STOP" again resets the display
//          COLON stopps blinking when timer is STOPPED (but visible)
// On Power On Reset:
//          Diplays shows 00:00
// When timer starts
//          Display clears leading Zeros, flashes colon at 1HZ i.e. "  : 1" for 1 second
#define SEGA   3
#define SEGB   4
#define SEGC   5
#define SEGD   6
#define SEGE   7
#define SEGF   8
#define SEGG   9
#define COLON  A0
// Upper colon tied to Digit3
// Lower colon tied to Digit 4
#define DIGIT1 10         // display tensMinutes
#define DIGIT2 11         // display unitMinutes
#define DIGIT3 12         // display tensSeconds
#define DIGIT4 13         // display unitMinutes  // also Built In LED
#define buttonStart A3    // Start Button input 
#define buttonStop  A2    // Stop Button input
#define debounceCount 250

unsigned long previousMillis = 0;
const long interval = 500;        // Flash COLON at 1 Hz
//int unitSeconds = 0;
//int tensSeconds = 0;
//int unitMinutes = 0;
//int tensMinutes = 0;
int unitSeconds = -1;
int tensSeconds = -1;
int unitMinutes = -1;
int tensMinutes = -1;
int halfSecond = 0;
int currentDigit = 0;
uint8_t startLock = 0;
uint8_t stopLock = 0;
// Setup input button structure for debouncing
typedef struct InputData {
  uint8_t state;
  uint8_t count;
};

InputData startButton, stopButton;

// Modes of operation
enum mode {
  stopped,
  pause,
  count,
  reset
} currentMode;

void setup() {
  Serial.begin(115200);
  Serial.println("Barun Awesome Timer is Awesome.");
  currentMode = stopped;
  // Initialise LED driver pins as outputs
  pinMode(SEGA, OUTPUT);
  pinMode(SEGB, OUTPUT);
  pinMode(SEGC, OUTPUT);
  pinMode(SEGD, OUTPUT);
  pinMode(SEGE, OUTPUT);
  pinMode(SEGF, OUTPUT);
  pinMode(SEGG, OUTPUT);
  pinMode(COLON, OUTPUT);
  pinMode(DIGIT1, OUTPUT);
  pinMode(DIGIT2, OUTPUT);
  pinMode(DIGIT3, OUTPUT);
  pinMode(DIGIT4, OUTPUT);
  pinMode(buttonStart, INPUT);
  pinMode(buttonStop, INPUT);
  // Turn all LEDs off (Digit Cathodes have to be set high)
  digitalWrite(SEGA, LOW);
  digitalWrite(SEGB, LOW);
  digitalWrite(SEGC, LOW);
  digitalWrite(SEGD, LOW);
  digitalWrite(SEGE, LOW);
  digitalWrite(SEGF, LOW);
  digitalWrite(SEGG, LOW);
  digitalWrite(COLON, HIGH);    // Show Colon at Power On Reset
  digitalWrite(DIGIT1, HIGH);
  digitalWrite(DIGIT2, HIGH);
  digitalWrite(DIGIT3, HIGH);
  digitalWrite(DIGIT4, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
  // debouce inputs here
  uint8_t current = digitalRead(buttonStart);
  // If start button is active, debounce it
  if ((debouce(current, startButton) == 0) && (startLock == 1)) {
    startLock = 0;
  }
  if ((debouce(current, startButton) == 1) && (startLock == 0)) {
    startLock = 1;                // stop cycling modes if button held down
    if ((currentMode == stopped) || (currentMode == pause) || (currentMode == reset)) {
      currentMode = count;
    } else {
      currentMode = pause;
    }
  }

  current = digitalRead(buttonStop);
  if ((debouce(current, stopButton) == 0) && (stopLock == 1)) {
    stopLock = 0;
  }
  if ((debouce(current, stopButton) == 1) && (stopLock == 0)) {
    stopLock = 1;
    if ((currentMode == stopped) || (currentMode == pause)) {
      currentMode = reset;
      unitSeconds = -1;
      tensSeconds = -1;
      unitMinutes = -1;
      tensMinutes = -1;
      digitalWrite(COLON, HIGH);
    } else {
      currentMode = stopped;
    }
  }

  if (currentMillis - previousMillis >= interval) {   // Every Half Second
    previousMillis = currentMillis;
    Serial.print("Current Mode is: "); Serial.print(currentMode); Serial.print(startLock); Serial.println(stopLock);
    if (currentMode == count) { // count = 2
      if (halfSecond == 0) {
        updateTime();
        digitalWrite(COLON, HIGH);
        halfSecond = 1;
      } else {
        digitalWrite(COLON, LOW);
        halfSecond = 0;
      }
    }
  }
  displayTime();
}

