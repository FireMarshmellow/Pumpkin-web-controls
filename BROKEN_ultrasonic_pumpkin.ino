#include <FastLED.h>
#include <HardwareSerial.h>

// Define the LED strip parameters
#define NUM_LEDS 20  // Adjust to the number of LEDs in your strip
#define DATA_PIN 7    // GPIO 7 for LED strip data

CRGB leds[NUM_LEDS];

// Define the HC-SR04 pins
const int trigPin = 5;
const int echoPin = 4;

// Define the distance thresholds (in cm)
float distanceMin = 10.0;   // Minimum distance for red (close)
float distanceMax = 200.0; // Maximum distance for sensor detection

const int pin = 6;

// Define RX and TX pins for the ESP32
#define RXD2 16
#define TXD2 17

// Initialize UART on Serial2 (use appropriate pins)
HardwareSerial mySerial(1);

// Variables for averaging distance
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 1000; // Update every 1 second
float distanceSum = 0;
int distanceCount = 0;
float averagedDistance = 0;

// Variable to keep track of the last played track
int lastTrackPlayed = 0;

// Variables for GPIO activation timing
unsigned long gpioActivationTime = 0;
const unsigned long gpioDuration = 10000; // GPIO HIGH for 10 seconds

void setup() {
  // Pull pin 6 LOW at start
  digitalWrite(pin, LOW);
  // Initialize serial communication
  Serial.begin(115200);
  
  // Initialize the LED strip
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();

  // Set the trigPin and echoPin as output and input
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Set pin 6 as output
  pinMode(pin, OUTPUT);

  // Begin communication with the MP3 module on Serial2
  mySerial.begin(9600, SERIAL_8N1, RXD2, TXD2);  // Speaker communication

  // Print a message
  Serial.println("HC-SR04, LED control, and MP3 module initialized...");
}

void loop() {
  // Get the distance from the ultrasonic sensor
  long distance = getDistance();

  // Accumulate distance readings
  distanceSum += distance;
  distanceCount++;

  unsigned long currentTime = millis();

  // Check if one second has passed
  if (currentTime - lastUpdateTime >= updateInterval) {
    // Calculate the average distance
    averagedDistance = distanceSum / distanceCount;

    Serial.print("Averaged Distance over ");
    Serial.print(updateInterval / 1000);
    Serial.print(" second(s): ");
    Serial.print(averagedDistance);
    Serial.println(" cm");

    // Reset the accumulators
    distanceSum = 0;
    distanceCount = 0;
    lastUpdateTime = currentTime;

    // Now use the averaged distance for the rest of the logic
    processDistance(averagedDistance);
  }

  // Check if GPIO should be turned off after duration
  if (digitalRead(pin) == HIGH && millis() - gpioActivationTime >= gpioDuration) {
    digitalWrite(pin, LOW);
    Serial.println("GPIO 6 set LOW after duration");
  }

  // Small delay to prevent overloading the sensor
  delay(50);
}

// Function to get the distance from HC-SR04
long getDistance() {
  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by setting it HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pin, returns the time in microseconds
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms

  // If no echo was received, return maximum distance
  if (duration == 0) {
    duration = 30000; // Maximum duration corresponds to ~500 cm
  }

  // Calculate the distance in cm (speed of sound is 343 m/s)
  long distance = duration * 0.034 / 2;  // Convert to cm

  return distance;
}

// Function to process the averaged distance
void processDistance(float distance) {
  // Calculate the step interval for audio playback
  float stepInterval = (distanceMax - distanceMin) / 5.0;
  Serial.print("Step interval: ");
  Serial.println(stepInterval);

  // If distance is above the max detection range, turn off LEDs
  if (distance > distanceMax) {
    Serial.println("Distance above max range, turning off LEDs.");
    FastLED.clear();
    FastLED.show();
    digitalWrite(pin, LOW); // Ensure GPIO is LOW
    lastTrackPlayed = 0;    // Reset last track
    return;
  }

  // Ensure distance stays within the valid range
  distance = constrain(distance, distanceMin, distanceMax);
  Serial.print("Constrained distance: ");
  Serial.println(distance);

  // Calculate color based on the distance (green to red transition)
  float distanceRange = distanceMax - distanceMin;
  float position = (distanceMax - distance) / distanceRange;  // 0.0 = far (green), 1.0 = close (red)
  Serial.print("Position: ");
  Serial.println(position);

  // Generate the color gradient from green (far) to red (close)
  CRGB color = blend(CRGB::Green, CRGB::Red, position * 255);

  // Set all LEDs to the calculated color
  fill_solid(leds, NUM_LEDS, color);

  // Show the LEDs
  FastLED.show();

  // Determine which track to play based on distance intervals
  int currentTrack = 0;
  for (int i = 1; i <= 5; i++) {
    float lowerBound = distanceMax - stepInterval * i;
    float upperBound = distanceMax - stepInterval * (i - 1);

    Serial.print("Checking interval for track ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(lowerBound);
    Serial.print(" cm to ");
    Serial.print(upperBound);
    Serial.println(" cm");

    if (distance > lowerBound && distance <= upperBound) {
      currentTrack = i;
      break;
    }
  }

  // Play the track only if it has changed
  if (currentTrack != lastTrackPlayed && currentTrack != 0) {
    Serial.print("Playing track ");
    Serial.println(currentTrack);
    play_track(currentTrack);
    lastTrackPlayed = currentTrack;

    if (currentTrack == 5) {
      Serial.println("Activating GPIO sequence for track 5");

      // Control GPIO 6 sequence
      digitalWrite(pin, HIGH); // Pull pin HIGH
      Serial.println("GPIO 6 set HIGH");
      gpioActivationTime = millis(); // Start timing
    } else {
      // Ensure GPIO is LOW for other tracks
      digitalWrite(pin, LOW);
    }
  } else if (currentTrack == 0) {
    // No track to play, ensure GPIO is LOW
    digitalWrite(pin, LOW);
    lastTrackPlayed = 0; // Reset last track
  } else {
    Serial.println("Track unchanged, not replaying.");
  }
}

// Helper function to send a command to the MP3 module
void send_command(uint8_t command[], size_t length) {
  Serial.print("Sending command: ");
  for (size_t i = 0; i < length; i++) {
    Serial.print(command[i], HEX);
    Serial.print(" ");
    mySerial.write(command[i]);
  }
  Serial.println();
}

// Playback Control Functions
void play_track(uint8_t track_number) {
  Serial.print("Playing track: ");
  Serial.println(track_number);
  uint8_t command[] = {0x7E, 0x03, 0x00, 0x02, 0x00, track_number, 0xEF};
  send_command(command, sizeof(command));
}
