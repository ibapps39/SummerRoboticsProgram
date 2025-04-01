/*
 * Basic Arduino Nano Introduction
 * Author: Ian Brown
 * Date: April 2025
 * 
 * A simple demonstration of:
 * - Controlling an LED
 * - Reading a button press
 * - Playing tones on a buzzer
 * 
 * Hardware needed:
 * - Arduino Nano
 * - 1 LED
 * - 1 Pushbutton
 * - 1 Passive buzzer
 * - 2 220 ohm resistors (for LED and buzzer)
 * - 1 10k ohm resistor (for button pull-down)
 * - Breadboard and jumper wires
 * 
 * Circuit connections:
 * - LED: positive leg to pin 9 through 220 ohm resistor, negative to GND
 * - Button: one side to 5V, other side to pin 2 and to GND through 10k resistor
 * - Buzzer: positive to pin 8 through 220 ohm resistor, negative to GND
 */

// Pin definitions
const int LED_PIN = 9;      // LED connected to pin 9
const int BUTTON_PIN = 2;   // Button connected to pin 2
const int BUZZER_PIN = 8;   // Buzzer connected to pin 8

// Variables
int buttonState = 0;        // Variable to store button state
int ledState = LOW;         // Current state of the LED

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  Serial.println("Arduino Nano Basic Example - Starting up...");
  
  // Configure pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initial LED state
  digitalWrite(LED_PIN, ledState);
}

void loop() {
  // Read the button state
  buttonState = digitalRead(BUTTON_PIN);
  
  // If button is pressed
  if (buttonState == HIGH) {
    Serial.println("Button pressed!");
    
    // Toggle LED state
    ledState = (ledState == LOW) ? HIGH : LOW;
    digitalWrite(LED_PIN, ledState);
    
    // Play a tone based on the LED state
    if (ledState == HIGH) {
      playTone(1000, 200); // High pitch tone when turning on
    } else {
      playTone(500, 200);  // Low pitch tone when turning off
    }
    
    // Small delay to avoid button bounce
    delay(300);
  }
}

// Function to play a tone on the buzzer
void playTone(int frequency, int duration) {
  tone(BUZZER_PIN, frequency, duration);
}
