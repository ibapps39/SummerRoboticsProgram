// Included Library, "Servo"
// Normally, we wouldn't use a <>, as servo isn't a standard library.
// I'd like access to the Servo library
#include <Servo.h>

// Which PIN is our PWM connected to on the nano? GREEN WIRE
#define SERVO_PIN_NANO 3

// Instance of Servo class, "Hey, bring the servo code in!"
Servo myservo;

// The angle of servo arm, we start at zero here.
int servoAngle = 0;
// Remember! 1000 miliseconds = 1 second
const unsigned long interval = 1000;

// Useful for tracking our passed time. We need to use long since it will get
// big fast!
unsigned long previous_milisecond = 0;

void setup() {
  // Tell the compiler, computer where our servo is, which pin.
  myservo.attach(SERVO_PIN_NANO);
}

void loop() {
  // Implement without delays, 

  // Track the current milisecond
  unsigned long current_milisecond = millis();
  // Repeat this every second
  if (current_milisecond - previous_milisecond >= interval)
  {
    // if 1 second has passed, reset! It's like a delay without
    // shutting down/interrupting
    previous_milisecond = current_milisecond;
    
    // Increase angle until 180, then decrement back down to 0
    if (servoAngle <= 180)
    {
      myservo.write(servoAngle++);
    } else {
      myservo.write(servoAngle--);
    }
    
  }
  // AlTERNATIVE CODE: Using Delays
  // myservo.write(0);
  // delay(1000);
  // myservo.write(90);
  // delay(1000);
  // myservo.write(servoAngle);
  // delay(1000);
  // myservo.write(servoAngle+10);
  // delay(100);
  // while (servoAngle <= 180) {
  //   servoAngle++;
  // }
  // delay(1000);
  // myservo.write(0);
}