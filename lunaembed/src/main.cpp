/*
#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}
//////////////////////////////////////////////////

#include <Arduino.h>

String command;

void setup() {
  Serial.begin(9600);
  pinMode(9, OUTPUT); // Example motor pin
  Serial.println("Arduino Ready");
}

void loop() {
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');

    if (command == "FORWARD") {
      digitalWrite(9, HIGH);
      Serial.println("Motor Forward");
    } else if (command == "STOP") {
      digitalWrite(9, LOW);
      Serial.println("Motor Stop");
    }
  }
}
*/
// #include <Arduino.h>

// void setup() {
//   Serial.begin(9600);
//   pinMode(5, OUTPUT); // left motor
//   pinMode(6, OUTPUT); // right motor
// }

// void loop() {
//   if (Serial.available()) {
//     char cmd = Serial.read();
//     if (cmd == 'F') {
//       digitalWrite(5, HIGH);
//       digitalWrite(6, HIGH);
//     } else if (cmd == 'S') {
//       digitalWrite(5, LOW);
//       digitalWrite(6, LOW);
//     }
//   }
// }