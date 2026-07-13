#include <Servo.h>

// Assign physical PWM pins on the Arduino Uno R4
const int SERVO1_PIN = 9;   // Waist (Yaw)
const int SERVO2_PIN = 10;  // Shoulder (Pitch)
const int SERVO3_PIN = 11;  // Elbow (Pitch)

Servo servo1;
Servo servo2;
Servo servo3;

void setup() {
  Serial.begin(115200); // High-speed baud rate for real-time control loops
  
  // Attach servos to pins
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  
  // Move arm to a safe starting configuration (Home position)
  servo1.write(90); 
  servo2.write(90);
  servo3.write(90);
}

void loop() {
  // Check if target angles are waiting in the serial buffer
  // Format expected: "angle1,angle2,angle3\n" (e.g., "90,45,120\n")
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    
    int comma1 = data.indexOf(',');
    int comma2 = data.indexOf(',', comma1 + 1);
    
    if (comma1 != -1 && comma2 != -1) {
      // Parse individual angle strings
      float angle1 = data.substring(0, comma1).toFloat();
      float angle2 = data.substring(comma1 + 1, comma2).toFloat();
      float angle3 = data.substring(comma2 + 1).toFloat();
      
      // Constrain inputs to protect physical hardware limits (0-180 degrees)
      int s1_cmd = constrain(int(angle1), 0, 180);
      int s2_cmd = constrain(int(angle2), 0, 180);
      int s3_cmd = constrain(int(angle3), 0, 180);
      
      // Write PWM signals directly to the MG996R motors
      servo1.write(s1_cmd);
      servo2.write(s2_cmd);
      servo3.write(s3_cmd);
      
      // Send confirmation back to Python control pipeline
      Serial.print("ACK:");
      Serial.print(s1_cmd); Serial.print(",");
      Serial.print(s2_cmd); Serial.print(",");
      Serial.println(s3_cmd);
    }
  }
}
