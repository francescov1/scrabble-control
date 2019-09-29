#define PORT_SPEED 115200


void setup() {
  Serial.begin(PORT_SPEED);
}

void loop() {
  while (Serial.available() > 0) {
   byte data[1];
   Serial.readBytes(data, 1);
   Serial.write(data, 1);
  }
}
