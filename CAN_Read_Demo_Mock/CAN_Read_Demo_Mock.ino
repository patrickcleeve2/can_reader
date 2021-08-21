void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  Serial.println("Sending Mock CAN Frame Data");

}

void loop() {

  // Create a fake CAN Frame for testing.
 
  // Create ID in hex (TODO: randomise ID too)
  Serial.print("can,0x690,");

  // Generate random data bytes in hex
  for (int i = 0; i <= 7; i++) {
    int rand_num = random(0, 255);
    Serial.print(rand_num, HEX);
    Serial.print(",");
  }
  Serial.println("");
  
  delay(200);
}
