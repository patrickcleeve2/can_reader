
String valid_can_ids[10] = {"0x420", "0x386", "0x2B0", "0x541", "0x251", "0x394", "0x371", "0x340", "0x4F1", "0x220"};
//String valid_can_ids[4] = {"0x690", "0x545", "0x80", "5e4"};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  Serial.println("Sending Mock CAN Frame Data");


}

void loop() {

  // Create a fake CAN Frame for testing.
  Serial.print("can,");

  // Select a random ID in hex
  int idx = sizeof(valid_can_ids) / sizeof(String);
  String frame_id = valid_can_ids[random(idx)];
  
  Serial.print(frame_id);
  Serial.print(",");

  // Generate random data bytes in hex
  for (int i = 0; i <= 7; i++) {
    int rand_num = random(0, 255);
    Serial.print(rand_num, HEX);
    Serial.print(",");
  }
  Serial.println("");
  
  delay(50);
}
