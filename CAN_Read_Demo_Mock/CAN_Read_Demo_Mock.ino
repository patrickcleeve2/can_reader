
String valid_can_ids[4] = {"0x690", "0x545", "0x80", "5e4"};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  Serial.println("Sending Mock CAN Frame Data");


}

void loop() {

  // Create a fake CAN Frame for testing.
  Serial.print("can,");

  // Create ID in hex (TODO: randomise ID too)

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
  
  delay(200);
}
