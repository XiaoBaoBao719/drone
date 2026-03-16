void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(0));
}

void loop() {
  // Generate three random numbers
  int num1 = random(0, 100);
  int num2 = random(0, 100);
  int num3 = random(0, 100);
  
  // Send as: num1,num2,num3
  Serial.print(num1);
  Serial.print(",");
  Serial.print(num2);
  Serial.print(",");
  Serial.println(num3);
  
  delay(1000); // Send every second
}