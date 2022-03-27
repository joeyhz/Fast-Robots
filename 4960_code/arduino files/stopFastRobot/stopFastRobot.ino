void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  pinMode(A14, OUTPUT);
  pinMode(A15, OUTPUT);
  pinMode(A16, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
    Serial.println("stop");
    analogWrite(A14, 0);
    analogWrite(13, 0);
    
    analogWrite(A16, 0);
    analogWrite(A15, 0);
}
