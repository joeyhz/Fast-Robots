
#define LFP 4
#define LBP A5
#define RFP 6
#define RBP 7


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis()% 15000 < 9000){
    Serial.println("stop");
    analogWrite(LFP, 0);
    analogWrite(LBP, 0);
  
    analogWrite(RFP, 0);
    analogWrite(RBP, 0);
  }else{
    //left forward
    analogWrite(LFP, 255);
    analogWrite(LBP, 0);

    //right forward
    analogWrite(RFP, 255);
    analogWrite(RBP, 0);
  }
  
}
