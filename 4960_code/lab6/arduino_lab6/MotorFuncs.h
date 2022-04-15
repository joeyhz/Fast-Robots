#ifndef MOTOR_FUNCS_H
#define MOTOR_FUNCS_H

#define LFP A5
#define LBP 4
#define RFP 7
#define RBP 6


void active_stop(){
      //left forward
      analogWrite(LFP, 10);
      analogWrite(LBP, 0);
  
      //right forward
      analogWrite(RFP, 0);
      analogWrite(RBP, 10);
}

void passive_stop(){
      analogWrite(LFP, 0);
      analogWrite(LBP, 0);
  
      //right forward
      analogWrite(RFP, 0);
      analogWrite(RBP, 0);
}

void move_speed (float pcnt_speed){
    int motor_input;
    if (pcnt_speed > 0){//move forward
      motor_input = (int)(20 + pcnt_speed * 0.85 );
      //left forward
      analogWrite(LFP, 0);
      analogWrite(LBP, motor_input);
  
      //right forward
      analogWrite(RFP, motor_input);
      analogWrite(RBP, 0);
    }
    else if (pcnt_speed < 0){
      //move backwards
      motor_input = (int)(20 + pcnt_speed * -0.5);
      //left forward
      analogWrite(LFP, motor_input);
      analogWrite(LBP, 0);
  
      //right forward
      analogWrite(RFP, 0);
      analogWrite(RBP, motor_input);
    }
//    else{
//      //active break 
//      digitalWrite(LFP, 0);
//      digitalWrite(LBP, 0);
//  
//      digitalWrite(RFP, 0);
//      digitalWrite(RBP, 0);
//    }
}
#endif
