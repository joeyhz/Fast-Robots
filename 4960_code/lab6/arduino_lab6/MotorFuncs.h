#ifndef MOTOR_FUNCS_H
#define MOTOR_FUNCS_H

#define LFP 4
#define LBP A5
#define RFP 6
#define RBP 7


void active_stop(){
    analogWrite(LFP, 0);
    analogWrite(LBP, 0);
  
    analogWrite(RFP, 0);
    analogWrite(RBP, 0);
}

void move_speed (float pcnt_speed){
    int motor_input;
    if (pcnt_speed > 0){//move forward
      motor_input = (int)(30 + pcnt_speed * 2.25);
      //left forward
      analogWrite(LFP, 0);
      analogWrite(LBP, motor_input);
  
      //right forward
      analogWrite(RFP, motor_input);
      analogWrite(RBP, 0);
    }
    else{//move backwards
      motor_input = (int)(30 + pcnt_speed * -2.25);
      //left forward
      analogWrite(LFP, motor_input);
      analogWrite(LBP, 0);
  
      //right forward
      analogWrite(RFP, 0);
      analogWrite(RBP, motor_input);
    }
}
#endif
