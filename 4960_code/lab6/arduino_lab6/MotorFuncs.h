#ifndef MOTOR_FUNCS_H
#define MOTOR_FUNCS_H

void active_stop(){
    analogWrite(A14, 0);
    analogWrite(13, 0);
  
    analogWrite(A16, 0);
    analogWrite(A15, 0);
}

void move_speed (float pcnt_speed){
    int motor_input;
    if (pcnt_speed > 0){//move forward
      motor_input = (int)(30 + pcnt_speed * 2.25);
      //left forward
      analogWrite(A14, 0);
      analogWrite(13, motor_input);
  
      //right forward
      analogWrite(A16, motor_input);
      analogWrite(A15, 0);
    }
    else{//move backwards
      motor_input = (int)(30 + pcnt_speed * -2.25);
      //left forward
      analogWrite(A14, motor_input);
      analogWrite(13, 0);
  
      //right forward
      analogWrite(A16, 0);
      analogWrite(A15, motor_input);
    }
}
#endif
