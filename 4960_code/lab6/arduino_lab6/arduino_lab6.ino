
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include "SensorFuncs.h"
#include "MotorFuncs.h"

#include <BasicLinearAlgebra.h>  //Use this library to work with matrices:
using namespace BLA;             //This allows you to declare a matrix


#include <ArduinoBLE.h>

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "9A48ECBA-2E92-082F-C079-9E75AAE428B1"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;


bool do_PID = false;
bool do_KF = false;
bool moving_forward = false;
float forward_spd = 0;
int measure_count = 0;
unsigned long startMillis = 0;

#define ARR_SIZE 100
int front_tof_readings[ARR_SIZE];
long times[ARR_SIZE];
float motor_pcnts[ARR_SIZE];

//for KF:
Matrix<2,1> kf_vals[ARR_SIZE];
Matrix<2,2> sigmas[ARR_SIZE];


int TARGET_DIST = 300;

float KP = 0.05;
#define KI 0.0
#define KD 0.5

int integrator = 0;
//////////// Global Variables ////////////

enum CommandTypes
{
    WALL_PID,
    WALL_KF,
    STOP,
    MOVE_FORWARD,
    SET_KP
};

void reset_data(){
  do_PID = false;
  moving_forward= false;
  forward_spd = 0;
  measure_count = 0;
  startMillis = 0;
}

void handle_command(){   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        case WALL_PID:
          do_PID = true;
          break;
        case WALL_KF:
          do_KF = true;
          break;
        case STOP:
          active_stop();
          break;
        case MOVE_FORWARD:
          success = robot_cmd.get_next_value(forward_spd);
          if (!success)
            return;
          moving_forward = true;
          break;
        case SET_KP:
          success = robot_cmd.get_next_value(KP);
          if (!success)
            return;
          break;
        default:
          Serial.print("Invalid Command Type: ");
          Serial.println(cmd_type);
          break;
    }
}

void setup(){
    Serial.begin(115200);

    setup_sensors();
    integrator = 0;

    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();
}

void write_pid_data(){
  Serial.println("Write data");
  for (int i=0; i<ARR_SIZE; i++)tx_characteristic_float.writeValue(front_tof_readings[i]*1.0);
  for (int i=0; i<ARR_SIZE; i++)tx_characteristic_float.writeValue(times[i]*1.0);
  for (int i=0; i<ARR_SIZE; i++)tx_characteristic_float.writeValue(motor_pcnts[i]); 
}

void write_kf_data(){
    for (int i=0; i<ARR_SIZE; i++)tx_characteristic_float.writeValue(kf_vals[i](1,0));
//  for (int i=0; i<ARR_SIZE; i++)tx_characteristic_float.writeValue(sigmas[i]());
}

void store_pid_data(int index, long curr_mill, int new_tof, float pcnt){  
  front_tof_readings[index] = new_tof;
  times[index] = curr_mill;
  motor_pcnts[index] = pcnt;
}

void store_kf_data(int index, Matrix<2,1> kf_x, Matrix<2,2> sigma){
  kf_vals[index] = kf_x;
  sigmas[index] = sigma;
}

void read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

float pid(int index, long time_stamp, int tof_dist){
  int error = tof_dist - TARGET_DIST;  
   
  float deriv;
  if (index == 0){
    deriv = 0;
    integrator = 0;
  }
  else{
    int dt = time_stamp - times[index-1];
    deriv = (tof_dist - front_tof_readings[index-1]) / dt;
    integrator = integrator + dt*error;
  }
  float raw = KP * error + KI * integrator + KD * deriv;
  
  float ret;
  if (raw > 100) ret= 100;
  if (raw < -100) ret= -100;
  store_pid_data(index, time_stamp, tof_dist, ret);
  return ret;
}


///////KALMAN FILTER////
Matrix<2,2> A = {0, 1,
                 0, -2.9};  
Matrix<2,1> B = {0, 5.235};
Matrix<1,2> C = {-1, 0};

Matrix<2,2> sigma_u = {100, 0,
                      0, 100};
Matrix<1> sigma_z = {400};

Matrix<2,2> I = {1, 0,
                0, 1};
                
#define STEP_SIZE 43.5

float kf(int index, long time_stamp, int tof_dist){
  if (index = 0){
    //store initial values
     Matrix<2,1> mu = {-tof_dist, 0};
     Matrix<2,2> sigma = {25,0,
                          0, 25};
     return mu(1,1);
  }

  // Get KF inputs for this iteration
  Matrix<2,1> mu = kf_vals[index-1];
  Matrix<2,2> sigma = sigmas[index-1];
  Matrix<1, 1> u = {motor_pcnts[index-1] / STEP_SIZE};
  Matrix<1> y = {tof_dist}; 
  int dt = time_stamp - times[index-1];
  Matrix<2,2> Ad = I + A * dt;
  Matrix<2,1> Bd = B * dt;

  //predict step
  Matrix<2,1> mu_p = Ad * mu + Bd * u;
  Matrix<2,2> sigma_p = Ad*sigma*~Ad + sigma_u; 

  //update step
  Matrix<1> y_m = y - C * mu_p;
  Matrix<1,1> sigma_m = C * sigma_p * ~C + sigma_z;

  Matrix<1,1> sigma_m_inv = sigma_m;
  Invert(sigma_m_inv);
  Matrix<2,1> kf_gain = sigma_p * ~C * sigma_m_inv; 

  //combine
  mu = mu_p + kf_gain * y_m;
  sigma = (I - kf_gain * C) * sigma_p;
  
  store_kf_data(index, mu, sigma);
}




void loop(){
    long curr_mil;
    int new_tof;
    float pcnt;
    float kf_state;
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());
        reset_data();

        // While central is connected
        while (central.connected()) {
            // Read data
            read_data();

            if (do_PID){
              if(sensor_data_ready() && measure_count < ARR_SIZE){
                //Doing PID and just got new sensor reading
                curr_mil = millis();
                new_tof = get_front_tof();
                pcnt = pid(measure_count, curr_mil, new_tof);
                
                move_speed(pcnt);
                store_pid_data(measure_count, curr_mil, new_tof, pcnt);
                measure_count ++;
                
              }
              else if (measure_count >= ARR_SIZE){
                active_stop();
                write_pid_data();
                do_PID = false;
              }
            }

            else if (do_KF){
               if(sensor_data_ready() && measure_count < ARR_SIZE){
                //Doing KF and just got new sensor reading
                curr_mil = millis();
                new_tof = get_front_tof();
                kf_state = kf(measure_count, curr_mil, new_tof);
                pcnt = pid(measure_count, curr_mil, kf_state);
                
                move_speed(pcnt);
                measure_count ++;
                
              }
              else if (measure_count >= ARR_SIZE){
                active_stop();
                write_pid_data();
                write_kf_data();
                do_KF = false;
              }
            }
            
            else if (moving_forward){
              if(sensor_data_ready()){
                curr_mil = millis();
                new_tof = get_front_tof();
                
                if (measure_count < 20 || (measure_count >= ARR_SIZE - 20 && measure_count < ARR_SIZE)){
                  //pre/post step
                  store_pid_data(measure_count, curr_mil, new_tof, 0);
                  measure_count ++;
                }
                else if (measure_count < ARR_SIZE - 20){
                  //step
                  if (new_tof < 250) active_stop();
                  else move_speed(forward_spd);
                  
                  store_pid_data(measure_count, curr_mil, new_tof, forward_spd);
                  measure_count ++;
                
                }
                else if (measure_count >= ARR_SIZE){
                  //done
                  passive_stop();
                  write_pid_data();
                  moving_forward = false;
                }
              }
            }
        }

        Serial.println("Disconnected");
        active_stop();
    }
}
