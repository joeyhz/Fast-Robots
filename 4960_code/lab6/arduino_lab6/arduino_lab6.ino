
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include "SensorFuncs.h"
#include "MotorFuncs.h"


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
int measure_count = 0;
unsigned long startMillis = 0;

#define ARR_SIZE 200
int front_tof_readings[ARR_SIZE];
long times[ARR_SIZE];
float motor_pcnts[ARR_SIZE];

int TARGET_DIST = 300;

#define KP 1.0
#define KI 0.0
#define KD 0.0

int integrator = 0;
//////////// Global Variables ////////////

enum CommandTypes
{
    WALL_PID,
    STOP,
    MOVE_FORWARD,
};

void
handle_command()
{   
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
        case STOP:
          break;
        case MOVE_FORWARD:
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

void write_data(){
  for (int i=0; i<ARR_SIZE; i++)tx_characteristic_float.writeValue(front_tof_readings[i]*1.0);
  for (int i=0; i<ARR_SIZE; i++)tx_characteristic_float.writeValue(times[i]*1.0);
  for (int i=0; i<ARR_SIZE; i++)tx_characteristic_float.writeValue(motor_pcnts[i]);
}

void store_data(int index, long curr_mill, int new_tof, float pcnt){
  Serial.println("Index: ");
  Serial.print(index);
  Serial.print(", Time (ms): ");
  Serial.print(curr_mill);
  Serial.print(", TOF dist (mm): ");
  Serial.print(new_tof);
  Serial.print(", Motor %: ");
  Serial.print(pcnt);
  
  front_tof_readings[index] = new_tof;
  times[index] = curr_mill;
  motor_pcnts[index] = pcnt;
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
  if (raw > 100) return 100;
  if (raw < -100) return -100;
  return raw;
}




void loop(){
    long curr_mil;
    int new_tof;
    float pcnt;
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected()) {
            // Read data
            read_data();

            if (do_PID && sensor_data_ready() && measure_count < ARR_SIZE){
              //Doing PID and just got new sensor reading
              curr_mil = millis();
              new_tof = get_front_tof();
              pcnt = pid(measure_count, curr_mil, new_tof);
              
              move_speed(pcnt);
              store_data(measure_count, curr_mil, new_tof, pcnt);
              measure_count ++;
              
            }
            else if (do_PID && measure_count >= ARR_SIZE){
             active_stop();
              write_data();
            }
        }

        Serial.println("Disconnected");
        active_stop();
    }
}
