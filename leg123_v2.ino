#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <SoftwareSerial.h>

#define RX_PIN 10  // RX for SC15 communication
#define TX_PIN 11  // TX for SC15 communication
#define NUM_SERVOS 6  // Number of servos

SoftwareSerial servoSerial(RX_PIN, TX_PIN);
ros::NodeHandle nh;

// Servo IDs (adjust as needed)
uint8_t servo_ids[NUM_SERVOS] = {1, 2, 3, 4, 5, 6};

// Function to send servo commands
void sendServoCommand(uint8_t id, int angle) {
    int position = map(angle, 0, 240, 0, 1000);  // Map angle to SC15 position

    uint8_t command[7];
    command[0] = 0x55; // Header byte 1
    command[1] = 0x55; // Header byte 2
    command[2] = id;   // Servo ID
    command[3] = 3;    // Data length
    command[4] = 1;    // Command type (Move to position)
    command[5] = position & 0xFF;         // Low byte
    command[6] = (position >> 8) & 0xFF;  // High byte

    servoSerial.write(command, sizeof(command));
}

// ROS subscriber callback
void servoCallback(const std_msgs::Float32MultiArray& msg) {
    if (msg.data.size() != NUM_SERVOS) {
        nh.logerror("Invalid number of servo commands!");
        return;
    }
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        int angle = msg.data[i];
        if (angle >= 0 && angle <= 240) {
            sendServoCommand(servo_ids[i], angle);
        } else {
            nh.logerror("Invalid angle! Must be 0-240°.");
        }
    }
}

// ROS subscriber
ros::Subscriber<std_msgs::Float32MultiArray> leg123("/servo_commands", servoCallback);

void setup() {
    Serial.begin(115200);
    servoSerial.begin(115200);  // SC15 baud rate
    nh.initNode();
    nh.subscribe(leg123);
}

void loop() {
    nh.spinOnce();  // Process ROS messages
    delay(10);
}
