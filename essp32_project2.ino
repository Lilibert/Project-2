// Include BasicLinearAlgebra library for matrix operations
#include <BasicLinearAlgebra.h>

// Include Bluetooth Serial library by Henry Abrahamsen
#include <BluetoothSerial.h>

// Create Bluetooth Serial object
BluetoothSerial SerialBT;

// Define UART2 pins (standard general-purpose UART)
#define RXD2 16 // GPIO16 (IO16) as RX
#define TXD2 17 // GPIO17 (IO17) as TX

// Packet header
#define PACKET_HEADER 0xAA

// Command types
#define CMD_MOTOR_VELOCITY_BASE 0x10 // Base command for motor velocity (0x1X)
#define CMD_MOTOR_ENCODER_BASE 0x20  // Base command for motor encoder (0x2X)
#define CMD_SENSORS_STATE_BASE 0x30  // Base command for sensors state (0x3X)

// Motor/Sensor direction modes (X value in 0x1X, 0x2X, and 0x3X)
#define MOTOR_MODE_BOTH_FORWARD 0  // 0xX0: Motor1/e2 +, Motor2/e3 +
#define MOTOR_MODE_BOTH_BACKWARD 1 // 0xX1: Motor1/e2 -, Motor2/e3 -
#define MOTOR_MODE_M1FWD_M2BWD 2   // 0xX2: Motor1/e2 +, Motor2/e3 -
#define MOTOR_MODE_M1BWD_M2FWD 3   // 0xX3: Motor1/e2 -, Motor2/e3 +

// Variables to store received sensor values
float e2_value = 0.0;        // Lateral error (mm)
float e3_value = 0.0;        // Angular error (rad)
uint8_t ultrasonic_dist = 0; // Ultrasonic distance (mm)

// MPC Controller Parameters
#define TS 0.01 // Sampling time in seconds (10ms)
float qw = 1.0; // Input weight (Q in matlab)
float pw = 1.0; // State penalty weight (P in matlab)

// Robot Parameters for Differential Drive
#define L 0.3               // Wheel base (distance between wheels) in meters
#define WHEEL_RADIUS 0.05   // Wheel radius in meters (adjust to your robot)
#define LINEAR_VELOCITY 0.3 // Linear velocity in m/s

// Control output
float w0 = 0.0; // MPC control output (angular velocity)

// Communication Configuration:
// Serial (UART0)  - Slave communication (sensor data receive)
// Serial2 (UART2) - Slave communication (motor commands send)
// SerialBT        - Debug/Log output via Bluetooth

// Function to calculate checksum (XOR of all bytes) - for 3-byte data
uint8_t calculateChecksum(uint8_t cmd, uint8_t byte1, uint8_t byte2, uint8_t byte3)
{
  return cmd ^ byte1 ^ byte2 ^ byte3;
}

// Function to calculate checksum for 2-byte data (for sensors packet)
uint8_t calculateChecksum2Bytes(uint8_t cmd, uint8_t byte1, uint8_t byte2)
{
  return cmd ^ byte1 ^ byte2;
}

// Function to send Motor Velocity Command Packet (0x1X)
// mode: motor direction mode (0-3)
//   0: Motor1 +, Motor2 + (both forward)
//   1: Motor1 -, Motor2 - (both backward)
//   2: Motor1 +, Motor2 - (turn right)
//   3: Motor1 -, Motor2 + (turn left)
// motor1_velocity: 12-bit value for first motor (0-4095)
// motor2_velocity: 12-bit value for second motor (0-4095)
void sendMotorVelocity(uint8_t mode, uint16_t motor1_velocity, uint16_t motor2_velocity)
{
  // Limit mode to 0-3
  mode &= 0x03;

  // Limit values to 12 bits (0-4095)
  motor1_velocity &= 0x0FFF;
  motor2_velocity &= 0x0FFF;

  // Create command byte: 0x1X where X is the mode
  uint8_t cmd = CMD_MOTOR_VELOCITY_BASE | mode; // 0x10, 0x11, 0x12, or 0x13

  // Pack 12-bit values into 3 bytes
  // Byte 1: motor1[11:4] (high 8 bits of motor1)
  // Byte 2: motor1[3:0] + motor2[11:8] (low 4 bits of motor1 + high 4 bits of motor2)
  // Byte 3: motor2[7:0] (low 8 bits of motor2)
  uint8_t byte1 = (motor1_velocity >> 4) & 0xFF;
  uint8_t byte2 = ((motor1_velocity & 0x0F) << 4) | ((motor2_velocity >> 8) & 0x0F);
  uint8_t byte3 = motor2_velocity & 0xFF;

  // Calculate checksum
  uint8_t checksum = calculateChecksum(cmd, byte1, byte2, byte3);

  // Send packet: [0xAA][0x1X][VELOCITY_VALUE][CHECKSUM]
  Serial2.write(PACKET_HEADER);
  Serial2.write(cmd);
  Serial2.write(byte1);
  Serial2.write(byte2);
  Serial2.write(byte3);
  Serial2.write(checksum);

  // Log to Bluetooth
  SerialBT.print("TX Motor Velocity: [0x");
  SerialBT.print(PACKET_HEADER, HEX);
  SerialBT.print("][0x");
  SerialBT.print(cmd, HEX);
  SerialBT.print("][0x");
  SerialBT.print(byte1, HEX);
  SerialBT.print("][0x");
  SerialBT.print(byte2, HEX);
  SerialBT.print("][0x");
  SerialBT.print(byte3, HEX);
  SerialBT.print("][0x");
  SerialBT.print(checksum, HEX);
  SerialBT.print("] M1=");
  SerialBT.print(motor1_velocity);
  SerialBT.print(" M2=");
  SerialBT.println(motor2_velocity);
}

// Convenience function: Auto-determine mode based on signed velocities
// Accepts signed int16_t values and automatically determines the mode
void sendMotorVelocitySigned(int16_t motor1_velocity, int16_t motor2_velocity)
{
  // Determine mode based on signs
  uint8_t mode;
  if (motor1_velocity >= 0 && motor2_velocity >= 0)
  {
    mode = MOTOR_MODE_BOTH_FORWARD; // 0: +, +
  }
  else if (motor1_velocity < 0 && motor2_velocity < 0)
  {
    mode = MOTOR_MODE_BOTH_BACKWARD; // 1: -, -
  }
  else if (motor1_velocity >= 0 && motor2_velocity < 0)
  {
    mode = MOTOR_MODE_M1FWD_M2BWD; // 2: +, -
  }
  else
  {                                // motor1_velocity < 0 && motor2_velocity >= 0
    mode = MOTOR_MODE_M1BWD_M2FWD; // 3: -, +
  }

  // Get absolute values
  uint16_t abs_motor1 = abs(motor1_velocity);
  uint16_t abs_motor2 = abs(motor2_velocity);

  // Send command
  sendMotorVelocity(mode, abs_motor1, abs_motor2);
}

// Function to send Motor Encoder Command Packet (0x2X)
// mode: motor direction mode (0-3)
//   0: Motor1 +, Motor2 + (both forward)
//   1: Motor1 -, Motor2 - (both backward)
//   2: Motor1 +, Motor2 - (turn right)
//   3: Motor1 -, Motor2 + (turn left)
// motor1_encoder: 12-bit value for first motor (0-4095)
// motor2_encoder: 12-bit value for second motor (0-4095)
void sendMotorEncoder(uint8_t mode, uint16_t motor1_encoder, uint16_t motor2_encoder)
{
  // Limit mode to 0-3
  mode &= 0x03;

  // Limit values to 12 bits (0-4095)
  motor1_encoder &= 0x0FFF;
  motor2_encoder &= 0x0FFF;

  // Create command byte: 0x2X where X is the mode
  uint8_t cmd = CMD_MOTOR_ENCODER_BASE | mode; // 0x20, 0x21, 0x22, or 0x23

  // Pack 12-bit values into 3 bytes
  // Byte 1: motor1[11:4] (high 8 bits of motor1)
  // Byte 2: motor1[3:0] + motor2[11:8] (low 4 bits of motor1 + high 4 bits of motor2)
  // Byte 3: motor2[7:0] (low 8 bits of motor2)
  uint8_t byte1 = (motor1_encoder >> 4) & 0xFF;
  uint8_t byte2 = ((motor1_encoder & 0x0F) << 4) | ((motor2_encoder >> 8) & 0x0F);
  uint8_t byte3 = motor2_encoder & 0xFF;

  // Calculate checksum
  uint8_t checksum = calculateChecksum(cmd, byte1, byte2, byte3);

  // Send packet: [0xAA][0x2X][ENCODER_VALUE][CHECKSUM]
  Serial2.write(PACKET_HEADER);
  Serial2.write(cmd);
  Serial2.write(byte1);
  Serial2.write(byte2);
  Serial2.write(byte3);
  Serial2.write(checksum);

  // Log to Bluetooth
  SerialBT.print("TX Motor Encoder: [0x");
  SerialBT.print(PACKET_HEADER, HEX);
  SerialBT.print("][0x");
  SerialBT.print(cmd, HEX);
  SerialBT.print("][0x");
  SerialBT.print(byte1, HEX);
  SerialBT.print("][0x");
  SerialBT.print(byte2, HEX);
  SerialBT.print("][0x");
  SerialBT.print(byte3, HEX);
  SerialBT.print("][0x");
  SerialBT.print(checksum, HEX);
  SerialBT.print("] M1=");
  SerialBT.print(motor1_encoder);
  SerialBT.print(" M2=");
  SerialBT.println(motor2_encoder);
}

// Convenience function: Auto-determine mode based on signed encoder values
// Accepts signed int16_t values and automatically determines the mode
void sendMotorEncoderSigned(int16_t motor1_encoder, int16_t motor2_encoder)
{
  // Determine mode based on signs
  uint8_t mode;
  if (motor1_encoder >= 0 && motor2_encoder >= 0)
  {
    mode = MOTOR_MODE_BOTH_FORWARD; // 0: +, +
  }
  else if (motor1_encoder < 0 && motor2_encoder < 0)
  {
    mode = MOTOR_MODE_BOTH_BACKWARD; // 1: -, -
  }
  else if (motor1_encoder >= 0 && motor2_encoder < 0)
  {
    mode = MOTOR_MODE_M1FWD_M2BWD; // 2: +, -
  }
  else
  {                                // motor1_encoder < 0 && motor2_encoder >= 0
    mode = MOTOR_MODE_M1BWD_M2FWD; // 3: -, +
  }

  // Get absolute values
  uint16_t abs_motor1 = abs(motor1_encoder);
  uint16_t abs_motor2 = abs(motor2_encoder);

  // Send command
  sendMotorEncoder(mode, abs_motor1, abs_motor2);
}

// Function to calculate left and right wheel angular velocities
// Inputs:
//   v: linear velocity (m/s)
//   omega: angular velocity (rad/s)
//   L: wheelbase distance (m)
//   r: wheel radius (m)
// Outputs:
//   omega_L: left wheel angular velocity in RPM
//   omega_R: right wheel angular velocity in RPM
//
// Formulas:
//   ω_R = (v + ω*L/2) / (2πr) × 60 (rpm)
//   ω_L = (v - ω*L/2) / (2πr) × 60 (rpm)
void calculateWheelOmegas(float v, float omega, float &omega_L, float &omega_R)
{
  float r = WHEEL_RADIUS;
  float wheelbase = L;

  // Calculate wheel angular velocities in RPM
  // ω_R = (v + ω*L/2) / (2πr) × 60
  omega_R = ((v + omega * wheelbase / 2.0) / (2.0 * PI * r)) * 60.0;

  // ω_L = (v - ω*L/2) / (2πr) × 60
  omega_L = ((v - omega * wheelbase / 2.0) / (2.0 * PI * r)) * 60.0;
}

// Function to send Sensors State Request (0x3X)
// mode: sensor sign mode (0-3)
//   0: e2 +, e3 + (both positive)
//   1: e2 -, e3 - (both negative)
//   2: e2 +, e3 -
//   3: e2 -, e3 +
// Send: [0xAA][0x3X][0x3X]
void requestSensorData(uint8_t mode)
{
  // Limit mode to 0-3
  mode &= 0x03;

  // Create command byte: 0x3X where X is the mode
  uint8_t cmd = CMD_SENSORS_STATE_BASE | mode; // 0x30, 0x31, 0x32, or 0x33

  // Send request packet: [0xAA][0x3X][0x3X]
  Serial2.write(PACKET_HEADER);
  Serial2.write(cmd);
  Serial2.write(cmd);

  // Log to Bluetooth
  SerialBT.print("TX Sensor Request: [0x");
  SerialBT.print(PACKET_HEADER, HEX);
  SerialBT.print("][0x");
  SerialBT.print(cmd, HEX);
  SerialBT.print("][0x");
  SerialBT.print(cmd, HEX);
  SerialBT.println("]");
}

// Function to receive Sensors State Packet (0x3X) from UART0 (Serial)
// Receive: [0xAA][0x3X][SENSORS_VALUE][CHECKSUM]
// SENSORS_VALUE: 3 bytes (e2_byte, e3_byte, ultrasonic_byte)
// Returns true if a valid packet was received
bool receiveSensorPacket(float &e2_value, float &e3_value, uint8_t &ultrasonic_dist)
{
  static uint8_t rxBuffer[6]; // [HEADER][CMD][E2_BYTE][E3_BYTE][ULTRASONIC_BYTE][CHECKSUM]
  static uint8_t rxIndex = 0;
  static bool packetStarted = false;

  // Check if data is available on Serial (UART0)
  while (Serial.available() > 0)
  {
    uint8_t inByte = Serial.read();

    // Look for packet header
    if (!packetStarted)
    {
      if (inByte == PACKET_HEADER)
      {
        packetStarted = true;
        rxIndex = 0;
        rxBuffer[rxIndex++] = inByte;
      }
    }
    else
    {
      // Store incoming byte
      rxBuffer[rxIndex++] = inByte;

      // Check if we received a complete packet (6 bytes total)
      if (rxIndex >= 6)
      {
        packetStarted = false;
        rxIndex = 0;

        // Parse packet
        uint8_t header = rxBuffer[0];
        uint8_t cmd = rxBuffer[1];
        uint8_t e2_byte = rxBuffer[2];
        uint8_t e3_byte = rxBuffer[3];
        uint8_t ultrasonic_byte = rxBuffer[4];
        uint8_t receivedChecksum = rxBuffer[5];

        // Verify it's a sensors packet (0x30-0x33)
        if ((cmd & 0xF0) == CMD_SENSORS_STATE_BASE)
        {
          // Calculate expected checksum (XOR of cmd + 3 data bytes)
          uint8_t expectedChecksum = calculateChecksum(cmd, e2_byte, e3_byte, ultrasonic_byte);

          // Verify checksum
          if (receivedChecksum == expectedChecksum)
          {
            // Get mode from command
            uint8_t mode = cmd & 0x0F;

            // Convert bytes back to floating point values
            // e2: 1 decimal place (245 -> 24.5 mm)
            float e2_raw = e2_byte / 10.0;
            // e3: 2 decimal places (157 -> 1.57 rad)
            float e3_raw = e3_byte / 100.0;

            // Apply signs based on mode
            switch (mode)
            {
            case 0: // e2 +, e3 +
              e2_value = e2_raw;
              e3_value = e3_raw;
              break;
            case 1: // e2 -, e3 -
              e2_value = -e2_raw;
              e3_value = -e3_raw;
              break;
            case 2: // e2 +, e3 -
              e2_value = e2_raw;
              e3_value = -e3_raw;
              break;
            case 3: // e2 -, e3 +
              e2_value = -e2_raw;
              e3_value = e3_raw;
              break;
            }

            // Ultrasonic distance (already in mm as uint8_t)
            ultrasonic_dist = ultrasonic_byte;

            // Debug: Print received packet to Bluetooth
            SerialBT.print("RX Sensor Data: [0x");
            SerialBT.print(header, HEX);
            SerialBT.print("][0x");
            SerialBT.print(cmd, HEX);
            SerialBT.print("][0x");
            SerialBT.print(e2_byte, HEX);
            SerialBT.print("][0x");
            SerialBT.print(e3_byte, HEX);
            SerialBT.print("][0x");
            SerialBT.print(ultrasonic_byte, HEX);
            SerialBT.print("][0x");
            SerialBT.print(receivedChecksum, HEX);
            SerialBT.println("]");

            // Print decoded values to Bluetooth
            SerialBT.print("  e2=");
            SerialBT.print(e2_value, 1);
            SerialBT.print("mm, e3=");
            SerialBT.print(e3_value, 2);
            SerialBT.print("rad, dist=");
            SerialBT.print(ultrasonic_dist);
            SerialBT.println("mm");

            return true; // Valid packet received
          }
          else
          {
            SerialBT.println("Error: Checksum mismatch!");
            SerialBT.print("  Expected: 0x");
            SerialBT.print(expectedChecksum, HEX);
            SerialBT.print(", Received: 0x");
            SerialBT.println(receivedChecksum, HEX);
          }
        }
      }
    }
  }

  return false; // No valid packet received yet
}

// Convenience function: Request and wait for sensor data
// Returns true if data received successfully
bool getSensorData(uint8_t mode, float &e2, float &e3, uint8_t &ultrasonic, uint16_t timeout_ms = 100)
{
  // Send request
  requestSensorData(mode);

  // Wait for response with timeout
  unsigned long startTime = millis();
  while (millis() - startTime < timeout_ms)
  {
    if (receiveSensorPacket(e2, e3, ultrasonic))
    {
      return true; // Success
    }
    delay(1); // Small delay to avoid busy-waiting
  }

  SerialBT.println("Timeout waiting for sensor data");
  return false; // Timeout
}

// MPC Controller Function
void calculate_mpc_control()
{
  using namespace BLA;

  // Discrete-time model matrices
  // A = [1, LINEAR_VELOCITY*Ts; 0, 1]
  Matrix<2, 2> A;
  A(0, 0) = 1.0;
  A(0, 1) = LINEAR_VELOCITY * TS;
  A(1, 0) = 0.0;
  A(1, 1) = 1.0;

  // B = [-L*Ts; -Ts]
  Matrix<2, 1> B;
  B(0, 0) = -L * TS;
  B(1, 0) = -TS;

  // Current state x0 = [e2; e3]
  Matrix<2, 1> x0;
  x0(0, 0) = e2_value;
  x0(1, 0) = e3_value;

  // Weights
  // P = pw * I (2x2 identity matrix scaled by pw)
  Matrix<2, 2> P;
  P(0, 0) = pw;
  P(0, 1) = 0.0;
  P(1, 0) = 0.0;
  P(1, 1) = pw;

  // Scalar input weight
  float Qu = qw;

  // Calculate: B' * P * B + Q
  // B' is 1x2, P is 2x2, B is 2x1 -> result is scalar (1x1)
  Matrix<1, 2> Bt = ~B;        // Transpose of B
  Matrix<1, 2> BtP = Bt * P;   // 1x2 * 2x2 = 1x2
  Matrix<1, 1> BtPB = BtP * B; // 1x2 * 2x1 = 1x1 (scalar)
  float BtPB_plus_Q = BtPB(0, 0) + Qu;

  // Calculate: x0' * A' * P * B
  // x0' is 1x2, A' is 2x2, P is 2x2, B is 2x1 -> result is scalar (1x1)
  Matrix<1, 2> x0t = ~x0;            // Transpose of x0
  Matrix<2, 2> At = ~A;              // Transpose of A
  Matrix<1, 2> x0tAt = x0t * At;     // 1x2 * 2x2 = 1x2
  Matrix<1, 2> x0tAtP = x0tAt * P;   // 1x2 * 2x2 = 1x2
  Matrix<1, 1> x0tAtPB = x0tAtP * B; // 1x2 * 2x1 = 1x1 (scalar)

  // Control law: w0 = ((B' * P * B + Q)^(-1) * (x0' * A' * P * B))
  w0 = x0tAtPB(0, 0) / BtPB_plus_Q;

  // Calculate wheel velocities
  float omega_L, omega_R; // Left and right wheel angular velocities in RPM
  calculateWheelOmegas(LINEAR_VELOCITY, w0, omega_L, omega_R);

  // Convert RPM to motor command values (can be negative)
  int16_t motor_L = int16_t(omega_L * 10);
  int16_t motor_R = int16_t(omega_R * 10);

  // Send motor velocity commands (auto-determines mode based on signs)
  sendMotorVelocitySigned(motor_L, motor_R);
}

void setup()
{
  // Initialize UART0 (Serial) at 115200 baud rate - for slave communication (RX sensor data)
  Serial.begin(115200);

  // Initialize Bluetooth Serial for debug/log output
  SerialBT.begin("ESP32_Robot"); // Bluetooth device name

  // Initialize UART2 with custom pins (TX=GPIO17, RX=GPIO16) - for slave communication (TX motor commands)
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Log to Bluetooth only
  SerialBT.println("=== ESP32 Robot Started ===");
  SerialBT.println("Bluetooth device: ESP32_Robot");
  SerialBT.print("UART0 (Serial): RX sensor data @ 115200 baud");
  SerialBT.println();
  SerialBT.print("UART2 (GPIO17/16): TX motor commands @ 115200 baud");
  SerialBT.println();
  SerialBT.println("MPC Controller Parameters:");
  SerialBT.print("  Ts=");
  SerialBT.print(TS, 3);
  SerialBT.print("s, LINEAR_VELOCITY=");
  SerialBT.print(LINEAR_VELOCITY, 2);
  SerialBT.print("m/s, L=");
  SerialBT.print(L, 2);
  SerialBT.println("m");
  SerialBT.print("  qw=");
  SerialBT.print(qw, 2);
  SerialBT.print(", pw=");
  SerialBT.println(pw, 2);
  SerialBT.println("=== Setup Complete ===");
  SerialBT.println();
}

void loop()
{
  // Check if data available from Bluetooth
  if (SerialBT.available()) {
    String btData = SerialBT.readStringUntil('\n');  // Read until newline
    btData.trim();  // Remove whitespace
    
    // Parse the two velocity values
    int spaceIndex = btData.indexOf(' ');
    if (spaceIndex > 0) {
      int leftVel = btData.substring(0, spaceIndex).toInt();
      int rightVel = btData.substring(spaceIndex + 1).toInt();
      
      // Send parsed velocities to motors
      sendMotorVelocitySigned(leftVel, rightVel);
      
      // Log to Bluetooth for confirmation
      SerialBT.print("Motor command: L=");
      SerialBT.print(leftVel);
      SerialBT.print(", R=");
      SerialBT.println(rightVel);
    }
  }
  
  // Original test command (optional - can remove or comment out)
  // sendMotorVelocitySigned(1800, 1800);
  // delay(10000);
}
