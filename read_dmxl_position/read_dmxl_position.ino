/* Dynamixel Read Position Example
 */
 /* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
/* Dynamixel ID defines */
#define J_ID 1
#define PRESENT_POS 54
 
Dynamixel Dxl(DXL_BUS_SERIAL1); 
 
void setup() {
  // Initialize the dynamixel bus:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps  
  Dxl.begin(3);
Dxl.maxTorque(J_ID,1); // it has maxtorque for weak movement  
//  Dxl.jointMode(J_ID); //jointMode() is to use position mode  
}
 
void loop() {
  int pos;
  // Wait for 20 milliseconds
  delay(20);
 
  pos = Dxl.readWord(J_ID, PRESENT_POS); // Read present position
  SerialUSB.print("Present Position: ");
  SerialUSB.println(pos);
 
 
}

