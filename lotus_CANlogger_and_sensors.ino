#include <math.h>
#include <mcp_can.h>
#include <SPI.h>
#include <SD.h>

//list of used digital pins
//D13 SCK
//D12 MISO
//D11 MOSI
const int SPI_CS_PIN_SD = 10; //D10 for SD shield
const int SPI_CS_PIN_CAN = 9; //D9 for CAN shield
const int O2_pwr_pin = 7; //D7 for FET actuation to power O2 sensor

//list of used analog pins
const int thermometerPin = 0; //A0 pin with thermometer (thermistor)
const int TMAP_IAT_pin = 2;//A2 pin with air temp thermometer
const int TMAP_MAP_pin = 3;//A3 pin with manifold pressure
const int O2_linear_pin = 4;//A4 pin with oxygen sensor output

//initialize some special variables
File dataFile;
MCP_CAN CAN(SPI_CS_PIN_CAN);//CAN shield device
unsigned long log_prev_time = 0;
unsigned char CAN_len = 8;
unsigned char CAN_read_buf[8];
unsigned char CAN_send_buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long CAN_Id = 0;
boolean new_CAN_Message = false;
unsigned long write_TMAP_to_ECU_prev_time = 0;
unsigned long write_UEGO_to_ECU_prev_time = 0;
unsigned long time_since_engine_start = 0;
unsigned long time_at_eng_count_increment = 0;
unsigned long clusterPacketTime = 0;
boolean engine_running = false;
int oxygen_sens_start_delay = 200;

void setup() {
  //delay while car voltage stabilizes...
  Serial.begin(9600);
  delay(2000);//2 seconds

  //start CAN shield - we'll get stuck here if we fail to initialize the CAN bus
START_INIT_CAN:
  if ( CAN_OK != CAN.begin(CAN_1000KBPS) ) // init can bus : baudrate = 1000k = 1MBps
  {
    Serial.println(F("Retry Starting CAN.."));
    delay(100);
    goto START_INIT_CAN;
  }

/*
  //start the SD card too - this will also get us stuck if it fails
START_INIT_SD:
  if ( !SD.begin(SPI_CS_PIN_SD) ) {
    Serial.println(F("Retry Starting SD shield.."));
    delay(100);
    goto START_INIT_SD;
  }

  //Now that we're initialized, let us write the data header
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(F("Time [ms], Oil Temperature [deg C], CANID, CAN0, CAN1, CAN2, CAN3, CAN4, CAN5, CAN6, CAN7,"));
    dataFile.close();
  }
  */

  //sweep the gauge cluster to indicate to the user that we've completed our initialization
  sweep_gauge_cluster();

  //initialize the FET power control
  pinMode(O2_pwr_pin, OUTPUT);
  delay(100);
  digitalWrite(O2_pwr_pin, LOW);

  //initialize our timers
  log_prev_time = millis();
  write_TMAP_to_ECU_prev_time = log_prev_time;
  write_UEGO_to_ECU_prev_time = log_prev_time;
  time_at_eng_count_increment = log_prev_time;
}

void loop() {
  
  //did we receive a CAN message? If so, read it in and set a flag
  while(CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&CAN_len, CAN_read_buf);
    CAN_Id = CAN.getCanId();
    new_CAN_Message = true;

    //interpret gauge cluster data so we can tell if the engine is running
    if (CAN_Id == 0x400)
    {
      clusterPacketTime = millis();
      uint16_t engine_speed = (CAN_read_buf[2] << 8) + CAN_read_buf[3];
      if (engine_speed > 500 && !engine_running)
      {
        //we're starting the engine, so read the coolant temperature and
        //decide how long to delay turning on the oxygen sensor
        uint8_t start_coolant_temp = (CAN_read_buf[5] * 160) / 256 - 40;
        oxygen_sens_start_delay = lookup_O2_start_delay(start_coolant_temp);
        engine_running = true;
      } else if (engine_speed < 100 && engine_running) {
        engine_running = false;
        time_since_engine_start = 0;
      }
    }
    else{
      //printCANpacket(CAN_Id, CAN_read_buf, CAN_len);
    }
    //printCANpacket(CAN_Id, CAN_read_buf, CAN_len);
  }

  unsigned long currentTime = millis();
  //read in the oil pan temperature
  double oil_temperature = tempRead(thermometerPin);
  
  //log the gauge cluster message (ID = 0x400) at 1Hz, and any other message immediately  
  /*
  if (currentTime - log_prev_time > 1000 || (CAN_Id != 0x400 && new_CAN_Message == true))
  {

    dataFile = SD.open("datalog.txt", FILE_WRITE);
    //then write to the SD card, if the file is available
    if (dataFile) {
      dataFile.print(currentTime);
      dataFile.print(F(","));
      dataFile.print(oil_temperature);
      dataFile.print(F(","));
      dataFile.print(CAN_Id, HEX);
      dataFile.print(F(","));

      for (uint8_t i = 0; i < CAN_len; i++)   //print the address of first byte
      {
        dataFile.print(CAN_read_buf[i]);
        dataFile.print(F(","));
      }
      dataFile.println();
      dataFile.close();
    }
    new_CAN_Message = false;
    log_prev_time = currentTime;

  }
  */

  //read in the other analog outputs from other sensors, and send
  //the values to the Lotus ECU at 10Hz
  //send 0x6 bytes to ID 0x55, first 4 are address, next 2 are data to write there
  //do this once for each 10-bit analog read value
  if (currentTime - write_TMAP_to_ECU_prev_time >= 100 && currentTime - clusterPacketTime <= 50) //don't send messages that may overlap with the cluster CAN packet
  {
    uint16_t TMAP_P_AtoD = analogRead(TMAP_MAP_pin);
    uint16_t TMAP_T_AtoD = analogRead(TMAP_IAT_pin);

    //send to 0x54 for 32-bit, 0x55 for 16-bit, and 0x56 for 8-bit data transfer 
    CAN_send_buf[0] = 0x00;
    CAN_send_buf[1] = 0x08;//0x82000 is unused RAM
    CAN_send_buf[2] = 0x20;
    CAN_send_buf[3] = 0x00;
    CAN_send_buf[4] = ((TMAP_P_AtoD >> 0x8) & 0xFF);
    CAN_send_buf[5] = (TMAP_P_AtoD & 0xFF);
    CAN_send_buf[6] = ((TMAP_T_AtoD >> 0x8) & 0xFF);//0x82002
    CAN_send_buf[7] = (TMAP_T_AtoD & 0xFF);
    CAN.sendMsgBuf(0x54, 0, 8, CAN_send_buf);
    //I need to figure out if this creates a response CAN message from the ECU.. I may need to read that in.
    //I checked this, and see no response from the ECU (only the gauge cluster packets being sent)
    /*
    Serial.print(currentTime);
    Serial.print(F(",P:"));
    Serial.print(TMAP_P_AtoD);
    Serial.print(F(","));
    Serial.print(CAN_send_buf[4]);
    Serial.print(F(","));
    Serial.println(CAN_send_buf[5]);
    */

    write_TMAP_to_ECU_prev_time = millis();
  }

  if (currentTime - write_UEGO_to_ECU_prev_time >= 100 && currentTime - clusterPacketTime <= 50 && (currentTime - write_TMAP_to_ECU_prev_time <= 80 && currentTime - write_TMAP_to_ECU_prev_time >= 10) ) //don't send messages that may overlap with the other CAN packets
  {
    uint16_t O2_AtoD = analogRead(O2_linear_pin);
    CAN_send_buf[0] = 0x00;
    CAN_send_buf[1] = 0x08;
    CAN_send_buf[2] = 0x20;
    CAN_send_buf[3] = 0x04;//0x82004
    CAN_send_buf[4] = ((O2_AtoD >> 0x8) & 0xFF);
    CAN_send_buf[5] = (O2_AtoD & 0xFF);
    CAN_send_buf[6] = (int)oil_temperature + 40;//0x82006
    CAN_send_buf[7] = 0x00;
    CAN.sendMsgBuf(0x54, 0, 8, CAN_send_buf);
    
    write_UEGO_to_ECU_prev_time = millis();
  }

  //also, we should manage the power given to the O2 sensor - if we power
  //it on too early then we risk cracking the element
  if (engine_running && currentTime - time_at_eng_count_increment >= 1000) {
    time_since_engine_start += 1;
    time_at_eng_count_increment = millis();
  }

  if (time_since_engine_start >= oxygen_sens_start_delay)
  {
    digitalWrite(O2_pwr_pin, HIGH);
  }
  else
  {
    digitalWrite(O2_pwr_pin, LOW);
  }

}

//input the analog pin which the oil temp. thermistor is on
double tempRead(int pin)
{
  double Vratio = 0;
  double data = 0;
  double resistance = 0;
  double temperature = 0;
  double logResistance = 0;
  data = analogRead(pin);
  Vratio = 1023 / data;
  resistance = 9990 * (Vratio - 1);
  logResistance = log10(resistance);
  temperature = 15.239 * (logResistance * logResistance) - 192.41 * logResistance + 590.84;
  return temperature;
}


//input coolant temperature at engine start from CAN data
int lookup_O2_start_delay(int startup_coolt_temp) {
  int temps[] = { -40, -15, 11, 37, 37}; //deg C
  int delays[] = {200, 150, 100, 12, 12};//second
  int index = 0;
  for (uint8_t i = 0; i < 4; i++) {
    index = i;
    if (startup_coolt_temp < temps[i])
    {
      break;
    }
  }

  //now, interpolate
  int delay_time_interp = delays[index] + ((startup_coolt_temp - temps[index]) * ( delays[index + 1] - delays[index] )) / ( temps[index + 1] - temps[index] );
  if (delay_time_interp > delays[0])
  {
    delay_time_interp = delays[0];
  } else if (delay_time_interp < delays[3])
  {
    delay_time_interp = delays[3];
  }
  return delay_time_interp;
}

void sweep_gauge_cluster() {
  //send the CAN messages that lead to the Tach and speedometer sweeping
  uint16_t tacho = 0;//range 0-10000 rpm
  uint8_t speedo = 0;//range 0-255 kph
  //CAN packet;
  //byte 0 == spd (kph)
  //byte 1 == unused
  //byte 2 == tacho high byte (rpm)
  //byte 3 == tacho low byte (rpm)
  //byte 4 == fuel level 0-255 is 0-100%
  //byte 5 == coolant temp, (160/256*byte - 40) (deg C)
  //byte 6 == bit flags for cluster lights
  //byte 7 == unused

  //loop and cycle the engine speed and tachometer over a few seconds
  double loopTime = 2.0;//sec
  int msgFreq = 50;//Hz
  int delayms = 1000/msgFreq;
  int numSteps = loopTime*msgFreq;
  double speedoStep = 255.00/(numSteps-1);
  double tachoStep = 10000.00/(numSteps-1);
  for(uint16_t i = 0; i < numSteps; i++)
  {
    //make sure we don't overload the CAN shield with ignored messages
    unsigned char len = 0;
    unsigned char buf[8];
    while(CAN_MSGAVAIL == CAN.checkReceive())// check if data coming
    {
      CAN.readMsgBuf(&len, buf);// read data,  len: data length, buf: data buf
    }
    speedo = i*speedoStep;
    tacho = i*tachoStep;
    CAN_send_buf[0] = speedo;
    CAN_send_buf[1] = 0x00;
    CAN_send_buf[2] = (tacho >> 8) & 0xFF;
    CAN_send_buf[3] = (tacho & 0xFF);
    CAN_send_buf[4] = 0x80;//50% fuel level
    CAN_send_buf[5] = 0x00;
    CAN_send_buf[6] = 0x00;
    CAN_send_buf[7] = 0x00;
    CAN.sendMsgBuf(0x400, 0, 8, CAN_send_buf);
    delay(delayms);
  }
  

  //reset the CAN message buffer
  CAN_send_buf[0] = 0x00;
  CAN_send_buf[1] = 0x00;
  CAN_send_buf[2] = 0x00;
  CAN_send_buf[3] = 0x00;
  CAN_send_buf[4] = 0x00;
  CAN_send_buf[5] = 0x00;
  CAN_send_buf[6] = 0x00;
  CAN_send_buf[7] = 0x00;
}

void printCANpacket(unsigned long ID, unsigned char bufferData[], unsigned char bufferLen) {
  Serial.print(F("ID: "));
  Serial.print(ID,HEX);
  Serial.print(F(" [ "));
  for(uint8_t i = 0; i < bufferLen; i++){
    Serial.print(bufferData[i],HEX);
    Serial.print(F(" "));
  }
  Serial.println(F("]"));
}
