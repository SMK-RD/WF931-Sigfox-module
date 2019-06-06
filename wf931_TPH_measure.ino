/*
 *  wf931.ino - WF931(Sigfox Module) sample application
 *  Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <LowPower.h>
#include <RTC.h>
#include <Wire.h>


//**************** 温湿度・気圧センサ 変数宣言 ***************************************************
uint8_t I2C_txBuffer[32];                   // I2C Transmit buffer
uint8_t I2c_rxBuffer[32];                   // I2C Receive buffer

// BME280用 キャリブレーション値
typedef struct _BME280_DIG
{
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
  int8_t  dig_H1;
  int16_t dig_H2;
  int8_t  dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  int8_t  dig_H6;
} BME280_DIG;
static BME280_DIG bme280_dig;
static int32_t t_fine;
// BME280用 温度、湿度、気圧格納変数
double bme280_Temperature;    // 温度
double bme280_Pressure;       // 気圧
double bme280_Humidity;       // 湿度
// TI OPT3001照度RAWデータ格納変数
//uint8_t i2c_illumi_data[2];   // 照度
unsigned int i2c_illumi_data[2];   // 照度

//**************************** BME280 内部関数宣言 ********************************
static void CompensateT_BME280(int32_t adc_T);
static void CompensateP_BME280(int32_t adc_P);
static void CompensateH_BME280(int32_t adc_H);

//***************** Cold start 定数設定 ******************************************
int sensorPin = A2;    // select the input pin for the analog voitage input
//int sensorValue = 0;  // variable to store the value coming from the sensor
const int numReadings = 5; // measure times for average
int readIndex;              // the index of the current reading
int total;                  // the running total
float averageSensorValue = 0.000;     // the average sensor voltage
float analogVref = 0.700; // Reference voltage
const int wakeupTime = 900;  // RTC timer 15分に設定 (900sec)
const int CorrectionValue = 3; // 時間補正値
const float alarmTh = 0.15; // alarm threshold
const uint8_t button2 = PIN_D22;  //  Pin used to trigger a wakeup

const char* boot_cause_strings[] = {
  "Power On Reset with Power Supplied",
  "System WDT expired or Self Reboot",
  "Chip WDT expired",
  "WKUPL signal detected in deep sleep",
  "WKUPS signal detected in deep sleep",
  "RTC Alarm expired in deep sleep",
  "USB Connected in deep sleep",
  "Others in deep sleep",
  "SCU Interrupt detected in cold sleep",
  "RTC Alarm0 expired in cold sleep",
  "RTC Alarm1 expired in cold sleep",
  "RTC Alarm2 expired in cold sleep",
  "RTC Alarm Error occurred in cold sleep",
  "Unknown(13)",
  "Unknown(14)",
  "Unknown(15)",
  "GPIO detected in cold sleep",
  "GPIO detected in cold sleep",
  "GPIO detected in cold sleep",
  "GPIO detected in cold sleep",
  "GPIO detected in cold sleep",
  "GPIO detected in cold sleep",
  "GPIO detected in cold sleep",
  "GPIO detected in cold sleep",
  "GPIO detected in cold sleep",
  "GPIO detected in cold sleep",
  "GPIO detected in cold sleep",
  "GPIO detected in cold sleep",
  "SEN_INT signal detected in cold sleep",
  "PMIC signal detected in cold sleep",
  "USB Disconnected in cold sleep",
  "USB Connected in cold sleep",
  "Power On Reset",
};

//************************* Boot Cause ********************************************
void printBootCause(bootcause_e bc)
{
  Serial.println("--------------------------------------------------");
  Serial.print("Boot Cause: ");
  Serial.print(boot_cause_strings[bc]);
  if ((COLD_GPIO_IRQ36 <= bc) && (bc <= COLD_GPIO_IRQ47)) {
    // Wakeup by GPIO    
    int pin = LowPower.getWakeupPin(bc);
    Serial.print(" <- pin ");
    Serial.print(pin);
  }
Serial.println();
Serial.println("--------------------------------------------------");
}

//***************************** ボタンが押された時にUART経由で通知 *********************
void pushed2()
{
  //Serial.println("Pushed D02!");
  Serial.println("Pushed D22!");
}

//****************************** wf931 UART 返信応答確認 ****************************
bool result() {
  char c;
  do{
   c = Serial2.read();
   printf("%c", c);
   if(c==0xFF){
    printf("Error!");
    return false;
   }
  }while(c!='\r');    

  // for LF
  c = Serial2.read();
  printf("%c", c);

  return true;
}

//************************ WF931のリセット、ID取得 ********************************
void wf931_job()
{
  // ********** WF931 リセット、ID取得 ***************
  // Configure baud rate to 9600
  Serial2.begin(9600);

  // Configure reset pin
  pinMode(PIN_D26, OUTPUT);

  // Do reset
  digitalWrite(PIN_D26, LOW);
  delay(100);

  // Release reset
  digitalWrite(PIN_D26, HIGH);
  delay(100);
  result();

  // Get chip ID
  printf("ID = ");
  Serial2.write("AT$ID?\r");
  delay(100);
  result();
}

//******************* WF931AT$SB=1の実行 ************************************
void wf931_send()
{
  // WF931 wakeup.
   Serial2.write("0x00");
   delay(60);
  
   // Send state by WF931
   Serial2.write("AT$SB=1\r");
   printf("Send State = 1\n");
   delay(100);
   result();
}

//**************** WF931をDeep Sleep状態にセット *******************************
void wf931Sleep()
{
   // WF931 wakeup.
   Serial2.write("0x00");
   delay(60);
  
   // Send deep sleep command to WF931
   Serial2.write("AT$DSLEEP\r");
   //printf("Send Deep Sleep command to WF931\n");
   delay(100);
   result();
}

//********************** DataをSigfox送信する　*********************************
void send_sigfox(int8_t i0, int8_t i1, int8_t i2, int8_t i3, int8_t i4, int16_t i5, int8_t i7, int8_t i8, int8_t i9, int8_t i10, int8_t i11)
{
  String output2;
  char val[24];
  // wakeup.
  Serial2.write("0x00");
  delay(60);

  // Send Data  
  sprintf(val,"%02x%02x%02x%02x%02x%04x%02x%02x%02x%02x%02x", i0, i1, i2, i3, i4, i5, i7, i8, i9, i10, i11);
  output2 = String(val);
  Serial2.write("AT$SF=");
  Serial2.write(val);
  Serial2.write("\r");
  
  delay(100);
  //result();
   
  Serial.print("SendData = ");
  Serial.println(output2);  
}

//***************** BOSCH製 BME280 温湿度/気圧センサー情報の取得 ****************************** 
void ReadDataBME280()
{
  // BOSCH製 BME280 温湿度/気圧センサー 初期化
  uint8_t osrs_t = 2;    // Temperature oversampling x 2
  uint8_t osrs_p = 5;  // Pressure oversampling x 16
  uint8_t osrs_h = 1;  // Humidity oversampling x 1
  uint8_t mode   = 3;  // Normal mode
  //uint8_t mode   = 1;  // Forced mode
  uint8_t t_sb   = 0;  // Tstandby 0.5ms 
  uint8_t filter = 4;  // Filter  16 //off
  uint8_t spi3w_en = 0;  // 3-wire SPI Disable

  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
  uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
  uint8_t ctrl_hum_reg  = osrs_h;
  uint8_t reset_reg = 0xE0;
  uint8_t reset = 0xB6;

  // グローバル変数クリア
  bme280_Temperature = 0.0; // 温度
  bme280_Pressure = 0.0;    // 気圧
  bme280_Humidity = 0.0;    // 湿度

  // 初期値のセット
  uint8_t I2CSLAVE_BME280 = 0x76; 
  I2C_txBuffer[0] = 0xF2;     // ctrl_hum register adr
  I2C_txBuffer[1] = ctrl_hum_reg; // Humidity oversampling x 1
  I2C_txBuffer[2] = 0xF4;     // ctrl_meas register adr
  I2C_txBuffer[3] = ctrl_meas_reg;  // Temperature oversampling x 1,Pressure oversampling x 1,Forced mode
  I2C_txBuffer[4] = 0xF5;     // config resister adr
  I2C_txBuffer[5] = config_reg;   // Tstandby 1000ms,Filter off,3-wire SPI Disable
  
  int i = 0;

  Wire.requestFrom(I2CSLAVE_BME280, 1);
  Wire.beginTransmission(I2CSLAVE_BME280);
  //Wire.write(reset_reg);  
  //Wire.write(reset); 
  for (i = 0; i < 6; i++)
  {
    Wire.write(I2C_txBuffer[i]);  
  }
  Wire.endTransmission(false);
   
  // BOSCH製 BME280 温湿度/気圧センサー キャリブレーション情報
  uint8_t calib_data[32];
  uint32_t bufData;
  //uint8_t i;
  memset(&bme280_dig, 0, sizeof(bme280_dig));

  // キャリブレーション情報
  // 0x88から24byte (温度、気圧のキャリブレーションデータの読込み）
  // I2Cアドレス 0x76 write
  I2C_txBuffer[0] = 0x88;
  Wire.requestFrom(I2CSLAVE_BME280, 1);
  Wire.beginTransmission(I2CSLAVE_BME280);
  Wire.write(I2C_txBuffer[0]);
  Wire.endTransmission(false);
  // I2Cアドレス 0x76 read  24bytes 
  for(i = 0; i < 24; i++)
  { 
    Wire.requestFrom(I2CSLAVE_BME280, 1); 
    calib_data[i] = Wire.read();
  }

  // 0xA1から1byte　(湿度のキャリブレーションデータの読込み-1）
  // I2Cアドレス 0x76 write
  I2C_txBuffer[0] = 0xA1;
  Wire.beginTransmission(I2CSLAVE_BME280);
  Wire.write(I2C_txBuffer[0]);
  Wire.endTransmission(false);
  // I2Cアドレス 0x76 read  1byte
  Wire.requestFrom(I2CSLAVE_BME280, 1);   
  i = 24;
  calib_data[i] = Wire.read();    // dig_H1[7:0]  i=24  
  
  // 0xE1から7byte　　(湿度のキャリブレーションデータの読込み-2）
  // I2Cアドレス 0x76 write
  I2C_txBuffer[0] = 0xE1;
  Wire.beginTransmission(I2CSLAVE_BME280);
  Wire.write(I2C_txBuffer[0]);
  Wire.endTransmission(false);
  // I2Cアドレス 0x76 read  7byte 
  for(int i = 0; i < 7; i++)
  {
    Wire.requestFrom(I2CSLAVE_BME280, 1); 
    calib_data[25 + i] = Wire.read();
  }

  bme280_dig.dig_T1 = (calib_data[1] << 8) | calib_data[0];
  bme280_dig.dig_T2 = (calib_data[3] << 8) | calib_data[2];
  bme280_dig.dig_T3 = (calib_data[5] << 8) | calib_data[4];
  bme280_dig.dig_P1 = (calib_data[7] << 8) | calib_data[6];
  bme280_dig.dig_P2 = (calib_data[9] << 8) | calib_data[8];
  bme280_dig.dig_P3 = (calib_data[11] << 8) | calib_data[10];
  bme280_dig.dig_P4 = (calib_data[13] << 8) | calib_data[12];
  bme280_dig.dig_P5 = (calib_data[15] << 8) | calib_data[14];
  bme280_dig.dig_P6 = (calib_data[17] << 8) | calib_data[16];
  bme280_dig.dig_P7 = (calib_data[19] << 8) | calib_data[18];
  bme280_dig.dig_P8 = (calib_data[21] << 8) | calib_data[20];
  bme280_dig.dig_P9 = (calib_data[23] << 8) | calib_data[22];
  bme280_dig.dig_H1 = calib_data[24];
  bme280_dig.dig_H2 = (calib_data[26] << 8) | calib_data[25];
  bme280_dig.dig_H3 = calib_data[27];
  bme280_dig.dig_H4 = (calib_data[28] << 4) | (0x0F & calib_data[29]);
  bme280_dig.dig_H5 = (calib_data[30] << 4) | ((calib_data[29] >> 4) & 0x0F);
  bme280_dig.dig_H6 = calib_data[31];

  // BOSCH製 BME280 温湿度/気圧センサー情報の取得
  uint8_t raw_data[8];
  i = 0;
  uint32_t temp_raw = 0;
  uint32_t pres_raw = 0;
  uint32_t hum_raw = 0;

  // 100msec delay for measuring time
  delay(1000);

  I2C_txBuffer[0] = 0xF7;
  Wire.beginTransmission(I2CSLAVE_BME280);
  Wire.write(I2C_txBuffer[0]);
  Wire.endTransmission(false);
  // I2Cアドレス 0x76 read  8byte
  for(i = 0; i < 8; i++)
  {        
    Wire.requestFrom(I2CSLAVE_BME280, 1); 
    raw_data[i] = Wire.read();
  }

  // 気圧、温度、湿度のRAWデータ
  pres_raw = (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4);
  temp_raw = (raw_data[3] << 12) | (raw_data[4] << 4) | (raw_data[5] >> 4);
  hum_raw  = (raw_data[6] << 8)  |  raw_data[7];
  
  // 値の変換 → グローバル変数に格納
  CompensateT_BME280((int32_t)temp_raw);
  CompensateP_BME280((int32_t)pres_raw);
  CompensateH_BME280((int32_t)hum_raw); 
}
  
//**************************** [内部関数] BOSCH製 BME280 温度データの補正と格納 ******************************
static void CompensateT_BME280(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)bme280_dig.dig_T1<<1))) * ((int32_t)bme280_dig.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)bme280_dig.dig_T1)) * ((adc_T>>4) - ((int32_t)bme280_dig.dig_T1))) >> 12) * ((int32_t)bme280_dig.dig_T3)) >> 14;
    
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  
  bme280_Temperature = (double)T / 100.0;
  Serial.print(bme280_Temperature);
  Serial.print(" ℃ \r\n");
}

//********************* BOSCH製 BME280 気圧データの補正と格納 ****************************************************
static void CompensateP_BME280(int32_t adc_P)
{
  int32_t var1, var2;
  uint32_t P;
  var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11) * ((int32_t)bme280_dig.dig_P6);
  var2 = var2 + ((var1*((int32_t)bme280_dig.dig_P5))<<1);
  var2 = (var2>>2)+(((int32_t)bme280_dig.dig_P4)<<16);
  var1 = (((bme280_dig.dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((int32_t)bme280_dig.dig_P2) * var1)>>1))>>18;
  var1 = ((((32768+var1))*((int32_t)bme280_dig.dig_P1))>>15);
  if (var1 == 0)
  {
    bme280_Pressure = 0.0;
    return;
  }    
  P = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
  if(P<0x80000000)
  {
     P = (P << 1) / ((uint32_t) var1);   
  }
  else
  {
      P = (P / (uint32_t)var1) * 2;    
  }
  var1 = (((int32_t)bme280_dig.dig_P9) * ((int32_t)(((P>>3) * (P>>3))>>13)))>>12;
  var2 = (((int32_t)(P>>2)) * ((int32_t)bme280_dig.dig_P8))>>13;
  P = (uint32_t)((int32_t)P + ((var1 + var2 + bme280_dig.dig_P7) >> 4));

  bme280_Pressure  = (double)P / 100.0;
  Serial.print(bme280_Pressure);
  Serial.print(" hPa \r\n");
}

//******************************* [内部関数] BOSCH製 BME280 湿度データの補正と格納 *****************************
static void CompensateH_BME280(int32_t adc_H)
{
  int32_t v_x1;
  uint32_t H;
    
  v_x1 = (t_fine - ((int32_t)76800));
  v_x1 = (((((adc_H << 14) -(((int32_t)bme280_dig.dig_H4) << 20) - (((int32_t)bme280_dig.dig_H5) * v_x1)) + 
            ((int32_t)16384)) >> 15) * (((((((v_x1 * ((int32_t)bme280_dig.dig_H6)) >> 10) * 
            (((v_x1 * ((int32_t)bme280_dig.dig_H3)) >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t)2097152)) * 
            ((int32_t)bme280_dig.dig_H2) + 8192) >> 14));
  v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32_t)bme280_dig.dig_H1)) >> 4));
  v_x1 = (v_x1 < 0 ? 0 : v_x1);
  v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
  H = (uint32_t)(v_x1 >> 12);
 
  bme280_Humidity = (double)H/1024.0;
  if(bme280_Humidity > 100.0)
  {
    bme280_Humidity = 100.0;
  }
  else if(bme280_Humidity < 0.0)
  {
    bme280_Humidity = 0.0;
  }
  Serial.print(bme280_Humidity);
  Serial.print("　％RH  \r\n");
}

//************************ 温度・湿度・気圧・照度測定結果のSigfox送信 ************************************* 
void tpmMeasure(void)
{
  int8_t msg[12] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int16_t msgP =0x0000;
  int16_t msgH =0x0000;
  double modf_int, modf_dec; // 整数部、小数部取り出し関数用
  msg[0] = 0x08;    // データタイプ：温湿度・気圧測定

  //BOSCH製 BME280を使用して温湿度気圧測定
  ReadDataBME280();
  // BME280 温度データの付加
  modf_int = (double)(int)bme280_Temperature;
  modf_dec = bme280_Temperature - modf_int;
  msg[1] = (uint8_t)modf_int;
  msg[2] = (uint8_t)(modf_dec * 100.0);

  // BME280 湿度データの付加
  modf_int = (double)(int)bme280_Humidity;
  modf_dec = bme280_Humidity - modf_int;
  msg[3] = (uint8_t)modf_int;
  msg[4] = (uint8_t)(modf_dec * 100.0);

  // BME280 気圧データの付加
  modf_int = (double)(int)bme280_Pressure;
  modf_dec = bme280_Pressure - modf_int;
  msgP = (uint16_t)modf_int;
  //msg[5] = (uint8_t)((uint16_t)modf_int >> 8);
  //msg[6] = (uint8_t)((uint16_t)modf_int & 0x00FF);
  msg[7] = (uint8_t)(modf_dec * 100.0);

  Serial.println("Sigfox send data");
  send_sigfox((int8_t)(msg[0]),
              (int8_t)(msg[1]),
              (int8_t)(msg[2]),
              (int8_t)(msg[3]),
              (int8_t)(msg[4]),
              (int16_t)(msgP),
              (int8_t)(msg[7]),
              (int8_t)(msg[8]),
              (int8_t)(msg[9]),
              (int8_t)(msg[10]),
              (int8_t)(msg[11]));
                
  Serial.println("End of Sigfox transmission");  
}

//************** セットアップ ***********************************************
void setup()
{
  wf931_job();

  Wire.begin(); // I2Cの接続

  // Configure baud rate to 115200 (ボードのUART）
  Serial.begin(115200);
  while (!Serial);

  // Initialize LowPower library
  LowPower.begin();

  // Get the boot cause
  bootcause_e bc = LowPower.bootCause();

  if ((bc == POR_SUPPLY) || (bc == POR_NORMAL)) 
  {
    Serial.println("wakeup RTC timer from sleep");
  } 
  else 
  {
    Serial.println("wakeup from cold sleep");
    tpmMeasure();
  }

  // Print the boot cause
  printBootCause(bc);
 
  // Print the current clock
  RTC.begin();
  RtcTime now = RTC.getTime();
  printf("%02d:%02d:%02d\n",
          now.hour(), now.minute(), now.second());

  // Button pin setting
  pinMode(button2, INPUT_PULLUP);
  attachInterrupt(button2, pushed2, FALLING);

  delay(10);

  // Enable wakeup by pushing button2
  LowPower.enableBootCause(button2);
  //Serial.println("Wait until PIN_D02 is low.");
  Serial.println("Wait until wakeup tme is reaching.");

  // Cold sleep
  Serial.print("Go to cold sleep...\n");

  // wf931 go to deep sleep
  wf931Sleep();
  
  // Go to deep sleep during about <wakeupTime> seconds
  //LowPower.deepSleep(wakeupTime - CorrectionValue);
  LowPower.coldSleep(wakeupTime - CorrectionValue);
  
}

//************************ main loop *******************************
void loop() 
{
}
