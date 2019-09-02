/*
 *  wf931.cpp - WF931(Sigfox Module) Library
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

#include <stdio.h>
#include <arduino.h>

#include "wf931.h"

//****************************** wf931 UART 返信応答確認 ****************************
bool wf931Class::result()
{
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
void wf931Class::begin()
{
  // ********** WF931 リセット、ID取得 ***************
  // Configure baud rate to 9600
  Serial2.begin(9600);

  // Configure reset pin
  pinMode(PIN_D26, OUTPUT);

  // Do reset
  digitalWrite(PIN_D26, LOW);
  usleep(100*1000);

  // Release reset
  digitalWrite(PIN_D26, HIGH);
  usleep(100*1000);
  result();

  // Get chip ID
  printf("ID = ");
  Serial2.write("AT$ID?\r");
  usleep(100*1000);
  result();
}

//******************* WF931AT$SB=1の実行 ************************************
void wf931Class::wakeup()
{
  // WF931 wakeup.
  Serial2.write("0x00");
  usleep(60*1000);
  
  // Send state by WF931
  Serial2.write("AT$SB=1\r");
  printf("Send State = 1\n");
  usleep(100*1000);
  result();
}

//**************** WF931をDeep Sleep状態にセット *******************************
void wf931Class::sleep()
{
  // WF931 wakeup.
  Serial2.write("0x00");
  usleep(60*1000);
  
  // Send deep sleep command to WF931
  Serial2.write("AT$DSLEEP\r");
  //printf("Send Deep Sleep command to WF931\n");
  usleep(100*1000);
  result();
}

//********************** DataをSigfox送信する　*********************************
void wf931Class::send(int8_t i0, int8_t i1, int8_t i2, int8_t i3, int8_t i4, int8_t i5, int8_t i6, int8_t i7, int8_t i8, int8_t i9, int8_t i10, int8_t i11)
{
  String output2;
  char val[24];
  // wakeup.
  Serial2.write("0x00");
  usleep(60*1000);

  // Send Data  
  sprintf(val,"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", i0, i1, i2, i3, i4, i5, i6, i7, i8, i9, i10, i11);
  output2 = String(val);
  Serial2.write("AT$SF=");
  Serial2.write(val);
  Serial2.write("\r");
  
  usleep(100*1000);
  //result();
   
  Serial.print("SendData = ");
  Serial.println(output2);  
}

//********************** DataをSigfox送信する　*********************************
void wf931Class::send(int16_t i0, int16_t i1, int16_t i2, int16_t i3, int16_t i4, int16_t i5)
{
  String output2;
  char val[24];
  // wakeup.
  Serial2.write("0x00");
  usleep(60*1000);

  // Send Data  
  sprintf(val,"%04x%04x%04x%04x%04x%04x", i0, i1, i2, i3, i4, i5);
  output2 = String(val);
  Serial2.write("AT$SF=");
  Serial2.write(val);
  Serial2.write("\r");
  
  usleep(100*1000);
  //result();
   
  Serial.print("SendData = ");
  Serial.println(output2);  
}

//********************** DataをSigfox送信する　*********************************
void wf931Class::send(float f0,float f1,float f2)
{
  union {float f; int i;} data;
  String output2;

  // wakeup.
  Serial2.write("0x00");
  usleep(60*1000);

  puts("send");
  Serial2.write("AT$SF=");
  data.f = f0;
  output2 = String(data.i, HEX);
  data.f = f1;
  output2 += String(data.i, HEX);
  data.f = f2;
  output2 += String(data.i, HEX);
  Serial2.println(output2);

  usleep(100*1000);
//  result();
  Serial.print("SendData = ");
  Serial.println(output2);  
}


wf931Class wf931;

