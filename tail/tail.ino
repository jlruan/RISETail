#include <Wire.h>
#include "wiring_private.h"

// base motors
#include <SCServo.h>
SMS_STS leftPivot;
SMS_STS rightPivot;
Uart Serial0 (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// fan pivot motor
#define MotFwd  8  // Motor Forward pin
#define MotRev  7 // Motor Reverse pin
#define encoderPin1 2 //Encoder Output 'A' must connected with intreput pin of arduino.
#define encoderPin2 3 //Encoder Otput 'B' must connected with intreput pin of arduino.

volatile int lastEncoded = 0; // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value

void SERCOM0_Handler()
{
    Serial0.IrqHandler();
}

void setup()
{
  // base
  pinPeripheral(5, PIO_SERCOM_ALT);
  pinPeripheral(6, PIO_SERCOM_ALT);

  Serial0.begin(1000000);
  Serial1.begin(1000000);
  leftPivot.pSerial = &Serial0;
  rightPivot.pSerial = &Serial1;
  delay(1000);

  // pivot
  pinMode(MotFwd, OUTPUT); 
  pinMode(MotRev, OUTPUT); 
  Serial.begin(9600); //initialize serial comunication

  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);
}

void loop()
{
  // base
  rightPivot.WritePosEx(2, 4095, 400, 10);//舵机(ID1)以最高速度V=2400步/秒，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
  leftPivot.WritePosEx(1, 4095, 400, 10);//舵机(ID1)以最高速度V=2400步/秒，加速度A=50(50*100步/秒^2)，运行至P1=4095位置
  delay(2240);//[(P1-P0)/V]*1000+[V/(A*100)]*1000

  // pivot
  for (int i = 0; i <= 500; i++){
    digitalWrite(MotFwd, LOW); 
    digitalWrite(MotRev, HIGH);
    Serial.print("Forward  ");
    Serial.println(encoderValue);
  }
  delay(1000);
  for (int i = 0; i <= 500; i++){
    digitalWrite(MotFwd, HIGH); 
    digitalWrite(MotRev, LOW);
    Serial.print("Reverse  ");
    Serial.println(encoderValue);
  }

  delay(1000);
}

// pivot encoder updates
void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue ++;

  lastEncoded = encoded; //store this value for next time

}

