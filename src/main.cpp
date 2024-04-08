#include <Arduino.h>
#define SDA_OUTPUT (DDRD |= (1 << DDB4))
#define SCL_OUTPUT (DDRD |= (1 << DDB3))
#define SDA_INPUT  (DDRD &= ~(1 << DDB4))
#define SCL_INPUT  (DDRD &= ~(1 << DDB3))
#define SDA_HIGH   (PORTD |=  (1 << PD4))
#define SCL_HIGH   (PORTD |=  (1 << PD3))
#define SDA_LOW    (PORTD &= ~(1 << PD4))
#define SCL_LOW    (PORTD &= ~(1 << PD3))
#define SDA_READ   ((PIND & (1 << PIND4)) >> PIND4)
#define SCL_READ   ((PIND & (1 << PIND3)) >> PIND3)
#define HALF 25
#define FULL (HALF * 2)

void I2C_Master_Start();
void I2C_Master_Stop();
uint8_t I2C_Master_ReadACK();
void I2C_Master_WriteByte(byte data);
int8_t I2C_Master_WriteData(byte address, char *data);

void setup(){
  delay(100);
  Serial.begin(9600);
}

void loop() {
  int8_t result;
  char data[]="abcd";
  result = I2C_Master_WriteData(0X55,data);
  if(result==1)   Serial.println("Master want to write data");
  else if(result==-1) Serial.println("Master write fail");
  delay(100);
}

void I2C_Master_Start() {
  SDA_HIGH;
  SCL_HIGH;
  SDA_OUTPUT;
  SCL_OUTPUT;
  delayMicroseconds(FULL);
  SDA_LOW;
  delayMicroseconds(HALF);
  SCL_LOW;
  delayMicroseconds(HALF);
}

void I2C_Master_Stop() {
  SDA_OUTPUT;
  SDA_LOW;
  delayMicroseconds(HALF);
  SCL_HIGH;
  delayMicroseconds(HALF);
  SDA_HIGH;
}

void I2C_Master_WriteByte(byte data) {
  SDA_OUTPUT;
  for (int i = 0; i < 8; i++) {
    if ((data & 0x80) != 0) {
      SDA_HIGH;
    } else {  SDA_LOW; }
    data = data << 1;
    delayMicroseconds(HALF);	
	 SCL_HIGH;
    delayMicroseconds(HALF);	
	 SCL_LOW;
  }
}

uint8_t I2C_Master_ReadACK() {
  uint8_t ACK;
  SDA_INPUT;
  delayMicroseconds(HALF);
  SCL_HIGH;
  ACK= SDA_READ; 
  delayMicroseconds(HALF);
  SCL_LOW;
  delayMicroseconds(HALF);
  return ACK;
}

int8_t I2C_Master_WriteData(byte address, char *data) { 
  int8_t ACK;
  I2C_Master_Start();
  address = address << 1;
  I2C_Master_WriteByte(address);
  ACK = I2C_Master_ReadACK();
  if (ACK == 1) {
    I2C_Master_Stop();
    return -1;
  }
  while (*data != '\0') {
    I2C_Master_WriteByte(*data);
    ACK = I2C_Master_ReadACK();
    if (ACK == 1) {
      I2C_Master_Stop();
      return -1;
    }
    data++;
  }
  I2C_Master_Stop();
  return 1;
}
