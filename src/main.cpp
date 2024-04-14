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
byte I2C_Master_ReadByte();
void I2C_Master_SendACK();
void I2C_Master_Stop();
void I2C_Master_WriteByte(byte data);
uint8_t I2C_Master_ReadACK();
int8_t I2C_Master_ReadData(byte address, byte *data,byte count);

void setup(){
  delay(100);
  Serial.begin(9600);
}

void loop() {
  byte rev[10]={0};
  int8_t res;
  res = I2C_Master_ReadData(0x55,rev,4);
  if(res==1)
  {
    Serial.println("read oke");
    Serial.println(String((char*)rev));
  }
  else
  {
    Serial.println("fail to read");
  }
  delay(100);
}

void I2C_Master_Start()
{
  SDA_HIGH;
  SCL_HIGH;
  SDA_OUTPUT;
  SCL_OUTPUT;
  delayMicroseconds(HALF);
  SDA_LOW;
  delayMicroseconds(HALF);
  SCL_LOW;
  delayMicroseconds(HALF);
}

void I2C_Master_Stop()
{
  SCL_LOW;
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

byte I2C_Master_ReadByte()
{
  byte data=0;
  SDA_INPUT;
  SCL_OUTPUT;
  SCL_LOW;
  for(int i=0;i<8;i++)
    {
      delayMicroseconds(HALF);
      SCL_HIGH;
      data = (data<<1) | SDA_READ;
      delayMicroseconds(HALF);
      SCL_LOW;
    }
    return data;
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

void I2C_Master_SendACK()
{
  SCL_LOW;
  SDA_OUTPUT;
  SDA_LOW; //SEND ACK
  delayMicroseconds(HALF);
  SCL_HIGH;
  delayMicroseconds(HALF);
  SCL_LOW;
  SDA_INPUT;
  delayMicroseconds(HALF);
}

int8_t I2C_Master_ReadData(byte address, byte *data,byte count)
{
  int8_t ACK;
  I2C_Master_Start();
  address = (address << 1) + 1;
  I2C_Master_WriteByte(address);
  ACK=I2C_Master_ReadACK();
  if(ACK==1)
  {
    I2C_Master_Stop();
    return -1;
  }
  for(int i=0;i<count;i++)
  {
    *data = I2C_Master_ReadByte();
    data++;
    I2C_Master_SendACK();
  }
  I2C_Master_Stop();
}



