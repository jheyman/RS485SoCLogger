const byte ENABLE_PIN = 9;
const byte BUTTON_PIN = 10;
const byte TRIG_PIN = 12;
const byte OUTPUT_PIN = 7;
const byte LED_PIN = 13;

#define ERROR_WRONG_DATA_EVERY 100
#define ERROR_SHORT_DATA_EVERY 250
char sendBuf[120];
char sendWrongBuf[120];
char sendTooShortBuf[60];
unsigned long loop_index=0;

void setup()
{
  Serial.begin(1000000);
  pinMode (ENABLE_PIN, OUTPUT);  // driver output enable
  pinMode (LED_PIN, OUTPUT);  // built-in LED
  pinMode (OUTPUT_PIN, OUTPUT);  // slave trigger output
  digitalWrite (OUTPUT_PIN, HIGH);  // slave trigger disabled at first
  pinMode (BUTTON_PIN, INPUT_PULLUP);  // button
  pinMode (TRIG_PIN, INPUT_PULLUP);  // trig input from master

  for (int i=0; i<120;i++)
  {
    sendBuf[i] = i;
    sendWrongBuf[i] = i;
    if (i < 60)
      sendTooShortBuf[i]=i;
  }

  sendWrongBuf[30] = 255;
} 
  
void loop()
{

  int trig, button;
  do {
    button = digitalRead(BUTTON_PIN);
    trig = digitalRead(TRIG_PIN);
  }
  while(trig != LOW && button != LOW);
 
  digitalWrite (OUTPUT_PIN, LOW);  // enable trigger to slave arduino
  digitalWrite (ENABLE_PIN, HIGH);  // enable sending
  digitalWrite (LED_PIN, HIGH);  // flash LED 

/*
  if (loop_index % ERROR_WRONG_DATA_EVERY ==0) {
    Serial.write(&sendWrongBuf[0], 60);
    delayMicroseconds(100);
    Serial.write(&sendWrongBuf[60], 60);    
  } else if (loop_index % ERROR_SHORT_DATA_EVERY ==0) {
    Serial.write(&sendTooShortBuf[0], 60);
  }
  else {
    Serial.write(&sendBuf[0], 60);
    delayMicroseconds(100);
    Serial.write(&sendBuf[60], 60);
  }
*/

  //Serial.write(&sendBuf[0], 60);
 // Serial.write(loop_index%256);

  //sendBuf[0]= loop_index%256;
  //Serial.write(&sendBuf[0],loop_index);
  
    int temp = loop_index%256;
   //Serial.write(loop_index%256);

   for(int i=0; i < 8;i++)
   {
    Serial.write(temp);
   }
   //delay(80);
   for(int i=0; i < 8;i++)
   {
    Serial.write(temp+1);
   }
   //delay(80);
   for(int i=0; i < 8;i++)
   {
    Serial.write(temp+2);
   }
   //delay(80);
      for(int i=0; i < 8;i++)
   {
    Serial.write(temp+3);
   }  
   /* 
   for(int i=0; i < 4094;i++)
   {
    //Serial.write(i%256);
    Serial.write(temp);
   }
*/

  // wait 1ms before release the enable: the last chars might still be in the RX buffer and not yet sent.
  delay(1);

  digitalWrite (ENABLE_PIN, LOW);  // disable sending

  // delay small enough to fit tested message rate, but long enough that the LED flash is visible
  delay(10);

  digitalWrite (LED_PIN, LOW);  // turn LED back off 




#if 0
delay(123);
 digitalWrite (ENABLE_PIN, HIGH);  // enable sending
  digitalWrite (LED_PIN, HIGH);  // flash LED 

  
   for(int i=0; i < 8;i++)
   {
    Serial.write(temp+4);
   }
   //delay(80);
   for(int i=0; i < 7;i++)
   {
    Serial.write(temp+5);
   }

  // wait 1ms before release the enable: the last chars might still be in the RX buffer and not yet sent.
  delay(1);

  digitalWrite (ENABLE_PIN, LOW);  // disable sending

  // delay small enough to fit tested message rate, but long enough that the LED flash is visible
  delay(10);

  digitalWrite (LED_PIN, LOW);  // turn LED back off 

#endif



  digitalWrite (OUTPUT_PIN, HIGH);  // reset trigger to slave arduino






  // the rest of the delay is such that the sum of the three delays = the desired message period.
  delay(989);

  loop_index+=4;
}

