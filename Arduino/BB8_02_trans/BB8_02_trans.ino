//transmitter

int ch0;
int ch1;
int ch2;
int ch3;
int ch4;
int but1;
int but2;

long previousMillis;
long interval = 10;

void setup()
{
   Serial.begin(38400); 
   pinMode  (2, INPUT_PULLUP);
   pinMode  (3, INPUT_PULLUP);
   
   delay(5000);
}

void loop()
{  
   
  but1 = digitalRead(2);
  but2 = digitalRead(3);
  ch0 = analogRead(A0); 
  ch1 = analogRead(A1); 
  ch2 = analogRead(A2); 
  ch3 = analogRead(A3); 
  ch4 = analogRead(A4);
  
  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis > interval) {
      previousMillis = currentMillis; 
 
      Serial.print (but1);
      Serial.print (",");
      Serial.print (but2);
      Serial.print (",");
      Serial.print (ch0);
      Serial.print (",");
      Serial.print (ch1);
      Serial.print (",");
      Serial.print (ch2);
      Serial.print (",");
      Serial.print (ch3);
      Serial.print (",");
      Serial.print (ch4); 
      Serial.print ("\n");
     
  } 

}




