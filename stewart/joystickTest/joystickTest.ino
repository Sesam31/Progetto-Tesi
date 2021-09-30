/* ground marrone
 * 5v rosso
 * switch verde?
 */
  
int Xin= A0; // X Input Pin   
int Yin = A1; // Y Input Pin
//int KEYin = 2; // Push Button
void setup ()
{
 //pinMode (KEYin, INPUT);
 Serial.begin (9600);
}
void loop ()
{
 int xVal, yVal, buttonVal;

 xVal = analogRead (Xin);
 yVal = analogRead (Yin);
 //buttonVal = digitalRead (KEYin);

 Serial.print("X = ");
 Serial.println (xVal, DEC);

 Serial.print ("Y = ");
 Serial.println (yVal, DEC);

 Serial.print("PULSANTE ");
 if (buttonVal == LOW){
 Serial.println(buttonVal);
 Serial.println ("NON PREMUTO");
 }
 else{
 Serial.println ("PREMUTO");
 }

 delay (500);
}
