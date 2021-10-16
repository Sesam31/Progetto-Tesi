// PARAMETRI PIANO RESISTIVO
#define sensePin A0
int x,y;
#define in1 12 
#define in2 13

void setup() {
  Serial.begin(2000000);      // tenere alto per evitare rallentamento processi

  // pin output controllo piano
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
}
void loop() {
  //Serial.print("ciao");
  getSense();
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.println(" ");
  delay(50);
}

void getSense(){
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  delay(10);
  x = analogRead(sensePin);

  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);

  delay(10);
  y = analogRead(sensePin);
}
