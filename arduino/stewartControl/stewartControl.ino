#include <Servo.h>
#include <math.h>
Servo servo[6];

const float pi = 3.141592653;
// range movimento inferiore e superiore
const int inf = 900;
const int sup = 2200;
// microsecondi per avere la manovella dei sevomotori orizzontali
const int oriz[6] = {1500,1550,1500,1560,1560,1500};
// microsecondi per avere la manovella dei servomotori in posizione di calibrazione
//const int cali[6] = {1325,1375,1325,1385,1385,1385};
// relazione tra microsecondi e radianti [us/rad]
const int usrad = (375/45)*(360/(2*pi));                        
// posizione richiesta {x,y,z,roll,pitch,yaw}

// VETTORI DI SISTEMA
// matrice rotazione globale
float Rg[3][3];   
// altezza piattaforma a riposo
float h0 = 106.5;                                                // ALTEZZA OTTENUTA SPERIMENTALMENTE, TEST: ROTAZIONE ASSE X 5°, DEVE ESSERE SIMMETRICO!!!
// vettore posizione {x,y,z} inizializzato con altezza riposo
float T[3][1] = {0,0,h0};
// vettore rotazione {roll,pitch,yaw}
float R[3][1] = {0,0,0};
// coordinate giunto biella/manovella al variare di alfa e beta (inizializzata durante il setup)
float giun[6][3];
// vettori origine base -> giunto piattaforma
float baseVec[6][3];
// vettori asse servo -> giunto piattaforma
float armVec[6][3];

//DESCRIZIONE TOPOLOGICA SISTEMA
const float alfa0[6];
// angolo manovella rispetto al piano xy
//float alfa[6];
float alfa[6] = {-0.34,-0.34,-0.34,-0.34,-0.34,-0.34};
// angolo manovella rispetto all'asse x
const float beta[6] = {-pi/2,pi/6,pi/6,5*pi/6,5*pi/6,pi/2};
// lunghezza manovella
const float lungman = 16;
// lunghezza biella
const float lungbie = 124;

//angolo asse rotazione servo rispetto a origine base
const float gamma = (13.38*2*pi)/360;
// distanza tra centro riferimento e asse rotazione servo [mm]
const float rd = 87.63;    // rotation distance (base)
// coordinate punti base
float base[6][3] = {{-rd*cos(gamma),-rd*sin(gamma),0},
                    {rd*cos(pi/3+gamma),-rd*sin(pi/3+gamma),0},
                    {rd*cos(pi/3-gamma),-rd*sin(pi/3-gamma),0},
                    {rd*cos(pi/3-gamma),rd*sin(pi/3-gamma),0},
                    {rd*cos(pi/3+gamma),rd*sin(pi/3+gamma),0},
                    {-rd*cos(gamma),rd*sin(gamma),0}};

//angolo asse giunto rispetto origine piattaforma
const float tau = (4.14*2*pi)/360;
const float pd = 80.52;    // pivot distance (piattaforma)
//coordinate punti piattaforma (inizializzato con altezza homing)
float piat[6][3] = {{pd*cos(2*pi/3+tau),-pd*sin(2*pi/3+tau),T[2][0]},
                    {pd*cos(2*pi/3-tau),-pd*sin(2*pi/3-tau),T[2][0]},
                    {pd*cos(tau),-pd*sin(tau),T[2][0]},
                    {pd*cos(tau),pd*sin(tau),T[2][0]},                        
                    {pd*cos(2*pi/3-tau),pd*sin(2*pi/3-tau),T[2][0]},
                    {pd*cos(2*pi/3+tau),pd*sin(2*pi/3+tau),T[2][0]}};

// distanza^2 tra asse e giunto piattaforma
float d2[6];
// distanza^2 manovella
const float m2 = pow(lungman,2);
// distanza^2 manovella
const float b2 = pow(lungbie,2);

// PARAMETRI EQUAZIONE ANGOLO
float L[6],M[6],N[6],angServo[6];


// PARAMETRI PIANO RESISTIVO
#define sensePin A0
// min 100 max 920, piatto 0.34x0.27m
float x, xOld;
float y, yOld;
float xorig,yorig;
double lastsense;
int on = 50;//25
int pause = 50;//10   Non mettere a 0 per nessun motivo
float sumX5;
float sumY5;
int sumcount = 19;//19
float averageX, averageY, averageXOld, averageYOld;
int count = 10;
double deltaTimeAv, newTimeAv, oldTimeAv;
//tempo inizio cerchio
int circlestart;
float draw;
int quadrante = 1;
bool circle = false;
bool infinity = false;
bool switchState = false;
bool oldclick, newclick;
#define in1 12  
#define in2 13 
#define swi 7

// PARAMETRI PID
float Kp = 25;//25;//30;//13;//13;//22.5;   //22.5 con foglio carta
float Ki = 13;//22;//15;//15;//22;//22;
float Kd = 25;//25;//35;//30;//30;//65;
double oldTime, newTime, deltaTime;
double sumErrX, sumErrY;
// double oldDerivataX, oldDerivataY;8
float setX = 0.173, setY = 0.133, errX, errY; // setpoint centrato 26.95 21.90
float xVel, yVel, xVelOld, yVelOld, xVelAv, yVelAv, xVelOldAv, yVelOldAv, xAccAv, yAccAv;
float xAcc, yAcc;
double pidX, pidY;
float oldDerivataX, oldDerivataY;
//bool pause = false;
char command;

//JOYSTICK
#define Xin A5
#define Yin A4
int joyX, joyY;

void setup() {
  Serial.begin(2000000);      // tenere alto per evitare rallentamento processi

  // LED usato in caso di emergenza
  pinMode(LED_BUILTIN, OUTPUT);

  // pin output controllo piano
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(swi,INPUT_PULLUP);

  // pin controllo servomotori
  servo[0].attach(3, inf, sup);
  servo[1].attach(5, inf, sup);
  servo[2].attach(6, inf, sup);
  servo[3].attach(9, inf, sup);
  servo[4].attach(10, inf, sup);
  servo[5].attach(11, inf, sup);

  // inizializza variabili
  getSense();
  oldTime = millis();
  xOld = x;
  yOld = y;
  
  /*while(true){
    setPosition(0,0,110,radians(0),radians(-5),radians(0)); 
  }*/
  
  /*while(true){
    for(float t = -3; t < 0; t+=0.1){
      setPosition(0,0,110,radians(t),radians(t+3),radians(0)); 
    }
    for(float t = 0; t < 3; t+=0.1){
      setPosition(0,0,110,radians(t),radians(3-t),radians(0)); 
    }
    for(float t = 3; t > 0; t-=0.1){
      setPosition(0,0,110,radians(t),radians(t-3),radians(0)); 
    }
    for(float t = 0; t > -3; t-=0.1){
      setPosition(0,0,110,radians(t),radians(-3-t),radians(0)); 
    }
  }
  */
  /*while(true){
  for(float t = -8; t < 8; t+=0.25){
    setPosition(0,0,110,radians(0),radians(0),radians(t));
    Serial.println(t);
  }
  for(float t = 8; t > -8; t-=0.25){
    setPosition(0,0,110,radians(0),radians(0),radians(t));
    Serial.println(t);
  }
  }*/
  
  /*while(true){
    for(float t = 0; t < 2*pi; t+=0.07){
      setPosition(10*cos(t),10*sin(t),110+5*sin(t),radians(0),radians(0),radians(0)); 
    }
  }*/
  /*while(true){
    setPosition(0,0,106.5,radians(0),radians(0),radians(0));
  }*/
  
}

void loop() {
  // loop esterno ~= 500us = 2kHz
  if(micros() > lastsense + pause){
    xOld = x;
    yOld = y;
  
    xVelOld = xVel;
    yVelOld = yVel;
    
    getSense();

    // calcolo tempo ciclo esterno
    newTime = micros();
    deltaTime = (newTime - oldTime)/1000000;
    oldTime = newTime;
    
    // velocità pallina in m/s
    xVel = (x - xOld)/deltaTime;
    yVel = (y - yOld)/deltaTime;
    
    //xAcc = (xVel - xVelOld)/(deltaTime);
    //yAcc = (yVel - yVelOld)/(deltaTime);
    
    // filto rumore posizione 
    float maxVel = 0.4;//0.45;
    if(abs(xVel) > maxVel){
      if(xVel > 0){
        x = xOld + maxVel*(deltaTime);
      }
      else{
        x = xOld - maxVel*(deltaTime);
      }
    }
    if(abs(yVel) > maxVel){
      if(yVel > 0){
        y = yOld + maxVel*(deltaTime);
      }
      else{
        y = yOld - maxVel*(deltaTime);
      }
    }
    
    if(sumcount > 0){
      sumX5 += x;
      sumY5 += y;
      sumcount--;
    }
    // loop interno ~= 10ms = 100Hz
    else{
      sumcount = 19; //19
      averageXOld = averageX;
      averageYOld = averageY;
      xVelOldAv = xVelAv;
      yVelOldAv = yVelAv;
      averageX = sumX5/sumcount;
      averageY = sumY5/sumcount;
      sumX5 = 0;
      sumY5 = 0;

      // calcola tempo ciclo interno
      newTimeAv = micros();
      deltaTimeAv =(newTimeAv - oldTimeAv)/(1000000);
      oldTimeAv = newTimeAv;

      xVelAv = (averageX-averageXOld)/deltaTimeAv;
      yVelAv = (averageY-averageYOld)/deltaTimeAv;
      
      xAccAv = (xVelAv - xVelOldAv)/deltaTimeAv;
      yAccAv = (yVelAv - yVelOldAv)/deltaTimeAv;

      // filtro posizione
      float maxVel = 0.4;//0.3;
      if(abs(xVelAv) > maxVel){
        if(xVelAv > 0){
          averageX = averageXOld + maxVel*(deltaTimeAv);
        }
        else{
          averageX = averageXOld - maxVel*(deltaTimeAv);
        }
      }
      if(abs(yVelAv) > maxVel){
        if(yVelAv > 0){
          averageY = averageYOld + maxVel*(deltaTimeAv);
        }
        else{
          averageY = averageYOld - maxVel*(deltaTimeAv);
        }
      }
      
      // filtro velocità (derivata)
      float maxAcc = 0.30;//0.20;   // max per 5 gradi
      if(abs(xAccAv) > maxAcc){
        if(xAccAv > 0){
          xVelAv = xVelOldAv + maxAcc*(deltaTimeAv);
        }
        else{
          xVelAv = xVelOldAv - maxAcc*(deltaTimeAv);
        }
      }
      if(abs(yAccAv) > maxAcc){
        if(yAccAv > 0){
          yVelAv = yVelOldAv + maxAcc*(deltaTimeAv);
        }
        else{
          yVelAv = yVelOldAv - maxAcc*(deltaTimeAv);
        }
      }

      runPID();

      joyX = 1023 - analogRead (Xin);
      joyY = 1023 - analogRead (Yin);
      newclick = digitalRead (swi);
      if(oldclick == false && newclick == true){
        if(switchState == true){
          switchState = !switchState;
        }
        else{
          switchState = !switchState;
        }
      }
      oldclick = newclick;

      
      
      if(Serial.available()){
        command = Serial.read();
      }

      runCommand();
      
      // Blocco di I/O 
      // (eseguito una volta ogni "count" cicli per non affollare la seriale)
      count = count - 1;
      if(count <= 0){    
        Serial.print(setX*1000);
        Serial.print(", ");
        Serial.print(setY*1000);
        Serial.print(", ");
        Serial.print(averageX*1000);
        Serial.print(", ");
        Serial.println(averageY*1000);
        count = 0;
      }
    }
  }
}

// costruisci matrice rotazione globale
void setMRot(float rol,float pit,float yaw){
  Rg[0][0] = cos(yaw)*cos(pit);
  Rg[0][1] = -sin(yaw)*cos(rol)+cos(yaw)*sin(pit)*sin(rol);
  Rg[0][2] = sin(yaw)*sin(rol)+cos(yaw)*sin(pit)*cos(rol);

  Rg[1][0] = sin(yaw)*cos(pit);
  Rg[1][1] = cos(yaw)*cos(rol)+sin(yaw)*sin(pit)*sin(rol);
  Rg[1][2] = -cos(yaw)*sin(rol)+sin(yaw)*sin(pit)*cos(rol);

  Rg[2][0] = -sin(pit);
  Rg[2][1] = cos(pit)*sin(rol);
  Rg[2][2] = cos(pit)*cos(rol);
}

// calcola la posizione del giunto biella/manovella
void setJLoc(float alfa, float beta, int n){      // alfa: angolo rispetto al piano xy. beta: angolo rispetto all'asse x.
  if(n % 2){    //n dispari
    giun[n][0] = lungman*cos(alfa)*cos(pi+beta)+base[n][0];     // per il sistema di angoli impiegato non è necessario indicare pi-alfa
    giun[n][1] = lungman*cos(alfa)*sin(pi+beta)+base[n][1];
    giun[n][2] = lungman*sin(alfa)+base[n][2];
  }
  else{
    giun[n][0] = lungman*cos(alfa)*cos(beta)+base[n][0];
    giun[n][1] = lungman*cos(alfa)*sin(beta)+base[n][1];
    giun[n][2] = lungman*sin(alfa)+base[n][2];
  }
}

// calcola moltiplicazione matrici
void getMult(float a[3][3], float b[3][1], float c[3][1]){
  for(int i=0; i < 3; i++){
    c[i][0] = 0;
    for (int j = 0; j < 3; j++){
      c[i][0] += a[i][j]*b[j][0];
    }
  }
}

// calcola somma matrici
void getSum(float a[3][1], float b[3][1], float c[3][1]){
  for(int i=0; i < 3; i++){
    c[i][0] = a[i][0]+b[i][0];
  }
}

// estrae una riga dalla matrice e la converte in colonna per semplificare moltiplicazione
void getRow(float a[3][1], float b[6][3], int n){
  for(int i = 0; i < 3; i++){
    a[i][0] = b[n][i];
  }
}

// costruisce matrice vettori origine base -> giunto piattaforma
void getBaseVec(){
  float tempMult[3][1];
  float tempSum[3][1];
  float piatVec[3][1];
  for(int i = 0; i < 6; i++){
    getRow(piatVec, piat, i);
    getMult(Rg,piatVec,tempMult);
    getSum(T,tempMult,tempSum);
    baseVec[i][0] = tempSum[0][0];
    baseVec[i][1] = tempSum[1][0];
    baseVec[i][2] = tempSum[2][0]-h0;
    /*Serial.println("altezza");
    Serial.println(tempSum[2][0]);*/
  }
}

// costruisce matrice vettori asse servo -> giunto piattaforma
void getArmVec(){
  for(int i=0; i < 6; i++){
    for(int j=0; j < 3; j++){
      armVec[i][j] = baseVec[i][j] - base[i][j];
    }
  }
}

// calcola il valore del quadrato del vettore fornito (come differenza tra inizio e fine)
float quadVec(float inizio[3][1], float fine[3][1]){
  float v2;
  return v2 = pow((inizio[0][0]-fine[0][0]),2)+
              pow((inizio[1][0]-fine[1][0]),2)+
              pow((inizio[2][0]-fine[2][0]),2);
}

// calcola ampiezza impulso per ottenere l'angolo richiesto
float getAmpImp(float angolo, int n){     
  float pw;
  if(n % 2){
    return pw = oriz[n]-(alfa[n]-alfa0[n])*usrad;
  }
  else{
    return pw = oriz[n]+(alfa[n]-alfa0[n])*usrad;
  }
}

// svolge i dovuti calcoli e imposta l'angolo dei servomotori
void setPosition(float x, float y, float z, float rol, float pit, float yaw){
  // imposta posizione desiderata
  T[0][0] = x;
  T[0][1] = y;
  T[0][2] = z;
  R[0][0] = rol;
  R[0][1] = pit;
  R[0][2] = yaw;
  // inizializza matrice posizione giunto biella/manovella
  for(int n = 0; n < 6; n++){
    setJLoc(alfa[n],beta[n],n);
  }
  // inizializza matrice rotazione globale
  setMRot(R[0][0],R[0][1],R[0][2]);
  // inizializza matrice vettori base -> giunto piattaforma
  getBaseVec();
  // inizializza matrice vettori asse -> giunto piattaforma
  getArmVec();
  // calcola distanza^2 tra asse e giunto piattaforma
  for(int i = 0; i < 6; i++){
    float inizio[3][1];
    float fine[3][1];
    getRow(inizio,baseVec,i);
    getRow(fine,base,i);
    d2[i] = quadVec(inizio,fine);
  }
  // calcolo angoli servomotori
  //Serial.println("Angoli servo:");
  for(int i = 0; i < 6; i++){
    L[i] = d2[i]-(b2-m2);
    M[i] = 2*lungman*(piat[i][2]-base[i][2]);
    N[i] = 2*lungman*(cos(beta[i]*(piat[i][0]-base[i][0])+sin(beta[i])*(piat[i][1]-base[i][1])));
    alfa[i]=asin(L[i]/sqrt(pow(M[i],2)+pow(N[i],2)))-atan(N[i]/M[i]);
    //Serial.println(alfa[i]);
  }
  // controllo microsecondi se ho raggiunto il limite
  int emergenza = 0;
  for(int i = 0; i < 6; i++){
    //Serial.println(constrain(getAmpImp(alfa[i],i),inf,sup));
    if(constrain(getAmpImp(alfa[i],i),inf,sup) == sup || constrain(getAmpImp(alfa[i],i),inf,sup) == inf || isnan(constrain(getAmpImp(alfa[i],i),inf,sup))){     // isnan controlla se la computazione è risultata in un valore impossibile.
      emergenza++;
    }
  }
  // assegnazione angolo servomotori
  if(emergenza == 0){
    //Serial.println("Microsecondi:");
    for(int i = 0; i < 6; i++){
        servo[i].writeMicroseconds(constrain(getAmpImp(alfa[i],i),inf,sup));
        //Serial.println(constrain(getAmpImp(alfa[i],i),inf,sup));
    }
  }
  else{
    Serial.println("Limite raggiunto, arresto motori.");
    fermoEmergenza();
  }
  
}

// in caso di emergenza non muoverti, entra in loop infinito lampeggiante allarme
void fermoEmergenza(){
  while(true){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

// ottieni posizione della pallina nel piano in mm
void getSense(){
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  delayMicroseconds(on);
  x = (analogRead(sensePin)-100)*(0.34/820);
  //Serial.print("x orig: "); //segnale ingresso non filtrato
  //Serial.println(x*100);

  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  delayMicroseconds(on);
  y = (analogRead(sensePin)-100)*(0.27/820);
  lastsense = micros();
}

// esegui procedure controllo PID
void runPID(){
  // calcolo errore
  float errXold = errX;
  float errYold = errY;
  errX = setX - averageX;
  errY = setY - averageY;

  // antiwindup
  float antiwindup = 0.08;
  sumErrX += errX * deltaTimeAv;
  sumErrY += errY * deltaTimeAv;
  if(abs(sumErrX) > antiwindup){
    if(sumErrX > 0){
      sumErrX = antiwindup;
    }
    else{
      sumErrX = -antiwindup;
    }
  }
  if(abs(sumErrY) > antiwindup){
    if(sumErrY > 0){
      sumErrY = antiwindup;
    }
    else{
      sumErrY = -antiwindup;
    }
  }

  //componenti PID per i due assi
  float proporzionaleX = Kp * errX;
  float integraleX = Ki * sumErrX;
  float derivativoX = -Kd * xVelAv; // - per il derivative o measurement

  float proporzionaleY = Kp * errY;
  float integraleY = Ki * sumErrY;
  float derivativoY = -Kd * yVelAv;

  pidX = proporzionaleX + integraleX + derivativoX;   // errX-errXold derivata normale x-xOld derivative on measurement
  pidY = proporzionaleY + integraleY + derivativoY;

  // limitazione uscita PID
  // angolo inclinazione massimo
  float maxangle = 6;
  float tiltX;
  float tiltY;
  if(abs(pidY) > maxangle){
    if(-pidY > 0){
      tiltX = maxangle;
    }
    if(-pidY < 0){
      tiltX = -maxangle;
    }
  }
  else{
    tiltX = -pidY;
  }

  if(abs(pidX) > maxangle){
    if(pidX > 0){
      tiltY = maxangle;
    }
    if(pidX < 0){
      tiltY = -maxangle;
    }
  }
  else{
    tiltY = pidX;
  }
  // assegnazione a piattaforma di Stewart
  setPosition(0,0,108,radians(tiltX),radians(tiltY),radians(0));
}

// analizza stato joystick e input, cambia il setpoint
void runCommand(){
  if(joyX < 300 && joyY < 700 && joyY > 300 && switchState == true){ 
    setX = 0.233;//0.173
    setY = 0.133;
  }
  if(joyX > 700 && joyY < 700 && joyY > 300 && switchState == true){ 
    setX = 0.113;//0.17
    setY = 0.133;
  }
  if(joyX > 300 && joyX < 700 &&joyY < 700 && joyY > 300 && switchState == true){ 
    setX = 0.173;//0.17
    setY = 0.133;
  }
  if(joyX > 300 && joyX < 700 && joyY < 300 && switchState == true){ 
    setX = 0.173;//0.17
    setY = 0.193;
  }
  if(joyX > 300 && joyX < 700 && joyY > 700 && switchState == true){        
    setX = 0.173;//0.17
    setY = 0.073;
  }
  
  if(joyX < 300 && joyY < 700 && joyY > 300 && switchState == false){
    circle = true;
    float Kp = 120;//25;//30;//13;//13;//22.5;   //22.5 con foglio carta
    float Ki = 0;//25;//22;//15;//15;//22;//22;
    float Kd = 0;//10;
    setX=0.173+0.05*cos(pi*(millis()-circlestart)/(1500*2));
    setY=0.133+0.04*sin(pi*(millis()-circlestart)/(1500*2));
  }
  if(joyX > 300 && joyX < 700 && joyY < 300 && switchState == false){ //verificare che funzioni disegno retta 45°
    float Kp = 120;//25;//30;//13;//13;//22.5;   //22.5 con foglio carta
    float Ki = 0;//25;//22;//15;//15;//22;//22;
    float Kd = 0;//10;
    setX=0.173+0.05*cos(pi*(millis()-circlestart)/(1500*2));
    setY=0.133+0.04*cos(pi*(millis()-circlestart)/(1500*2));
  }
  if(joyX > 300 && joyX < 700 && joyY > 700 && switchState == false){ //verificare che funzioni disegno u
    float Kp = 120;//25;//30;//13;//13;//22.5;   //22.5 con foglio carta
    float Ki = 0;//25;//22;//15;//15;//22;//22;
    float Kd = 0;//10;
    setX=0.173+0.05*cos(pi*(millis()-circlestart)/(1500*2));
    setY=0.133+0.04*cos(2*pi*(millis()-circlestart)/(1500*2));
  }
  if(joyX > 700 && joyY < 700 && joyY > 300 && switchState == false){ 
    infinity = true;
    if(draw < 0.05){
      draw += 0.0006;//0.007
    }
    else{
      draw = 0;
      quadrante++;
      if(quadrante == 5){
        quadrante = 1;
      }
    }
    if(quadrante == 1){
      setX = 0.173 + draw;
      setY = 0.133 + 7*draw*sqrt(0.06-draw);
    }
    if(quadrante == 2){
      setX = 0.173 + (0.06 - draw);
      setY = 0.133 - 7*(0.06 - draw)*sqrt(0.06-(0.06 - draw));
    }
    if(quadrante == 3){
      setX = 0.173 - draw;
      setY = 0.133 + 7*draw*sqrt(0.06-draw);
    }
    if(quadrante == 4){
      setX = 0.173 - (0.06 - draw);
      setY = 0.133 - 7*(0.06 - draw)*sqrt(0.06- (0.06 - draw));
    }
    
  }

  if(joyX > 300 && joyX < 700 &&joyY < 700 && joyY > 300 && switchState == false){ 
    setX = 0.173;
    setY = 0.133;
    circle = false;
    infinity = false;
    draw = 0;
    quadrante = 1;
    float Kp = 25;//25;//30;//13;//13;//22.5;   //22.5 con foglio carta
    float Ki = 13;//22;//15;//15;//22;//22;
    float Kd = 25;//25;//35;//30;//30;//65;
  }

  if(command == 'p'){
    pause = true;
    while(pause){
      setPosition(0,0,110,radians(0),radians(0),radians(0));
      delay(500);
      if(Serial.read() == 'p'){
        pause = false;
      }
    }
  }
}
