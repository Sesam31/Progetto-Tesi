#include <Servo.h>
#include <math.h>
Servo servo[6];

const double pi = 3.141592653;
// range movimento inferiore e superiore
const int inf = 900;
const int sup = 2000;
// microsecondi per avere la manovella dei sevomotori orizzontali
const int oriz[6] = {1500,1550,1500,1560,1560,1500};
// microsecondi per avere la manovella dei servomotori in posizione di calibrazione
//const int cali[6] = {1325,1375,1325,1385,1385,1385};
// relazione tra microsecondi e radianti [us/rad]
const int usrad = (375/45)*(360/(2*pi));                        
// posizione richiesta {x,y,z,roll,pitch,yaw}

// VETTORI DI SISTEMA
// matrice rotazione globale
double Rg[3][3];   
// altezza piattaforma a riposo
double h0 = 106.5;                                                // ALTEZZA OTTENUTA SPERIMENTALMENTE, TEST: ROTAZIONE ASSE X 5°, DEVE ESSERE SIMMETRICO!!!
// vettore posizione {x,y,z} inizializzato con altezza riposo
double T[3][1] = {0,0,h0};
// vettore rotazione {roll,pitch,yaw}
double R[3][1] = {0,0,0};
// coordinate giunto biella/manovella al variare di alfa e beta (inizializzata durante il setup)
double giun[6][3];
// vettori origine base -> giunto piattaforma
double baseVec[6][3];
// vettori asse servo -> giunto piattaforma
double armVec[6][3];

//DESCRIZIONE TOPOLOGICA SISTEMA
const double alfa0[6];
// angolo manovella rispetto al piano xy
//double alfa[6];
double alfa[6] = {-0.34,-0.34,-0.34,-0.34,-0.34,-0.34};
// angolo manovella rispetto all'asse x
const double beta[6] = {-pi/2,pi/6,pi/6,5*pi/6,5*pi/6,pi/2};
// lunghezza manovella
const double lungman = 16;
// lunghezza biella
const double lungbie = 124;

//angolo asse rotazione servo rispetto a origine base
const double gamma = (13.38*2*pi)/360;
// distanza tra centro riferimento e asse rotazione servo [mm]
const double rd = 87.63;    // rotation distance (base)
// coordinate punti base
double base[6][3] = {{-rd*cos(gamma),-rd*sin(gamma),0},
                    {rd*cos(pi/3+gamma),-rd*sin(pi/3+gamma),0},
                    {rd*cos(pi/3-gamma),-rd*sin(pi/3-gamma),0},
                    {rd*cos(pi/3-gamma),rd*sin(pi/3-gamma),0},
                    {rd*cos(pi/3+gamma),rd*sin(pi/3+gamma),0},
                    {-rd*cos(gamma),rd*sin(gamma),0}};

//angolo asse giunto rispetto origine piattaforma
const double tau = (4.14*2*pi)/360;
const double pd = 80.52;    // pivot distance (piattaforma)
//coordinate punti piattaforma (inizializzato con altezza homing)
double piat[6][3] = {{pd*cos(2*pi/3+tau),-pd*sin(2*pi/3+tau),T[2][0]},
                    {pd*cos(2*pi/3-tau),-pd*sin(2*pi/3-tau),T[2][0]},
                    {pd*cos(tau),-pd*sin(tau),T[2][0]},
                    {pd*cos(tau),pd*sin(tau),T[2][0]},                        
                    {pd*cos(2*pi/3-tau),pd*sin(2*pi/3-tau),T[2][0]},
                    {pd*cos(2*pi/3+tau),pd*sin(2*pi/3+tau),T[2][0]}};

// distanza^2 tra asse e giunto piattaforma
double d2[6];
// distanza^2 manovella
const double m2 = pow(lungman,2);
// distanza^2 manovella
const double b2 = pow(lungbie,2);

// PARAMETRI EQUAZIONE ANGOLO
double L[6],M[6],N[6],angServo[6];

void setup() {
  Serial.begin(9600);

  // LED usato in caso di emergenza
  pinMode(LED_BUILTIN, OUTPUT);
  
  servo[0].attach(3, inf, sup);
  servo[1].attach(5, inf, sup);
  servo[2].attach(6, inf, sup);
  servo[3].attach(9, inf, sup);
  servo[4].attach(10, inf, sup);
  servo[5].attach(11, inf, sup);

  
  while(true){
    for(float t = 0; t < 2*pi; t+=0.2){
    //setPosition(10*cos(t),10*sin(t),115+5*sin(t),radians(0),radians(0),radians(0));
    setPosition(10*cos(t),10*sin(t),110+3*sin(t),radians(0),radians(0),radians(0));
    Serial.println("valore sin");
    Serial.println(115+5*sin(t));   

    }
  }
  

  
  /*
  /for(int i = 0; i < 6; i++){
    servo[i].writeMicroseconds(constrain(getAmpImp(alfa[i],i),inf,sup));
    //Serial.println(getAmpImp(alfa[i],i));
  }
  delay(1000);
  for(int n = 0; n < 6; n++){
    alfa[n]=0;
  }
  for(int i = 0; i < 6; i++){
    servo[i].writeMicroseconds(getAmpImp(alfa[i],i));
  }
  
  delay(1000);
  pos[3] = radians(5);
  
  //inizializza matrice posizione giunto biella/manovella
  for(int n = 0; n < 6; n++){
    setJLoc(alfa[n],beta[n],n);
  }
  setMRot(pos[3],pos[4],pos[5]);
  
  getBaseVec();
  
  getArmVec();
  
  for(int i=0; i < 6; i++){
    for(int j=0; j < 3; j++){
      Serial.print(armVec[i][j]);
      Serial.print(" ");
    }
    Serial.println(" ");
  }
  
  for(int i = 0; i < 6; i++){
    double inizio[3][1];
    double fine[3][1];
    getRow(inizio,baseVec,i);
    getRow(fine,base,i);
    d2[i] = quadVec(inizio,fine);
    //Serial.print(d2[i]);
    //Serial.println(" ");
  }
  for(int i=0; i < 6; i++){
    for(int j=0; j < 3; j++){
      Serial.print(piat[i][j]);
      Serial.print(" ");
    }
    Serial.println(" ");
  }
  Serial.println("----------------");
  for(int i=0; i < 6; i++){
    for(int j=0; j < 3; j++){
      Serial.print(base[i][j]);
      Serial.print(" ");
    }
    Serial.println(" ");
  }
  Serial.println("Angoli servo:");
  for(int i = 0; i < 6; i++){
    L[i] = d2[i]-(b2-m2);
    M[i] = 2*lungman*(piat[i][2]-base[i][2]);
    N[i] = 2*lungman*(cos(beta[i]*(piat[i][0]-base[i][0])+sin(beta[i])*(piat[i][1]-base[i][1])));
    alfa[i]=asin(L[i]/sqrt(pow(M[i],2)+pow(N[i],2)))-atan(N[i]/M[i]);
    
    Serial.println(alfa[i]);
  }
  for(int i = 0; i < 6; i++){
    servo[i].writeMicroseconds(getAmpImp(alfa[i],i));
    //Serial.println(getAmpImp(alfa[i],i));
  }

  */

  
  /*

  //inizializza matrice posizione giunto biella/manovella
  for(int n = 0; n < 6; n++){
    setJLoc(alfa[n],beta[n],n);
  }
  
  setMRot(0,0,0);
  getBaseVec();
  getArmVec();

  // cacola distanza^2 tra asse servo e giunto piattaforma
  for(int i = 0; i < 6; i++){
    double inizio[3][1];
    double fine[3][1];
    getRow(inizio,piat,i);
    getRow(fine,base,i);
    d2[i] = quadVec(inizio,fine);
  }

  // inizializza valori L
  for(int i = 0; i < 6; i++){
    L[i] = d2[i]-(b2-m2);
    M[i] = 2*lungman*(piat[i][2]-base[i][2]);
    N[i] = 2*lungman*(cos(beta[i]*(piat[i][0]-base[i][0])+sin(beta[i])*(piat[i][1]-base[i][1])));
    angServo[i]=asin(L[i]/sqrt(pow(M[i],2)+pow(N[i],2)))-atan(N[i]/M[i]);
  }
  
  */


  // FUNZIONI UTILI
  
  /*for(int j=0; j < 6; j++){
      Serial.print(angServo[j]);
      Serial.print(" ");
   }*/

  // Serial.println(sqrt(b2+m2-pow((piat[0][0]-base[0][0]),2)-pow((piat[0][1]-base[0][1]),2)));     // RIVEDERE EQ H0
    
  /*for(int i=0; i < 6; i++){
    for(int j=0; j < 3; j++){
      Serial.print(armVec[i][j]);
      Serial.print(" ");
    }
    Serial.println(" ");
  }*/

  /*int a[3][3] = {{2,3,1}, {4,2,1}, {3,1,5} };
  int b[3][1] = {1,2,3};
  int c[3][1];

  mult(a,b,c);
   for(int i=0; i < 3; i++){
    for(int j=0; j < 1; j++){
      Serial.print(c[i][j]);
      Serial.print(" ");
    }
    Serial.println(" ");
  }*/
}

void loop() {
  

}

// costruisci matrice rotazione globale
void setMRot(double rol,double pit,double yaw){
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
void setJLoc(double alfa, double beta, int n){      // alfa: angolo rispetto al piano xy. beta: angolo rispetto all'asse x.
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
void getMult(double a[3][3], double b[3][1], double c[3][1]){
  for(int i=0; i < 3; i++){
    c[i][0] = 0;
    for (int j = 0; j < 3; j++){
      c[i][0] += a[i][j]*b[j][0];
    }
  }
}

// calcola somma matrici
void getSum(double a[3][1], double b[3][1], double c[3][1]){
  for(int i=0; i < 3; i++){
    c[i][0] = a[i][0]+b[i][0];
  }
}

// estrae una riga dalla matrice e la converte in colonna per semplificare moltiplicazione
void getRow(double a[3][1], double b[6][3], int n){
  for(int i = 0; i < 3; i++){
    a[i][0] = b[n][i];
  }
}

// costruisce matrice vettori origine base -> giunto piattaforma
void getBaseVec(){
  double tempMult[3][1];
  double tempSum[3][1];
  double piatVec[3][1];
  for(int i = 0; i < 6; i++){
    getRow(piatVec, piat, i);
    getMult(Rg,piatVec,tempMult);
    getSum(T,tempMult,tempSum);
    baseVec[i][0] = tempSum[0][0];
    baseVec[i][1] = tempSum[1][0];
    baseVec[i][2] = tempSum[2][0]-h0;
    Serial.println("altezza");
    Serial.println(tempSum[2][0]);
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
double quadVec(double inizio[3][1], double fine[3][1]){
  double v2;
  return v2 = pow((inizio[0][0]-fine[0][0]),2)+
              pow((inizio[1][0]-fine[1][0]),2)+
              pow((inizio[2][0]-fine[2][0]),2);
}

// calcola ampiezza impulso per ottenere l'angolo richiesto
double getAmpImp(double angolo, int n){     
  double pw;
  if(n % 2){
    return pw = oriz[n]-(alfa[n]-alfa0[n])*usrad;
  }
  else{
    return pw = oriz[n]+(alfa[n]-alfa0[n])*usrad;
  }
}

// svolge i dovuti calcoli e imposta l'angolo dei servomotori
void setPosition(double x, double y, double z, double rol, double pit, double yaw){
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
  /*for(int i=0; i < 6; i++){
    for(int j=0; j < 3; j++){
      Serial.print(armVec[i][j]);
      Serial.print(" ");
    }
    Serial.println(" ");
  }*/
  // calcola distanza^2 tra asse e giunto piattaforma
  for(int i = 0; i < 6; i++){
    double inizio[3][1];
    double fine[3][1];
    getRow(inizio,baseVec,i);
    getRow(fine,base,i);
    d2[i] = quadVec(inizio,fine);
  }
  // calcolo angoli servomotori
  Serial.println("Angoli servo:");
  for(int i = 0; i < 6; i++){
    L[i] = d2[i]-(b2-m2);
    M[i] = 2*lungman*(piat[i][2]-base[i][2]);
    N[i] = 2*lungman*(cos(beta[i]*(piat[i][0]-base[i][0])+sin(beta[i])*(piat[i][1]-base[i][1])));
    alfa[i]=asin(L[i]/sqrt(pow(M[i],2)+pow(N[i],2)))-atan(N[i]/M[i]);
    Serial.println(alfa[i]);
  }
  // controllo microsecondi se ho raggiunto il limite
  int emergenza = 0;
  for(int i = 0; i < 6; i++){
    //Serial.println(constrain(getAmpImp(alfa[i],i),inf,sup));
    if(constrain(getAmpImp(alfa[i],i),inf,sup) == sup || constrain(getAmpImp(alfa[i],i),inf,sup) == inf){
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
