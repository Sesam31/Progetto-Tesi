#include <Servo.h>
#include <math.h>
Servo servo[6];

const double pi = 3.141592653;
// range movimento inferiore e superiore
const int inf = 1800;
const int sup = 900;
// microsecondi per avere i bracci dei sevomotori orizzontali
int oriz[6] = {1500,1550,1500,1560,1560,1500};
// relazione tra microsecondi e radianti [us/rad]
int usrad = (400/45)*(360/(2*pi));                        //CALCOLARE ANGOLO SPERIMENTALE
// posizione richiesta {x,y,z,roll,pitch,yaw}
double pos[6] = {0,0,0,0,0,0};

// VETTORI DI SISTEMA
// matrice rotazione globale
double Rg[3][3];   
// vettore posizione {x,y,z} inizializzato con altezza riposo
double T[3][1] = {0,0,114.5};                                                                          // double T[3][1] = {0,0,114.5}; 
// coordinate giunto biella/manovella al variare di alfa e beta (inizializzata durante il setup)
double giun[6][3];
// vettori origine base -> giunto piattaforma
double baseVec[6][3];
// vettori asse servo -> giunto piattaforma
double armVec[6][3];

//DESCRIZIONE TOPOLOGICA SISTEMA
double alfa0[6];
// angolo manovella rispetto al piano xy
double alfa[6] = {0,0,0,0,0,0};
// angolo manovella rispetto all'asse x
double beta[6] = {-pi/2,pi/6,pi/6,5*pi/6,5*pi/6,pi/2};
// lunghezza manovella
double lungman = 16;
// lunghezza biella
double lungbie = 124;

//angolo asse rotazione servo rispetto a origine base
double gamma = (13.38*2*pi)/360;
// distanza tra centro riferimento e asse rotazione servo [mm]
double rd = 87.63;    // rotation distance (base)
// coordinate punti base
double base[6][3] = {{-rd*cos(gamma),-rd*sin(gamma),0},
                    {rd*cos(pi/3+gamma),-rd*sin(pi/3+gamma),0},
                    {rd*cos(pi/3-gamma),-rd*sin(pi/3-gamma),0},
                    {rd*cos(pi/3-gamma),rd*sin(pi/3-gamma),0},
                    {rd*cos(pi/3+gamma),rd*sin(pi/3+gamma),0},
                    {-rd*cos(gamma),rd*sin(gamma),0}};

//angolo asse giunto rispetto origine piattaforma
double tau = (4.14*2*pi)/360;
double pd = 80.52;    // pivot distance (piattaforma)
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
double m2 = pow(lungman,2);
// distanza^2 manovella
double b2 = pow(lungbie,2);

// PARAMETRI EQUAZIONE ANGOLO
double L[6],M[6],N[6],angServo[6];

void setup() {
  Serial.begin(9600);
  
  servo[0].attach(3, inf, sup);
  servo[1].attach(5, inf, sup);
  servo[2].attach(6, inf, sup);
  servo[3].attach(9, inf, sup);
  servo[4].attach(10, inf, sup);
  servo[5].attach(11, inf, sup);

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

  for(int j=0; j < 6; j++){
      Serial.print(angServo[j]);
      Serial.print(" ");
   }
    
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
    baseVec[i][2] = tempSum[2][0];
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

double getAmpImp(double angolo, int n){      //RIVEDERE VALORI DI ZERO
  double pw;
  if(n % 2){
    return pw = oriz[1]-(alfa-alfa0)*usrad;
  }
  else{
    return pw = oriz[0]+(alfa-alfa0)*usrad;
  }
}
