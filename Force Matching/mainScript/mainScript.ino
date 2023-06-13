#include <LiquidCrystal.h>

//writing to txt
import processing.serial.*
Serial myData;
PrintWriter output; 

//rand num generation
long randNumber;
int FSS = 13;

//configurations for the PWM
const int freq = 53600; //motor controler requires 53.6 kHz
const int resolution = 12; //mototr controler runs at 12 bit resolution (0 tp 4096)
const int duty cycle;
const long r = 0.1 //length of momnet arm in m
const int CurrentLimit 

// Force max and min for rand vlaue generator 
const long FMin = 0;
const long FMax = 501;
const long FOUT,FIN;

//motor constants
const long torqurConstant = 110;


void setup() {
  // serial monitor 
  Serial.begin(9600);
  
  // Motor pin I/O
  const int currnetLimit = 14, currentMonitor = 19, EnableAndDirection= 15, H1=16, H2=17, H3=18, ;

  pinMode(H1, INPUT)
  pinMode(H2, INPUT)
  pinMode(H3, INPUT)
  pinMode(currentMonitor, INPUT)
  pinMode(currnetLimit, OUTPUT)
  pinMode(EnableAndDirection, OUTPUT)

  //pinMode(DCMotor, OUTPUT)o
  //Honywell force sensor
  pinMode(FSS, INPUT);




  //LCD scree setup, this will change whith screen
  const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
  LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

  //writing data to text file (change to SD card later)
  myData = new Serial (this,Serial.list()[0],9600);
  output = createWriter ("data.txt")
}

void loop() {

//write the data to a txt file
  if (myData.avaliable()>0){
    String value =myData.readString();
  }
  if (value != null){
    output.println(value)
  }

//get randomized force
  FOUT = random(FMin, FMax);
  Serial.println(force);
  FIN = analogRead(FSS)

// Actuate motor
  
  analogWrite ()

}

//function to calculate the force applied to the sensor
//this could either be calibrate with the force sensor or if unreliable, use force sensor input to control increase or decrease force
int forceFunction (long current){
  long result;
  result = (torqueConstant * current)/r;
  return result; //this is a long
}
