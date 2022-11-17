/*
  Camera Slider
  by dahansi.de
  visit: www.glohbe.de
 */

#include <LiquidCrystal_I2C.h>  // Display
//Display:
#define I2C_ADDR    0x27  // Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7
LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);
String myTxt;

/*
 * 0 = Hauptmenü
 * 1 = Setup
 * 2 = Manuell
 */
int state = 0;                // Aktueller Programmteil
int stateAlt = -1;            // Alter Status

// Drehknopf
#define DIR_CCW 0x10
#define DIR_CW 0x20
int Rotary1 = 10;              // Rotary Drehknopf
int Rotary2 = 11;              // Rotary Drehknopf
int RotaryButton = 12;         // Rotary Button
int rAlt = 0;                 // Rotary PressStatus Alt
int rWert = 0;                // Rotary DrehWert

// Poti
int potPin = 2;
int stepSize = 0;


const int resolution = 16;
const long int maxSteps = 52600; // von Links nach Rechts 17 ganze Schritt a 200
int speed = 800;
long int steps = 0;
long int maxPics = 0;
long int maxDelay = 0;
int pictureCounter = 0;

short int shutterPin = 2;  // Auslösen der Kamera
short int motorEnable = 6; // HIGH = Motor deaktiviert | LOW = Motor aktiviert
short int motorStep = 5; // Step 
short int motorDir = 4; // Dir | LOW hin zum Motor | High = Weg vom Motor
boolean motorDirection = HIGH; //LOW hin zum Motor | High = Weg vom Motor

short int motorM1 = 7; // 
short int motorM2 = 8; // -> Bestimmen die Revolution | M1,M2,M3 High = 1/16, M1,M2,M3 auf LOW = 1
short int motorM3 = 9; //

/* Pfeil */
byte arrow[8] = {
  B00000,
  B11000,
  B01100,
  B00110,
  B01100,
  B11000,
  B00000,
};


void setup() {
  lcd.begin(20,4);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home();
  lcd.print("Camera Slider v1");
  lcd.setCursor(0,1);
  lcd.print("Initialisiere...");
  lcd.setCursor(0,3);
  lcd.print("[24.11.16] by Hansi");
  Serial.begin(9600);

  // Rotary Encoder:
  pinMode(Rotary1, INPUT);  // Drehknopf initialisieren
  pinMode(Rotary2, INPUT);  // Drehknopf initialisieren
  digitalWrite(Rotary1, HIGH);
  digitalWrite(Rotary2, HIGH);
  pinMode(RotaryButton, INPUT);      // Button initialisieren
  digitalWrite(RotaryButton, HIGH);  // Button aktivieren

  // Motor
  pinMode(motorEnable,OUTPUT); // Enable 
  digitalWrite(motorEnable,HIGH); // Motor deaktivieren
  pinMode(motorStep,OUTPUT); 
  pinMode(motorDir,OUTPUT); 
  pinMode(motorM1,OUTPUT); 
  pinMode(motorM2,OUTPUT); 
  pinMode(motorM3,OUTPUT); 
  setRes(16);

  // Foto
  pinMode (shutterPin, OUTPUT);
  delay(1000);
}


int menuAkt = 0;
int menu[] = {0,1,2};
int menuSize = sizeof(menu) / sizeof(int);
void loop() {
  // Weg vom Motor HIGH
  // Hin zum Motor LOW
  if(state != stateAlt){
    stateAlt = state;
    switch(state){
      case 1: state = loadSetup(); break;
      case 2: state = loadManuell(); break;
      default: state = loadMenu(); break;
    }
  }
  delay(1000);
  /*
  Serial.println(maxSteps);
  speed = 800;
  while(steps < maxSteps){
    Move(200);
    delay(500);
    shutter();
    delay(500);
    steps += 200;
    Serial.print("Schritt ");
    Serial.print(steps);
    Serial.print(" von ");
    Serial.print(maxSteps);
    Serial.print(" | #Fotos: ");
    Serial.println(pictureCounter);
  }
  goBack();
  exit;
  */
}


/*
 *  Setup:
 */

int setupStep = 0;
boolean loadSetup(){
  boolean doSetup = true;
  while(doSetup){
    switch(setupStep){
      case 0: 
        showBilder(); // Max. 10520
        setupStep++;
        break;
      case 1:
        showDelay();
        setupStep++;
        break;
      case 2:
        showDirection();
        setupStep++;
        break;
      case 3:
        showUndlos();
        setupStep++;
        break;
      case 4:
        doTimelapse();
        doSetup = false;
        setupStep = 0;
        break;
    }
  }
  return 0;
}

/*
 *  Manueller Modus
 */
int rotary = 0;
void showBilder(){
  boolean dome = true;
  lcd.clear();
  lcd.print("Wie viele Bilder?");
  int addPics = analogRead(potPin);
  
  while(dome){
    rotary = getRotary();
    Serial.println(analogRead(potPin));
    if(rotary){
      addPics = analogRead(potPin);
      if(addPics < 1)
        addPics = 1;
      if(rotary > 0){
        maxPics += addPics;
        if(maxPics > maxSteps)
          maxPics = maxSteps;
      }
      if(rotary < 0){
        maxPics -= addPics;
        if(maxPics < 0)
          maxPics = 0;
      }
      if(maxPics > 10520)
        maxPics = 10520;
      clearLine(2);
      myTxt = maxPics + String(" / ") + maxSteps / 5;
      lcd.setCursor(0,2); lcd.print(myTxt);
      myTxt = String("Video: ") + String((float) maxPics / 30) + String("s   ");
      lcd.setCursor(0,3); lcd.print(myTxt);
    }
    if(pushed())
      dome = false;
  }
}

void showDelay(){
  boolean dome = true;
  lcd.clear();
  lcd.print("Wie viel Zeit");
  lcd.setCursor(0,1); lcd.print("zw. den Bildern?");
  while(dome){
    rotary = getRotary();
    if(rotary){
      if(rotary > 0)
        maxDelay++;
      if(rotary < 0)
        maxDelay--;
      clearLine(2);
      myTxt = String("Abstand: ") + maxDelay + ("s   ");
      lcd.setCursor(0,2); lcd.print(myTxt);
    }
    if(pushed())
      dome = false;
  }
}

void showDirection(){
  boolean dome = true;
  lcd.clear();
  lcd.print("Welche Richtung?");
  lcd.setCursor(0,2); lcd.print("Bewegung");
  lcd.setCursor(0,3); lcd.print("Wohin soll er gehn?");
  while(dome){
    rotary = getRotary();
    if(rotary){
      if(rotary > 0){
        lcd.setCursor(0,3); lcd.print("weg vom Motor");
        motorDirection = HIGH;
      }
      if(rotary < 0){
        lcd.setCursor(0,3); lcd.print("hin zum Motor");
        motorDirection = LOW;
      }
    }
    if(pushed())
      dome = false;
  }
}

void showUndlos(){
   boolean dome = true;
  lcd.clear();
  lcd.print("Kann es losgehen?");
  while(dome){
    if(pushed())
      dome = false;
  }
}

void doTimelapse(){
  steps = 0;
  pictureCounter = 0;
  setRes(16);
  lcd.clear();
  lcd.print("Timelapse...");
  digitalWrite(motorEnable,LOW);
  digitalWrite(motorDir,motorDirection);
  speed = 800;
  long int maxStepsNow = maxPics;
  int nowMove = (maxSteps / maxStepsNow);
  Serial.println(nowMove);
  while(steps < maxSteps){
    lcd.clear();
    shutter();
    lcd.setCursor(0,0); lcd.print("Timelapse...");
    myTxt = String("Bilder: ") + pictureCounter + (" / ") + (maxPics +  1);
    lcd.setCursor(0,1); lcd.print(myTxt);
    float prozent = ((float) steps / float(maxSteps)) * 100;
    myTxt = String("Fortschritt: ") + prozent + String("%");
    lcd.setCursor(0,2); lcd.print(myTxt);
    myTxt = String("Restdauer: ~") + ((maxPics - pictureCounter) * (maxDelay + 1) + ((maxSteps - steps) / 1000 ) * 1.63) + String("s");
    lcd.setCursor(0,3); lcd.print(myTxt);
    delay(maxDelay * 1000);
    Move(nowMove);
    delay(750);
    steps += nowMove;
  }
  lcd.clear();
  shutter();
  lcd.setCursor(0,0); lcd.print("Timelapse...");
  myTxt = String("Bilder: ") + pictureCounter + (" / ") + (maxPics +  1);
  lcd.setCursor(0,1); lcd.print(myTxt);
  float prozent = ((float) steps / float(maxSteps)) * 100;
  myTxt = String("Fortschritt: ") + prozent + String("%");
  lcd.setCursor(0,2); lcd.print(myTxt);
  myTxt = String("Restdauer: ~") + ((maxPics - pictureCounter) * (maxDelay + 1) + ((maxSteps - steps) / 1000 ) * 1.63) + String("s  ");
  lcd.setCursor(0,3); lcd.print(myTxt);
  delay(maxDelay * 1000);
  digitalWrite(motorEnable,HIGH);
  if(motorDirection == HIGH)
    motorDirection = LOW;
  else
    motorDirection = HIGH;
  goBack();
}

boolean loadManuell(){
  setRes(16);
  digitalWrite(motorEnable,LOW);
  lcd.clear();
  myTxt = String("Manueller Modus");
  lcd.print(myTxt);
  int stepSize = 10;
  while(1){
    rotary = getRotary();
    stepSize = analogRead(potPin);
    if(stepSize < 5)
      stepSize = 5;
    if(rotary){
      if(rotary > 0){
        digitalWrite(motorDir,HIGH);
        steps += stepSize;
      }else{
        digitalWrite(motorDir,LOW);
        steps -= stepSize;
      }
      Move(stepSize);
      float prozent = ((float) steps / float(maxSteps)) * 100;
      //prozent = prozent / 100;
      myTxt = String("Distanz: ") + prozent + String("%   ");
      lcd.setCursor(0,1); lcd.print(myTxt);
    }
    if(pushed()){
      Serial.println("Bild!");
      delay(500);
      shutter();
      delay(500);
      myTxt = String("# Fotos: ") + pictureCounter;
      lcd.setCursor(0,2); lcd.print(myTxt);
    }
  }
  digitalWrite(motorEnable,HIGH);
  return 0;
}

/*
 * Hauptmenü
 */
boolean menuTrigger = false;
int loadMenu(){
  // 0: Setup starten
  // 1: Manueller Modus
  int menuAlt = 0;
  changeMenu(0);
  while(1){
    int rotary = getRotary();
    if(rotary){
      changeMenu(rotary);
    }

    if(pushed() && menuTrigger){
      menuAkt++;
      return menuAkt;
    }
    menuTrigger = true;
  }
  return 0;
}

void changeMenu(int s){
  if(!s)
    showMenu(0);
  else{
    if(s < 0)
      menuAkt--;
    else
      menuAkt++;
    
    if(menuAkt > menuSize)
      menuAkt = 0;
    else if(menuAkt < 0)
      menuAkt = menuSize;
    showMenu(menuAkt);
  }
}

void showMenu(int s){
  lcd.createChar(0, arrow);
  lcd.clear();
  lcd.setCursor(2,0); lcd.print("Setup starten");
  lcd.setCursor(2,1); lcd.print("Manueller Modus");
  
  switch(s){
    case 1: lcd.setCursor(0,1); lcd.write(byte(0)); break;
    default: lcd.setCursor(0,0); lcd.write(byte(0)); break;
  }
  menuAkt = s;
}

void Move(int steps){
  for(int i = 0;i<steps;i++){
    digitalWrite(motorStep,HIGH); // Output high 
    delayMicroseconds(speed); // Wait 1/2 a ms 
    digitalWrite(motorStep,LOW); // Output low 
    delayMicroseconds(speed); // Wait 1/2 a ms 
  }
}

void shutter(){
  digitalWrite(shutterPin, HIGH);  
  delay(50);
  digitalWrite(shutterPin, LOW);  
  pictureCounter++;
}

void goBack(){
  digitalWrite(motorEnable,LOW);
  digitalWrite(motorDir,motorDirection);
  setRes(0);
  speed = 1000;
  if(steps > maxSteps)
    steps = maxSteps;
  while(steps > 0){
    Move(100);
    steps -= 100 * 16;
    Serial.println(steps);
  }
  digitalWrite(motorEnable,HIGH);
  Serial.println("Fertig!");
}

void setRes(int i){
  if(i == 16){
    digitalWrite(motorM1,HIGH);
    digitalWrite(motorM2,HIGH);
    digitalWrite(motorM3,HIGH);
  }else{
    digitalWrite(motorM1,LOW);
    digitalWrite(motorM2,LOW);
    digitalWrite(motorM3,LOW);
  }
}

/*
 * Knopfdruck abfragen
 */
boolean pushed(){
  int rStatus = digitalRead(RotaryButton);
  if (rStatus != rAlt) {
    rAlt = rStatus;
    if(rStatus == HIGH)
      return true;
  }
  return false;
}

/*
 * Drehimpuls abfragen
 */
 
const unsigned char ttable[7][4] = {
  {0x0, 0x2, 0x4,  0x0}, {0x3, 0x0, 0x1, 0x10},
  {0x3, 0x2, 0x0,  0x0}, {0x3, 0x2, 0x1,  0x0},
  {0x6, 0x0, 0x4,  0x0}, {0x6, 0x5, 0x0, 0x20},
  {0x6, 0x5, 0x4,  0x0},
};

volatile unsigned char stateR = 0;

int getRotary() {
  unsigned char pinstate = (digitalRead(Rotary2) << 1) | digitalRead(Rotary1);
  stateR = ttable[stateR & 0xf][pinstate];
  unsigned char result = (stateR & 0x30);

  if(result)
    return (result == DIR_CCW ? -1 : 1);
  return false;
}

void clearLine(int i){
  lcd.setCursor(0,i);
  lcd.print("                    ");
  return;
}

