



#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);

int led =12;
int ledG =11;
int ledR =10;

 
Adafruit_MMA8451 mma = Adafruit_MMA8451();


#define ADC_ref 5.0
#define zero_x 1.575
#define zero_y 1.575
#define zero_z 1.575
#define sensitivity_x 0.315
#define sensitivity_y 0.315
#define sensitivity_z 0.315
unsigned int x_value;
unsigned int y_value;
unsigned int z_value;

float x_v;
float y_v;
float z_v;


float angle_x;
float angle_y;
float angle_z;

void setup(void) {
  Serial.begin(9600);
  lcd.begin(20,4); 
  lcd.setBacklightPin(3,POSITIVE);
   lcd.setBacklight(HIGH);
     analogReference(DEFAULT);

   
  
  Serial.println("Adafruit MMA8451 test!");
  pinMode(led,OUTPUT);
  pinMode(ledR,OUTPUT);
  pinMode(ledG,OUTPUT);



  
  mma.setRange(MMA8451_RANGE_2_G);
  
  
}

void loop() {

 // read the analog input pins:
  x_value= analogRead(A0);
  y_value= analogRead(A1);
  z_value= analogRead(A2);
  
// get the correct vlues of the analog sensor 

  x_v=((x_value/1024.0*ADC_ref)-zero_x)/sensitivity_x;
 
  y_v=((y_value/1024.0*ADC_ref)-zero_y)/sensitivity_y;
    
  z_v=((z_value/1024.0*ADC_ref)-zero_z)/sensitivity_z;
 


  
  // Read the 'raw' data in 14-bit counts
  mma.read();

  /* Get a new sensor event */ 
  sensors_event_t event; 
  mma.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: \t"); Serial.print(event.acceleration.x); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(event.acceleration.y); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(event.acceleration.z); Serial.print("\t");
  Serial.println("m/s^2 ");


  // X axis acceleration 
     lcd.setCursor(0,0);
     lcd.print("X:");  
     lcd.setCursor(2,0);
     lcd.print(event.acceleration.x);  
      // Y axis acceleration
     lcd.setCursor(0,1);
     lcd.print("Y:");  
     lcd.setCursor(2,1);
     lcd.print(event.acceleration.y);  
      // Z axis acceleration 
     lcd.setCursor(0,2);
     lcd.print("Z:");  
     lcd.setCursor(2,2);
     lcd.print(event.acceleration.z);  


  /* Get the orientation of the sensor */
  uint8_t o = mma.getOrientation();

     
  
  switch (o) {
    case MMA8451_PL_PUF: 
      Serial.println("Portrait Up Front");
      lcd.setCursor(8,2);
      break;
    case MMA8451_PL_PUB: 
      Serial.println("Portrait Up Back");
      lcd.setCursor(8,2); 
      break;    
    case MMA8451_PL_PDF: 
      Serial.println("Portrait Down Front");
      break;
    case MMA8451_PL_PDB: 
      Serial.println("Portrait Down Back");
      break;
    case MMA8451_PL_LRF: 
      Serial.println("Landscape Right Front");
      break;
    case MMA8451_PL_LRB: 
      Serial.println("Landscape Right Back");
      break;
    case MMA8451_PL_LLF: 
      Serial.println("Landscape Left Front");
      break;
    case MMA8451_PL_LLB: 
      Serial.println("Landscape Left Back");
      break;
    }



             lcd.setCursor(8,0);
             lcd.print("a_X");  
             lcd.setCursor(13,0);
             lcd.print(x_v);


              lcd.setCursor(8,1);
             lcd.print("a_Y");  
             lcd.setCursor(13,1);
             lcd.print((y_v));



             lcd.setCursor(8,2);
             lcd.print("a_Z");  
             lcd.setCursor(13,2);
             lcd.print(z_v);
             

if (event.acceleration.z>9){
        digitalWrite(led,HIGH);
}else if (event.acceleration.y>9){
        digitalWrite(ledG,HIGH);
}else if (event.acceleration.x>9){
      digitalWrite(ledR,HIGH);
}   else 
  digitalWrite(led,LOW);
    digitalWrite(ledG,LOW);


    
    lcd.setCursor(0,3);
     lcd.print("Angle from Z:");  
     angle_x =atan2(-(event.acceleration.y),-(event.acceleration.z))+180;
     lcd.setCursor(14,3);
      lcd.print(angle_x);
     
  //   lcd.print((event.acceleration.x*-9.085)-1); 

   


      
  Serial.println();
  delay(1000);
  
}
