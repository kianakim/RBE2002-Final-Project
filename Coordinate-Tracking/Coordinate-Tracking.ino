#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#include <LiquidCrystal.h>

const int rs = 18, en = 19, d4 = 17, d5 = 16, d6 = 15, d7 = 14;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Storing distance data
// store drive forward values and direction to keep track of position

// coordinate variables
double candle_x = 0;
double candle_y = 0;
double candle_z = 0;
double numberTurns = 0;
double val = 3300; 
double distance;
volatile double numberofTurns = 0;
int turning_dir;




Encoder myEncoder(12, 8); 


void setup() {
  //myEncoder.attach(12, 8); 
  myEncoder.write(0); // set the encoder position to 0
   lcd.begin(16, 2);
  Serial.begin(9600);
}

void loop() {
  // forward is +x //for example
  // whenever you use driveForward(), after completed update variable
 if(numberofTurns = 1){
  candle_y = candle_y + distance;
  }
// else if(numberofTurns = 2){
//      myEncoder.write(0);
//      candle_x = candle_x + distance;
//     // val = 0;
//     }
// else if(numberofTurns = 3){
//      myEncoder.write(0);
//      candle_y = candle_y - distance;
//     // val = 0;
//     }
// else if(numberofTurns = 4){
//  myEncoder.write(0);
//  candle_x = candle_x - distance;
// // val = 0;
// }
// 

   lcd.setCursor(0, 0);
  lcd.print("x:");
  lcd.print(candle_x);
  lcd.print("in");
  lcd.setCursor(0,1);  //delete line if using z
  lcd.print("y:");
  lcd.print(candle_y);
  lcd.print("in");

//  Serial.println(candle_y);

  //distanceCovered();
}


//void encoderPos(){        // checks if the robot turned right and if the robot turned it adds 1 to turns count
//        val = 3300 ;          // read position
//        Serial.print("Encoder is at: "); // print the position
//        Serial.println(val);
//        delay(100);
//  }

void numberTurn(){
  if(turning_dir = 1){
        numberofTurns = numberofTurns + 1;
}}

void distanceCovered(){   // calculates the distance covered by the robot based on the encoders count
//       encoderPos();
       distance = (val/1632.67)*(3.14*2.75) ;  // inches
//       Serial.print("Distance Covered: "); // print the position
//       Serial.println(distance);
        delay(100);
           
}






