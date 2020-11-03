#include <NewPing.h>

#define TRIGGER_PIN  11  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


unsigned short forward_right_wheel=5;
unsigned short backward_right_wheel=3;

unsigned short forward_left_wheel=6;
unsigned short backward_left_wheel=9;

const int water_sensor_pin=A0;


String motor_Direction;

int distance;
int water_value;

String Direction;

unsigned int pins[]={3,5,6,9};

char data_reading;


////////////////////////////////////////
void forward_car(){
  
    analogWrite(backward_right_wheel,0);
   analogWrite(forward_right_wheel,255);

   analogWrite(backward_left_wheel,0);
   analogWrite(forward_left_wheel,255);

}
void backward_car(){
  
   analogWrite(backward_right_wheel,255);
   analogWrite(forward_right_wheel,0);

   analogWrite(backward_left_wheel,255);
   analogWrite(forward_left_wheel,0);

}

void stop_car(){
    analogWrite(backward_right_wheel,0);
   analogWrite(forward_right_wheel,0);

   analogWrite(backward_left_wheel,0);
   analogWrite(forward_left_wheel,0);
}


void right_car(){
  analogWrite(backward_right_wheel,0);
   analogWrite(forward_right_wheel,150);

   analogWrite(backward_left_wheel,0);
   analogWrite(forward_left_wheel,0);
}

void left_car(){
  analogWrite(backward_right_wheel,0);
   analogWrite(forward_right_wheel,0);

   analogWrite(backward_left_wheel,0);
   analogWrite(forward_left_wheel,150);
}

void down_car_speed(){
   analogWrite(backward_right_wheel,0);
   analogWrite(forward_right_wheel,100);

   analogWrite(backward_left_wheel,0);
   analogWrite(forward_left_wheel,100);

}
/////////////////////////////////////////////
int measure_distance(){
  int distance_measure=sonar.ping_cm();
  Serial.print("distance :");
  Serial.print(distance_measure);
  Serial.println(" cm");
  delay(500);
  return distance_measure;
  
}

int measure_water_sensor(){
  int water_sensor_value=analogRead(water_sensor_pin);
   Serial.print("water : ");
   Serial.println(water_sensor_value);

    delay(500);
   return water_sensor_value;

   
}
/////////////////////////////////////////
void run_car(){
    distance=measure_distance();
    water_value=measure_water_sensor();

    if(distance>20){
    forward_car();

   }

   else{
    stop_car();
   }
   
    while(water_value<980){
      water_value=measure_water_sensor();
      distance=measure_distance();

      if(distance>20){
        down_car_speed();
      }
      if (distance<20){
        stop_car();
      }
      
      if(water_value>980){
        break;
      }
    }

}


void send_command(char data_reading){
  switch(data_reading){
    case 'f': run_car();break;
    case 'b': backward_car();break;
    case 's': stop_car();break;
    case 'r': right_car();break;
    case 'l': left_car();break;

    default:Serial.print("Inalid Command\n");
    
  }
}

void setup() {
 Serial.begin(9600);
  for(int i=0;i<4;i++)
  {
    pinMode(pins[i],OUTPUT);
  }
  
}


void loop() {

   //receive command
  if (Serial.available() > 0){
    data_reading = Serial.read();
  }
  
   send_command(data_reading);

     
}
    

