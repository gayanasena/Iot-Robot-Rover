#include <Servo.h>          //standard library for the servo
#include <NewPing.h>        //for the Ultrasonic sensor function library.
#include <AFMotor.h>

//L293D motor control
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

//sensor pins
#define trig_pin A3 //analog input 3
#define echo_pin A4 //analog input 4

// Variable for serial communication
int command = 0;

// DEFAULT mode
int mode = 1;

#define maximum_distance 100 // maxium distance for head ultrasonic sensor
boolean goesForward = false; //robots direction movemnt state
int distance = 40; // obstacle blocking distance
int speed = 0; // wheel speed variable

NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
Servo servo_motor; //servo motor object

//function for get distance mesurment
int readPing() {
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 250;
  }
  return cm;
}

// move head sensor and get mesurments
int lookRight() {
  servo_motor.write(62);// right corner
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(117);// back to default
  return distance;
}

int lookLeft() {
  servo_motor.write(172);//left corner
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(117);// back to default
  return distance;
  delay(100);
}

// Functions for movements

void setSpeed() {
  //setup wheel speeds on looping
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);
}

void moveStop() {
  speed = 0;
  setSpeed();
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void moveForward(int getFSpeed) {

  if (!goesForward) {

    goesForward = true;
    speed = getFSpeed;
    setSpeed();
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
  }
}

void moveBackward() {
  goesForward = false;
  speed  = 220;
  setSpeed();
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(200);
  moveStop();
}

void turnRight(int getTRspeed) {
  speed = getTRspeed;
  setSpeed();
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(300);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void turnLeft(int getTLSpeed) {
  speed = getTLSpeed;
  setSpeed();
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);

  delay(300);

  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

//functions for modes for robot
void RCMode()
{
  if (Serial.available() > 0) {
    command = Serial.read();
    Serial.println(command);
  }

  switch (command) {
    case 'F': goesForward = false; moveForward(250); break;
    case 'B': moveBackward(); break;
    case 'R': turnRight(220); break;
    case 'L': turnLeft(220); break;
    /*case 'Q': goAheadRight(); break;
      case 'Z': goAheadLeft(); break;
      case 'Y': goBackRight(); break;
      case 'X': goBackLeft(); break;
      case '200': speed = 200; break;*/
    case '1': mode = 1; loop(); break;
    case '2': mode = 2; loop(); break;
    case '3': mode = 3; loop(); break;
    case 'S': moveStop(); break;
  }
}

void obstacleAvoidMode() {
  int distanceRight = 0;
  int distanceLeft = 0;

  delay(50);

  if (distance <= 30) { // if something in front stop moving and step backward then looking for a direction that can go
    moveStop();
    delay(150);
    moveBackward();
    delay(500);
    moveStop();
    delay(200);
    //get mesurments to find a free direction
    distanceRight = lookRight();
    delay(200);
    distanceLeft = lookLeft();
    delay(200);
    //move to free direction
    if (distance >= distanceLeft) {// if left direction is shorter than front(straight) distance
      Serial.println("case 1");
      turnRight(220);// if someting near left side robot turn to right side
      moveStop();
    }
    else if (distance >= distanceRight) {
      Serial.println("case 2");
      turnLeft(220); // if someting near right side robot turn to left side
      moveStop();
    }
    else if (distance >= distanceRight && distance >= distanceLeft)
    {
      Serial.println("case 3");
      distanceRight = lookRight();
      delay(200);
      distanceLeft = lookLeft();
      delay(200);
      moveBackward();
    }
    else {
      Serial.println("case 4");
      moveBackward();
      Serial.println(distanceRight);
      Serial.println(distanceLeft);
      if (distanceRight >= distanceLeft)
      {
        turnLeft(220);
      }
      else
      {
        turnRight(220);
      }
    }
  }
  else {
    moveForward(240);  // if nothing in straight direction robot move forward
  }
  distance = readPing(); // get distance mesurments

  //mode changeing if required
  if (Serial.available() > 0) {
    command = Serial.read();
    //set mode;
    switch (command)
    {
      case '1': mode = 1; loop(); break;
      case '2': mode = 2; loop(); break;
      case '3': mode = 3; distance = 30 ; loop(); break;
      default: break;
    }
    Serial.println("Sub - ");
    Serial.println(command);
  }
}

void lineFollowMode()
{
  //define variables and get IR sensor input (Analog)
  int IRLeft = digitalRead(A2);
  int IRMiddle = digitalRead(A1);
  int IRRight = digitalRead(A0);

  /*Serial.print(IRLeft);
    Serial.print(IRMiddle);
    Serial.print(IRRight);
    Serial.println("");*/

  //switch for line following
  // IR - States
  // white - 0 (sensor out on)
  // black - 1 (sensor out off)

  if (IRLeft == 1 && IRMiddle == 1 && IRRight == 1) // check if all black - stop
  {
    moveStop();
  }
  else if (IRLeft == 0 && IRMiddle == 0 && IRRight == 0) // check if all white - stop
  {
    moveStop();
  }
  else if (IRLeft == 0 && IRMiddle == 1 && IRRight == 0) // check if middle only black - run forward
  {
    goesForward = false;
    moveForward(150);
  }
  else if (IRLeft == 1 && IRRight == 0) // check if middle and right only black - turn left
  {
    turnLeft(190);
  }
  else if (IRLeft == 1 && IRMiddle == 0 && IRRight == 0) // check if middle and right only black - turn left
  {
    turnLeft(190);
  }
  else if (IRLeft == 0  && IRRight == 1) // check if middle and left only black - turn right
  {
    turnRight(190);
  }
  else if (IRLeft == 0 && IRMiddle == 0 && IRRight == 1) // check if middle and left only black - turn right
  {
    turnRight(190);
  }
  else
  {
    moveStop();
  }

  //mode changeing if required
  if (Serial.available() > 0) {
    command = Serial.read();
    //set mode;
    switch (command)
    {
      case '1': mode = 1; loop(); break;
      case '2': mode = 2; loop(); break;
      case '3': mode = 3; distance = 30 ; loop(); break;
      default: break;
    }
    Serial.println("Sub - ");
    Serial.println(command);
  }
}

void setup() {

  Serial.begin(115200); //setup serial monitor(baud rate - 9600)
  servo_motor.attach(10); //head servo pin

  servo_motor.write(117); //default servo angle
  delay(1500);

  //setup IR pins
  pinMode(A0, INPUT); //IR Right
  pinMode(A1, INPUT); //IR Middle
  pinMode(A2, INPUT); //IR Left

  //get default ongoing distance mesurment from head
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}

void loop() {
  
  if (Serial.available() > 0) {
    command = Serial.read();
  }
  switch (mode)
  {
    case 1 :
      RCMode();
      break;

    case 2:
      lineFollowMode();
      break;

    case 3:
      obstacleAvoidMode();
      break;

    default:
      break;
  }

}
