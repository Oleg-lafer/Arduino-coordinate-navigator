#include <IRremote.h>

int RECV_PIN = 2;
IRrecv irrecv(RECV_PIN);
decode_results results;

// Define motor control pins
const int LeftPWM = 3;
const int LeftForward = 5;
const int LeftBackward = 4;

const int RightPWM = 6;
const int RightForward = 7;
const int RightBackward = 8;

const int rot_speed = 7;  // 1:7
const int straight_speed = 7;  // 1:7
const int delays_time = 2000;  // [ms]

const double straight_length_2_time = 2;
const double rot_ang_2_time = 1.8;


// Define the list of (x,y) pairs
const int numPairs = 15; // replace with the actual number of pairs
int xy_list[numPairs][2] = {
{512, 508},
{512, 432},
{553, 496},
{547, 575},
{485, 642},
{412, 699},
{351, 695},
{298, 667},
{253, 643},
{215, 601},
{199, 542},
{286, 533},
{349, 526},
{411, 519},
{467, 513},
    // ...
    // fill in the rest of the pairs
};


void setup(){
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver

    // Set motor control pins as outputs
    pinMode(LeftPWM, OUTPUT);
    pinMode(LeftForward, OUTPUT);
    pinMode(LeftBackward, OUTPUT);

    pinMode(RightPWM, OUTPUT);
    pinMode(RightForward, OUTPUT);
    pinMode(RightBackward, OUTPUT);

    // Initialize motors (stopped)
    digitalWrite(LeftForward, HIGH);
    digitalWrite(LeftBackward, LOW);
    digitalWrite(RightForward, HIGH);
    digitalWrite(RightBackward, LOW);

    analogWrite(RightPWM, 0);
    analogWrite(LeftPWM, 0);

}

//Infinite loop
void loop() {
  delay(delays_time); // drive for 1 second
  for (int list_ind = 1; list_ind < numPairs-1; list_ind++) {    
    // Calculate angle and speed here
    int rot_time_and_dir, straigh_time;
    calculateAngle(list_ind, &rot_time_and_dir, &straigh_time);

    // Rotate the car
    if (rot_time_and_dir > 0){ // right turn
      move(rot_speed, -rot_speed);
    }else{  // left turn
      move(-rot_speed, rot_speed);
    }
    Serial.println("rotate");
    delay(abs(rot_time_and_dir)); // drive for 1 second

    move(0, 0);
    Serial.println("pause");
    delay(delays_time); // drive for 1 second

    // Drive straight for some time
    move(straight_speed, straight_speed);
    Serial.println("straight");
    delay(straigh_time); // drive straight

    move(0, 0);
    Serial.println("pause");
    delay(delays_time);
  }

  move(0, 0);
  delay(9999999);
}


void calculateAngle(int list_ind, int* rot_time_and_dir, int* straight_time){
  int rot_angle, straight_length;
  straight_length = 0;
  rot_angle = 0;

  int three_dots_xy[3][2] = {};  // last, current, and next points
  // Copy last point
  three_dots_xy[0][0] = xy_list[list_ind-1][0];
  three_dots_xy[0][1] = xy_list[list_ind-1][1];

  // Copy current point
  three_dots_xy[1][0] = xy_list[list_ind][0];
  three_dots_xy[1][1] = xy_list[list_ind][1];

  // Copy next point
  three_dots_xy[2][0] = xy_list[list_ind+1][0];
  three_dots_xy[2][1] = xy_list[list_ind+1][1];

  // Calculate distances
  double distance_0_1 = sqrt(pow(three_dots_xy[1][0] - three_dots_xy[0][0], 2) + pow(three_dots_xy[1][1] - three_dots_xy[0][1], 2));
  double distance_1_2 = sqrt(pow(three_dots_xy[2][0] - three_dots_xy[1][0], 2) + pow(three_dots_xy[2][1] - three_dots_xy[1][1], 2));
  double distance_0_2 = sqrt(pow(three_dots_xy[2][0] - three_dots_xy[0][0], 2) + pow(three_dots_xy[2][1] - three_dots_xy[0][1], 2));

  // Calculate the square of the distances
  double distance_0_1_sq = pow(distance_0_1, 2);
  double distance_1_2_sq = pow(distance_1_2, 2);
  double distance_0_2_sq = pow(distance_0_2, 2);

  // // Print the dists
  // Serial.println("distance_0_1 = " + String(distance_0_1));
  // Serial.println("distance_1_2 = " + String(distance_1_2));
  // Serial.println("distance_0_2 = " + String(distance_0_2));

  // Calculate cos(γ)
  double cos_gamma = (distance_0_1_sq + distance_1_2_sq - distance_0_2_sq) / (2 * distance_0_1 * distance_1_2);
  double gamma_rad = acos(cos_gamma);
  double gamma_deg = gamma_rad * (180.0 / M_PI);
  rot_angle = 180 - gamma_deg;

  *rot_time_and_dir = rot_ang_2_time * rot_angle;

  straight_length = distance_1_2;
  *straight_time = straight_length_2_time * straight_length;

  Serial.println("rot ang = " + String(rot_angle) + ", straight length = " + String(straight_length));
}


// -7 <= V <= 7
void move(int V_left, int V_right) {
    if(V_left>0){
      //left wheel moving forward
      digitalWrite(LeftForward, HIGH);   // Left motor forward
      digitalWrite(LeftBackward, LOW);   // Left motor not backward
    }else{
      //left wheel moving backward
      digitalWrite(LeftForward, LOW);  // Left motor not forward
      digitalWrite(LeftBackward, HIGH);  // Left motor backward
    }

    if(V_right>0){
      //right wheel moving forward
      digitalWrite(RightForward, HIGH);   // Right motor forward
      digitalWrite(RightBackward, LOW);   // Right motor not backward
    }else{
      //right wheel moving backward
      digitalWrite(RightForward, LOW);  // Right motor not forward
      digitalWrite(RightBackward, HIGH);  // Right motor backward
    }

    int left_PWM_value = map(abs(V_left), 0 ,7, 0, 255);
    int right_PWM_value = map(abs(V_right), 0 ,7, 0, 255);

    // Serial.println("");
    // Serial.println("Motor left is forawrd: "+String(V_left>0)+" speed: "+String(left_PWM_value));
    // Serial.println("Motor right is forawrd: "+String(V_right>0)+" speed: "+String(right_PWM_value));
    // Serial.println("");
  
    analogWrite(LeftPWM, left_PWM_value);   //  speed forward for left motor
    analogWrite(RightPWM, right_PWM_value);  //  speed forward for right motor
    delay(30);
}
