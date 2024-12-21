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

const int rot_speed = 7;
const int straight_speed = 7;
const int delays_time = 200;

// Define driving propotionals
const double straight_length_2_time = 3;
const double rot_ang_2_time = 1.8;

// Define traffic light
const int red_light = 13;
const int yellow_light = 12;
const int green_light = 11;
const int buzzer = 10;
const int traffic_light_time = 300;
const int buzzer_time = 100;

// Define the list of (x,y) pairs
const int constant_xy_list_length = 15;
int constant_xy_list[constant_xy_list_length][2] = {
  { 512, 508 },
  { 512, 432 },
  { 553, 496 },
  { 547, 575 },
  { 485, 642 },
  { 412, 699 },
  { 351, 695 },
  { 298, 667 },
  { 253, 643 },
  { 215, 601 },
  { 199, 542 },
  { 286, 533 },
  { 349, 526 },
  { 411, 519 },
  { 467, 513 },
};

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();

  pinMode(LeftPWM, OUTPUT);
  pinMode(LeftForward, OUTPUT);
  pinMode(LeftBackward, OUTPUT);

  pinMode(RightPWM, OUTPUT);
  pinMode(RightForward, OUTPUT);
  pinMode(RightBackward, OUTPUT);

  analogWrite(RightPWM, 0);
  analogWrite(LeftPWM, 0);
  digitalWrite(LeftForward, HIGH);
  digitalWrite(LeftBackward, LOW);
  digitalWrite(RightForward, HIGH);
  digitalWrite(RightBackward, LOW);

  pinMode(red_light, OUTPUT);
  pinMode(yellow_light, OUTPUT);
  pinMode(green_light, OUTPUT);
  pinMode(buzzer, OUTPUT);

  digitalWrite(red_light, LOW);
  digitalWrite(yellow_light, LOW);
  digitalWrite(green_light, LOW);
  digitalWrite(buzzer, LOW);
}

void loop() {
  start_traffic_light_and_buzzer();

  drive_according_to_xy_list(constant_xy_list, constant_xy_list_length);

  move(0, 0);
  delay(9999999);
}

void calculateAngle(int list_ind, int xy_list[constant_xy_list_length][2], int* rot_time_and_dir, int* straight_time) {
  int rot_angle, straight_length;
  straight_length = 0;
  rot_angle = 0;

  int three_dots_xy[3][2] = {};
  three_dots_xy[0][0] = xy_list[list_ind - 1][0];
  three_dots_xy[0][1] = xy_list[list_ind - 1][1];
  three_dots_xy[1][0] = xy_list[list_ind][0];
  three_dots_xy[1][1] = xy_list[list_ind][1];
  three_dots_xy[2][0] = xy_list[list_ind + 1][0];
  three_dots_xy[2][1] = xy_list[list_ind + 1][1];

  double distance_0_1 = sqrt(pow(three_dots_xy[1][0] - three_dots_xy[0][0], 2) + pow(three_dots_xy[1][1] - three_dots_xy[0][1], 2));
  double distance_1_2 = sqrt(pow(three_dots_xy[2][0] - three_dots_xy[1][0], 2) + pow(three_dots_xy[2][1] - three_dots_xy[1][1], 2));
  double distance_0_2 = sqrt(pow(three_dots_xy[2][0] - three_dots_xy[0][0], 2) + pow(three_dots_xy[2][1] - three_dots_xy[0][1], 2));

  double distance_0_1_sq = pow(distance_0_1, 2);
  double distance_1_2_sq = pow(distance_1_2, 2);
  double distance_0_2_sq = pow(distance_0_2, 2);

  double cos_gamma = (distance_0_1_sq + distance_1_2_sq - distance_0_2_sq) / (2 * distance_0_1 * distance_1_2);
  double gamma_rad = acos(cos_gamma);
  double gamma_deg = gamma_rad * (180.0 / M_PI);
  rot_angle = 180 - gamma_deg;

  *rot_time_and_dir = rot_ang_2_time * rot_angle;

  straight_length = distance_1_2;
  *straight_time = straight_length_2_time * straight_length;

  Serial.println("rot ang = " + String(rot_angle) + ", straight length = " + String(straight_length));
}

void move(int V_left, int V_right) {
  if (V_left > 0) {
    digitalWrite(LeftForward, HIGH);
    digitalWrite(LeftBackward, LOW);
  } else {
    digitalWrite(LeftForward, LOW);
    digitalWrite(LeftBackward, HIGH);
  }

  if (V_right > 0) {
    digitalWrite(RightForward, HIGH);
    digitalWrite(RightBackward, LOW);
  } else {
    digitalWrite(RightForward, LOW);
    digitalWrite(RightBackward, HIGH);
  }

  int left_PWM_value = map(abs(V_left), 0, 7, 0, 255);
  int right_PWM_value = map(abs(V_right), 0, 7, 0, 255);

  analogWrite(LeftPWM, left_PWM_value);
  analogWrite(RightPWM, right_PWM_value);
  delay(30);
}

void start_traffic_light_and_buzzer(){
  delay(traffic_light_time);

  digitalWrite(red_light, HIGH);
  digitalWrite(buzzer, HIGH);
  delay(buzzer_time);
  digitalWrite(buzzer, LOW);
  delay(traffic_light_time - buzzer_time);

  digitalWrite(yellow_light, HIGH);
  digitalWrite(buzzer, HIGH);
  delay(buzzer_time);
  digitalWrite(buzzer, LOW);
  delay(traffic_light_time - buzzer_time);

  digitalWrite(green_light, HIGH);
  digitalWrite(buzzer, HIGH);
  delay(buzzer_time);
  digitalWrite(buzzer, LOW);
  delay(traffic_light_time - buzzer_time);

  digitalWrite(buzzer, HIGH);
  delay(buzzer_time*3);
  digitalWrite(buzzer, LOW);  

  digitalWrite(red_light, LOW);
  digitalWrite(yellow_light, LOW);
  digitalWrite(green_light, LOW);
}

void drive_according_to_xy_list(int xy_list[constant_xy_list_length][2], int list_length){
  for (int list_ind = 1; list_ind < list_length - 1; list_ind++) {
    int rot_time_and_dir, straight_time;
    calculateAngle(list_ind, xy_list, &rot_time_and_dir, &straight_time);

    if (rot_time_and_dir > 0) {
      move(rot_speed, -rot_speed);
    } else {
      move(-rot_speed, rot_speed);
    }
    delay(abs(rot_time_and_dir));

    move(0, 0);
    delay(delays_time);

    move(straight_speed, straight_speed);
    delay(straight_time);

    move(0, 0);
    delay(delays_time);
  }
}
