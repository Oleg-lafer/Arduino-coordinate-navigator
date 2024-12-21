#include <IRremote.h>

#define joyX A2
#define joyY A0
int joy_coords[2] = {-1, -1};
int filtered_coords[2] = {512, 512};
const int min_distance = 25;
const int max_stored_points = 50;
int stored_coords[max_stored_points][2];
int stored_counter = 0;

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
const int delays_time = 50;

// Define driving propotionals
const double straight_length_2_time = 1.5;
const double rot_ang_2_time = 2.25;

// Define traffic light
const int red_light = 13;
const int yellow_light = 12;
const int green_light = 11;
const int buzzer = 10;
const int traffic_light_time = 300;
const int buzzer_time = 100;

// Low-pass filter function
int low_pass_filter(int new_value, int prev_value, float alpha = 0.15) {
  return alpha * new_value + (1 - alpha) * prev_value;
}

// Calculate Euclidean distance between two points
float calculate_distance(int coord1[2], int coord2[2]) {
  return sqrt(pow(coord2[0] - coord1[0], 2) + pow(coord2[1] - coord1[1], 2));
}

// Filter coordinates based on minimum distance
void store_filtered_coordinates(int new_coord[2]) {
  if (stored_counter == 0 || calculate_distance(stored_coords[stored_counter - 1], new_coord) >= min_distance) {
      stored_coords[stored_counter][0] = new_coord[0];
      stored_coords[stored_counter][1] = new_coord[1];
      stored_counter++;
  }
}

// Print stored coordinates
void print_stored_coords() {
  Serial.println("Stored Coordinates:");
  for (int i = 0; i < stored_counter; i++) {
    Serial.println(String("X: ") + stored_coords[i][0] + ", Y: " + stored_coords[i][1]);
  }
}

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

  digitalWrite(red_light, HIGH);
  digitalWrite(yellow_light, LOW);
  digitalWrite(green_light, HIGH);
  digitalWrite(buzzer, LOW);
}

void loop() {
  get_path_coordinates();
  
  start_traffic_light_and_buzzer();

  drive_according_to_xy_list(stored_coords, stored_counter);

  move(0, 0);
  while(true) {
    delay(1000);
  }
}

void get_path_coordinates(){
  while (stored_counter < max_stored_points) {
    get_joy_coords(joy_coords);
    filtered_coords[0] = low_pass_filter(joy_coords[0], filtered_coords[0]);
    filtered_coords[1] = low_pass_filter(joy_coords[1], filtered_coords[1]);
    store_filtered_coordinates(filtered_coords);
    Serial.println(String("Filtered X: ") + filtered_coords[0] + ", Filtered Y: " + filtered_coords[1]);
    
    delay(100);
  }
  print_stored_coords();
}

void calculateAngle(int list_ind, int xy_list[max_stored_points][2], int* rot_time_and_dir, int* straight_time) {
  int rot_angle, straight_length;
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

  double cos_gamma = (pow(distance_0_1, 2) + pow(distance_1_2, 2) - pow(distance_0_2, 2)) / (2 * distance_0_1 * distance_1_2);
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

void drive_according_to_xy_list(int xy_list[max_stored_points][2], int list_length){
  for (int list_ind = 1; list_ind < list_length - 1; list_ind++) {
    int rot_time_and_dir, straight_time;
    calculateAngle(list_ind, xy_list, &rot_time_and_dir, &straight_time);

    if (rot_time_and_dir != 0){
      if (rot_time_and_dir > 0) {
        move(rot_speed, -rot_speed);
      } else {
        move(-rot_speed, rot_speed);
      }
      delay(abs(rot_time_and_dir));

      move(0, 0);
      delay(delays_time);
    }

    move(straight_speed, straight_speed);
    delay(straight_time);

    move(0, 0);
    delay(delays_time);
  }
}

void get_joy_coords(int coords[2]) {
  coords[0] = analogRead(joyY);
  coords[1] = analogRead(joyX);
}