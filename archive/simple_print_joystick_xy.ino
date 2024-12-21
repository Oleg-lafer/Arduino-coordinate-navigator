#define joyX A2
#define joyY A0
int joy_coords[2] = {-1, -1};

void setup() {
  // Start serial communication
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  get_joy_coords(joy_coords);
  Serial.println(String("X: ") + joy_coords[0] + ", Y: " + joy_coords[1]);

  delay(100);
}

void get_joy_coords(int coords[2]) {
  coords[0] = analogRead(joyY);
  coords[1] = analogRead(joyX);
}
