#define joyX A2
#define joyY A0
int joy_coords[2] = {-1, -1};
int filtered_coords[2] = {0, 0};
int counter = 0;
const int min_distance = 20;
const int max_points = 100;
int stored_coords[max_points][2];
int stored_counter = 0;

// Low-pass filter function
int low_pass_filter(int new_value, int prev_value, float alpha = 0.15) {
  return alpha * new_value + (1 - alpha) * prev_value;
}

// Calculate Euclidean distance between two points
float calculate_distance(int coord1[2], int coord2[2]) {
  return sqrt(pow(coord2[0] - coord1[0], 2) + pow(coord2[1] - coord1[1], 2));
}

// Filter coordinates based on minimum distance
void filter_coordinates(int new_coord[2]) {
  if (stored_counter == 0 || calculate_distance(stored_coords[stored_counter - 1], new_coord) >= min_distance) {
    if (stored_counter < max_points) {
      stored_coords[stored_counter][0] = new_coord[0];
      stored_coords[stored_counter][1] = new_coord[1];
      stored_counter++;
    }
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
  // Start serial communication
  Serial.begin(9600);
}

void loop() {
  if (counter < 100) {
    // Get joystick coordinates
    get_joy_coords(joy_coords);
    filtered_coords[0] = low_pass_filter(joy_coords[0], filtered_coords[0]);
    filtered_coords[1] = low_pass_filter(joy_coords[1], filtered_coords[1]);
    
    // Filter and store coordinates
    filter_coordinates(filtered_coords);
    
    // Print only the filtered coordinates
    Serial.println(String("Filtered X: ") + filtered_coords[0] + ", Filtered Y: " + filtered_coords[1]);

    // Increment the counter
    counter++;
    
    // Delay for 100 milliseconds
    delay(100);
  } else {
    // Print stored coordinates once the loop is finished
    print_stored_coords();
    // Stop the loop
    while (true) {}
  }
}

void get_joy_coords(int coords[2]) {
  coords[0] = analogRead(joyY);
  coords[1] = analogRead(joyX);
}
