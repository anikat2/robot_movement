#include <Arduino.h>
#include <robot_movement.h>

FourWheelDrive drive(2, 3, 4, 5);

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(A0));

  drive.a_star_init(8, 8);
  drive.clear_grid_obstacles();
  drive.set_grid_obstacle(3, 3);
  drive.set_grid_obstacle(3, 4);
  drive.set_grid_obstacle(4, 3);

  if (drive.a_star(0, 0, 7, 7)) {
    Serial.print(F("A* path len="));
    Serial.println(drive.path_length());
    for (uint8_t i = 0; i < drive.path_length(); i++) {
      uint8_t x = 0;
      uint8_t y = 0;
      drive.get_grid_path_point(i, x, y);
      Serial.print(x);
      Serial.print(',');
      Serial.println(y);
    }
  } else {
    Serial.println(F("A* failed"));
  }

  drive.rrt_init(0, 8, 0, 8, 0.5f, 0.1f, 400);
  drive.clear_continuous_obstacles();
  drive.add_circle_obstacle(4, 4, 1.2f);

  if (drive.rrt(0.5f, 0.5f, 7.5f, 7.5f)) {
    Serial.print(F("RRT path len="));
    Serial.println(drive.path_length());
  } else {
    Serial.println(F("RRT failed"));
  }

  drive.rrt_star_init(0, 8, 0, 8, 0.5f, 0.1f, 2.0f, 400);
  drive.clear_continuous_obstacles();
  drive.add_circle_obstacle(4, 4, 1.2f);

  if (drive.rrt_star(0.5f, 0.5f, 7.5f, 7.5f)) {
    Serial.print(F("RRT* path len="));
    Serial.println(drive.path_length());
  } else {
    Serial.println(F("RRT* failed"));
  }
}

void loop() {}
