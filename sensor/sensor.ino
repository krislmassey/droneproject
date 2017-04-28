#include <math.h>

// pins
#define FRONT_TRIG  13
#define FRONT_ECHO  12
#define RIGHT_TRIG  11
#define RIGHT_ECHO  10
#define TOP_TRIG    9
#define TOP_ECHO    8
#define LEFT_TRIG   7
#define LEFT_ECHO   6

// direction index
#define FRONT 0
#define RIGHT 1
#define TOP  2
#define LEFT  3

// pin index
#define TRIG 0
#define ECHO 1

// pin array
int pin_arr[4][2] = {
  {FRONT_TRIG, FRONT_ECHO},
  {RIGHT_TRIG, RIGHT_ECHO},
  {TOP_TRIG, TOP_ECHO},
  {LEFT_TRIG, LEFT_ECHO}
};

long getDistanceRaw(int);
void sendDistances(long);
long* getRawDistances();
long* getAverageDistances(int);
long* getFilteredDistances(int);

void setup() {
  Serial.begin(9600);
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);
  pinMode(TOP_TRIG, OUTPUT);
  pinMode(TOP_ECHO, INPUT);
  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);
  Serial.println("Set up..");
}

void loop() {
  long* distances;

  distances = getRawDistances();
  //distances = getAverageDistances(10);
  //distances = getFilteredDistances(10);
  
  sendDistances(distances);
  //delay(1000);
}

// get raw distance from dir sensor
long getDistanceRaw(int dir) {
  long distance, duration;
  int trig_pin = pin_arr[dir][TRIG];
  int echo_pin = pin_arr[dir][ECHO];

  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  duration = pulseIn(echo_pin, HIGH);
  
  distance = (duration/2) / 29.1;
  if (distance > 200) return 200;
  if (distance < 20) return 20;
  return distance;
}

// serial print the direction and distances delimited by semicolons
// direction and distance seperated by one space
// Example: "F 20;R 27;T 32;L 17;"
void sendDistancesSlow(long distances[]) {
  //Serial.println("Constructing distance string..");
  String data = "";
  for (int dir = 0; dir < 4; dir++) {
    if (dir == FRONT) data = data += "F ";
    else if (dir == RIGHT) data += "R ";
    else if (dir == TOP) data += "T ";
    else if (dir == LEFT) data += "L ";
    //Serial.println("Adding distance for dir");
    data += distances[dir];
    data += ';';
  }
  Serial.println(data);
}

void sendDistances(long distances[]) {
  String data = "";
  for (int dir = 0; dir < 4; dir++) {
    data += distances[dir];
    data += ' ';
//    if (dir == FRONT) Serial.print("F ");
//    else if (dir == RIGHT) Serial.print("R ");
//    else if (dir == TOP) Serial.print("T ");
//    else if (dir == LEFT) Serial.print("L ");
//    Serial.print(distances[dir]);
//    Serial.print(';');
  }
  Serial.println(data);
}

// get raw distances for each direction
long* getRawDistances() {
  long distances[4];
  for (int dir = 0; dir < 4; dir++) {
    distances[dir] = getDistanceRaw(dir);
  }
  return distances;
}

// get average of num_samples of raw distances for each direction
long* getAverageDistances(int num_samples) {
  long distances[4];
  // takes num_samples of raw distances and average them
  for (int dir = 0; dir < 4; dir++) {
    for (int s_num = 1; s_num <= num_samples; s_num++) {
      if (s_num == 1) distances[dir] = getDistanceRaw(dir);
      else {
        distances[dir] = ((distances[dir] * (s_num - 1)) + getDistanceRaw(dir)) / s_num;
      }
    }
  }
  return distances;
}

// get filtered average of num_samples of raw distances for each direction
long* getFilteredDistances(int num_samples) {
  long distances[4];
  long readings[num_samples];
  long average;
  long distance;
  long std_dev;
  int keep;

  // for each direction
  for (int dir = 0; dir < 4; dir++) {
    average = 0;
    //Serial.println("Getting averages for dir ");
    // get num_samples of raw distance and average them
    // keep up with each raw reading for later filtering
    for (int s_num = 0; s_num <= num_samples; s_num++) {
      distance = getDistanceRaw(dir);
      readings[s_num] = distance;
      
      if (s_num == 1) average = distance;
      else {
        average = ((average * (s_num - 1)) + distance) / s_num;
      }
    }

    std_dev = 0;
    //Serial.println("Getting standard deviation for dir ");
    // get the standard deviation
    for (int s_num = 0; s_num < num_samples; s_num++) {
      std_dev += (readings[s_num] - average)*(readings[s_num] - average);
    }
    std_dev = sqrt(std_dev / (num_samples - 1));

    keep = 0;
    // get rid of readings outside of standard deviation from average
    for (int s_num = 0; s_num < num_samples; s_num++) {
      if (readings[s_num] > (average + std_dev) |
          readings[s_num] < (average - std_dev)) {
          continue;
      }
      // average the good readings
      if (keep == 0) {
        distances[dir] = readings[s_num];
        keep++;
      }
      else {
        distances[dir] = ((distances[dir] * (keep - 1)) + readings[s_num]) / keep;
        keep++;
      }
    }
  }
  return distances;
}
