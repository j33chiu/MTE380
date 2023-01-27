const int board_green_led = PA5;
const int us_sensor_trig = PA9;
const int us_sensor_echo = PA8;

// benchmark
int lastMillis = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // pause

  // setup nucleo onboard green led
  pinMode(board_green_led, OUTPUT);

  // setup ultrasonic sensor
  pinMode(us_sensor_trig, OUTPUT);
  pinMode(us_sensor_echo, INPUT);

  lastMillis = millis();
}

void loop() {
  // read from ultrasonic sensor
  digitalWrite(us_sensor_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(us_sensor_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(us_sensor_trig, LOW);
  long us_sensor_pulse_duration = pulseIn(us_sensor_echo, HIGH);
  long us_sensor_dist = us_sensor_pulse_duration * 0.034 / 2;
  Serial.print("Ultrasonic Sensor Distance: ");
  Serial.print(us_sensor_dist);
  Serial.print("cm. (");
  if (us_sensor_dist > 10) {
    digitalWrite(board_green_led, HIGH);
  } else {
    digitalWrite(board_green_led, LOW);
  }
  int curTime = millis();
  Serial.print(curTime - lastMillis);
  Serial.println("ms)"); // ~5ms for measurment loop, ~200 measurements per second

  lastMillis = curTime;

  delay(20);
}
