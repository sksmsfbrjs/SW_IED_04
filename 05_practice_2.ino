#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  count = toggle = 0;
  digitalWrite(PIN_LED, toggle);
  delay(1000);
}

void loop() {
  if (count == 11) {
  while(1){}
}
  Serial.println(++count);
  toggle = toggle_state(toggle);
  digitalWrite(PIN_LED, toggle);
  delay(100);
}

int toggle_state(int toggle) {
  return 1-toggle;
}
