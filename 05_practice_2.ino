#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count = 0;
  toggle = 0;
  digitalWrite(PIN_LED, toggle); // turn off LED.
}

void loop() {
  for (count; count < 10; count ++) {
    Serial.println(count);
    toggle = toggle_state(toggle); // toggle LED value.
    digitalWrite(PIN_LED, toggle); // update LED status.
    delay(100); // wait for 1,000 milliseconds
  }
  toggle = 1;
  digitalWrite(PIN_LED, toggle);
  while (1) {
  }
}

int toggle_state(int toggle) {
  if (toggle == 0) {
    toggle += 1;
  }
  else {
    toggle -= 1;
  }
  return toggle;
}
