// Apollo Blink Program
// It will blink the on-board blue LED and print to the Serial Terminal

int LED_pin = 32;

void setup() {
  // Start a serial connection.  It can be viewed with through Tools->Serial Monitor
  // Make sure to set the baud rate to 115200 in the bottom right corner
  Serial.begin(115200);
  // Set the LED pin as an Output
  pinMode(LED_pin, OUTPUT);
}

void loop() {
  // Turn the LED on/off every 500 millisecondsl
  digitalWrite(LED_pin, HIGH);
  delay(500);
  digitalWrite(LED_pin, LOW);
  delay(500);
  // Send text to the Serial Terminal for debugging
  Serial.print("Current milliseconds = ");
  Serial.println(millis());
}
