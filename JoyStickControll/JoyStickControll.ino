/*
Based on BLE LED control example to send a joystick signal
*/

#include <ArduinoBLE.h>

// Define pins for the joystick
const int joyXPin = A0;  // X-axis of joystick
const int joyYPin = A1;  // Y-axis of joystick

// Variables to store previous joystick positions
int prevJoyXValue = 0;
int prevJoyYValue = 0;

// variables for button
const int buttonPin = 2;
int oldButtonState = LOW;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // configure the button pin as input
  pinMode(buttonPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // initialize the Bluetooth® Low Energy hardware
  BLE.begin();

  Serial.println("Bluetooth® Low Energy Central - LED control");

  // start scanning for peripherals
  BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();
  Serial.println(peripheral);

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);

    if (peripheral.localName() != "LED") {
      return;
    }

    // stop scanning
    BLE.stopScan();

    controlLed(peripheral);

    // peripheral disconnected, start scanning again
    BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");
  }
}

void controlLed(BLEDevice peripheral) {
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the LED characteristic
  BLECharacteristic ledCharacteristic = peripheral.characteristic("19b10001-e8f2-537e-4f6c-d104768a1214");

  if (!ledCharacteristic) {
    Serial.println("Peripheral does not have LED characteristic!");
    peripheral.disconnect();
    return;
  } else if (!ledCharacteristic.canWrite()) {
    Serial.println("Peripheral does not have a writable LED characteristic!");
    peripheral.disconnect();
    return;
  }

  while (peripheral.connected()) {
    // while the peripheral is connected
    
    // Read analog values from joystick
    int joyXValue = analogRead(joyXPin);
    int joyYValue = analogRead(joyYPin);

    if (joyXValue != prevJoyXValue || joyYValue != prevJoyYValue) {
      // Map analog values to IR signal range
      int mappedXValue = map(joyXValue, 0, 1023, 0, 255);
      int mappedYValue = map(joyYValue, 0, 1023, 0, 255);

      Serial.println("Sending:");
      Serial.println(mappedXValue);
      Serial.println(mappedYValue);
      Serial.println((byte)mappedXValue);
      Serial.println((byte)mappedYValue);
      ledCharacteristic.writeValue((byte)mappedXValue);
      ledCharacteristic.writeValue((byte)mappedYValue);
      // Update previous joystick positions
      prevJoyXValue = joyXValue;
      prevJoyYValue = joyYValue;
    }
    // read the button pin
    /*int buttonState = digitalRead(buttonPin);

    if (oldButtonState != buttonState) {
      // button changed
      oldButtonState = buttonState;

      if (buttonState) {
        Serial.println("button pressed");

        // button is pressed, write 0x01 to turn the LED on
        ledCharacteristic.writeValue((byte)0x01);
      } else {
        Serial.println("button released");

        // button is released, write 0x00 to turn the LED off
        ledCharacteristic.writeValue((byte)0x00);
      }
    }*/
  }

  Serial.println("Peripheral disconnected");
}
