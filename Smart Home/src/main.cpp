#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <DHT.h>
#include <Servo.h>

/////////////////////////////////////////Definitions///////////////////////////////////////

#define ROWS 4 // Number of rows in the keypad matrix
#define COLS 4 // Number of columns in the keypad matrix

// PINS
#define flameSensorPin A0 // Analog input pin
#define GasSensorPin A1
#define DHT_PIN A2
#define GAS_led A3

#define FLAME_led 0
#define ledsPin 1
#define Button 6
#define ServoPin 9
#define echoPin 10
#define triggerPin 11

#define DHT_TYPE DHT11
// comment if you want analog modes
#define Digital_mode 0

char keys[ROWS][COLS] = {
    {'1', '2', '3', '-'},
    {'4', '5', '6', '*'},
    {'7', '8', '9', '/'},
    {'C', '0', '=', '+'}};

byte colPins[ROWS] = {7, 8, 12, 13}; // Connect these to the row pinouts of the keypad
byte rowPins[COLS] = {4, 3, 2, 5};   // Connect these to the column pinouts of the keypad

/////////////////////////////////////////Variables///////////////////////////////////////

const char *validUsername = "Student";
const char *validPassword = "12345";
int maxAttempts = 3;
int attempts = 0;
bool regestered = 0;
bool system_start = 0;
long duration;
int distance;

/////////////////////////////////////////Instances///////////////////////////////////////

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
LiquidCrystal_I2C lcd(0x20, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
DHT dht(DHT_PIN, DHT_TYPE);
Servo myservo; // create servo object to control a servo

/////////////////////////////////////////Functions/////////////////////////////////////////

void Flame_readings()
{
  lcd.clear();

// Read the analog value from the flame sensor
#ifdef Digital_mode
  int sensorValue = digitalRead(flameSensorPin);
  if (sensorValue == 1)
  {
    lcd.println("Flame detected!");
    // Play a tone at 2000Hz for 1000 milliseconds (1 second)
    digitalWrite(FLAME_led, HIGH);
    // Wait for 1 second
    delay(200);
  }
  else
    digitalWrite(FLAME_led, LOW);
#else
  // analog reading in proteus is a shit
  int sensorValue = analogRead(flameSensorPin);
  // Print the sensor value to the Serial Monitor for debugging
  Serial.print("Sensor Value: ");
  Serial.println(sensorValue);

  // Check if the sensor value exceeds a threshold to detect a flame
  if (sensorValue > 500)
  {
    Serial.println("Flame detected!");
  }
  else
  {
    // Serial.println("No flame detected.");
  }
#endif

  // Add a delay to reduce serial output frequency
  delay(200); // Delay for 1 second
}
void DH11_readings()
{
  lcd.clear();

  float temperature = dht.readTemperature(); // in Celsius
  float humidity = dht.readHumidity();       // in percentage

  // Check if the readings are valid (not NaN)
  if (!isnan(temperature) && !isnan(humidity))
  {
    // Print temperature and humidity to the serial monitor
    lcd.print("Temperature: ");
    lcd.print(temperature);
    lcd.setCursor(0, 1);
    lcd.print(" C, Humidity: ");
    lcd.print(humidity);
    lcd.println("%");
  }
  else
  {
    // If there was an issue with the sensor reading, print an error
    lcd.print("Failed to read from the sensor. Check the wiring.");
  }
  // Delay for a few seconds before the next reading
  delay(500); // Adjust the delay as needed
}
void Gas_readings()
{
  lcd.clear();
// Read the analog value from the flame sensor
#ifdef Digital_mode
  int sensorValue = digitalRead(GasSensorPin);
  if (sensorValue == 1)
  {
    lcd.print("GAS detected!");
    // Play a tone at 1000Hz for 1000 milliseconds (1 second)
    digitalWrite(GAS_led, HIGH);
  }
  else
  digitalWrite(GAS_led, LOW);
#else
  // analog reading in proteus is a shit
  int sensorValue = analogRead(GasSensorPin);
  // Print the sensor value to the Serial Monitor for debugging
  Serial.print("GAS Value: ");
  Serial.println(sensorValue);

  // Check if the sensor value exceeds a threshold to detect a GAS
  if (sensorValue > 500)
  {
    Serial.println("GAS detected!");
  }
  else
  {
    // Serial.println("No GAS detected.");
  }
#endif

  // Add a delay to reduce serial output frequency
  delay(500); // Delay for 1 second
}
void US_readings()
{
  // Clear the trigger pin
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  // Send a 10us pulse to trigger the ultrasonic sensor
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Measure the duration of the echo pulse
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters
  distance = duration * 0.034 / 2;

  // Print the distance to the serial monitor
  lcd.clear();
  lcd.print("Distance: ");
  lcd.print(distance);
  lcd.println(" cm");
  delay(300);
}

char getKeyWithLCDPrint()
{
  char key = keypad.getKey();
  if (key)
  {
    lcd.setCursor(0, 1);
    lcd.print("Pressed: ");
    lcd.print(key);
    delay(200); // Display for a moment and then clear
    lcd.setCursor(0, 1);
    lcd.print("            "); // Clear the line
  }
  return key;
}

void handleAccessDenied()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Access denied!");
  attempts++;
  if (attempts < maxAttempts)
  {
    lcd.setCursor(0, 1);
    lcd.print("Tries left: ");
    lcd.print(maxAttempts - attempts);
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Max attempts reached!");
    delay(500);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Please contact admin");
    while (true)
      ; // Lock the system
  }
  delay(200);
  lcd.clear();
}
void Registeration()
{
  lcd.clear();
  lcd.print("press C to start");
  delay(200);
  char key = getKeyWithLCDPrint();

  if (key == 'C')
  {
    if (attempts < maxAttempts)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Enter password:");
      char enteredPassword[10]; // Adjust the size as needed
      getKeyWithLCDPrint();     // Clear any remaining keypresses
      int passwordIndex = 0;
      while (true)
      {
        char passwordKey = getKeyWithLCDPrint();
        if (passwordKey && passwordKey != 'C')
        {
          enteredPassword[passwordIndex++] = passwordKey;
        }
        if (passwordKey == 'C')
        {
          enteredPassword[passwordIndex] = '\0';
          break;
        }
      }

      if (strcmp(enteredPassword, validPassword) == 0)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Access granted!");
        delay(500);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Welcome, ");
        lcd.setCursor(0, 1);
        lcd.print(validUsername);
        delay(500);
        lcd.clear();
        regestered = 1;
      }
      else
      {
        handleAccessDenied();
      }
    }
  }
}
void Open_Door()
{
  myservo.write(0);
  delay(200); // Wait for 500 milliseconds
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Door Open");
  myservo.write(90); // Close the door
  delay(200);        // Wait for 500 milliseconds
  // Display the door status on the LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Door Closed");
  system_start = 1;
}
void setup()
{
  lcd.init(); // initialize the lcd
  lcd.backlight();
  lcd.setCursor(0, 0);
  myservo.attach(ServoPin); // attaches the servo on pin 9 to the servo object
  pinMode(flameSensorPin, INPUT);
  pinMode(GasSensorPin, INPUT);
  // Set the trigger pin as an OUTPUT
  pinMode(triggerPin, OUTPUT);
  // Set the echo pin as an INPUT
  pinMode(echoPin, INPUT);
  pinMode(ledsPin, OUTPUT);
  pinMode(GAS_led, OUTPUT);
  pinMode(FLAME_led, OUTPUT);

  dht.begin();

  lcd.print("SMART HOME ... ");
  delay(200);
}

void loop()
{

  if (regestered)
  {
    if (system_start)
    {
      US_readings();
      digitalWrite(ledsPin, (distance <= 400) ? HIGH : LOW);
      Gas_readings();
      DH11_readings();
      Flame_readings();
    }
    else
      Open_Door();
  }
  else
    Registeration();
}
