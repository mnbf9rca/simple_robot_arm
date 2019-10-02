/******************************************************/
//                                                    //
// simple_robot_arm                                   //
// designed to work with a simple 4DOF robot arm      //
// with 4 servo motors                                //
// which receives a JSON instructing angle of any one //
// or more servos                                     //
// MIT licence                                        //
// from https://github.com/mnbf9rca/simple_robot_arm  //
//                                                    //
/******************************************************/

#include <ArduinoJson.h>

// JSON key names for the 4 servos
#define SERVO_BASE_NAME "base"
#define SERVO_LEFT_NAME "left"
#define SERVO_RIGHT_NAME "right"
#define SERVO_GRIP_NAME "grip"

// pinout for each servo
#define SERVO_BASE_PIN D0
#define SERVO_LEFT_PIN D1
#define SERVO_RIGHT_PIN D2
#define SERVO_GRIP_PIN D3

//min and max ranges for each servo
#define SERVO_BASE_MIN 0
#define SERVO_BASE_MAX 180
#define SERVO_LEFT_MIN 30
#define SERVO_LEFT_MAX 80
#define SERVO_RIGHT_MIN 60
#define SERVO_RIGHT_MAX 150
#define SERVO_GRIP_MIN 5
#define SERVO_GRIP_MAX 40

// macros to accomodate different size key names
#define JSON_TEMPLATE "{\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%d}"
#define LENGTH_OF_JSON_KEYS (sizeof(SERVO_BASE_NAME) + sizeof(SERVO_LEFT_NAME) + sizeof(SERVO_RIGHT_NAME) + sizeof(SERVO_GRIP_NAME))

Servo servoBase;  // base
Servo servoLeft;  // left
Servo servoRight; // right
Servo servoGrip;  // gripper

int targetBase = 90;
int targetLeft = 30;
int targetRight = 90;
int targetGrip = 20;

// helper function to publish integers e.g. for debugging
void publishInt(const char *name, int integer)
{
  char *buf;
  buf = (char *)malloc(sizeof(int));
  sprintf(buf, "%d", integer);
  Particle.publish(name, buf, 0, PRIVATE);
  free(buf);
}

int midpoint(int minimum, int maximum)
{
  return minimum + ((maximum - minimum) / 2);
}

void toggleServos()
{
  // when starting up, servos seem to get stuck. this moves them to one way then another.

  char *buf;                                   //hold the JSON string
  buf = (char *)malloc(sizeof(JSON_TEMPLATE) - //raw template
                       16 * sizeof(char) +     // 4 x %s and 4 x %d
                       LENGTH_OF_JSON_KEYS +   // the length of the text of the JSON keys
                       4 * sizeof(int));       // 4 integer values

  // first, let's set to the min values
  sprintf(buf,
          JSON_TEMPLATE,
          SERVO_BASE_NAME,
          SERVO_BASE_MIN,
          SERVO_LEFT_NAME,
          SERVO_LEFT_MIN,
          SERVO_RIGHT_NAME,
          SERVO_RIGHT_MIN,
          SERVO_GRIP_NAME,
          SERVO_GRIP_MIN);

  parseAndStoreJson(buf);
  setServos();
  delay(500); // wait for it to move

  // now return all to midpoints
  sprintf(buf,
          JSON_TEMPLATE,
          SERVO_BASE_NAME,
          midpoint(SERVO_BASE_MIN, SERVO_BASE_MAX),
          SERVO_LEFT_NAME,
          midpoint(SERVO_LEFT_MIN, SERVO_LEFT_MAX),
          SERVO_RIGHT_NAME,
          midpoint(SERVO_RIGHT_MIN, SERVO_RIGHT_MAX),
          SERVO_GRIP_NAME,
          midpoint(SERVO_GRIP_MIN, SERVO_GRIP_MAX));

  parseAndStoreJson(buf);
  setServos();
  free(buf);
}

void setup()
{
  if (!didAttachServer(servoBase.attach(SERVO_BASE_PIN), 0))
  {
    return;
  } //base
  if (!didAttachServer(servoLeft.attach(SERVO_LEFT_PIN), 1))
  {
    return;
  } //left
  if (!didAttachServer(servoRight.attach(SERVO_RIGHT_PIN), 2))
  {
    return;
  } //right
  if (!didAttachServer(servoGrip.attach(SERVO_GRIP_PIN), 3))
  {
    return;
  } //gripper

  Particle.function("moveServoFunc", moveServoFunc);

  // to let us monitor the values
  Particle.variable("targetBase", targetBase);
  Particle.variable("targetLeft", targetLeft);
  Particle.variable("targetRight", targetRight);
  Particle.variable("targetGrip", targetGrip);
  pinMode(D7, OUTPUT);
  toggleServos();
}
bool didAttachServer(bool result, uint16_t pin)
{
  // attach a servo to a pin and report success
  char servoUsed[3];
  sprintf(servoUsed, "%d", pin);
  if (result)
  {
    // Particle.publish("attached servo", servoUsed, 10, PRIVATE);
  }
  else
  {
    Particle.publish("Cannot attach to servo", servoUsed, 10, PRIVATE);
  }
  return result;
}

int moveServoFunc(String extra)
{
  parseAndStoreJson(extra);
  setServos();
  return 0;
}

/***************
/
/ parse + store JSON
/
***************/
bool parseJson(JsonDocument(*doc), const char *json)
// Parse the json document, return in *doc, bool if ok
{
  DeserializationError err = deserializeJson(*doc, json);
  if (err)
  {
    Particle.publish("DeserializationError", err.c_str(), 10, PRIVATE);
    return false;
  }
  else
  {
    Particle.publish("deserialized json", json, 21600, PRIVATE);
    return true;
  }
}

bool safeSetTargetValue(int *target, int *receivedValue, int minValue, int maxValue)
// compares the receivedValue to min + max, and if it's between them, assigns it to *target
{
  if ((*receivedValue >= minValue) && (*receivedValue <= maxValue))
  {
    *target = *receivedValue;
    return true;
  }
  return false;
}

void checkIfOutsideRange(int *target, int *receivedValue, const char *valueToCheck)
// checks if a value is wihtin allowed range for this specific servo
{
  bool setServo = false;
  if (strcmp(valueToCheck, SERVO_BASE_NAME) == 0)
  {
    setServo = safeSetTargetValue(target, receivedValue, SERVO_BASE_MIN, SERVO_BASE_MAX);
  }
  else if (strcmp(valueToCheck, SERVO_LEFT_NAME) == 0)
  {
    setServo = safeSetTargetValue(target, receivedValue, SERVO_LEFT_MIN, SERVO_LEFT_MAX);
  }
  else if (strcmp(valueToCheck, SERVO_RIGHT_NAME) == 0)
  {
    setServo = safeSetTargetValue(target, receivedValue, SERVO_RIGHT_MIN, SERVO_RIGHT_MAX);
  }
  else if (strcmp(valueToCheck, SERVO_GRIP_NAME) == 0)
  {
    setServo = safeSetTargetValue(target, receivedValue, SERVO_GRIP_MIN, SERVO_GRIP_MAX);
  }
  if (!setServo)
  {
    char *buf;
    buf = (char *)malloc(strlen(valueToCheck) + sizeof(int) + 8 * sizeof(char));
    sprintf(buf, "s: %s, v: %d", valueToCheck, *receivedValue);
    Particle.publish("failed to set servo", buf, 10, PRIVATE);
    free(buf);
  }
}

// fetches a value if it exists in teh JSON
bool safeGetKeyValue(JsonObject(*obj), int *target, const char *(valueToFetch))
{
  if (obj->containsKey(valueToFetch))
  {
    *target = obj->getMember(valueToFetch).as<int>();
    return true;
  }
  return false;
}

// fetches the value if it exists, then checks if it's inside the valid range before setting
void fetchValueAndSet(JsonObject(*objptr), int(*targetPtr), const char *(valueToFetch))
{
  int *receivedValuePtr = (int *)malloc(sizeof(int));
  if (safeGetKeyValue(objptr, receivedValuePtr, valueToFetch))
  {
    checkIfOutsideRange(targetPtr, receivedValuePtr, valueToFetch);
  }
  free(receivedValuePtr);
}

void parseAndStoreJson(const char *json)
{
  // receive the JSON for the servo states and store

  const size_t capacity = JSON_OBJECT_SIZE(4) + LENGTH_OF_JSON_KEYS;
  StaticJsonDocument<capacity> doc;

  if (!parseJson(&doc, json))
  {
    // didnt get a document...
    return;
  }

  // fetch values and set servos
  JsonObject obj = doc.as<JsonObject>();

  fetchValueAndSet(&obj, &targetBase, SERVO_BASE_NAME);
  fetchValueAndSet(&obj, &targetLeft, SERVO_LEFT_NAME);
  fetchValueAndSet(&obj, &targetRight, SERVO_RIGHT_NAME);
  fetchValueAndSet(&obj, &targetGrip, SERVO_GRIP_NAME);

  doc.clear(); // release memory
}

/***************
/
/ servo control
/
***************/
void setServos()
{
  // sets all 4 servos to the currently set values
  Particle.publish("setting servos", "", 21600, PRIVATE);
  writeToServo(&servoBase, targetBase);
  writeToServo(&servoLeft, targetLeft);
  writeToServo(&servoRight, targetRight);
  writeToServo(&servoGrip, targetGrip);

  // toggle LED to indicate complete
  digitalWrite(D7, HIGH - digitalRead(D7));
}
void writeToServo(Servo *servo, int target)
{
  //  writes a specific servo
  // separated out to allow e.g. a delay to be added etc.
  servo->write(target);
  delay(2);
}

void loop()
{
}