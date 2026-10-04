#include <math.h>

const byte trigPin = 9;
const byte echoPin = 10;

const byte led1Pin = 3;
const byte led2Pin = 5;
const byte buzzerPin = 8;

/*
  The sketch measures distance about every 70 ms.

  This makes the distance history update quickly enough to detect
  a rapid movement toward the sensor.
*/
const unsigned long sampleIntervalMs = 70;
const unsigned long echoTimeoutUs = 30000;

/*
  HC-SR04 measurements below about 2 cm are not dependable.
  A value outside these limits is treated as invalid.
*/
const float minValidDistanceCm = 2.0;
const float maxValidDistanceCm = 350.0;

/*
  Take three raw readings and use the middle one. This filters a
  single noisy reading.
*/
const byte filterSampleCount = 3;

/*
  Ignore one extremely unrealistic valid distance jump.
*/
const float maxDistanceJumpCm = 70.0;

/*
  The last valid distance must be this close before an invalid next
  reading can be considered part of a possible fall.
*/
const float approachingZeroDistanceCm = 12.0;

/*
  The object must move toward the sensor quickly.

  Distance dropping by at least 3 cm in one processed sample and
  6 cm across two processed samples rejects slow movement toward zero.
*/
const float minimumDistanceDropPerSampleCm = 3.0;
const float minimumTwoSampleDropCm = 6.0;

/*
  High acceleration threshold.

  fabs(a1) is used, which means the code checks acceleration magnitude.
  It triggers whether acceleration calculates as positive or negative,
  as long as it is large enough.
*/
const float highAccelerationThreshold = 200.0;

/*
  If p1 and p2 are similar, the logarithmic comparison is near 100.

  Values below 85 or above 115 count as different.
*/
const float similarityLow = 85.0;
const float similarityHigh = 115.0;

/*
  Fall alert outputs stay active for 8 seconds.
*/
const unsigned long fallAlertHoldMs = 8000;
const unsigned int fallBuzzerFrequency = 1500;

/*
  Manual reminder timer.

  Set this to the number of minutes after upload/reset before the
  reminder begins. It is 1440 minutes = 24 hours by default, so it
  does not interfere with immediate fall-detection testing.
*/
const unsigned long reminderDelayMinutes = 1440;

const unsigned long reminderDurationMs = 30000;
const unsigned long reminderBlinkIntervalMs = 600;
const unsigned int reminderBuzzerFrequency = 550;

const unsigned long reminderDelayMs =
  reminderDelayMinutes * 60UL * 1000UL;


/*
  Distance history:

  d0 = oldest valid distance
  d1 = middle valid distance
  d2 = newest valid distance
*/
float d0 = 0.0;
float d1 = 0.0;
float d2 = 0.0;

/*
  Velocity history:

  v0 = prior velocity
  v1 = newest velocity
*/
float v0 = 0.0;
float v1 = 0.0;

/*
  Acceleration history:

  a0 = prior acceleration
  a1 = newest acceleration
*/
float a0 = 0.0;
float a1 = 0.0;

bool haveD0 = false;
bool haveD1 = false;
bool haveD2 = false;

bool haveV0 = false;
bool haveV1 = false;

bool haveA0 = false;
bool haveA1 = false;

float lastAcceptedDistance = -1.0;

unsigned long lastSampleTime = 0;
unsigned long lastValidSampleAt = 0;

bool fallAlertActive = false;
unsigned long fallAlertStartedAt = 0;

bool reminderHasStarted = false;
bool reminderActive = false;
bool reminderOutputsOn = false;

unsigned long reminderStartedAt = 0;
unsigned long reminderLastBlinkAt = 0;

unsigned long sketchStartedAt = 0;
unsigned long lastCountdownPrintAt = 0;


void allOutputsOff() {
  digitalWrite(led1Pin, LOW);
  digitalWrite(led2Pin, LOW);
  noTone(buzzerPin);
}

void startFallAlert() {
  /*
    A fall alert overrides the reminder.
  */
  reminderActive = false;

  digitalWrite(led1Pin, HIGH);
  digitalWrite(led2Pin, HIGH);
  tone(buzzerPin, fallBuzzerFrequency);

  fallAlertActive = true;
  fallAlertStartedAt = millis();
}

void updateFallAlert() {
  if (!fallAlertActive) {
    return;
  }

  if (millis() - fallAlertStartedAt >= fallAlertHoldMs) {
    fallAlertActive = false;
    allOutputsOff();
  }
}

void startReminder() {
  if (fallAlertActive) {
    return;
  }

  reminderHasStarted = true;
  reminderActive = true;
  reminderOutputsOn = true;

  reminderStartedAt = millis();
  reminderLastBlinkAt = millis();

  digitalWrite(led1Pin, HIGH);
  digitalWrite(led2Pin, HIGH);
  tone(buzzerPin, reminderBuzzerFrequency);

  Serial.println("Manual reminder started");
}

void updateReminder() {
  if (!reminderActive || fallAlertActive) {
    return;
  }

  unsigned long now = millis();

  if (now - reminderStartedAt >= reminderDurationMs) {
    reminderActive = false;
    allOutputsOff();

    Serial.println("Manual reminder finished");
    return;
  }

  if (now - reminderLastBlinkAt >= reminderBlinkIntervalMs) {
    reminderLastBlinkAt = now;
    reminderOutputsOn = !reminderOutputsOn;

    if (reminderOutputsOn) {
      digitalWrite(led1Pin, HIGH);
      digitalWrite(led2Pin, HIGH);
      tone(buzzerPin, reminderBuzzerFrequency);
    } else {
      digitalWrite(led1Pin, LOW);
      digitalWrite(led2Pin, LOW);
      noTone(buzzerPin);
    }
  }
}

void checkManualReminderTimer() {
  if (!reminderHasStarted &&
      millis() - sketchStartedAt >= reminderDelayMs) {
    startReminder();
  }
}

void printReminderCountdown() {
  if (reminderHasStarted) {
    return;
  }

  unsigned long now = millis();

  if (now - lastCountdownPrintAt < 1000) {
    return;
  }

  lastCountdownPrintAt = now;

  unsigned long elapsedMs = now - sketchStartedAt;
  unsigned long remainingMs = 0;

  if (elapsedMs < reminderDelayMs) {
    remainingMs = reminderDelayMs - elapsedMs;
  }

  unsigned long remainingSeconds = remainingMs / 1000UL;

  unsigned long days = remainingSeconds / 86400UL;
  remainingSeconds %= 86400UL;

  unsigned long hours = remainingSeconds / 3600UL;
  remainingSeconds %= 3600UL;

  unsigned long minutes = remainingSeconds / 60UL;
  unsigned long seconds = remainingSeconds % 60UL;

  Serial.print("Reminder countdown: ");

  if (days < 10) Serial.print("0");
  Serial.print(days);
  Serial.print("d ");

  if (hours < 10) Serial.print("0");
  Serial.print(hours);
  Serial.print("h ");

  if (minutes < 10) Serial.print("0");
  Serial.print(minutes);
  Serial.print("m ");

  if (seconds < 10) Serial.print("0");
  Serial.print(seconds);
  Serial.println("s");
}


/*
  Send an ultrasonic trigger and measure the returned Echo pulse.

  Returns:
  - A positive distance in centimeters for a valid echo.
  - -1.0 if the Echo pulse timed out or is outside the accepted range.
*/
float readRawDistanceCm() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  /*
    The HC-SR04 starts a measurement from a 10 microsecond HIGH pulse.
  */
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, echoTimeoutUs);

  /*
    pulseIn() gives 0 when a complete echo pulse does not arrive before
    the timeout. That is treated as invalid, not as a true zero distance.
  */
  if (duration == 0) {
    return -1.0;
  }

  /*
    Sound travels out to the target and back, so divide by 2.

    distance = echo time × speed of sound / 2
  */
  float distanceCm = (duration * 0.0343) / 2.0;

  if (distanceCm < minValidDistanceCm ||
      distanceCm > maxValidDistanceCm) {
    return -1.0;
  }

  return distanceCm;
}

void sortValues(float values[], byte count) {
  for (byte i = 0; i < count - 1; i++) {
    for (byte j = i + 1; j < count; j++) {
      if (values[j] < values[i]) {
        float temporary = values[i];
        values[i] = values[j];
        values[j] = temporary;
      }
    }
  }
}

float readFilteredDistanceCm() {
  float readings[filterSampleCount];
  byte validCount = 0;

  for (byte i = 0; i < filterSampleCount; i++) {
    float reading = readRawDistanceCm();

    if (reading > 0.0) {
      readings[validCount] = reading;
      validCount++;
    }

    delay(5);
  }

  if (validCount < 2) {
    return -1.0;
  }

  sortValues(readings, validCount);

  return readings[validCount / 2];
}

void resetMotionHistory() {
  haveD0 = false;
  haveD1 = false;
  haveD2 = false;

  haveV0 = false;
  haveV1 = false;

  haveA0 = false;
  haveA1 = false;

  d0 = 0.0;
  d1 = 0.0;
  d2 = 0.0;

  v0 = 0.0;
  v1 = 0.0;

  a0 = 0.0;
  a1 = 0.0;

  lastAcceptedDistance = -1.0;
  lastValidSampleAt = 0;
}

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  allOutputsOff();

  Serial.begin(9600);

  sketchStartedAt = millis();

  Serial.println("Fall detection ready");
  Serial.println("Distance readings print continuously.");
}


/*
  Main program
*/
void loop() {
  printReminderCountdown();

  updateFallAlert();
  updateReminder();
  checkManualReminderTimer();

  /*
    Do not overwrite the LED/buzzer state while a fall alert is active.
  */
  if (fallAlertActive) {
    return;
  }

  unsigned long now = millis();

  if (now - lastSampleTime < sampleIntervalMs) {
    return;
  }

  lastSampleTime = now;

  float newDistance = readFilteredDistanceCm();

  /*
    An invalid reading is only checked as a possible fall after the
    valid readings leading into it approached close range quickly.
  */
  if (newDistance < 0.0) {
    Serial.println("Distance: INVALID / NO ECHO");

    float oneSampleDistanceDrop = d1 - d2;
    float twoSampleDistanceDrop = d0 - d2;

    bool enoughHistory =
      haveD0 &&
      haveD1 &&
      haveD2 &&
      haveV1 &&
      haveA0 &&
      haveA1;

    bool closeToSensor =
      d2 <= approachingZeroDistanceCm;

    bool movingTowardSensor =
      v1 < 0.0;

    /*
      This prevents slow changes from qualifying for a fall check.

      Example of a slow approach:
      20 cm -> 18 cm -> 16 cm

      Example of a faster approach:
      20 cm -> 13 cm -> 6 cm
  */
    bool rapidlyApproachingZero =
      oneSampleDistanceDrop >= minimumDistanceDropPerSampleCm &&
      twoSampleDistanceDrop >= minimumTwoSampleDropCm;

    bool shouldCheckForFall =
      enoughHistory &&
      closeToSensor &&
      movingTowardSensor &&
      rapidlyApproachingZero;

    if (shouldCheckForFall) {
      Serial.println();
      Serial.println("========== CHECKING FOR FALL ==========");

      /*
        THE VELOCITY IDEA

        Velocity is the first derivative of distance:

            velocity = change in distance / change in time

        A negative velocity means the distance got smaller, so the
        target moved toward the sensor.
      */

      /*
        THE ACCELERATION IDEA

        Acceleration is the derivative of velocity:

            acceleration = change in velocity / change in time

        A large acceleration magnitude means the speed changed sharply.
        This code requires the magnitude to be at least 200 cm/s^2.
      */

      /*
        p1 and p2 are built from two acceleration-related values.

        p1 is the change between the current and previous acceleration.
        p2 is the size of the current acceleration.

        fabs() makes both values positive.
        +2 keeps both values greater than 1, which is important because
        logarithm bases cannot be 0, 1, or negative.
      */
      float p1 = fabs(a1 - a0) + 2.0;
      float p2 = fabs(a1) + 2.0;

      p1 = constrain(p1, 2.0, 100000.0);
      p2 = constrain(p2, 2.0, 100000.0);

      /*
        HOW THE LOGARITHM COMPARISON WORKS

        The requested math is:

            p2 ^ (log base p1 of 100)

        A logarithm answers this question:

            "What exponent must the base be raised to in order
             to equal a chosen number?"

        For example:

            log base 10 of 100 = 2

        because:

            10 ^ 2 = 100

        Arduino's log(x) is the natural logarithm, so Arduino does not
        directly write log(value, base). The change-of-base rule is:

            log base p1 of 100 = log(100) / log(p1)

        Therefore, the Arduino calculation becomes:

            pow(p2, log(100) / log(p1))

        pow(base, exponent) raises the base to the exponent.

        If p1 equals p2 exactly:
        - log base p1 of 100 is the exponent that makes p1 equal 100.
        - Raising p2, which equals p1, to that same exponent also gives 100.
        - So similarity becomes 100.

        Therefore:
        - similarity near 100 means p1 and p2 are similar.
        - similarity far from 100 means p1 and p2 differ.
      */
      float similarity = pow(p2, log(100.0) / log(p1));

      /*
        A fall is declared only if:
        1. The fast close approach happened.
        2. The next reading became invalid.
        3. Acceleration magnitude was high.
        4. The power comparison was different from 100.
      */
      bool highAcceleration =
        fabs(a1) >= highAccelerationThreshold;

      bool powersAreDifferent =
        similarity < similarityLow ||
        similarity > similarityHigh;

      bool likelyFall =
        highAcceleration &&
        powersAreDifferent;

      Serial.print("Distances: ");
      Serial.print(d0, 1);
      Serial.print(" -> ");
      Serial.print(d1, 1);
      Serial.print(" -> ");
      Serial.print(d2, 1);
      Serial.println(" cm");

      Serial.print("One-sample drop: ");
      Serial.print(oneSampleDistanceDrop, 1);
      Serial.print(" cm | Required: ");
      Serial.println(minimumDistanceDropPerSampleCm, 1);

      Serial.print("Two-sample drop: ");
      Serial.print(twoSampleDistanceDrop, 1);
      Serial.print(" cm | Required: ");
      Serial.println(minimumTwoSampleDropCm, 1);

      Serial.print("Velocity: ");
      Serial.print(v1, 1);
      Serial.println(" cm/s");

      Serial.print("Acceleration: ");
      Serial.print(a1, 1);
      Serial.println(" cm/s^2");

      Serial.print("Acceleration magnitude: ");
      Serial.print(fabs(a1), 1);
      Serial.print(" cm/s^2 | Required: ");
      Serial.println(highAccelerationThreshold, 1);

      Serial.print("p1: ");
      Serial.print(p1, 1);
      Serial.print(" | p2: ");
      Serial.println(p2, 1);

      Serial.print("Power similarity: ");
      Serial.print(similarity, 1);
      Serial.print(" | Different if below ");
      Serial.print(similarityLow, 1);
      Serial.print(" or above ");
      Serial.println(similarityHigh, 1);

      if (likelyFall) {
        Serial.println("RESULT: FALL");
        startFallAlert();
      } else {
        Serial.println("RESULT: NOT A FALL");
      }

      Serial.println("========================================");
    }

    resetMotionHistory();
    return;
  }

  /*
    Reject a single impossible large jump.
  */
  if (lastAcceptedDistance > 0.0 &&
      fabs(newDistance - lastAcceptedDistance) > maxDistanceJumpCm) {
    Serial.println("Distance: REJECTED LARGE JUMP");
    return;
  }

  lastAcceptedDistance = newDistance;

  /*
    Print each valid filtered distance measurement.
  */
  Serial.print("Distance: ");
  Serial.print(newDistance, 1);
  Serial.println(" cm");

  unsigned long acceptedAt = millis();

  if (!haveD0) {
    d0 = newDistance;
    haveD0 = true;
    lastValidSampleAt = acceptedAt;
    return;
  }

  if (!haveD1) {
    d1 = newDistance;
    haveD1 = true;
    lastValidSampleAt = acceptedAt;
    return;
  }

  if (!haveD2) {
    d2 = newDistance;
    haveD2 = true;
    lastValidSampleAt = acceptedAt;
    return;
  }

  /*
    Use the real time between accepted distance readings.

    This is more accurate than assuming every filtered reading took
    exactly 70 ms.
  */
  float dt = (acceptedAt - lastValidSampleAt) / 1000.0;
  lastValidSampleAt = acceptedAt;

  if (dt <= 0.0) {
    return;
  }

  d0 = d1;
  d1 = d2;
  d2 = newDistance;

  /*
    First derivative of distance: velocity.

        velocity = (new distance - previous distance) / time

    Negative velocity means the target moved closer.
  */
  float newVelocity = (d2 - d1) / dt;

  if (!haveV0) {
    v0 = newVelocity;
    haveV0 = true;
    return;
  }

  if (!haveV1) {
    v1 = newVelocity;
    haveV1 = true;
    return;
  }

  v0 = v1;
  v1 = newVelocity;

  /*
    Derivative of velocity: acceleration.

        acceleration = (new velocity - previous velocity) / time
  */
  float newAcceleration = (v1 - v0) / dt;

  if (!haveA0) {
    a0 = newAcceleration;
    haveA0 = true;
    return;
  }

  if (!haveA1) {
    a1 = newAcceleration;
    haveA1 = true;
    return;
  }

  a0 = a1;
  a1 = newAcceleration;
}