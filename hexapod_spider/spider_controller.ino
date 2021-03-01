#include <Wire.h>
#include <SPI.h>

#include <Adafruit_PWMServoDriver.h>

#include <movingAvg.h> // https://github.com/JChristensen/movingAvg
#include <GeneralUtils.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

uint8_t JOINT_COUNT = 18;

#define JOINT_RIGHT_FRONT_0 0
#define JOINT_RIGHT_FRONT_1 1
#define JOINT_RIGHT_FRONT_2 2
#define JOINT_RIGHT_MIDDLE_0 3
#define JOINT_RIGHT_MIDDLE_1 4
#define JOINT_RIGHT_MIDDLE_2 5
#define JOINT_RIGHT_REAR_0 6
#define JOINT_RIGHT_REAR_1 7
#define JOINT_RIGHT_REAR_2 8

#define JOINT_LEFT_FRONT_0 9
#define JOINT_LEFT_FRONT_1 10
#define JOINT_LEFT_FRONT_2 11
#define JOINT_LEFT_MIDDLE_0 12
#define JOINT_LEFT_MIDDLE_1 13
#define JOINT_LEFT_MIDDLE_2 14
#define JOINT_LEFT_REAR_0 15
#define JOINT_LEFT_REAR_1 16
#define JOINT_LEFT_REAR_2 17

int JOINT_PWM_LOOKUP[] = {
    // Right front, board 1
    2,
    1,
    0,
    // Right middle, board 2
    16 + 2,
    16 + 1,
    16 + 0,
    // Right rear, board 1
    6,
    5,
    4,
    // Left front, board 2
    16 + 13,
    16 + 14,
    16 + 15,
    // Left middle, board 1
    13,
    14,
    15,
    // Left rear, board 2
    16 + 9,
    16 + 10,
    16 + 11,
};

// Mainly used for testing individual joints
// Turning all on at once may cause some serious power draw and jerking.
bool JOINT_ENABLED[] = {
    // Right front, shoulder, elbow, wrist
    true,
    true,
    true,
    // Right middle
    true,
    true,
    true,
    // Right rear
    true,
    true,
    true,
    // Left front
    true,
    true,
    true,
    // Left middle
    true,
    true,
    true,
    // Left rear
    true,
    true,
    true,
};

#define MOVE_TO_REST_POSITION_JOINT_DELAY 300
float JOINT_REST_ANGLE[] = {
    // Right front, shoulder, elbow, wrist
    90.0,
    90.0,
    90.0,
    // Right middle
    90.0,
    90.0,
    90.0,
    // Right rear
    90.0,
    90.0,
    90.0,
    // Left front
    90.0,
    90.0,
    90.0,
    // Left middle
    90.0,
    90.0,
    90.0,
    // Left rear
    90.0,
    90.0,
    90.0,
};

// Start servos all in center
#define INITIAL_JOINT_ANGLE 90.0
float jointAngle[] = {
    // Right front, shoulder, elbow, wrist
    INITIAL_JOINT_ANGLE,
    INITIAL_JOINT_ANGLE,
    INITIAL_JOINT_ANGLE,
    // Right middle
    INITIAL_JOINT_ANGLE,
    INITIAL_JOINT_ANGLE,
    INITIAL_JOINT_ANGLE,
    // Right rear
    INITIAL_JOINT_ANGLE,
    INITIAL_JOINT_ANGLE,
    INITIAL_JOINT_ANGLE,
    // Left front
    INITIAL_JOINT_ANGLE,
    INITIAL_JOINT_ANGLE,
    INITIAL_JOINT_ANGLE,
    // Left middle
    INITIAL_JOINT_ANGLE,
    INITIAL_JOINT_ANGLE,
    INITIAL_JOINT_ANGLE,
    // Left rear
    INITIAL_JOINT_ANGLE,
    INITIAL_JOINT_ANGLE,
    INITIAL_JOINT_ANGLE,
};

#define JOYSTICK_MIDPOINT 512
// How much joystick must move from midpoint to activate joint movement
#define JOYSTICK_SLOW_THRESHOLD 90
#define JOYSTICK_MEDIUM_THRESHOLD 300
#define JOYSTICK_FAST_THRESHOLD 462

// See: https://docs.google.com/spreadsheets/d/1TmXDisWLFvfgmEzKd0akO-7d5ffvYGMOTqIH83Ui-l8
#define STEPPER_PULSE_WIDTH_MIN 110
#define STEPPER_PULSE_WIDTH_MAX 540

#define TARGET_TICK_TIME 20

// Amount pulse width changes during a slow move
#define JOINT_ANGLE_DELTA_MOVE_SLOW 0.04
#define JOINT_ANGLE_DELTA_MOVE_MID 0.2
#define JOINT_ANGLE_DELTA_MOVE_FAST 1.0

// TODO: Seeing power issues, or over-drive issues, maybe speed adjustments here
// are not the correct approach, but seem to help.
float JOINT_SPEED_FACTOR[] = {
    // Right front, shoulder, elbow, wrist
    1.0,
    1.0,
    1.0,
    // Right middle
    1.0,
    1.0,
    1.0,
    // Right rear
    1.0,
    1.0,
    1.0,
    // Left front
    1.0,
    1.0,
    1.0,
    // Left middle
    1.0,
    1.0,
    1.0,
    // Left rear
    1.0,
    1.0,
    1.0,
};

// FIXME: J1 min angle should depend on angle of J2, 40 is generally safe with no attachment
// Many others are same way.
uint8_t JOINT_ANGLE_MIN[] = {
    // Right front, shoulder, elbow, wrist
    60, // 40 min, 80 safe
    30,
    30,
    // Right middle
    50,
    30,
    30,
    // Right rear
    30,
    30,
    30,
    // Left front
    60,
    35,
    30,
    // Left middle
    30,
    30,
    30,
    // Left rear
    30,
    35,
    30,
};

#define FRONT_ELBOW_MAX 150

uint8_t JOINT_ANGLE_MAX[] = {
    // Right front, shoulder, elbow, wrist
    160,
    FRONT_ELBOW_MAX,
    130,
    // Right middle
    105,
    130,
    150,
    // Right rear
    160,
    160,
    160,
    // Left front
    120, // 140 max, 100 safe
    FRONT_ELBOW_MAX,
    150,
    // Left middle
    160,
    160,
    160,
    // Left rear
    100,
    140,
    160,
};

bool JOINT_INVERT_ANGLE[] = {
    // Right front
    false,
    false,
    true,
    // Right middle
    false,
    false,
    true,
    // Right rear
    false,
    false,
    true,
    // Left front
    false,
    true,
    false,
    // Left middle
    false,
    true,
    false,
    // Left rear
    false,
    true,
    false,
};

// #define PIN_BUTTON_1 2
// #define PIN_BUTTON_2 3
// #define PIN_BUTTON_3 4

// unsigned int BUTTON_COUNT = 3;
// unsigned int BUTTON_PIN[] = {2, 3, 4};

// bool lastButtonState[] = {false, false, false};

#define STATE_SETUP 0
#define STATE_RUNNING 1

static int state = STATE_SETUP;

movingAvg averageTickDuration(8);

int lastAverageTickDuration = 0;

// static unsigned long lastButtonTwoUpdateTime = 0;
// static bool buttonTwoWantsUpdate = true;

// FIXME Run this with esp coop scheduler, should get better fps on robot
// task, and can de prioritize as frequent UI updates.

void setJointAngle(unsigned int jointIndex, float newAngle)
{
  float originalAngle = jointAngle[jointIndex];

  // Clamp angle
  if (newAngle < JOINT_ANGLE_MIN[jointIndex])
  {
    // Serial.print(jointIndex);
    // Serial.println("Angle less than min for this joint");

    newAngle = JOINT_ANGLE_MIN[jointIndex];
  }
  else if (newAngle > JOINT_ANGLE_MAX[jointIndex])
  {
    // Serial.print(jointIndex);
    // Serial.println("Angle greater than max for this joint");

    newAngle = JOINT_ANGLE_MAX[jointIndex];
  }

  // Map newAngle to a valid pulse width for servo
  int pulseWidth = map(
      newAngle,
      0,
      180,
      STEPPER_PULSE_WIDTH_MIN,
      STEPPER_PULSE_WIDTH_MAX);

  // !! No changes to newAngle past here !!

  // Save for next time
  jointAngle[jointIndex] = newAngle;

  // Send updated angle to pwm
  if (JOINT_ENABLED[jointIndex])
  {
    int jointPwm = JOINT_PWM_LOOKUP[jointIndex];
    if (jointPwm < 16)
    {
      pwm1.setPWM(jointPwm, 0, pulseWidth);
    }
    else
    {
      pwm2.setPWM(jointPwm - 16, 0, pulseWidth);
    }
  }

  if (originalAngle != newAngle)
  {
    Serial.print("joint i=");
    Serial.print(jointIndex);
    Serial.print(" d=");
    Serial.print(newAngle);
    Serial.print(" pw=");
    Serial.print(pulseWidth);
    Serial.print("\n");
  }
}

/**
 * Set joint angle as a percentage of min to max.
 * `0.5` is the midpoint.
 */
void setJointAngleNormal(int jointIndex, float newAngleNormal)
{
  if (newAngleNormal < 0.0)
  {
    newAngleNormal = 0.0;
  }
  else if (newAngleNormal > 1.0)
  {
    newAngleNormal = 1.0;
  }

  if (JOINT_INVERT_ANGLE[jointIndex])
  {
    newAngleNormal = 1.0 - newAngleNormal;
  }

  float targetAngle = ((float)(JOINT_ANGLE_MAX[jointIndex] - JOINT_ANGLE_MIN[jointIndex]) *
                           newAngleNormal +
                       (float)JOINT_ANGLE_MIN[jointIndex]);

  Serial.print("setJointAngleNormal: j=");
  Serial.print(jointIndex);
  Serial.print(" norm=");
  Serial.print(newAngleNormal);
  Serial.print(" deg=");
  Serial.println(targetAngle);

  setJointAngle(jointIndex, targetAngle);
}

void moveJointToRestPosition(int jointIndex, bool useDelay = true)
{
  Serial.print("Moving joint to rest position:");
  Serial.println(jointIndex);

  setJointAngle(jointIndex, JOINT_REST_ANGLE[jointIndex]);
  if (useDelay)
    delay(MOVE_TO_REST_POSITION_JOINT_DELAY);
}

// FIXME: Detect last position using FRAM IC. Detect power loss with VRAM IC.
// In case of power loss, servos may have lost positioning, and system should
// require guided user bring up ("press button to confirm J1").
void moveToRestPosition()
{
  Serial.println("Moving to rest position...");

  // Certain joints should be activated before others.
  // Group 1: Lift legs in air
  int group1[] = {
      JOINT_LEFT_FRONT_1,
      JOINT_LEFT_MIDDLE_1,
      JOINT_LEFT_REAR_1,
      JOINT_RIGHT_FRONT_1,
      JOINT_RIGHT_MIDDLE_1,
      JOINT_RIGHT_REAR_1,
  };
  for (int i = 0; i < 6; i++)
  {
    setJointAngle(group1[i], JOINT_REST_ANGLE[i]);
  }
  delay(MOVE_TO_REST_POSITION_JOINT_DELAY);

  // Group 2: Place shoulders
  int group2[] = {
      JOINT_LEFT_FRONT_0,
      JOINT_LEFT_MIDDLE_0,
      JOINT_LEFT_REAR_0,
      JOINT_RIGHT_FRONT_0,
      JOINT_RIGHT_MIDDLE_0,
      JOINT_RIGHT_REAR_0,
  };
  for (int i = 0; i < 6; i++)
  {
    setJointAngle(group2[i], JOINT_REST_ANGLE[i]);
  }
  delay(MOVE_TO_REST_POSITION_JOINT_DELAY);

  // Group 3: Place wrists
  int group3[] = {
      JOINT_LEFT_FRONT_2,
      JOINT_LEFT_MIDDLE_2,
      JOINT_LEFT_REAR_2,
      JOINT_RIGHT_FRONT_2,
      JOINT_RIGHT_MIDDLE_2,
      JOINT_RIGHT_REAR_2,
  };
  for (int i = 0; i < 6; i++)
  {
    setJointAngle(group3[i], JOINT_REST_ANGLE[i]);
  }
  delay(MOVE_TO_REST_POSITION_JOINT_DELAY);

  // Group 4: Place elbows
  int group4[] = {
      JOINT_LEFT_FRONT_1,
      JOINT_LEFT_MIDDLE_1,
      JOINT_LEFT_REAR_1,
      JOINT_RIGHT_FRONT_1,
      JOINT_RIGHT_MIDDLE_1,
      JOINT_RIGHT_REAR_1,
  };
  for (int i = 0; i < 6; i++)
  {
    setJointAngle(group4[i], JOINT_REST_ANGLE[i]);
  }
  delay(MOVE_TO_REST_POSITION_JOINT_DELAY);

  delay(100);
}

void flexJointToMin(int jointIndex)
{
  setJointAngleNormal(jointIndex, 0.1);
  delay(300);
  setJointAngle(jointIndex, JOINT_REST_ANGLE[jointIndex]);
  delay(200);
}

void flexArmToMin(int armJointStartIndex)
{
  int j0 = armJointStartIndex;
  int j1 = armJointStartIndex + 1;
  int j2 = armJointStartIndex + 2;

  setJointAngleNormal(j1, 0.01);
  setJointAngleNormal(j2, 0.01);
  delay(175);

  setJointAngleNormal(j0, 0.2);
  delay(175);
  setJointAngleNormal(j0, 0.8);
  delay(175);
  setJointAngle(j0, JOINT_REST_ANGLE[j0]);
  delay(175);

  setJointAngleNormal(j1, 0.8);
  setJointAngleNormal(j2, 0.8);
  delay(200);

  setJointAngle(j1, JOINT_REST_ANGLE[j1]);
  setJointAngle(j2, JOINT_REST_ANGLE[j2]);
  delay(100);
}

void setup()
{
  Serial.begin(9600);
  Serial.println("spider_controller");

  Serial.println("Starting PWM");
  pwm1.reset();
  pwm1.begin();
  // Analog servos run at ~50 Hz updates
  pwm1.setPWMFreq(50);

  pwm2.reset();
  pwm2.begin();
  // Analog servos run at ~50 Hz updates
  pwm2.setPWMFreq(50);

  averageTickDuration.begin();

  Serial.println("Setup done.");

  moveToRestPosition();

  state = STATE_RUNNING;
  delay(200);
  Serial.println("Running...");
}

unsigned long lastTickTime = 0;

void loop()
{
  unsigned long now = millis();

  if (Serial.available() > 0)
  {
    // read the incoming byte:
    int incomingByte = Serial.read();

    int incomingJointNumber = incomingByte - (int)'a';
    bool secondIncomingByte = Serial.read() == (int)']';
    if (incomingJointNumber >= 0 && incomingJointNumber < JOINT_COUNT)
    {
      // say what you got:
      Serial.print("I received: ");
      Serial.println(incomingJointNumber, DEC);
      Serial.println(secondIncomingByte ? "up" : "down");

      int direction = secondIncomingByte ? 1 : -1;

      setJointAngle(incomingJointNumber, jointAngle[incomingJointNumber] + 5 * direction);

      Serial.print("joint=");
      Serial.print(incomingJointNumber);
      Serial.print("angle=");
      Serial.println(jointAngle[incomingJointNumber]);
    }
    else if (incomingByte == 'z')
    {
      moveToRestPosition();
    }
    else if (incomingByte == 'x')
    {
      moveToRestPosition();

      // Flex each shoulder
      // flexJointToMin(JOINT_RIGHT_FRONT_1);
      // flexJointToMin(JOINT_LEFT_FRONT_1);
      // flexJointToMin(JOINT_RIGHT_MIDDLE_1);
      // flexJointToMin(JOINT_LEFT_MIDDLE_1);
      // flexJointToMin(JOINT_RIGHT_REAR_1);
      // flexJointToMin(JOINT_LEFT_REAR_1);

      flexArmToMin(JOINT_RIGHT_FRONT_0);
      flexArmToMin(JOINT_RIGHT_MIDDLE_0);
      flexArmToMin(JOINT_RIGHT_REAR_0);
      flexArmToMin(JOINT_LEFT_FRONT_0);
      flexArmToMin(JOINT_LEFT_MIDDLE_0);
      flexArmToMin(JOINT_LEFT_REAR_0);
    }
    else if (incomingByte == 'w')
    {
      // Walk command

      // First move to rest
      moveToRestPosition();

      // Walk forward one step
      for (int step_counter = 0; step_counter < 4; step_counter++)
      {
        // Lift right front leg
        setJointAngleNormal(JOINT_RIGHT_FRONT_1, 0);
        setJointAngleNormal(JOINT_RIGHT_FRONT_2, 0);
        // Lift left middle leg
        setJointAngleNormal(JOINT_LEFT_MIDDLE_1, 0);
        setJointAngleNormal(JOINT_LEFT_MIDDLE_2, 0);
        delay(25);
        // Move right front leg forward
        setJointAngleNormal(JOINT_RIGHT_FRONT_0, 0);
        // Move left middle leg forward
        setJointAngleNormal(JOINT_LEFT_MIDDLE_0, 0.8);
        delay(250);
        // Put right front leg on ground
        moveJointToRestPosition(JOINT_RIGHT_FRONT_1, false);
        moveJointToRestPosition(JOINT_RIGHT_FRONT_2, false);
        // Put left middle leg on ground
        moveJointToRestPosition(JOINT_LEFT_MIDDLE_1, false);
        moveJointToRestPosition(JOINT_LEFT_MIDDLE_2, false);
        delay(50);

        // // Lift left middle leg
        // setJointAngleNormal(JOINT_LEFT_MIDDLE_1, 0);
        // setJointAngleNormal(JOINT_LEFT_MIDDLE_2, 0);
        // delay(25);
        // // Move left middle leg forward
        // setJointAngleNormal(JOINT_LEFT_MIDDLE_0, 0.8);
        // delay(250);
        // // Put left middle leg on ground
        // moveJointToRestPosition(JOINT_LEFT_MIDDLE_1, false);
        // moveJointToRestPosition(JOINT_LEFT_MIDDLE_2, false);
        // delay(50);

        // Lift left front leg
        setJointAngleNormal(JOINT_LEFT_FRONT_1, 0);
        setJointAngleNormal(JOINT_LEFT_FRONT_2, 0);
        // Lift right middle leg
        setJointAngleNormal(JOINT_RIGHT_MIDDLE_1, 0);
        setJointAngleNormal(JOINT_RIGHT_MIDDLE_2, 0);
        delay(25);
        // Move right front leg forward
        setJointAngleNormal(JOINT_LEFT_FRONT_0, 1);
        delay(250);
        // Move right middle leg forward
        setJointAngleNormal(JOINT_RIGHT_MIDDLE_0, 0.2);
        // Put right front leg on ground
        moveJointToRestPosition(JOINT_LEFT_FRONT_1, false);
        moveJointToRestPosition(JOINT_LEFT_FRONT_2, false);
        // Put right middle leg on ground
        moveJointToRestPosition(JOINT_RIGHT_MIDDLE_1, false);
        moveJointToRestPosition(JOINT_RIGHT_MIDDLE_2, false);
        delay(50);

        // // Lift right middle leg
        // setJointAngleNormal(JOINT_RIGHT_MIDDLE_1, 0);
        // setJointAngleNormal(JOINT_RIGHT_MIDDLE_2, 0);
        // delay(25);
        // // Move right middle leg forward
        // setJointAngleNormal(JOINT_RIGHT_MIDDLE_0, 0.2);
        // delay(250);
        // // Put right middle leg on ground
        // moveJointToRestPosition(JOINT_RIGHT_MIDDLE_1, false);
        // moveJointToRestPosition(JOINT_RIGHT_MIDDLE_2, false);
        // delay(50);

        // Move front and middle to rest
        setJointAngle(JOINT_RIGHT_MIDDLE_0, JOINT_REST_ANGLE[JOINT_RIGHT_MIDDLE_0]);
        setJointAngle(JOINT_LEFT_MIDDLE_0, JOINT_REST_ANGLE[JOINT_LEFT_MIDDLE_0]);
        setJointAngle(JOINT_RIGHT_FRONT_0, JOINT_REST_ANGLE[JOINT_RIGHT_FRONT_0]);
        setJointAngle(JOINT_LEFT_FRONT_0, JOINT_REST_ANGLE[JOINT_LEFT_FRONT_0]);

        // Move back legs back, and stretch a bit
        setJointAngleNormal(JOINT_LEFT_REAR_0, 0.1);
        setJointAngle(
            JOINT_LEFT_REAR_2,
            JOINT_REST_ANGLE[JOINT_LEFT_REAR_2] - 10);

        setJointAngleNormal(JOINT_RIGHT_REAR_0, 0.9);
        setJointAngle(
            JOINT_RIGHT_REAR_2,
            JOINT_REST_ANGLE[JOINT_RIGHT_REAR_2] + 10);
        delay(300);

        // pick up left rear leg then move to rest pos
        setJointAngleNormal(JOINT_LEFT_REAR_1, 0);
        delay(50);
        setJointAngle(
            JOINT_LEFT_REAR_0,
            JOINT_REST_ANGLE[JOINT_LEFT_REAR_0] - 10);
        delay(250);
        setJointAngle(JOINT_LEFT_REAR_1, JOINT_REST_ANGLE[JOINT_LEFT_REAR_1]);
        setJointAngle(
            JOINT_LEFT_REAR_2,
            JOINT_REST_ANGLE[JOINT_LEFT_REAR_2] + 10);
        delay(175);

        // pick up right rear leg then move to rest pos
        setJointAngleNormal(JOINT_RIGHT_REAR_1, 0);
        delay(50);
        setJointAngle(
            JOINT_RIGHT_REAR_0,
            JOINT_REST_ANGLE[JOINT_RIGHT_REAR_0] + 10);
        delay(250);
        setJointAngle(JOINT_RIGHT_REAR_1, JOINT_REST_ANGLE[JOINT_RIGHT_REAR_1]);
        setJointAngle(
            JOINT_RIGHT_REAR_2,
            JOINT_REST_ANGLE[JOINT_RIGHT_REAR_2] - 10);
        delay(175);
      }
    }
  }

  // Target 20ms tick
  delayForTargetTickDuration(&now, TARGET_TICK_TIME, &lastTickTime);
}
