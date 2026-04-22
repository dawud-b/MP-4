
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "student_attitude_controller.h"
#include "sensfusion6.h"
#include "controller_student.h"

#include "log.h"
#include "debug.h"

#include "param.h"
#include "math3d.h"

//delta time between calls to the update function
#define STUDENT_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

//desired vehicle state as calculated by PID controllers
static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float thrustDesired;

//variables used only for logging PID command outputs
static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

//Dummy variables for test stand
static float angle;
static float rate;

void controllerStudentInit(void)
{
  studentAttitudeControllerInit(STUDENT_UPDATE_DT);
}

bool controllerStudentTest(void)
{
  bool pass = true;
  //controller passes check if attitude controller passes
  pass &= studentAttitudeControllerTest();

  return pass;
}


/**
 * Limit the input angle between -180 and 180
 * 
 * @param angle 
 * @return float 
 */
static float capAngle(float angle) {
  if (angle > 180)
    return 180;
  else if (angle < -180)
    return -180;

  return angle;
}


/**
 * This function is called periodically to update the PID loop,
 * Reads state estimate and setpoint values and passes them
 * to the functions that perform PID calculations,
 * attitude PID and attitude rate PID
 * 
 * @param control Output, struct is modified as the ouput of the control loop
 * @param setpoint Input, setpoints for thrust, attitude, position, velocity etc. of the quad
 * @param sensors Input, Raw sensor values (typically want to use the state estimated instead) includes gyro, 
 * accelerometer, barometer, magnatometer 
 * @param state Input, A more robust way to measure the current state of the quad, allows for direct
 * measurements of the orientation of the quad. Includes attitude, position, velocity,
 * and acceleration
 * @param tick Input, system clock
 */
void controllerStudent(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick)
{

  // Main Controller Function

  // check if time to update the attutide controller
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {

    //only support attitude and attitude rate control
    if(setpoint->mode.x != modeDisable || setpoint->mode.y != modeDisable || setpoint->mode.z != modeDisable){
      DEBUG_PRINT("Student controller does not support vehicle position or velocity mode. Check flight mode.");
      control->thrust = 0;
      control->roll = 0;
      control->pitch = 0;
      control->yaw = 0;
      return;
    }
    
    // 488 TODO set desired attitude, roll, pitch, and yaw angles
    attitudeDesired.roll = setpoint->attitude.roll;
    attitudeDesired.pitch = setpoint->attitude.pitch;
    attitudeDesired.yaw = setpoint->attitude.yaw;

    // 488 TODO if yaw is in rate mode, move the yaw angle setpoint accordingly


    // 488 TODO set desired thrust
    thrustDesired = setpoint->thrust;

    // 488 TODO Run the attitude controller update with the actual attitude and desired attitude
    // outputs the desired attitude rates
      // attitude control
      // use values given by setpoint.attitude
    studentAttitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw, 
      attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
      &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

    // 488 TODO if velocity mode, overwrite rateDesired output
    // from the attitude controller with the setpoint value
    // Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.x == modeDisable && setpoint->mode.y == modeDisable && setpoint->mode.z == modeDisable && setpoint->mode.roll == modeVelocity && setpoint->mode.yaw == modeVelocity && setpoint->mode.pitch == modeVelocity)
    {
      rateDesired.roll = setpoint->attitudeRate.roll;
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      rateDesired.yaw = setpoint->attitudeRate.yaw;
      studentAttitudeControllerResetAllPID();
    }

    // 488 TODO update the attitude rate PID, given the current angular rate 
    // read by the gyro and the desired rate
    
    // attitude rate control
    studentAttitudeControllerCorrectRatePID(sensors->gyro.y, sensors->gyro.x, sensors->gyro.z, 
      rateDesired.roll, rateDesired.pitch, rateDesired.yaw,
      &(control->roll), &(control->pitch), &(control->yaw));

  }

  //488 TODO set control->thrust
  control->thrust = thrustDesired;

  //488 TODO if no thrust active, set all outputs to 0 and reset PID variables
  if (control->thrust == 0) {
    control->thrust = 0;
    control->roll   = 0;
    control->pitch  = 0;
    control->yaw    = 0;
    studentAttitudeControllerResetAllPID();
  }

  //copy values for logging
  cmd_thrust = control->thrust;
  cmd_roll = control->roll;
  cmd_pitch = control->pitch;
  cmd_yaw = control->yaw;
  r_roll = sensors->gyro.x;
  r_pitch = sensors->gyro.y;
  r_yaw = sensors->gyro.z;
  accelz = sensors->acc.z;
}

/**
 * Logging variables for the command and reference signals for the
 * student PID controller
 */
LOG_GROUP_START(ctrlStdnt)

// 488 TODO setup logging parameters, replace null with pointer to globabl variable

/**
 * @brief Thrust command output
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command output
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command output
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command output
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in degrees
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in degrees
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Gyro yaw rate measurement in degrees
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the z axis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll, NULL)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch, NULL)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw, NULL)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate, NULL)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, NULL)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate, NULL)

LOG_GROUP_STOP(ctrlStdnt)

/**
 *  Test Stand Logging Variables
 */

LOG_GROUP_START(Test_Stand)

/**
 * @brief Test_Stand.angle
 */
LOG_ADD(LOG_FLOAT, angle, &angle)
/**
 * @brief Test_Stand.rate
 */
LOG_ADD(LOG_FLOAT, rate, &rate)

LOG_GROUP_STOP(Test_Stand)



/**
 * Controller parameters
 */
PARAM_GROUP_START(ctrlStdnt)

//488 TODO optionally add any parameters to modify the controller code while running

PARAM_ADD(PARAM_FLOAT, placeHolder, NULL)

PARAM_GROUP_STOP(ctrlStdnt)