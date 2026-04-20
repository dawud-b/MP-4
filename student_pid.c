#include "student_pid.h"
#include "num.h"
#include <math.h>
#include <float.h>

/**
 * PID object initialization.
 *
 * @param[out] pid   A pointer to the pid object to initialize.
 * @param[in] desired  The initial set point.
 * @param[in] kp        The proportional gain
 * @param[in] ki        The integral gain
 * @param[in] kd        The derivative gain
 * @param[in] dt        Delta time since the last call
 * @param[in] samplingRate Frequency the update will be called
 * @param[in] cutoffFreq   Frequency to set the low pass filter cutoff at
 * @param[in] enableDFilter Enable setting for the D lowpass filter
 */
void studentPidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt,
             const float samplingRate, const float cutoffFreq,
             bool enableDFilter)
{
  //488 TODO initialize all the values in the PidObject struct

  studentPidSetDesired(pid, desired);
  studentPidSetKp(pid, kp);
  studentPidSetKi(pid, ki);
  studentPidSetKd(pid, kd);
  studentPidSetDt(pid, dt);
  studentPidReset(pid);

  studentPidSetIntegralLimit(pid, DEFAULT_PID_INTEGRATION_LIMIT);
  pid->last_error = 0;
  pid->ki_accumulation = 0;
  pid->output_limit = 0;
 
  //additional initialization for optional low pass filter
  pid->enableDFilter = enableDFilter;
  if (pid->enableDFilter)
  {
    lpf2pInit(&pid->dFilter, samplingRate, cutoffFreq);
  }
}

/**
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if studentPidSetError() has been used.
 * @return PID algorithm output
 */
float studentPidUpdate(PidObject* pid, const float measured, const bool updateError)
{

  // 488 TODO write base PID algorithm

  float error;
  if (updateError)
    error = pid->setpoint - measured;
  else
    error = pid->error;
  
  studentPidSetError(pid, error);
  
  float kp_component = pid->kp * error;
  float kd_component = pid->kd * ((error - pid->last_error) / pid->dt);
  pid->last_error = error;
  pid->ki_accumulation += error * pid->dt;
  float ki_component = pid->ki * pid->ki_accumulation;
  


    // 488 TODO optionally enable derivative low pass filtering
    /*
    if (pid->enableDFilter)
    {
      pid->deriv = lpf2pApply(&pid->dFilter, deriv);
      if (isnan(pid->deriv)) {
        pid->deriv = 0;
      }
    } else {
      pid->deriv = deriv;
    }
    */


    // 488 TODO Constrain the integral (unless the integral limit is zero), use the constrain function

    if (pid->integrationLimit != 0 && pid->ki_accumulation > pid->integrationLimit) {
      pid->ki_accumulation = pid->integrationLimit;
      ki_component = pid->ki * pid->ki_accumulation;
    }
    

    // 488 TODO Constrain the total PID output (unless the output Limit is zero)

    float pid_output = kp_component + ki_component + kd_component;
    if (pid->output_limit != 0 && pid_output > pid->output_limit)
      pid_output = pid->output_limit;

    return pid_output;
}

/**
 * Set the integral limit for this PID in deg.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] limit Pid integral swing limit.
 */
void studentPidSetIntegralLimit(PidObject* pid, const float limit) {
  pid->integrationLimit = limit;
}

/**
 * Reset the PID error values
 *
 * @param[in] pid   A pointer to the pid object.
 */
void studentPidReset(PidObject* pid)
{
  // 488 TODO
  pid->error = 0;
}

/**
 * Set a new error. Use if a special error calculation is needed.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] error The new error
 */
void studentPidSetError(PidObject* pid, const float error)
{
  pid->error = error;
}

/**
 * Set a new set point for the PID to track.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] angle The new set point
 */
void studentPidSetDesired(PidObject* pid, const float desired)
{
  pid->setpoint = desired;
}

/**
 * Get the current desired setpoint
 * 
 * @param[in] pid  A pointer to the pid object.
 * @return The set point
 */
float studentPidGetDesired(PidObject* pid)
{
  return pid->setpoint;
}


/**
 * Find out if PID is active
 * @return TRUE if active, FALSE otherwise
 */
bool studentPidIsActive(PidObject* pid)
{
  //488 TODO is active if the constants kp ki kd are above some small threshold
  return pid->kp > pid->kd_active_threshold && pid->ki > pid->ki_active_threshold && pid->kd > pid->kd_active_threshold;
}

/**
 * Set a new proportional gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kp    The new proportional gain
 */
void studentPidSetKp(PidObject* pid, const float kp)
{
  pid->kp = kp;
}

/**
 * Set a new integral gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] ki    The new integral gain
 */
void studentPidSetKi(PidObject* pid, const float ki)
{
  pid->ki = ki;
}

/**
 * Set a new derivative gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kd    The derivative gain
 */
void studentPidSetKd(PidObject* pid, const float kd)
{
  pid->kd = kd;
}

/**
 * Set a new dt gain for the PID. Defaults to IMU_UPDATE_DT upon construction
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] dt    Delta time
 */
void studentPidSetDt(PidObject* pid, const float dt) {
  pid->dt = dt;
}