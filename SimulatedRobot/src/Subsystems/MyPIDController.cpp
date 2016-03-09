/*
 * MyPIDController.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: dean
 */

#include <Subsystems/MyPIDController.h>

#define USE_MUTEX

#ifdef USE_MUTEX
#define SYNC_MUTEX std::lock_guard<std::recursive_mutex> sync(m_mutex)
#define LOCK_MUTEX std::lock_guard<std::recursive_mutex> lock(m_mutex)
#else
#define SYNC_MUTEX
#define LOCK_MUTEX
#endif
MyPIDController::MyPIDController(float Kp, float Ki, float Kd, PIDSource *source, PIDOutput *output, float rate)
{
	Initialize(Kp, Ki, Kd,source, output,rate);
}

MyPIDController::~MyPIDController() {
	// TODO Auto-generated destructor stub
}


void MyPIDController::Initialize(float Kp, float Ki, float Kd,
								PIDSource *source, PIDOutput *output,float rate)
{

	m_P = Kp;
	m_I = Ki;
	m_D = Kd;

	m_maximumOutput = 1.0;
	m_minimumOutput = -1.0;

	m_maximumInput = 0;
	m_minimumInput = 0;

	m_enabled = false;
	m_setpoint = 0;

	m_prevError = 0;
	m_totalError = 0;
	m_tolerance = .05;

	m_result = 0;

	m_pidInput = source;
	m_pidOutput = output;
	m_period = rate;

	m_controlLoop = std::make_unique<MyNotifier>(&MyPIDController::Calculate, this);
	m_controlLoop->StartPeriodic(m_period);

	static int32_t instances = 0;
	instances++;
}

void MyPIDController::Calculate()
{
	bool enabled;
	PIDSource *pidInput;

	{
		LOCK_MUTEX;
		if (m_pidInput == 0) return;
		if (m_pidOutput == 0) return;
		enabled = m_enabled;
		pidInput = m_pidInput;
	}

	if (enabled)
	{
		float input = pidInput->PIDGet();
		float result;
		PIDOutput *pidOutput;

		{
			SYNC_MUTEX;
			m_error = m_setpoint - input;
            if (m_pidInput->GetPIDSourceType() == PIDSourceType::kRate) {
              if (m_P != 0) {
                double potentialPGain = (m_totalError + m_error) * m_P;
                if (potentialPGain < m_maximumOutput) {
                  if (potentialPGain > m_minimumOutput) {
                    m_totalError += m_error;
                  }
                  else {
                    m_totalError = m_minimumOutput / m_P;
                  }
                }
                else {
                  m_totalError = m_maximumOutput / m_P;
                }
              }

              m_result = m_D * m_error + m_P * m_totalError;
            }
            else {
              if (m_I != 0) {
                double potentialIGain = (m_totalError + m_error) * m_I;
                if (potentialIGain < m_maximumOutput) {
                  if (potentialIGain > m_minimumOutput) {
                    m_totalError += m_error;
                  }
                  else {
                    m_totalError = m_minimumOutput / m_I;
                  }
                }
                else {
                  m_totalError = m_maximumOutput / m_I;
                }
              }

              m_result = m_P * m_error + m_I * m_totalError +
                         m_D * (m_error - m_prevError);
            }
			m_prevError = m_error;

			if (m_result > m_maximumOutput) m_result = m_maximumOutput;
			else if (m_result < m_minimumOutput) m_result = m_minimumOutput;

			pidOutput = m_pidOutput;
			result = m_result;
		}

		pidOutput->PIDWrite(result);
	}
}
/**
 * Set the PID Controller gain parameters.
 * Set the proportional, integral, and differential coefficients.
 * @param p Proportional coefficient
 * @param i Integral coefficient
 * @param d Differential coefficient
 */
void MyPIDController::SetPID(double p, double i, double d)
{
	{
		LOCK_MUTEX;
		m_P = p;
		m_I = i;
		m_D = d;
	}
}

/**
 * Get the Proportional coefficient
 * @return proportional coefficient
 */
double MyPIDController::GetP() const
{
	LOCK_MUTEX;
	return m_P;
}

/**
 * Get the Integral coefficient
 * @return integral coefficient
 */
double MyPIDController::GetI() const
{
	LOCK_MUTEX;
	return m_I;
}

/**
 * Get the Differential coefficient
 * @return differential coefficient
 */
double MyPIDController::GetD() const
{
	LOCK_MUTEX;
	return m_D;
}
/**
 * Return the current PID result
 * This is always centered on zero and constrained the the max and min outs
 * @return the latest calculated output
 */
float MyPIDController::Get() const
{
	LOCK_MUTEX;
	return m_result;
}

/**
 * Sets the maximum and minimum values expected from the input.
 *
 * @param minimumInput the minimum value expected from the input
 * @param maximumInput the maximum value expected from the output
 */
void MyPIDController::SetInputRange(float minimumInput, float maximumInput)
{
	{
		LOCK_MUTEX;
		m_minimumInput = minimumInput;
		m_maximumInput = maximumInput;
	}

	SetSetpoint(m_setpoint);
}

/**
 * Sets the minimum and maximum values to write.
 *
 * @param minimumOutput the minimum value to write to the output
 * @param maximumOutput the maximum value to write to the output
 */
void MyPIDController::SetOutputRange(float minimumOutput, float maximumOutput)
{
	LOCK_MUTEX;
	m_minimumOutput = minimumOutput;
	m_maximumOutput = maximumOutput;
}

/**
 * Set the setpoint for the MYPIDController
 * @param setpoint the desired setpoint
 */
void MyPIDController::SetSetpoint(float setpoint)
{
	{
		LOCK_MUTEX;

		if (m_maximumInput > m_minimumInput)
		{
			if (setpoint > m_maximumInput)
				m_setpoint = m_maximumInput;
			else if (setpoint < m_minimumInput)
				m_setpoint = m_minimumInput;
			else
				m_setpoint = setpoint;
		}
		else
		{
			m_setpoint = setpoint;
		}
	}
}
/**
 * Returns the current setpoint of the MYPIDController
 * @return the current setpoint
 */
double MyPIDController::GetSetpoint() const
{
	LOCK_MUTEX;
	return m_setpoint;
}


/**
 * Retruns the current difference of the input from the setpoint
 * @return the current error
 */
float MyPIDController::GetError() const
{
	double pidInput;
	{
		LOCK_MUTEX;
		pidInput = m_pidInput->PIDGet();
	}
	return GetSetpoint() - pidInput;
}

/**
 * Sets what type of input the PID controller will use
 */
void MyPIDController::SetPIDSourceType(PIDSourceType pidSource) {
  m_pidInput->SetPIDSourceType(pidSource);
}

/**
 * Returns the type of input the PID controller is using
 * @return the PID controller input type
 */
PIDSourceType MyPIDController::GetPIDSourceType() const {
  return m_pidInput->GetPIDSourceType();
}


/*
 * Set the percentage error which is considered tolerable for use with
 * OnTarget.
 * @param percentage error which is tolerable
 */
void MyPIDController::SetTolerance(float tol)
{
	LOCK_MUTEX;
	m_tolerance = tol;
}

/*
 * Return true if the error is within the percentage of the total input range,
 * determined by SetTolerance. This asssumes that the maximum and minimum input
 * were set using SetInput.
 * Currently this just reports on target as the actual value passes through the setpoint.
 * Ideally it should be based on being within the tolerance for some period of time.
 */
bool MyPIDController::OnTarget() const
{
	SYNC_MUTEX;
	double error = GetError();
	return fabs(error) < m_tolerance;
}

/**
 * Begin running the MYPIDController
 */
void MyPIDController::Enable()
{
	{
		LOCK_MUTEX;
		//if(!m_started)
		//	m_controlLoop->StartPeriodic(m_period);
		m_started=true;
		m_enabled = true;
	}
}

/**
 * Stop running the MYPIDController, this sets the output to zero before stopping.
 */
void MyPIDController::Disable()
{
	{
		LOCK_MUTEX;
		m_pidOutput->PIDWrite(0);
		m_enabled = false;
	}
}

/**
 * Return true if MYPIDController is enabled.
 */
bool MyPIDController::IsEnabled() const
{
	LOCK_MUTEX;
	return m_enabled;
}

/**
 * Reset the previous error,, the integral term, and disable the controller.
 */
void MyPIDController::Reset()
{
	Disable();

	LOCK_MUTEX;
	m_prevError = 0;
	m_totalError = 0;
	m_result = 0;
}
