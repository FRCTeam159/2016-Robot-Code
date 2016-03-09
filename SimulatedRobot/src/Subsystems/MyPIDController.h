/*
 * MYPIDController.h
 *
 *  Created on: Mar 7, 2016
 *      Author: dean
 */

#ifndef SRC_SUBSYSTEMS_MYPIDCONTROLLER_H_
#define SRC_SUBSYSTEMS_MYPIDCONTROLLER_H_

#include "WPILib.h"
#include <Subsystems/MyNotifier.h>


class MyPIDController: public PIDInterface {
	PIDSource *m_pidInput;
	PIDOutput *m_pidOutput;

	float m_P = 1.0;              // factor for "proportional" control
	float m_I = 0.0;              // factor for "integral" control
	float m_D = 0.0;              // factor for "derivative" control
	float m_maximumOutput = 1.0;  // |maximum output|
	float m_minimumOutput = -1.0;  // |minimum output|
	float m_maximumInput = 0;   // maximum input - limit setpoint to this
	float m_minimumInput = 0;   // minimum input - limit setpoint to this
	bool m_enabled = false;  // is the pid controller enabled
	float m_prevError = 0;  // the prior error (used to compute velocity)
	double m_totalError = 0; // the sum of the errors for use in the integral calc

	float m_tolerance = 0.05;
	float m_setpoint = 0;
	float m_prevSetpoint = 0;
	float m_error = 0;
	float m_result = 0;
	float m_period;
	bool m_started=false;
	mutable std::recursive_mutex m_mutex;

	std::unique_ptr<MyNotifier> m_controlLoop;

	void Initialize(float p, float i, float d,PIDSource *source,
			PIDOutput *output, float rate);

public:
	MyPIDController(float p, float i, float d, PIDSource *source,PIDOutput *output, float rate);

	virtual ~MyPIDController();
	virtual void Calculate();

	virtual float Get() const;
	virtual void SetInputRange(float minimumInput, float maximumInput);
	virtual void SetOutputRange(float minimumOutput, float maximumOutput);
	virtual void SetPID(double p, double i, double d) override;
	virtual double GetP() const override;
	virtual double GetI() const override;
	virtual double GetD() const override;

	virtual void SetSetpoint(float setpoint) override;
	virtual double GetSetpoint() const override;

	virtual float GetError() const;

	virtual void SetPIDSourceType(PIDSourceType pidSource);
	virtual PIDSourceType GetPIDSourceType() const;

	virtual void SetTolerance(float percent);
	virtual bool OnTarget() const;

	virtual void Enable() override;
	virtual void Disable() override;
	virtual bool IsEnabled() const override;

	virtual void Reset() override;

};

#endif /* SRC_SUBSYSTEMS_MYPIDCONTROLLER_H_ */
