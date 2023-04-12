
class PIDController {
    public:
	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;
	PIDController(float KpIn, float KiIn, float KdIn, float tauIn = 0, float limMinIn = 0, float limMaxIn = 0, float limMinIntIn = 0, float limMaxIntIn = 0)
	{
		Kp = KpIn; Ki = KiIn; Kd = KdIn; tau = tauIn; limMin = limMinIn; limMax = limMaxIn; limMinInt = limMinIntIn; limMaxInt = limMaxIntIn;
	}
    /* METHODS */

    void Init() {

	/* Clear controller variables */
	integrator = 0.0f;
	prevError  = 0.0f;

	differentiator  = 0.0f;
	prevMeasurement = 0.0f;

	out = 0.0f;
	}

	void Update(float setpoint, float measurement, float dt) {
		/*
		* Error signal
		*/
		float error = setpoint - measurement;


		/*
		* Proportional
		*/
		float proportional = Kp * error;


		/*
		* Integral
		*/
		integrator = integrator + 0.5f * Ki * (dt) * (error + prevError);

		/* Anti-wind-up via integrator clamping */
		if (integrator > limMaxInt) {

			integrator = limMaxInt;

		} else if (integrator < limMinInt) {

			integrator = limMinInt;

		}


		/*
		* Derivative (band-limited differentiator)
		*/
			
		differentiator = -(2.0f * Kd * (measurement - prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
							+ (2.0f * tau - dt) * differentiator)
							/ (2.0f * tau + dt);


		/*
		* Compute output and apply limits
		*/
		out = proportional + integrator + differentiator;

		if (out > limMax) {

			out = limMax;

		} else if (out < limMin) {

			out = limMin;

		}

		/* Store error and measurement for later use */
		prevError       = error;
		prevMeasurement = measurement;
	}
};