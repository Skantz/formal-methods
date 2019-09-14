//Array index mnemonics for negative big, medium, and small, zero, and positive small, medium and big.
#define NB 0
#define NM 1
#define NS 2
#define ZE 3
#define PS 4
#define PM 5
#define PB 6

/*
 *	Optimum wheel slip under braking: S_ref = 0.15. Multiplied by 1000.
 */
#define S_ref 150

/*
 *	The control signal to hydraulic modulator is calculated periodically once
 *	every 20 ms. Multiplied by 1000.
 */
#define delta_t 20

/*
 *	The radius of the wheels is 0.25 m. Multiplied by 1000.
 */
#define R 250

/*
 *	The table u used to compute the control signal. First index is for error
 *	(variable e in the function compute_control_signal) and second index for
 *	error prime (variable ep in the function compute_control_signal). Each
 *	entry has been multiplied by 1000 and rounded to nearest integer.
 */
//					  	  	  NB	NM		NS		ZE		PS		PM		PB
const int u[7][7] = /*NB*/	{{1000,	1000,	1000,	1000,	667,	333,	0},
					/*NM*/	 {1000,	1000,	1000,	667,	667,	0,		-333},
					/*NS*/	 {1000,	667,	667,	333,	0,		-333,	-667},
					/*ZE*/	 {1000,	667,	333,	0,		-333,	-667,	-1000},
					/*PS*/	 {667,	333,	0,		-333,	-667,	-667,	-1000},
					/*PM*/	 {333,	0,		-667,	-667,	-1000,	-1000,	-1000},
					/*PB*/	 {0,	-333,	-667,	-1000,	-1000,	-1000,	-1000}};

/*
 *	Dummy variable representing the wheel velocity sensor in radians/s.
 */
int wt_sensor;

/*
 *	Dummy variable representing the vehicle acceleration sensor in m/s².
 */
int at_sensor;

/*
 *	Dummy variable representing the sensor of whether the brake pedal is pushed
 *	or not. Nonzero value is true and zero is false.
 */
int bp_sensor;

/*
 *	Stores the wheel slip value computed from the last computation of the
 *	control signal. That is, the last computed wheel slip value.
 */
int S_previous;

/*
 *	Stores the velocity of the vehicle just before braking. Written each time
 *	the top-level function hydraulic_modulator_driver is invoked and the brake
 *	pedal is not pushed. Used to compute the current velocity of the vehicle
 *	during braking.
 */
int velocity_before_braking;

/*
 *	Stores the sum of the acceleration samples of the vehicle read during
 *	braking. Written by hydraulic_modulator_driver. If the brake pedal is not
 *	pushed, then acceleration_sum is set to the current acceleration of the
 *	vehicle. If the brake pedal is pushed, then acceleration_sum is added with
 *	the current acceleration of the vehicle. Hence, acceleration_sum = Σa_i.
 *	acceleration_sum is used to compute the current velocity of the vehicle.
 */
int acceleration_sum;

/*
 *	Dummy variable representing the hydraulic modulator. This variable holds
 *	the value currently being sent to the hydraulic modulator.
 */
int signal_to_hydraulic_modulator;

/******************************************************************************
 * The Membership functions µₘ*************************************************
 ******************************************************************************/

/*
 *µNB(x) ≔ 1			if x ≤ -1
 *µNB(x) ≔ -2x - 1		if -1 < x < -0.5
 *µNB(x) ≔ 0			if -0.5 ≤ x
 */
int mNB(int x) {
	if (x <= -1000)
		return 1000;
	else if (-1000 < x && x <-500)
		return -2*x - 1000;
	else
		return 0;
}

/*
 *µNM(x) ≔ 0			if x ≤ -1
 *µNM(x) ≔ 2x + 2		if -1 < x ≤ -0.5
 *µNM(x) ≔ -4x - 1		if -0.5 < x < -0.25
 *µNM(x) ≔ 0			if -0.25 ≤ x
 */
int mNM(int x) {
	if (x <= -1000)
		return 0;
	else if (-1000 < x && x <= -500)
		return 2*x + 2000;
	else if (-500 < x && x < -250)
		return -4*x - 1000;
	else
		return 0;
}

/*
 *µNS(x) ≔ 0			if x ≤ -0.5
 *µNS(x) ≔ 4x + 2		if -0.5 < x ≤ -0.25
 *µNS(x) ≔ -4x			if -0.25 < x < 0
 *µNS(x) ≔ 0			if 0 ≤ x
 */
int mNS(int x) {
	if (x <= -500)
		return 0;
	else if (-500 < x && x <= -250)
		return 4*x + 2000;
	else if (-250 < x && x < 0)
		return -4*x;
	else
		return 0;
}

/*
 *µZE(x) ≔ 0			if x ≤ -0.25
 *µZE(x) ≔ 4x + 1		if -0.25 < x ≤ 0
 *µZE(x) ≔ -4x + 1		if 0 < x < 0.25
 *µZE(x) ≔ 0			if 0.25 ≤ x
 */
int mZE(int x) {
	if (x <= -250)
		return 0;
	else if (-250 < x && x <= 0)
		return 4*x + 1000;
	else if (0 < x && x < 250)
		return -4*x + 1000;
	else
		return 0;
}

/*
 *µPS(x) ≔ 0			if x ≤ 0
 *µPS(x) ≔ 4x			if 0 < x ≤ 0.25
 *µPS(x) ≔ -4x + 2		if 0.25 < x < 0.5
 *µPS(x) ≔ 0			if 0.5 ≤ x
 */
int mPS(int x) {
	if (x <= 0)
		return 0;
	else if (0 < x && x <= 250)
		return 4*x;
	else if (250 < x && x < 500)
		return -4*x + 2000;
	else
		return 0;
}

/*
 *µPM(x) ≔ 0			if x ≤ 0.25
 *µPM(x) ≔ 4x - 1		if 0.25 < x ≤ 0.5
 *µPM(x) ≔ -2x + 2		if 0.5 < x < 1
 *µPM(x) ≔ 0			if 1 ≤ x
 */
int mPM(int x) {
	if (x <= 250)
		return 0;
	else if (250 < x && x <= 500)
		return 4*x - 1000;
	else if (500 < x && x < 1000)
		return -2*x + 2000;
	else
		return 0;
}

/*
 *µPB(x) ≔ 0			if x ≤ 0.5
 *µPB(x) ≔ 2x - 1 		if 0.5 < x < 1
 *µPB(x) ≔ 1			if 1 ≤ x
 */
int mPB(int x) {
	if (x <= 500)
		return 0;
	else if (500 < x && x < 1000)
		return 2*x - 1000;
	else
		return 1000;
}

/*
 *	Computes the membership degree.
 *	index ∈ {NB, NM, NS, ZE, PS, PM, PB}.
 *	x is e or ep.
 */
int md(int index, int x) {
	if (index == NB)
		return mNB(x);
	else if (index == NM)
		return mNM(x);
	else if (index == NS)
		return mNS(x);
	else if (index == ZE)
		return mZE(x);
	else if (index == PS)
		return mPS(x);
	else if (index == PM)
		return mPM(x);
	else
		return mPB(x);
}

/******************************************************************************
 * End of Membership functions ************************************************
 ******************************************************************************/



/******************************************************************************
 * Dummy functions used to perform input/output *******************************
 ******************************************************************************/

/*
 *	Returns the current angular wheel velocity.
 */
int read_wheel_angular_velocity(void) {
	return wt_sensor;
}

/*
 *	Returns the current acceleration of the vehicle.
 */
int read_acceleration_of_vehicle(void) {
	return at_sensor;
}

/*
 *	Returns non-zero if the brake pedal is pushed, and zero if the brake pedal
 *	is not pushed.
 */
int read_brake_pedal(void) {
	return bp_sensor;
}

/*
 *	Writes uc to the hydraulic modulator.
 */
void write_control_signal_to_hydraulic_modulator(int uc) {
	signal_to_hydraulic_modulator = uc;
}

/******************************************************************************
 * End of dummy functions used to perform input/output ************************
 ******************************************************************************/


/*
 *	Output: Velocity of vehicle = Σa*dt + v0 = v0 + dt⋅Σa.
 *	First term is divided by 1000 to keep the quantities in terms of 1000,
 *	since both acceleration_sum and delta_t are already multiplied by 1000.
 */
int compute_velocity_of_vehicle(void) {
	return acceleration_sum*delta_t/1000 + velocity_before_braking;
}

/*
 *	v: Vehicle velocity m/s.
 *	wt: Angular wheel velocity radians/s.
 *
 *	Output: New wheel slip S.
 */
int compute_wheel_slip(int v, int wt) {
	return (v - wt*R/1000)/v;
}

/*
 *	Computes the control signal to the hydraulic modulator.
 */
int compute_control_signal(void) {
	int wt = read_wheel_angular_velocity();

	int v = compute_velocity_of_vehicle();

	int S = compute_wheel_slip(v, wt);

	int e = S - S_ref;									//error.

	int ep = (S - S_previous)/delta_t;					//error prime.

	S_previous = S;										//Updates old value of wheel slip.

	//Computes the control signal for each of the error and error derivative combinations.
	//The code below this for loop is equivalent to this for loop, hopefully. Except for scaling.
//	for (error_index = 0; error_index < 7; error_index = error_index + 1)
//		for (error_prime_index = 0; error_prime_index < 7; error_prime_index = error_prime_index + 1) {
//			int m = md(error_index, error)*md(error_prime_index, error_prime);
//			numerator += m*u_table[error_index][error_prime_index];
//			denominator += m;
//		}

	int numerator = 0, denominator = 0;

	int ep_sum = 0;

	for (int i = 0; i < 7; i++)	//i ∈ {NB, NM, NS, ZE, PS, PM, PB}
		ep_sum += md(i, ep);

	//error is NB. Go through all of {NB, NM, NS, ZE, PS, PM, PB} for error prime.
	for (int ep_index = 0; ep_index < 7; ep_index++)
		numerator += md(NB, e)*md(ep_index, ep)*u[NB][ep_index]/1000;

	//error is NM. Go through all of {NB, NM, NS, ZE, PS, PM, PB} for error prime.
	for (int ep_index = 0; ep_index < 7; ep_index++)
		numerator += md(NM, e)*md(ep_index, ep)*u[NM][ep_index]/1000;

	//error is NS. Go through all of {NB, NM, NS, ZE, PS, PM, PB} for error prime.
	for (int ep_index = 0; ep_index < 7; ep_index++)
		numerator += md(NS, e)*md(ep_index, ep)*u[NS][ep_index]/1000;

	//error ∈ {ZE, PS, PM, PB}.
	for (int e_index = 3; e_index < 7; e_index++)
		for (int ep_index = 0; ep_index < 7; ep_index++)	//ep_index ∈ {NB, NM, NS, ZE, PS, PM, PB}
			numerator += md(e_index, e)*md(ep_index, ep)*u[e_index][ep_index]/1000;

	for (int i = 0; i < 7; i++)	//i ∈ {NB, NM, NS, ZE, PS, PM, PB}
		denominator += md(i, e)*ep_sum/1000;

	//Returns uc.
	return numerator/denominator;
}

/*
 *	It is assumed that the interrupt service routine calls this main function
 *	each time a timer interrupt occurs (once every 0.02 seconds).
 */
void hydraulic_modulator_driver(void) {
	//The brake pedal is not pushed.
	if (read_brake_pedal() == 0) {
		//Reads the current angular velocity of the wheel to compute the
		//current velocity of vehicle.
		int wt = read_wheel_angular_velocity();
		velocity_before_braking = wt*R;
		//Stores the current acceleration of the vehicle. The first time
		//hydraulic_modulator_driver is invoked when the brake pedal is pushed,
		//acceleration_sum is equal to the acceleration of the vehicle just
		//before braking. This means that the integration of the acceleration
		//over time is done over the time interval that starts when braking
		//starts.
		acceleration_sum = read_acceleration_of_vehicle();
		//No wheel slip since the brakes are not applied and therefore the
		//wheels are rolling freely.
		S_previous = 0;
		//Instructs the hydraulic modulator to not cause any brake pressure.
		write_control_signal_to_hydraulic_modulator(-1000);
	} else {
		//The brake pedal is pushed.
		//Adds the current acceleration of the vehicle.
		acceleration_sum += read_acceleration_of_vehicle();
		//Computes the control signal.
		int uc = compute_control_signal();
		//Sends the control signal to the hydraulic modulator.
		write_control_signal_to_hydraulic_modulator(uc);
	}
}

//Dummy function. GCC requires a main function.
int main(void) {
	return 0;
}
