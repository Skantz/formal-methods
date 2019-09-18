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
const int u[7][7] =                      	{{1000,	1000,	1000,	1000,	667,	333,	0},
					/*NM*/	 {1000,	1000,	1000,	667,	667,	0,	-333},
					/*NS*/	 {1000,	667,	667,	333,	0,	-333,	-667},
					/*ZE*/	 {1000,	667,	333,	0,	-333,	-667,	-1000},
					/*PS*/	 {667,	333,	0,	-333,	-667,	-667,	-1000},
					/*PM*/	 {333,	0,	-667,	-667,	-1000,	-1000,	-1000},
					/*PB*/	 {0,	-333,	-667,	-1000,	-1000,	-1000,	-1000}};
/*@ predicate u_init =
   (u[NB][NB] == 1000 && u[NB][NM] == 1000 && u[NB][NS] == 1000 && u[NB][ZE] == 1000 && u[NB][PS] == 667  && u[NB][PM] == 333  && u[NB][PB] == 0 &&
    u[NM][NB] == 1000 && u[NM][NM] == 1000 && u[NM][NS] == 1000 && u[NM][ZE] == 667  && u[NM][PS] == 667  && u[NM][PM] == 0    && u[NM][PB] == -333 &&
    u[NS][NB] == 1000 && u[NS][NM] == 667  && u[NS][NS] == 667  && u[NS][ZE] == 333  && u[NS][PS] == 0    && u[NS][PM] == -333 && u[NS][PB] == -667 &&
    u[ZE][NB] == 1000 && u[ZE][NM] == 667  && u[ZE][NS] == 333  && u[ZE][ZE] == 0    && u[ZE][PS] == -333 && u[ZE][PM] == -667 && u[ZE][PB] == -1000 &&
    u[PS][NB] == 667  && u[PS][NM] == 333  && u[PS][NS] == 0    && u[PS][ZE] == -333 && u[PS][PS] == -667 && u[PS][PM] == -667 && u[PS][PB] == -1000 &&
    u[PM][NB] == 333  && u[PM][NM] == 0    && u[PM][NS] == -667 && u[PM][ZE] == -667 && u[PM][PS] == -1000&& u[PM][PM] == -1000&& u[PM][PB] == -1000 &&
    u[PB][NB] == 0    && u[PB][NM] == -333 && u[PB][NS] == -667 && u[PB][ZE] == -1000&& u[PB][PS] == -1000&& u[PB][PM] == -1000&& u[PB][PB] == -1000);
*/


/*@ ghost int uf(int a, int b){return a == NB ?   (b == NB ? 1000 : b == NM ? 1000 : b == NS ? 1000 : b == ZE ? 1000 :b == PS ? 667  : b == PM ? 333  : 0 )
                          :  a == NM ? (b == NB ? 1000 : b == NM ? 1000 : b == NS ? 1000 : b == ZE ? 667  : b == PS ? 667 : b == PM ? 0    : -333)
			  :  a == NS ? (b == NB ? 1000 : b == NM ? 667  : b == NS ? 667  : b == ZE ? 337  : b == PS ? 0   : b == PM ? -333 : -333)
			  :  a == ZE ? (b == NB ? 1000 : b == NM ? 667  : b == NS ? 337  : b == ZE ? 0    : b == PS ? -333: b == PM ? -667 : -1000)
                          :  a == PS ? (b == NB ? 667  : b == NM ? 333  : b == NS ? 0    : b == ZE ? -333 : b == PS ? -667: b == PM ? -667 : -1000)
                          :  a == PM ? (b == NB ? 333  : b == NM ? 0    : b == NS ? -667 : b == ZE ? -667 : b == PS ?-1000: b == PM ?-1000 : -1000)
                          :            (b == NB ? 0    : b == NM ? -333 : b == NS ? -667 : b == ZE ?-1000 : b == PS ?-1000: b == PM ?-1000 : -1000);}
*/

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


//@ logic integer mNB(integer x) =  x <= -1000 ? 1000 : -1000 < x <= -500 ? -2*x -1000 : 0;

/*@
  assigns \nothing;
  behavior a:
    assumes x <= -1000;
    ensures \result == 1000;
    ensures \result == mNB(x);

  behavior b:
    assumes -1000 < x < -500;
    ensures \result == -2*x - 1000;
    ensures \result == mNB(x);
  behavior c:
    assumes -500 <= x;
    ensures \result == 0;
    ensures \result == mNB(x);

  complete behaviors a, b, c;
  disjoint behaviors a, b, c;
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

//@ logic integer mNM(integer x) =  x <= -1000 ? 0 : -1000 < x <= -500 ? 2*x + 2000 : -500 < x < -250 ? -4*x - 1000 : 0;
//@ logic integer mNMM(integer x) =  x <= -1000 ? 0 : -1000 < x <= -500 ? 2*x + 2000 : -500 < x < -250 ? -4*x - 1000 : 0;


/*.@
  assigns \nothing;
  ensures (x <= -1000 && \result == 0)
        || (-1000 < x <= 500 && \result == 2*x + 2000)
	|| (-500  < x < -250 && \result == -4*x - 1000)
	|| (-250 <= x && \result == 0);
  ensures \result == mNM(x);
*/

/*@
  assigns \nothing;
  behavior mNM_1:
    assumes x <= -1000;
    ensures \result == 0;
  ensures \result == mNM(x);
  behavior mNM_2:
    assumes -1000 < x && x <= -500;
    ensures \result == 2*x + 2000;
  ensures \result == mNM(x);
  behavior nNM_3:
    assumes -500 < x < -250;
    ensures \result == -4*x - 1000;
  ensures \result == mNM(x);
  behavior nNM_4:
    assumes -250 <= x;
    ensures \result == 0;
  ensures \result == mNM(x);
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

//@ logic integer mNS(integer x) =  x <= -500 ? 0 : -500 < x <= -250 ? 4*x + 2000 : -250 < x < 0 ? -4*x  : 0;
/*@
  assigns \nothing;
  ensures (x <= -500 && \result == 0)
       || (-500 < x <= -250 && \result ==  4*x + 2000)
       || (-250 < x <   0   && \result == -4*x)
       || (x >= 0 && \result == 0);
  ensures \result == mNS(x);
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
//@ logic integer mZE(integer x) =  x <= - 250 ? 0 : -250 < x <= 0 ? 4*x + 1000 : 0 < x < 250 ? -4*x + 1000 : 0;
/*@
  assigns \nothing;
  ensures (x <= -250 && \result == 0)
       || (-250 < x <= -0 && \result ==  4*x + 1000)
       || (0 < x < 250 && \result == -4*x + 1000)
       || (x >= 250 && \result == 0);
  ensures \result == mZE(x);
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
//@ logic integer mPS(integer x) =  x <= 0 ? 0 : 0 < x <= 250 ? 4*x : 250 < x < 500 ? -4*x + 2000 : 0;
/*@
  assigns \nothing;
  ensures (x <= 0 && \result == 0)
       || (0 < x <= 250 && \result ==  4*x)
       || (250 < x < 500 && \result == -4*x + 2000)
       || (x >= 500 && \result == 0);
  ensures \result == mPS(x);
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

//@ logic integer mPM(integer x) =  x <= 250 ? 0 : 250 < x <= 500 ? 4*x - 1000 : 500 < x < 1000 ? -2*x + 2000 : 0;
/*@
  assigns \nothing;
  ensures (x <= 250 && \result == 0)
       || (250 < x <= 500 && \result ==  4*x - 1000)
       || (500 < x < 1000 && \result == -2*x + 2000)
       || (x >= 1000 && \result == 0);
  ensures \result == mPM(x);
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

//@ logic integer mPB(integer x) = (x <= 500)? 0 : ((500 < x  < 1000)? (2*x - 1000) : 1000);
/*@
  assigns \nothing;
  //ensures 0 <= \result <= 1000;
  ensures (x <= 500 && \result == 0)
       || (500 < x < 1000 && \result == 2*x - 1000)
       || (x >= 1000 && \result == 1000);
  ensures -1000 <= \result <= 1000;

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


// /*@
//  ensures (index == NB && \result == mNB(x))
//      || (index == NM && \result == mNM(x))
//      || (index == NS && \result == mNS(x))
//      || (index == ZE && \result == mZE(x))
//      || (index == PS && \result == mPS(x))
//      || (index == PM && \result == mPM(x))
//      || (index != NB && index != NS && index != ZE && index != PS && index != PM && \result == mPB(x));
// */

/*@ logic integer md(integer index, integer x) = index == NB? mNB(x) : 
                                                 index == NM? mNM(x) :
						 index == NS? mNS(x) :
						 index == ZE? mZE(x) :
						 index == PS? mPS(x) :
						 index == PM? mPM(x) :
                                                              mPB(x);
*/

/*@ logic integer mdd(integer index, integer x) = 
                         index == NB? (x <= -1000 ? 1000 : -1000 < x <= -500 ? -2*x -1000 : 0) :
                         index == NM? (x <= -1000 ? 0 : -1000 < x <= -500 ? 2*x + 2000 : -500 < x < -250 ? -4*x - 1000 : 0): 
                         index == NS? (x <= -500 ? 0 : -500 < x <= -250 ? 4*x + 2000 : -250 < x < 0 ? -4*x  : 0) :
                         index == ZE? (x <= - 250 ? 0 : -250 < x <= 0 ? 4*x + 1000 : 0 < x < 250 ? -4*x + 1000 : 0) :
                         index == PS? (x <= 0 ? 0 : 0 < x <= 250 ? 4*x : 250 < x < 500 ? -4*x + 2000 : 0) :
                         index == PM? (x <= 250 ? 0 : 250 < x <= 500 ? 4*x - 1000 : 500 < x < 1000 ? -2*x + 2000 : 0) :
                                      ((x <= 500)? 0 : ((500 < x  < 1000)? (2*x - 1000) : 1000));
*/


/*@ logic integer md_sum(integer index, integer x, integer factor) = 

    index < 0 ? 0 :
    index == 0 ? md(index, x) * factor :
    mdd(index, x) + md_sum(index - 1, x, factor) * factor;
*/

/*@
    assigns \nothing;
    ensures 0 <= \result <= 1000;
    ensures \result == md(index, x);
    behavior nb_1:
      assumes index == NB && x <= -1000;
      ensures \result == mNB(x);
      ensures \result == 1000;
    behavior nb_2:
      assumes index == NB && -1000 < x <= -500;
      ensures \result == mNB(x);
      ensures \result == -2*x - 1000;
    behavior nb_3:
      assumes index == NB && -500 < x;
      ensures \result == mNB(x);
      ensures \result == 0;
    behavior nm_1:
      assumes index == NM && x <= -1000;
      ensures \result == mNM(x);
      ensures \result == 0;
    behavior nm_2:
      assumes index == NM && -1000 < x <= -500;
      ensures \result == mNM(x);
      ensures \result == 2*x + 2000;
    behavior nm_3:
      assumes index == NM && -500 < x < -250;
      ensures \result == mNM(x);
      ensures \result == -4*x - 1000;
    behavior nm_4:
      assumes index == NM && -250 <= x;
      ensures \result == mNM(x);
      ensures \result == 0;
    behavior ns_1:
      assumes index == NS && x <= -500;
      ensures \result == mNS(x);
      ensures \result == 0;
    behavior ns_2:
      assumes index == NS && -500 < x <= -250;
      ensures \result == mNS(x);
      ensures \result == 4*x + 2000;
    behavior ns_3:
      assumes index == NS && -250 < x < 0;
      ensures \result == mNS(x);
      ensures \result == -4*x;
    behavior ns_4:
      assumes index == NS && 0 <= x;
      ensures \result == mNS(x);
      ensures \result == 0;
    behavior ze_1:
      assumes index == ZE && x <= -250;
      ensures \result == mZE(x);
      ensures \result == 0;
    behavior ze_2:
      assumes index == ZE && -250 < x <= 0;
      ensures \result == mZE(x);
      ensures \result == 4*x + 1000;
    behavior ze_3:
      assumes index == ZE && 0 < x < 250;
      ensures \result == mZE(x);
      ensures \result == -4*x + 1000;
    behavior ze_4:
      assumes index == ZE && 250 <= x;
      ensures \result == mZE(x);
      ensures \result == 0;
    behavior ps_1:
      assumes index == PS && x <= 0;
      ensures \result == mPS(x);
      ensures \result == 0;
    behavior ps_2:
      assumes index == PS && 0 < x <= 250;
      ensures \result == mPS(x);
      ensures \result == 4*x;
    behavior ps_3:
      assumes index == PS && 250 < x < 500;
      ensures \result == mPS(x);
      ensures \result == -4*x + 2000;
    behavior ps_4:
      assumes index == PS && 500 <= x;
      ensures \result == mPS(x);
      ensures \result == 0;
    behavior pm_1:
      assumes index == PM && x <= 250;
      ensures \result == mPM(x);
      ensures \result == 0;
    behavior pm_2:
      assumes index == PM && 250 < x <= 500;
      ensures \result == mPM(x);
      ensures \result == 4*x - 1000;
    behavior pm_3:
      assumes index == PM && 500 < x < 1000;
      ensures \result == mPM(x);
      ensures \result == -2*x + 2000;
    behavior pm_4:
      assumes index == PM && 1000 <= x;
      ensures \result == mPM(x);
      ensures \result == 0;
    behavior pb_1:
      assumes index != NB && index != NM && index != NS && index != ZE && index != PS && index != PM && x <= 500;
      ensures \result == mPB(x);
      ensures \result == 0;
    behavior pb_2:
      assumes index != NB && index != NM && index != NS && index != ZE && index != PS && index != PM && 500 < x < 1000;
      ensures \result == mPB(x);
      ensures \result == 2*x - 1000;
    behavior pb_3:
      assumes index != NB && index != NM && index != NS && index != ZE && index != PS && index != PM && 1000 <= x;
      ensures \result == mPB(x);
      ensures \result == 1000;
  complete behaviors nb_1, nb_2, nb_3, nm_1, nm_2, nm_3, nm_4, ns_1, ns_2, ns_3, ns_4, ze_1, ze_2, ze_3, ze_4, ps_1, ps_2, ps_3, ps_4, pm_1, pm_2, pm_3, pm_4, pb_1, pb_2, pb_3;
  disjoint behaviors nb_1, nb_2, nb_3, nm_1, nm_2, nm_3, nm_4,  ns_1, ns_2, ns_3, ns_4, ze_1, ze_2, ze_3, ze_4, ps_1, ps_2, ps_3, ps_4, pm_1, pm_2, pm_3, pm_4, pb_1, pb_2, pb_3;
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
/*@
    assigns \nothing; 
    ensures \result == wt_sensor;
*/
int read_wheel_angular_velocity(void) {
	return wt_sensor;
}

/*
 *	Returns the current acceleration of the vehicle.
 */
/*@
    assigns \nothing;
    ensures \result == at_sensor;
*/
int read_acceleration_of_vehicle(void) {
	return at_sensor;
}

/*
 *	Returns non-zero if the brake pedal is pushed, and zero if the brake pedal
 *	is not pushed.
 */

/*@
    assigns \nothing;
    ensures \result == bp_sensor;
*/
int read_brake_pedal(void) {
	return bp_sensor;
}

/*
 *	Writes uc to the hydraulic modulator.
 */
/*@
    assigns signal_to_hydraulic_modulator;
    ensures signal_to_hydraulic_modulator == uc;
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
//@logic integer velocity = acceleration_sum*delta_t/1000 + velocity_before_braking;
/*@
  assigns \nothing;
  ensures \result == acceleration_sum*delta_t/1000 + velocity_before_braking;
  ensures \result == velocity;
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

//@logic integer wheel_slip(integer v, integer wt) = (v - wt*R/1000/v);

/*@
  assigns \nothing;
  ensures \result == (v - wt*R/1000)/v;
*/
int compute_wheel_slip(int v, int wt) {
	return (v - wt*R/1000)/v;
}

/*
 *	Computes the control signal to the hydraulic modulator.
 */

// i ≤ 2147483647



// numerator += md(NM, e)*md(ep_index, ep)*u[NM][ep_index]/1000;
/*@ logic integer loop_2_3_4_no_recursion(integer ep_index, integer index, integer e, integer ep) =
    ep_index < 0 ? 0 :
    ep_index == 0 ? md(index, e)*md(0, ep)*u[index][0]/1000 :
    ep_index == 1 ? md(index, e)*md(0, ep)*u[index][0]/1000 + md(index, e)*md(1, ep)*u[index][1]/1000 :
    ep_index == 2 ? md(index, e)*md(0, ep)*u[index][0]/1000 + md(index, e)*md(1, ep)*u[index][1]/1000 + md(index, e)*md(2, ep)*u[index][2]/1000 :
    ep_index == 3 ? md(index, e)*md(0, ep)*u[index][0]/1000 + md(index, e)*md(1, ep)*u[index][1]/1000 + md(index, e)*md(2, ep)*u[index][2]/1000 + md(index, e)*md(3, ep)*u[index][3]/1000 :
    ep_index == 4 ? md(index, e)*md(0, ep)*u[index][0]/1000 + md(index, e)*md(1, ep)*u[index][1]/1000 + md(index, e)*md(2, ep)*u[index][2]/1000 + md(index, e)*md(3, ep)*u[index][3]/1000 + md(index, e)*md(4, ep)*u[index][4]/1000 :
    ep_index == 5? md(index, e)*md(0, ep)*u[index][0]/1000 + md(index, e)*md(1, ep)*u[index][1]/1000 + md(index, e)*md(2, ep)*u[index][2]/1000 + md(index, e)*md(3, ep)*u[index][3]/1000 + md(index,e)*md(4, ep)*u[index][4]/1000 + md(index, e)*md(5, ep)*u[index][5]/1000 :
    md(index, e)*md(0, ep)*u[index][0]/1000 + md(index, e)*md(1, ep)*u[index][1]/1000 + md(index, e)*md(2, ep)*u[index][2]/1000 + md(index, e)*md(3, ep)*u[index][3]/1000 + md(index,e)*md(4, ep)*u[index][4]/1000 + md(index, e)*md(5, ep)*u[index][5]/1000 + md(index, e)*md(6, ep)*u[index][6]/1000;
*/

/*@ logic integer loop_2_3_4(integer ep_index, integer index, integer e, integer ep) = 
    ep_index <  0 ? 0 :
    ep_index == 0 ? md(index, e) * md(ep_index, ep)*u[index][ep_index]/1000 :
    loop_2_3_4(ep_index - 1, index, e, ep) + md(index, e) * md(ep_index, ep)*u[index][ep_index]/1000;
*/


/*@ logic integer loop_2_3_4_(integer ep_index, integer index, integer e, integer ep) = 
    ep_index <  0 ? 0 :
    ep_index == 0 ? md(index, e)*md(ep_index, ep)*u[index][ep_index]/1000 :
                    md(index, e)*md(ep_index, ep)*u[index][ep_index]/1000 + loop_2_3_4(ep_index - 1, index, e, ep);
*/

/*@
   requires u_init;
   assigns S_previous;
   behavior case_of_interest:
     //assumes rte: -2147483647 <= wt_sensor <= 2147483647 && -2147483647 <= acceleration_sum*delta_t/1000 + velocity_before_braking <= 2147483647 && -2147483647 <= S_previous <= 2147483647;
     assumes e_lower_than_minus_501: (acceleration_sum*delta_t/1000 + velocity_before_braking - wt_sensor*R/1000)/(acceleration_sum*delta_t/1000 + velocity_before_braking) <= -500;
     assumes ep_lower_than_minus_501: ((acceleration_sum*delta_t/1000 + velocity_before_braking - wt_sensor*R/1000)/(acceleration_sum*delta_t/1000 + velocity_before_braking) - S_previous)/delta_t <= -500;
     ensures expected_u_signal_max: \result == 1000;
*/
int compute_control_signal(void) {
	int wt = read_wheel_angular_velocity();
	//@ assert wt_pre: wt == wt_sensor;
	int v = compute_velocity_of_vehicle();
        //@ assert v_pre: v == acceleration_sum*delta_t/1000 + velocity_before_braking;
	int S = compute_wheel_slip(v, wt);
        //@ assert S_pre: S == (v - wt*R/1000)/v;
	int e = S - S_ref;									//error.
        //@ assert e_pre: e == S - S_ref;
	int ep = (S - S_previous)/delta_t;					//error prime.
        //@ assert ep_pre: ep == (S - S_previous)/delta_t;
	S_previous = S;		//Updates old value of wheel slip.
        //@ assert S_previous_pre: S_previous == S;

	int numerator = 0, denominator = 0;
        int ep_sum = 0;
        //@ assert numerator_pre: numerator == 0;
        //@ assert denominator_pre: denominator == 0;
        //@ assert ep_sum_pre: ep_sum == 0;

	/*@
	   loop assigns i, ep_sum;
	   loop invariant 0 <= i <= 7;
           loop invariant ep_sum_value: ep_sum == md_sum(i-1, ep, 1);
           loop variant 7-i;
        */	
 	for (int i = 0; i < 7; i++)   //i ∈ {NB, NM, NS, ZE, PS, PM, PB}
          ep_sum += md(i, ep);

        //.@ assert presumably_unecessary_ep_sum_post_check: ep_sum == md(0, ep) + md(1, ep) + md(2, ep) + md(3, ep) + md(4, ep) + md(5, ep) + md(6, ep); //md_sum(6, ep, 1);
        //@ assert THE_ep_check: ep <= -500 ==> ep_sum == 1000;   

        /*@
           loop assigns ep_index, numerator;
           loop invariant 0 <= ep_index <= 7;
           //loop invariant numerator_loop_1_bounds: -7 * 1000 * 1000 <= numerator <= 7 * 1000 * 1000;
           loop invariant numerator_loop_1_value: numerator == loop_2_3_4(ep_index -1, NB, e, ep);
           loop variant 7-ep_index;
        */
	for (int ep_index = 0; ep_index < 7; ep_index++)
                numerator += md(NB, e)*md(ep_index, ep)*u[NB][ep_index]/1000;

        //.@ assert numerator_holds: numerator == loop_2_3_4_no_recursion(6, NB, e, ep);
        //.@ assert numerator_between_bounds: -7 * 1000 * 1000 <= numerator <= 7 * 1000 * 1000;
        //.@ assert num_strict_double: e == 500 && ep == -500 ==> numerator == -2000*e - 1000000;
        //.@ assert num_strict_single: e == 500 && ep <= -500 ==> numerator == -2000*e - 1000000;
        //@ assert num_1: -1000 < e <= -900 && ep <= -500 ==> numerator == -2000*e - 1000000;
        //.@ assert num_2: -900 < e <= -800 && ep <= -500 ==> numerator == -2000*e - 1000000;
        //.@ assert num_3: -800 < e <= -700 && ep <= -500 ==> numerator == -2000*e - 1000000;
        //.@ assert num_4: -700 < e <= -600 && ep <= -500 ==> numerator == -2000*e - 1000000;
        //.@ assert num_5: -600 < e <= -500 && ep <= -500 ==> numerator == -2000*e - 1000000;
        //.@ assert num_6: -1000 < e <= -500 && ep <= -500 ==> numerator == -2000*e - 1000000;

        /*@
           loop assigns ep_index, numerator;
           loop invariant 0 <= ep_index <= 7 ;
           loop invariant numerator_loop_2_value: numerator == \at(numerator, LoopEntry) + loop_2_3_4(ep_index -1, NM, e, ep);
           loop variant 7-ep_index;
        */
	for (int ep_index = 0; ep_index < 7; ep_index++)
                numerator += md(NM, e)*md(ep_index, ep)*u[NM][ep_index]/1000;

        /*@
           loop assigns ep_index, numerator;
           loop invariant 0 <= ep_index <= 7 ;
           //loop invariant numerator_loop_3_value: numerator == \at(numerator, LoopEntry) + loop_2_3_4(ep_index -1, NS, e, ep);
           loop variant 7-ep_index;
        */             
	for (int ep_index = 0; ep_index < 7; ep_index++)
                numerator += md(NS, e)*md(ep_index, ep)*u[NS][ep_index]/1000;
        /*@
           loop assigns e_index, numerator;
           //loop invariant 3 <= e_index <= 7;
           loop variant 7-e_index+3;
        */        
	for (int e_index = 3; e_index < 7; e_index++)
        /*@
           loop assigns ep_index, numerator;
           loop invariant 0 <= ep_index <= 7 ;
           //loop invariant numerator_loop_4_value_inner_loop: numerator == \at(numerator, LoopEntry) + loop_2_3_4(ep_index -1, e_index , e, ep);
           loop variant 7-ep_index;
        */
 		 for (int ep_index = 0; ep_index < 7; ep_index++)        //ep_index ∈ {NB, NM, NS, ZE, PS, PM, PB}
                        numerator += md(e_index, e)*md(ep_index, ep)*u[e_index][ep_index]/1000;

        /*@
           loop assigns i, denominator;
           loop invariant 0 <= i <= 7;
           //loop invariant denominator_loop_value: denominator == md_sum(i-1, ep, ep_sum/1000);
           loop variant 7-i;
        */
        for (int i = 0; i < 7; i++)     //i ∈ {NB, NM, NS, ZE, PS, PM, PB}
                denominator += md(i, e)*ep_sum/1000;

        //Returns uc.
        return numerator/denominator;

}

/*
 *	It is assumed that the interrupt service routine calls this main function
 *	each time a timer interrupt occurs (once every 0.02 seconds).
 */ 


//  assigns signal_to_hydraulic_modulator, S_previous, velocity_before_braking, acceleration_sum;
//  assigns signal_to_hydraulic_modulator, S_previous, velocity_before_braking, acceleration_sum
//  assigns acceleration_sum, signal_to_hydraulic_modulator;

/*@
  requires u_init;
  behavior bp_sensor_zero:
    assumes bp_sensor == 0;    
    ensures signal_to_hydraulic_modulator == -1000; 
    ensures acceleration_sum == at_sensor;
    ensures S_previous == 0;
  behavior bp_sensor_not_zero:
    assumes bp_sensor != 0;
    ensures acceleration_sum == (\old(acceleration_sum) + at_sensor);
  complete behaviors bp_sensor_zero, bp_sensor_not_zero;
  disjoint behaviors bp_sensor_zero, bp_sensor_not_zero;
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
/*@ ensures \result == 0;
*/
int main(void) {
//@ assert mNM(-999) == 2;
//false. @ assert \exists integer k;  mdd(k, -500) == 1000;
//@ assert mdd(0, -500) == 0;
//@ assert mdd(1, -500) == 1000;
//false. @ assert mdd(2, -500) == 1000;


	return 0;
}
