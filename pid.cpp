/*▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽
⎧=============================================================================⎫
⎪ Francesco Martina PID Controller Implementation.                            ⎪
⎪—————————————————————————————————————————————————————————————————————————————⎪
⎪ © All Right Reserved                                                        ⎪
⎩=============================================================================⎭
△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△*/

#include "pid.h"

//////////////////// Function Code ////////////////////////

//Default controller constructor, OFF
PID_Controller::PID_Controller(){
	Set_Param(PID_Param_Default);
	Time_Rate = 0;
	Use_Limits = false;
	Reset_State();
}

//Setup
PID_Controller::PID_Controller(const PID_Param& parameters, const float& frequency){
	Set_Param(parameters);
	Set_Frequency(frequency);
	Use_Limits = false;
	Reset_State();
}

PID_Controller::PID_Controller(const PID_Param& parameters, const float& frequency, const Limits& outL, const Limits& intL){
	Set_Param(parameters);
	Set_Frequency(frequency);
	Set_Limits(outL, intL);
	Reset_State();
}

//Copy Constructor
PID_Controller::PID_Controller(const PID_Controller& controller){
	//Member to Member auto copy
	Reset_State();
}

//Assignement operator
PID_Controller& PID_Controller::operator=(const PID_Controller& other_controller){
	Param = other_controller.Param;
	Time_Rate = other_controller.Time_Rate;
	Out_Limits = other_controller.Out_Limits;
	Int_Limits = other_controller.Int_Limits;
	Controller = other_controller.Controller;
	Use_Limits = other_controller.Use_Limits;
	Reset_State();
	return *this;
}

//Reset state
inline void PID_Controller::Reset_State(){
	Integration_Term = 0;
	Old_Error = 0;
	Avoid_First = true;
}

//Parameter loading and sets control type
void PID_Controller::Set_Param(const PID_Param& parameters){
	Param = parameters;
	Controller = P;
	if(Param.I) Controller = PI;
	if(Param.D) Controller = PID;
}

//Sets Time rate for integration and differentiation
void PID_Controller::Set_Frequency(const float& frequency){
	Time_Rate = 1 / frequency;
}

//Sets integration var limits and output limits
void PID_Controller::Set_Limits(const Limits& outL, const Limits& intL){
	if(outL.Hi || outL.Low || intL.Hi || intL.Low){
		Use_Limits = true;
		Out_Limits = outL;
		Int_Limits = intL;
		Reset_State();
	}else Use_Limits = false;
}

///////////////////////////////////////////////////////////
////////////////////// Computation ////////////////////////
///////////////////////////////////////////////////////////
Com_Result PID_Controller::Compute(const float& error, float& out){
	switch(Controller){
		case P: P_Compute(error, out); break;
		case PI: PI_Compute(error, out); break;
		case PID: PID_Compute(error, out); break;
	}
	if(Use_Limits) return Saturate(out);
	return SUCCEEDED;
}

void PID_Controller::P_Compute(const float& error, float& out){
	out = error * Param.P;
}

void PID_Controller::PI_Compute(const float& error, float& out){
	Integration_Term += error * Time_Rate; 
	out = error * Param.P + Integration_Term * Param.I;
}

void PID_Controller::PID_Compute(const float& error, float& out){
	//Wait First sample to be stored, stability issues
	if(!Avoid_First){
		float Derivative_Term;
		Derivative_Term = (error - Old_Error) / Time_Rate;
		Integration_Term += error * Time_Rate; 
		out = error * Param.P + Integration_Term * Param.I + Derivative_Term * Param.D;
	}else{
		out = 0;
		Avoid_First = false;
	}

	//Store error for differentiation
	Old_Error = error;
}

//Actuate limits
Com_Result PID_Controller::Saturate(float& out){
	Com_Result response = SUCCEEDED;

	if(out > Out_Limits.Hi){out = Out_Limits.Hi; response = OUT_OVERFLOW;}
	if(out < Out_Limits.Low){out = Out_Limits.Low; response = OUT_OVERFLOW;}

	if(Integration_Term > Int_Limits.Hi){
		Integration_Term = Int_Limits.Hi; 
		if(response == OUT_OVERFLOW) response = BOTH_OVERFLOW; else response = INT_OVERFLOW;
	}

	if(Integration_Term < Int_Limits.Low){
		Integration_Term = Int_Limits.Low; 
		if(response == OUT_OVERFLOW) response = BOTH_OVERFLOW; else response = INT_OVERFLOW;
	}

	return response;
}
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////











