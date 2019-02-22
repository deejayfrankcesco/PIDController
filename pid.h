/*▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽▽
⎧=============================================================================⎫
⎪ Francesco Martina PID Controller Implementation.                            ⎪
⎪—————————————————————————————————————————————————————————————————————————————⎪
⎪ © All Right Reserved                                                        ⎪
⎩=============================================================================⎭
△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△△*/

///////////////// Structures Definition ///////////////////
//PID parameters
//controller Out = ((P)) * Error + ((I)) * Series[Error * Time_Rate] + ((D)) * (Error - Old_Error) / Time_Rate
struct PID_Param{
	float P;
	float I;
	float D;
};

const PID_Param PID_Param_Default = {0, 0, 0};	//Controller OFF

//Controller limits
struct Limits{
	float Low;
	float Hi;
};


////////////////// Enumerate Definition ///////////////////
enum Control_Type{
	P,
	PI,
	PID
};

enum Com_Result{
	SUCCEEDED,
	INT_OVERFLOW,
	OUT_OVERFLOW,
	BOTH_OVERFLOW
};

///////////////////////////////////////////////////////////
/////////////////// Class Definition //////////////////////
///////////////////////////////////////////////////////////

class PID_Controller{

private:

	//Settings
	PID_Param Param;			//Settings controller parameters
	float Time_Rate;			//Time working step (in seconds)

	Limits Out_Limits;			//Out saturation
	Limits Int_Limits;			//Integral term saturation

	//Internal State
	Control_Type Controller;	//Type of controller used 
	float Old_Error;			//Stores Old Error (needed for D term)
	float Integration_Term;		//Store Integration
	bool  Avoid_First;			//Wait to store first Old_Error sample avoiding First computation 


	//Computation function Declarations
	void P_Compute(const float& Error, float& out);
	void PI_Compute(const float& Error, float& out);
	void PID_Compute(const float& Error, float& out);
	Com_Result Saturate(float& out);


public:

	PID_Controller();
	PID_Controller(const PID_Param& parameters, const float& frequency);
	PID_Controller(const PID_Param& parameters, const float& frequency, const Limits& outL, const Limits& intL);
	PID_Controller(const PID_Controller&);

	//Utility Functions
	inline void Reset_State();

	//Setup Functions
	void Set_Param(const PID_Param& parameters);
	void Set_Frequency(const float& frequency);		//Frequency in Hz

	bool Use_Limits;
	void Set_Limits(const Limits& outL, const Limits& intL);

	PID_Controller& operator=(const PID_Controller& other_controller);

	Com_Result Compute(const float& error, float& out);

};
