#ifndef Head_h
#define Head_h


/** A head will be returned by the AttitudeSensor and
 * contains the angles of the current head direction
 * 
 */
class Head {

public:

	Head();
	typedef struct HeadDirection {
		float yawDeg;
		float rollDeg;
		float pitchDeg;
	} HeadDirection;

	HeadDirection angles;

	void printToStdOut();
};


#endif // Head_h
