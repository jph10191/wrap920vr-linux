#ifndef Middleware_Head_h
#define Middleware_Head_h

namespace Middleware {

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

} /* End of namespace Middleware */

#endif // Middleware_Head_h
