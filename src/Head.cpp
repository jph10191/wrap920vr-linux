#include "Head.h"
#include <stdio.h>


/** 
 * @brief constructor of Head
 * @author Christian Herz <christian.herz@uni-oldenburg.de>
 */
Head::Head() {

	// initialize with zero degree
	angles.pitchDeg = angles.rollDeg = angles.pitchDeg = 0;
}

/** 
 * @brief This method prints all values for the head direction to std
 * @author Christian Herz <christian.herz@uni-oldenburg.de>
 */
void Head::printToStdOut() {
	printf("%f deg LR,%f deg NLR ,%f deg OU\n", 
    angles.yawDeg, angles.rollDeg, angles.pitchDeg);
}

