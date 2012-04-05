#ifndef ATTITUDESENSOREXCEPTION_H
#define ATTITUDESENSOREXCEPTION_H

#include <string>

#define ERROR_VUZIX_NOT_EXIST_MSG "device is not connected"
#define ERROR_VUZIX_NOT_ENOUGH_MEMORY_MSG "not enough memory"
#define ERROR_VUZIX_NOT_SUPPORTED_MSG "tracker does not support this function"
#define ERROR_VUZIX_NO_CALIBRATION_DATA_MSG "manager does not send calibration data for yaw"

using namespace std;

/**
 * @author Christian Herz <christian.herz@uni-oldenburg.de>
 */
class AttitudeSensorException {
    public:
		enum error {
			ERROR_VUZIX_NOT_EXIST,
			ERROR_VUZIX_NOT_ENOUGH_MEMORY,
			ERROR_VUZIX_NOT_SUPPORTED,
			ERROR_VUZIX_NO_CALIBRATION_DATA
		};

        AttitudeSensorException(error c); 
        const string message();

    private:
        error code;
        static string errmsg[];
};

#endif
