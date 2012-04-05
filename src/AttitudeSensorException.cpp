#include <string>

#include "AttitudeSensorException.h"

using namespace std;

/**
 * @brief Constructor for exception
 * @author Christian Herz <christian.herz@uni-oldenburg.de>
 */
AttitudeSensorException::AttitudeSensorException(error c) {
	code = c;
}

/**
 * @brief Exception message
 * @author Christian Herz <christian.herz@uni-oldenburg.de>
 */
const string AttitudeSensorException::message() {
	return errmsg[(int) this->code];
};

/**
 * @brief Exception messages
 * @author Christian Herz <christian.herz@uni-oldenburg.de>
 */
string AttitudeSensorException::errmsg[] = {
            ERROR_VUZIX_NOT_EXIST_MSG,
			ERROR_VUZIX_NOT_ENOUGH_MEMORY_MSG,
			ERROR_VUZIX_NOT_SUPPORTED_MSG,
			ERROR_VUZIX_NO_CALIBRATION_DATA_MSG
};

