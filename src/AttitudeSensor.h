#include "Head.h"
#include "AttitudeSensorException.h"

#define ATTITUDE_SENSOR_LOGGER_NAME "AttitudeSensor"

#include <stdlib.h>
#include <stdint.h>
#include <iostream>
#include <fstream>

#define ATTITUDE_SENSOR_VENDOR 0x1bae
#define ATTITUDE_SENSOR_PRODUCT 0x014b
#define ATTITUDE_SENSOR_HIDRAW "/dev/vuzix"
#define ATTITUDE_SENSOR_BUFFERSIZE 26
#define ATTITUDE_SENSOR_RINGBUFFER_SIZE 10 
#define ATTITUDE_SENSOR_GEOMETRIC_PROBABILITY 0.5

using namespace std;

namespace Middleware {

	typedef struct tag_IWRSENSOR_PARSED {
        int16_t x, y, z;
    } IWRSENSOR_PARSED;

	typedef struct tag_IWRSENSOR_PARSED_F {
        float x, y, z;
    } IWRSENSOR_PARSED_F;


    typedef struct tag_IWRSENSDATA_PARSED {
        IWRSENSOR_PARSED mag_sensor, acc_sensor, gyro_sensor;
    } IWRSENSDATA_PARSED;

    typedef struct tag_IWRSENSOR {
        unsigned char x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb;
    } IWRSENSOR, *PIWRSENSOR;
    
    typedef struct tag_IWRSENSDATA {
        IWRSENSOR mag_sensor, acc_sensor, gyro_sensor;
    } IWRSENSDATA, *PIWRSENSDATA;

	typedef struct tag_ANGLES {
		float yaw, pitch, roll;
	} ANGLES;
	
	typedef struct tag_RINGBUFFER {
		float measures[ATTITUDE_SENSOR_RINGBUFFER_SIZE]; 
		unsigned int pointer;
	} RINGBUFFER;

/*
 * @author Christian Herz <christian.herz@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 */
class AttitudeSensor {

public :
	static bool vuzixConnected;
	Head* head;

    AttitudeSensor() throw (AttitudeSensorException);
	~AttitudeSensor();
    void timerProc() throw (AttitudeSensorException);
	void resetHeadDirection();
	const Head* getHeadDirection(); 

	void toggleUseYaw(void);
	void toggleUsePitch(void);
	void toggleUseRoll(void);

    void readConfiguration(ifstream &configFile);
    void writeConfiguration(void);

	static float normalizeValue(
        int16_t &min, 
        int16_t &max,  
        int16_t &value);
	
	static IWRSENSOR_PARSED_F normalizeSensor(
        IWRSENSOR_PARSED &calibMin,  
	    IWRSENSOR_PARSED &calibMax, 
        IWRSENSOR_PARSED &sensor);
	
	static IWRSENSOR_PARSED normalizeGyro(
        IWRSENSOR_PARSED &biasGyro, 
	    IWRSENSOR_PARSED &sensor );
	
	static ANGLES calculateAngles( 
		ANGLES & currentAngles,		
		IWRSENSOR_PARSED_F &normalizedMagSensorData, 
		IWRSENSOR_PARSED_F &normalizedAccSensorData,
		IWRSENSOR_PARSED &normalizedGyrSensorData, 
		ANGLES &currentGyro, 
		RINGBUFFER &ringbufferAccPitch, 
		RINGBUFFER &ringbufferAccRoll, 
		float &currentAccPitch,
		float &currentAccRoll);
	
	static float calculatePitch(
	    float &currentPitch,		
	    IWRSENSOR_PARSED_F &normalizedAccSensorData, 
	    IWRSENSOR_PARSED &normalizedGyrSensorData, 
	    ANGLES &currentGyro, 
	    RINGBUFFER &ringbufferAccPitch, 
	    float &currentAccPitch);

	static float calculateRoll(
	    float &currentRoll,		
	    IWRSENSOR_PARSED_F &normalizedAccSensorData, 
	    IWRSENSOR_PARSED &normalizedGyrSensorData, 
	    ANGLES &currentGyro, 
	    RINGBUFFER &ringbufferAccRoll, 
	    float &currentAccRoll);
	
	static float calculateYaw(
        IWRSENSOR_PARSED_F &normalizedMagSensorData, 
        IWRSENSOR_PARSED &normalizedGyrSensorData, 
	    ANGLES &currentGyro,
	    float &currentYaw,
		float &currentPitch,
		float &currentRoll);

	static float geometricDistribution(float p, int k);

	int fileDevice, bytesRead;
	IWRSENSDATA sensdata;
	IWRSENSDATA_PARSED parsed;
	IWRSENSOR_PARSED calibMagMin;
	IWRSENSOR_PARSED calibMagMax;
	IWRSENSOR_PARSED calibAccMin;
	IWRSENSOR_PARSED calibAccMax;
	IWRSENSOR_PARSED biasGyro;

	ANGLES zeroAngles;
	ANGLES currentGyro;
	ANGLES currentAcc;
	ANGLES currentAngles;

	RINGBUFFER ringbufferAccPitch;
	RINGBUFFER ringbufferAccRoll;
	float currentAccPitch;
	float currentAccRoll;

	unsigned char buf[28]; //TODO magic
	IWRSENSOR_PARSED estimateGyroBias();
	void calibrate();
	//void calculateAngles(double & yaw, double & pitch, double & roll, bool useCalib);
	void receive();

	IWRSENSDATA_PARSED parseData();

	private:
		bool useYaw;
		bool usePitch;
		bool useRoll;		      
};
}
