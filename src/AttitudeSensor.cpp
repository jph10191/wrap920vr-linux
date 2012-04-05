#include "AttitudeSensor.h"
#include "AttitudeSensorException.h"

#include <Logger.h>

#ifdef WIN32
#include "stdafx.h"
#define IWEARDRV_EXPLICIT
#include "iweardrv.h"
#include <tchar.h>
#include "stdio.h"
#include "conio.h"
#endif

#ifdef UNIX
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <testdog/unit_test.hpp>
#endif

namespace Middleware {

#ifdef UNIX
TDOG_TEST_SUITE(AttitudeSensorTest) {
	TDOG_TEST_CASE(normalizeValue) {
		TDOG_SET_AUTHOR("Jendrik und Justin");
		int16_t min=100;
		int16_t max=200;
		int16_t sensdata=150;
		float normalizedSensdata=AttitudeSensor::normalizeValue(min, max, sensdata);
		TDOG_ASSERT_DOUBLE_EQUAL(normalizedSensdata, 0.0, 0.01);
		
		min=-150;
		max=200;
		sensdata=100;
		normalizedSensdata=AttitudeSensor::normalizeValue(min, max, sensdata);
		TDOG_ASSERT_DOUBLE_EQUAL(normalizedSensdata, 75.0, 0.01);
	}
	
	TDOG_TEST_CASE(normalizeSensor) {
		TDOG_SET_AUTHOR("Jendrik und Justin");
		IWRSENSOR_PARSED calibMin={100,-150,-150};
		IWRSENSOR_PARSED calibMax={200,200,200};
		IWRSENSOR_PARSED sensor={150,100,100};

		IWRSENSOR_PARSED_F 
		normalizedSensdata=AttitudeSensor::normalizeSensor(calibMin, 
		calibMax, sensor);
		TDOG_ASSERT_DOUBLE_EQUAL(normalizedSensdata.x, 0.0, 0.01);
		TDOG_ASSERT_DOUBLE_EQUAL(normalizedSensdata.y, 75.0, 0.01);
		TDOG_ASSERT_DOUBLE_EQUAL(normalizedSensdata.z, 75.0, 0.01);
	}
}
#endif 

// first set vuzixConnected to false, later it will be tested
bool AttitudeSensor::vuzixConnected = false;

/** 
 * @brief  Constructor of AttitudeSensor
 * @author Christian Herz <christian.herz@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 * @author Matthias Bruns <matthias.bruns@uni-oldenburg.de>
 */
AttitudeSensor::AttitudeSensor() throw (AttitudeSensorException) {

    Logger::setLogger(
        ATTITUDE_SENSOR_LOGGER_NAME, 
        Logger::INFO, 
        &std::cout);

    LOG(ATTITUDE_SENSOR_LOGGER_NAME) 
            << "AttitudeSensor instantiated." << Logger::endl;

	head = new Head();

#ifdef UNIX
	this->fileDevice = open(
		ATTITUDE_SENSOR_HIDRAW,
		O_RDWR | O_NONBLOCK);

	if(this->fileDevice < 0) {
		LOG(ATTITUDE_SENSOR_LOGGER_NAME) 
			<< "Could not open device." << Logger::endl;
		
	    return;
	}	
	
	this->zeroAngles.yaw = 0.0;
	this->zeroAngles.pitch = 0.0;
	this->zeroAngles.roll = 0.0;
	this->currentAngles.yaw = 0.0;
	this->currentAngles.pitch = 0.0;
	this->currentAngles.roll = 0.0;
	this->useYaw = true;
	this->usePitch = true;
	this->useRoll = true;

	// Try to read config file
	ifstream configFile;
	configFile.open("attitudesensor.conf");

	if(configFile.is_open()){
		//read calibration data from config file
        this->readConfiguration(configFile);
	} else{
		this->biasGyro = this->estimateGyroBias();
		this->calibrate();
        this->writeConfiguration();
	}

	for(int i=0; i<ATTITUDE_SENSOR_RINGBUFFER_SIZE; i++){
		this->ringbufferAccPitch.measures[i]=0.0;
		this->ringbufferAccRoll.measures[i]=0.0;
	}

	this->ringbufferAccPitch.pointer = 0;
	this->ringbufferAccRoll.pointer = 0;
	this->currentAccPitch = 0.0;
	this->currentAccRoll = 0.0;
	
	this->vuzixConnected = true;
#endif

#ifdef WIN32
    LOG(ATTITUDE_SENSOR_LOGGER_NAME) << 
        "Using win32 init code." << Logger::endl;

	// loading windows driver for vuzix
    m_hIwear = LoadLibrary(_T("IWEARDRV.DLL"));

	// driver successfully loaded
    if (m_hIwear) {
		// Proc address of each needed procedure will be caught
		IWROpenTracker = (PIWROPENTRACKER)GetProcAddress(
			m_hIwear, "IWROpenTracker");
		IWRZeroSet = (PIWRZEROSET)GetProcAddress(
			m_hIwear, "IWRZeroSet");
		IWRGet6DTracking = (PIWRGET6DTRACKING)GetProcAddress(
			m_hIwear, "IWRGet6DTracking");
		IWRGetSensorData = (PIWRGETSENSORDATA)GetProcAddress(
			m_hIwear, "IWRGetSensorData");
		IWRGetMagYaw = (PIWRGETMAGYAW)GetProcAddress(
			m_hIwear,"IWRGetMagYaw");
		IWRSetMagAutoCorrect = (PIWRSETMAGAUTOCORRECT)GetProcAddress(
			m_hIwear, "IWRSetMagAutoCorrect");
		IWRGetVersion = (PIWRGETVERSION)GetProcAddress(
			m_hIwear, "IWRGetVersion");

		// starting the poll for getting data from the vuzix
		try {
			this->startPoll();
		} catch (AttitudeSensorException &ex) {
			LOG(ATTITUDE_SENSOR_LOGGER_NAME) 
					<< ex.message() << Logger::endl;
		}
	}
#endif
}

/** 
 * @brief Destructor frees memory if allocated
 * @author Christian Herz <christian.herz@uni-oldenburg.de>
 */
AttitudeSensor::~AttitudeSensor() {
	if(head) {
		delete head;
	}
}


const Head* AttitudeSensor::getHeadDirection(){
	return head;
}


#ifdef WIN32
/** 
 * @brief This method resets the head direction
 * @author Christian Herz <christian.herz@uni-oldenburg.de>
 */
void AttitudeSensor::resetHeadDirection(){
	IWRZeroSet();
}
#endif

#ifdef WIN32
/** 
 * @brief This method opens the tracker
 * @author Christian Herz <christian.herz@uni-oldenburg.de>
 * @author Matthias Bruns <matthias.bruns@uni-oldenburg.de>
 */
void AttitudeSensor::startPoll() throw (AttitudeSensorException) {

	// opens the tracker
	switch(IWROpenTracker()) {
		case ERROR_SUCCESS:
			// vuzix is connected
			this->vuzixConnected = true;
			break;
		// Device is not connected, throw ex
		case ERROR_DEV_NOT_EXIST: { 
			this->vuzixConnected = false;
			AttitudeSensorException ex 
                (AttitudeSensorException::ERROR_VUZIX_NOT_EXIST);
			
            LOG(ATTITUDE_SENSOR_LOGGER_NAME) 
				<< ex.message() << Logger::endl;
			throw ex;
			break;
		}
		// Not enough memory, throw ex
		case ERROR_NOT_ENOUGH_MEMORY: { 
			AttitudeSensorException ex 
                (AttitudeSensorException::ERROR_VUZIX_NOT_ENOUGH_MEMORY);
			
            LOG(ATTITUDE_SENSOR_LOGGER_NAME) 
				<< ex.message() << Logger::endl;
			throw ex;
			break;
		}
    }

	// for pulling calibration data from V-Monitor
	IWRVERSION ver;

	// NECESSARY!! pulling calibration data from V-Monitor
	if (IWRGetVersion(&ver) != ERROR_SUCCESS) {
		AttitudeSensorException ex 
            (AttitudeSensorException::ERROR_VUZIX_NO_CALIBRATION_DATA);

		LOG(ATTITUDE_SENSOR_LOGGER_NAME) 
			<< ex.message() << Logger::endl;
		throw ex;
	}

	resetHeadDirection();

	// using magneticfield-data for the yaw-value
	IWRSetMagAutoCorrect(true);
} 
#endif

#ifdef UNIX

/**
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 */
void AttitudeSensor::readConfiguration(ifstream &configFile) {
    LOG(ATTITUDE_SENSOR_LOGGER_NAME) << "Reading configuration" << Logger::endl;
    	
	string line;
	getline(configFile, line);
	this->biasGyro.x=atoi(line.c_str());
	getline(configFile, line);
	this->biasGyro.y=atoi(line.c_str());
	getline(configFile, line);
	this->biasGyro.z=atoi(line.c_str());
	getline(configFile, line);
	this->calibMagMin.x=atoi(line.c_str());
	getline(configFile, line);
	this->calibMagMin.y=atoi(line.c_str());
	getline(configFile, line);
	this->calibMagMin.z=atoi(line.c_str());
	getline(configFile, line);
	this->calibMagMax.x=atoi(line.c_str());
	getline(configFile, line);
	this->calibMagMax.y=atoi(line.c_str());
	getline(configFile, line);
	this->calibMagMax.z=atoi(line.c_str());
	getline(configFile, line);
	this->calibAccMin.x=atoi(line.c_str());
	getline(configFile, line);
	this->calibAccMin.y=atoi(line.c_str());
	getline(configFile, line);
	this->calibAccMin.z=atoi(line.c_str());
	getline(configFile, line);
	this->calibAccMax.x=atoi(line.c_str());
	getline(configFile, line);
	this->calibAccMax.y=atoi(line.c_str());
	getline(configFile, line);
	this->calibAccMax.z=atoi(line.c_str());
}

/**
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 */
void AttitudeSensor::writeConfiguration() {
	LOG(ATTITUDE_SENSOR_LOGGER_NAME) << "Writing configuration" << Logger::endl;
    ofstream configFileOut("attitudesensor.conf");
    configFileOut << this->biasGyro.x<<endl;
	configFileOut << this->biasGyro.y<<endl;
	configFileOut << this->biasGyro.z<<endl;
	configFileOut << this->calibMagMin.x<<endl;
	configFileOut << this->calibMagMin.y<<endl;
	configFileOut << this->calibMagMin.z<<endl;
	configFileOut << this->calibMagMax.x<<endl;
	configFileOut << this->calibMagMax.y<<endl;
	configFileOut << this->calibMagMax.z<<endl;
	configFileOut << this->calibAccMin.x<<endl;
	configFileOut << this->calibAccMin.y<<endl;
	configFileOut << this->calibAccMin.z<<endl;
	configFileOut << this->calibAccMax.x<<endl;
	configFileOut << this->calibAccMax.y<<endl;
	configFileOut << this->calibAccMax.z<<endl;
	configFileOut.close();

}

/**
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 */
void AttitudeSensor::receive() {

	bytesRead = read(
		this->fileDevice, 
		buf, 
		ATTITUDE_SENSOR_BUFFERSIZE);
	
	if (bytesRead < 0) {
	} else {
		memcpy(
			&this->sensdata, 
			&buf[2],  //offset
			sizeof(unsigned char) * 24);
		this->parsed = this->parseData();
	}
}

/**
 * @author Justin Philipp Heinermann<justin.philipp.heinermann@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 */
ANGLES AttitudeSensor::calculateAngles( 
		ANGLES &currentAngles,		
		IWRSENSOR_PARSED_F &normalizedMagSensorData, 
		IWRSENSOR_PARSED_F &normalizedAccSensorData, 
		IWRSENSOR_PARSED &normalizedGyrSensorData, 
		ANGLES &currentGyro,
		RINGBUFFER &ringbufferAccPitch,
		RINGBUFFER &ringbufferAccRoll,
		float &currentAccPitch,
		float &currentAccRoll) {

	ANGLES retVal;

	retVal.yaw = AttitudeSensor::calculateYaw(normalizedMagSensorData, 
		normalizedGyrSensorData, currentGyro,currentAngles.yaw, currentAccPitch, currentAccRoll);

	retVal.pitch = AttitudeSensor::calculatePitch(
            currentAngles.pitch, 
		    normalizedAccSensorData, 
			normalizedGyrSensorData, 
			currentGyro, 
			ringbufferAccPitch, 
			currentAccPitch);
	
	retVal.roll = AttitudeSensor::calculateRoll(
			currentAngles.roll,
			normalizedAccSensorData, 
			normalizedGyrSensorData, 
			currentGyro, 
			ringbufferAccRoll, 
			currentAccRoll);
	
	return retVal;
}

/*
 * Calculate the pitch angle from accelerometer and gyroscope input.
 * Side-effects: be aware that this function changes the ringbuffer for
 * the accelerometer in order to filter the data.
 *
 * @brief Calculate the pitch angle from accelerometer and gyroscope input
 *
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 */
float AttitudeSensor::calculatePitch(
	float &currentPitch,
	IWRSENSOR_PARSED_F &normalizedAccSensorData,
	IWRSENSOR_PARSED
	&normalizedGyrSensorData, 
	ANGLES &currentGyro,
	RINGBUFFER &ringbufferAccPitch,
	float &currentAccPitch){

	float retVal;

	//Filter Acc Data
	ringbufferAccPitch.measures[ringbufferAccPitch.pointer] = 
		atan2(
            normalizedAccSensorData.x, 
			sqrt(normalizedAccSensorData.z * normalizedAccSensorData.z + 
			normalizedAccSensorData.y * normalizedAccSensorData.y)
		) / M_PI; 

    float accAvg = 0.0;

	for(unsigned int i = 0; i < ATTITUDE_SENSOR_RINGBUFFER_SIZE; i++){
		
        unsigned int currentIndex =
			(ringbufferAccPitch.pointer + i) % 
                ATTITUDE_SENSOR_RINGBUFFER_SIZE;
		
        float coefficient=geometricDistribution(
				ATTITUDE_SENSOR_GEOMETRIC_PROBABILITY,
				i);

		accAvg += coefficient * ringbufferAccPitch.measures[currentIndex];	
	}

	ringbufferAccPitch.pointer = ringbufferAccPitch.pointer + 1;
	
    if(ringbufferAccPitch.pointer == ATTITUDE_SENSOR_RINGBUFFER_SIZE){
		ringbufferAccPitch.pointer = 0;	
	}
		
	//Gyro is difference, acc is absolute position
	accAvg*=90.0;
	currentAccPitch=accAvg;

	//Add gyro to currentPitch
	currentPitch = currentPitch + normalizedGyrSensorData.y * (90.0/32768.0);

	float possibleErrorPitch=currentAccPitch-currentPitch;
//	std::cout<<"Pitch gyr acc dif: "<<currentPitch<<" "<<currentAccPitch<<" "<<possibleErrorPitch<<std::endl;

	if(possibleErrorPitch > 2.0 || possibleErrorPitch < -2.0){
		currentPitch = currentPitch + 0.05 * possibleErrorPitch;
	}


	retVal=currentPitch;
	return retVal; 
}

/**
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 */
float AttitudeSensor::calculateRoll(
	float &currentRoll,
	IWRSENSOR_PARSED_F &normalizedAccSensorData,
	IWRSENSOR_PARSED &normalizedGyrSensorData, 
	ANGLES &currentGyro,
	RINGBUFFER &ringbufferAccRoll,
	float &currentAccRoll){

	float retVal;

    //Filter Acc Data
	ringbufferAccRoll.measures[ringbufferAccRoll.pointer]=
		atan2(	
			sqrt(normalizedAccSensorData.x* normalizedAccSensorData.x + 
			normalizedAccSensorData.z*normalizedAccSensorData.z),
			normalizedAccSensorData.y
		)/M_PI; 
	
    float accAvg=0.0;
	
    for(unsigned int i=0; i < ATTITUDE_SENSOR_RINGBUFFER_SIZE; i++){

		unsigned int currentIndex =
			(ringbufferAccRoll.pointer + i) % ATTITUDE_SENSOR_RINGBUFFER_SIZE;
	
        float coefficient = geometricDistribution(
				ATTITUDE_SENSOR_GEOMETRIC_PROBABILITY,i);
		
        accAvg += coefficient * ringbufferAccRoll.measures[currentIndex];	
	}

	ringbufferAccRoll.pointer = ringbufferAccRoll.pointer + 1;
	
    if(ringbufferAccRoll.pointer == ATTITUDE_SENSOR_RINGBUFFER_SIZE){
		ringbufferAccRoll.pointer = 0;	
	}
		
	currentAccRoll = accAvg*90.0;

	//Add gyro to currentPitch
	currentRoll = currentRoll + normalizedGyrSensorData.z *0.5*(180.0/32768.0);
	float possibleErrorRoll=currentAccRoll-currentRoll;

	//std::cout<<"Roll gyr acc dif: "<<currentRoll<<" "<<currentAccRoll<<" "<<possibleErrorRoll<<std::endl;
	if(possibleErrorRoll > 1.0){
		currentRoll = currentRoll + 0.05 * possibleErrorRoll;
	}else if(possibleErrorRoll < -1.0){
		currentRoll = currentRoll + 0.05 * possibleErrorRoll;
	}
	retVal=currentRoll;
	return retVal; 
}

/**
 * @author Justin Philipp Heinermann<justin.philipp.heinermann@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 */
float AttitudeSensor::calculateYaw(
    IWRSENSOR_PARSED_F &normalizedMagSensorData, 
    IWRSENSOR_PARSED &normalizedGyrSensorData, 
    ANGLES &currentGyro,
    float &currentYaw,
    float &currentPitch,
    float &currentRoll
    ) {
	static double lastMagYaw=0.0;
    IWRSENSOR_PARSED_F mag = normalizedMagSensorData;
	float retVal;
	float magYaw;
	
	float gyrDiff=normalizedGyrSensorData.x * 0.2 * (180.0 / 32768.0);

	double xh=mag.x;
	double yh=mag.y;
	double length = sqrt(xh * xh + yh * yh);
	xh = xh / length;
	yh = yh / length;
	magYaw = atan2(xh, yh); 
	magYaw*= 0.2 * (180.0 / 32768.0);
	
	float magDiff=magYaw-lastMagYaw;
	
	float possibleErrorYaw=gyrDiff-magDiff;

	if(possibleErrorYaw > 1.0){
		retVal = currentGyro.yaw + gyrDiff - 0.05 * possibleErrorYaw;
	}else if(possibleErrorYaw < -1.0){
		retVal = currentGyro.yaw + gyrDiff + 0.05 * possibleErrorYaw;
	}else{
		retVal=currentGyro.yaw + gyrDiff;
	}
	currentGyro.yaw = retVal;
	return retVal;
}

/**
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 */
float AttitudeSensor::geometricDistribution(float p, int k){
	return p * pow((1-p), k-1);
}

/**
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 */
void AttitudeSensor::calibrate() {
	receive();
	
    this->calibMagMin=parsed.mag_sensor;
	this->calibMagMax=parsed.mag_sensor;
	this->calibAccMin=parsed.acc_sensor;
	this->calibAccMax=parsed.acc_sensor;
	
    int16_t *ptr = (int16_t *) &this->parsed;
	int16_t *ptrMagMin = (int16_t *) &this->calibMagMin;
	int16_t *ptrMagMax = (int16_t *) &this->calibMagMax;
	int16_t *ptrAccMin = (int16_t *) &this->calibAccMin;
	int16_t *ptrAccMax = (int16_t *) &this->calibAccMax;
	
	for(int i = 1; i < 250; i++) {//TODO define
		receive();
		for(int k = 0; k < 3; k++) {
//			printf("%i\n",ptr[k]);
			if(ptrMagMin[k]>ptr[k]){
//				printf("doh\n");
				ptrMagMin[k] = ptr[k];
			}else if(ptrMagMax[k]<ptr[k]){
				ptrMagMax[k] = ptr[k];
			}
		}
		   
		for(int k = 3; k < 6; k++) {
				if(ptrAccMin[k-3]>ptr[k]){
					ptrAccMin[k-3] = ptr[k];
				}else if(ptrAccMax[k-3]<ptr[k]){
					ptrAccMax[k-3] = ptr[k];
				}
		}
		boost::posix_time::milliseconds workTime(50);
		boost::this_thread::sleep(workTime);
		printf("calibMin: %i %i %i %i %i %i \n", 
		    this->calibMagMin.x,
			this->calibMagMin.y,
			this->calibMagMin.z,
			this->calibAccMin.x,
			this->calibAccMin.y,
			this->calibAccMin.z);

		printf("calibMax: %i %i %i %i %i %i \n", 
			this->calibMagMax.x,
			this->calibMagMax.y,
			this->calibMagMax.z,
			this->calibAccMax.x,
			this->calibAccMax.y,
			this->calibAccMax.z);
	}
}

/**
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 */
IWRSENSOR_PARSED AttitudeSensor::estimateGyroBias() {
	IWRSENSOR_PARSED retVal;
	long sums[3]={0,0,0};	

	// read data
	// glasses must lie on the desk
	// and must not be moved
	for(int i = 0; i < 4000; i++) {//TODO define
		receive();
		//int16_t **ptr = (int16_t **) &parsed; //TODO wieso nicht?
		int16_t *ptr = (int16_t *) &parsed;
		for(int k = 6; k < 9; k++) {
			LOG(ATTITUDE_SENSOR_LOGGER_NAME) <<
                "sum["<<k-6<<"]+="<<ptr[k]<<"="<<sums[k-6]<<Logger::endl;
			sums[k-6] += (long) ptr[k];
		}
	}

	// calculate average
	retVal.x = ((float) sums[0])/4000.0;
	retVal.y = ((float) sums[1])/4000.0;
	retVal.z = ((float) sums[2])/4000.0;
	LOG(ATTITUDE_SENSOR_LOGGER_NAME)<<"Gyroscope Bias x: "
	<<retVal.x<<Logger::endl;
	LOG(ATTITUDE_SENSOR_LOGGER_NAME)<<"Gyroscope Bias y: "
	<<retVal.y<<Logger::endl;
	LOG(ATTITUDE_SENSOR_LOGGER_NAME)<<"Gyroscope Bias z: " 
	<<retVal.z<<Logger::endl;	
	LOG(ATTITUDE_SENSOR_LOGGER_NAME)<<"Calibrate Gyro done"<<Logger::endl;
	return retVal;
}	

/**
 * @author Justin Philipp Heinermann<justin.philipp.heinermann@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 */
float AttitudeSensor::normalizeValue(
    int16_t &min, 
    int16_t &max,
    int16_t &value){

    if(value < min) value = min;
	if(value > max) value = max;
	
// wrong...
// float retVal=((float) value - ((float) min + (float) max)/2);


	float retVal=
		2.0f*
		(((float)value - (float)min))/
		((float)max - (float)min)
		-1.0f;

	if(retVal<0.001f && retVal>-0.001f){
		std::cout<<"sozusagen 0"<<std::endl;
	}

	return retVal;
}

/**
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 */
IWRSENSOR_PARSED AttitudeSensor::normalizeGyro(
    IWRSENSOR_PARSED &biasGyro, 
	IWRSENSOR_PARSED &sensor){

	IWRSENSOR_PARSED retVal;	
	retVal.x = sensor.x - biasGyro.x;
	retVal.y = sensor.y - biasGyro.y;
	retVal.z = sensor.z - biasGyro.z;
	
    return retVal;
}


/**
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 */
IWRSENSOR_PARSED_F AttitudeSensor::normalizeSensor(
    IWRSENSOR_PARSED &calibMin,  
	IWRSENSOR_PARSED &calibMax, 
    IWRSENSOR_PARSED &sensor) {

	IWRSENSOR_PARSED_F retVal;		
	
    retVal.x=normalizeValue(calibMin.x,calibMax.x,sensor.x);
	retVal.y=normalizeValue(calibMin.y,calibMax.y,sensor.y);
	retVal.z=normalizeValue(calibMin.z,calibMax.z,sensor.z);



    return retVal;
}

/**
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 * @author Justin Philipp Heinermann<justin.philipp.heinermann@uni-oldenburg.de>
 */
IWRSENSDATA_PARSED AttitudeSensor::parseData() {

	IWRSENSDATA_PARSED ret;
	unsigned char *ptrData = (unsigned char*) &this->sensdata;
	signed short *ptrRet = (signed short*) &ret;

	for(int i = 0, j=0; i < 12; i+=2,  j++) { //mag and acc
		unsigned char *lsb = ptrData + i;
		unsigned char *msb = ptrData + i + 1;
		*(ptrRet + j) = (((unsigned short) *msb << 8) | (unsigned short) *lsb);
	} 

//	for(int i = 12, j=6; i < 18; i+=2,  j++) { //high bandwidth gyro
	for(int i = 18, j=6; i < 24; i+=2,  j++) { //low bandwidth gyro
		unsigned char *lsb = ptrData + i;
		unsigned char *msb = ptrData + i + 1;
		*(ptrRet + j) = (((unsigned short) *msb << 8) | (unsigned short) *lsb);
	} 
	return ret;
}

/**
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 */
void AttitudeSensor::timerProc() throw (AttitudeSensorException){
	if(!this->vuzixConnected){
			return;
	}
	receive();

	double yaw, pitch, roll;
	
	IWRSENSOR_PARSED_F normalizedMagSensorData =
	    AttitudeSensor::normalizeSensor(
        this->calibMagMin, 
	    this->calibMagMax, 
        this->parsed.mag_sensor);
	
	IWRSENSOR_PARSED_F normalizedAccSensorData = 
        AttitudeSensor::normalizeSensor(
        this->calibAccMin, 
	    this->calibAccMax, 
        this->parsed.acc_sensor);

	IWRSENSOR_PARSED normalizedGyrSensorData = 
	    AttitudeSensor::normalizeGyro(
        this->biasGyro, 
	    this->parsed.gyro_sensor);	
	
	ANGLES angles=AttitudeSensor::calculateAngles(
	    this->currentAngles,		
	    normalizedMagSensorData, 
	    normalizedAccSensorData, 
	    normalizedGyrSensorData, 
	    this->currentGyro, 
	    this->ringbufferAccPitch, 
	    this->ringbufferAccRoll, 
	    this->currentAccPitch,
	    this->currentAccRoll);

	pitch = angles.pitch;
	roll = angles.roll;
	yaw = angles.yaw;
	
	if(this->useYaw)
		this->head->angles.yawDeg = angles.yaw - this->zeroAngles.yaw; 
	
	if(this->usePitch)
		this->head->angles.pitchDeg = angles.pitch- this->zeroAngles.pitch; 
	
	if(this->useRoll)
		this->head->angles.rollDeg = angles.roll -this->zeroAngles.roll;
}

/**
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 */
void AttitudeSensor::resetHeadDirection() {
	this->zeroAngles.yaw=this->currentGyro.yaw;
	this->zeroAngles.pitch=this->currentAngles.pitch;
	this->zeroAngles.roll=this->currentAngles.roll;
	
	printf("Zeroes: %f || %f || %f\n", this->zeroAngles.yaw, 
	this->zeroAngles.pitch, this->zeroAngles.roll);
}

/**
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 */
void AttitudeSensor::toggleUseYaw() {
	this->useYaw= (!this->useYaw);
}	

/**
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 */
void AttitudeSensor::toggleUsePitch() {
	this->usePitch= (!this->usePitch);
}	

/**
 * @author Justin Philipp Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 */
void AttitudeSensor::toggleUseRoll() {
	this->useRoll= (!this->useRoll);
}	

#endif

/** 
 * @brief This method catches the data from the Vuzix and saves them to a struct.
 * 
 * WARNING: UNIX code only works on little endian architectures.
 *          Don't panic, x86 is little endian.
 *
 * @author Christian Herz <christian.herz@uni-oldenburg.de>
 * @author Matthias Bruns <matthias.bruns@uni-oldenburg.de>
 * @author Jendrik Poloczek <jendrik.poloczek@uni-oldenburg.de>
 * @author Justin Philip Heinermann <justin.philipp.heinermann@uni-oldenburg.de>
 */
 #ifdef WIN32
void AttitudeSensor::timerProc() throw (AttitudeSensorException) {

	while (true) {
		// If pointer is not yet allocated
		if(this->head == NULL)
			this->head = new Head();	

		IWRSENSDATA sensdata;
		LONG yaw = 0, pitch = 0, roll = 0, myaw = 0;
        LONG xtrn = 0, ytrn = 0, ztrn = 0;

		// if errors come up restart the poll
		if (IWRGet6DTracking(&yaw, &pitch, &roll, &xtrn, &ytrn, &ztrn) 
            != ERROR_SUCCESS) {
			
            try {
				this->startPoll();
			} catch (AttitudeSensorException &ex) {
				LOG(ATTITUDE_SENSOR_LOGGER_NAME) 
					<< ex.message() << Logger::endl;
			}
		}

		// getting data from the vuzix
		switch(IWRGetSensorData(&sensdata)) {
			case ERROR_SUCCESS:
				// everything went ok
				break;
			// Device does not support this function, throw ex
			case ERROR_NOT_SUPPORTED: { 
				AttitudeSensorException ex 
                    (AttitudeSensorException::ERROR_VUZIX_NOT_SUPPORTED);
				
                LOG(ATTITUDE_SENSOR_LOGGER_NAME) 
					<< ex.message() << Logger::endl;
				throw ex;
				break;
			}

			// Device is not connected, throw ex
			case ERROR_DEV_NOT_EXIST: { 
				this->vuzixConnected = false;
				
                AttitudeSensorException ex 
                    (AttitudeSensorException::ERROR_VUZIX_NOT_EXIST);
				
                LOG(ATTITUDE_SENSOR_LOGGER_NAME) 
					<< ex.message() << Logger::endl;
				throw ex;
				break;
			}
		}

		// get lates yaw values
		switch(IWRGetMagYaw(&myaw)) {
			case ERROR_SUCCESS:
				// everything went ok
				break;
			// Device does not support this function, throw ex
			case ERROR_NOT_SUPPORTED: { 
				
                AttitudeSensorException ex 
                    (AttitudeSensorException::ERROR_VUZIX_NOT_SUPPORTED);
				
                LOG(ATTITUDE_SENSOR_LOGGER_NAME) 
					<< ex.message() << Logger::endl;
				
                throw ex;
				break;
			}

			// Device is not connected, throw ex
			case ERROR_DEV_NOT_EXIST: { 
				this->vuzixConnected = false;

				AttitudeSensorException ex 
                    (AttitudeSensorException::ERROR_VUZIX_NOT_EXIST);
				
                LOG(ATTITUDE_SENSOR_LOGGER_NAME) 
					<< ex.message() << Logger::endl;
				throw ex;
				break;
			}
		}

		// angles in degree
		this->head->angles.yawDeg = yaw/180;
		this->head->angles.rollDeg = roll/180;
		this->head->angles.pitchDeg = pitch/180;
	}
}
#endif

} /* End of namespace Middleware */
