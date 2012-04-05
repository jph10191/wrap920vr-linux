#include "../Logger.h" 
#include "../AttitudeSensor.h"
#include <stdio.h>
#include <iostream>

AttitudeSensor * as;
const Head * head;
bool run=true;

int main(){
	as = new AttitudeSensor();
	if(as->vuzixConnected){
		while(run){
			if(!as->vuzixConnected){
				run=false;
				break;
			}
			as->timerProc();
			head=as->getHeadDirection();
			std::cout<<"yaw: "<<head->angles.yawDeg<<std::endl;
			std::cout<<"pitch: "<<head->angles.pitchDeg<<std::endl;
			std::cout<<"roll: "<<head->angles.rollDeg<<std::endl;
			usleep(5000); //5 millis
		}
	}
}
