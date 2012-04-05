#include "../Logger.h" 
#include "../AttitudeSensor.h"
#include <stdio.h>
#include <iostream>

AttitudeSensor * as;
bool run=true;
int main(){
	as = new AttitudeSensor();
	if(as->vuzixConnected){
		while(run){
			if(as->vuzixConnected){
				run=false;
				break;
			}
			as->timerProc();
			std::cout<<"bla"<<std::endl;
		}
	}
}
