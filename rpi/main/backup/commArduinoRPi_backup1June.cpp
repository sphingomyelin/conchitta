//include system librarys
#include <stdio.h> //for printf
#include <stdint.h> //uint8_t definitions
#include <stdlib.h> //for exit(int);
#include <string.h> //for errno
#include <errno.h> //error output

//opencv librarys
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <raspicam/raspicam_cv.h>
 
//wiring Pi
#include <wiringPi.h>
#include <wiringSerial.h>

#include <unistd.h>

using namespace std;
using namespace cv;

const char device[]= "/dev/ttyACM0";
const unsigned long baud = 9600;

const int NBLOOP = 10;

const char BOTTLE_DETECTION = 'a';
const char GO_HOME = 'b';
const char SHUTDOWN_RPi = 'c';

const int SEC = 1;
const int FIVESEC = 5;

char state;
char oldState;

int main(){
 
	int fd;
	
	//~ int xvalue = 567;
	//~ int yvalue = 9;
	
	int compteur = 0;
	int valueAvailable = 0;
	char xcharvalue[] = "x567.";
	char ycharvalue[] = "y89 .";
	
	oldState = 'a';
	
	
/*****ouverture communication******/
	if((fd = serialOpen(device,baud))<0){
		cout<<"trouble opening"<<endl;
		//exit(-1);
	}
	else cout<<"serialOpen OK"<<endl;
	sleep(SEC); //TEST


	if(wiringPiSetup() == -1){
		cout<<"unable to start wiringPi"<<endl;
		//exit(-1);
	}
	else cout<<"wiringPiSetup OK"<<endl;
	sleep(SEC); //TEST
	
	cout<<"wait 1s..."<<endl; //TEST
	sleep(FIVESEC);//TEST
	
	while(compteur<NBLOOP){	
	/*****SEND DATA******/		
		serialPuts(fd,xcharvalue);
		serialPuts(fd,ycharvalue);
		cout<<"data "<<compteur<<" send"<<endl;
		compteur++;
	}
	
	while(1){
	/*****RECEPTION DATA******/
		if(valueAvailable = serialDataAvail(fd)){
			cout<<"value of available: "<< valueAvailable<<endl;
			state = serialGetchar (fd);
			cout<<"received: "<<state<<endl;
			sleep(SEC);
		}
		//else {cout<<"NO reception"<<endl;}
		
		//cout<<xcharvalue[0]<<","<<xcharvalue[1]<<endl; //RETOURNE x5
	
		//~ //if(state != oldState){
		//~ switch(state){
			//~ case BOTTLE_DETECTION: 
				//~ cout<< "STATE: bottle"<<endl; 
				//~ //oldState = state; 
				//~ //bottleDetection();
				//~ break;
			//~ case GO_HOME: 
				//~ cout<<"STATE: go home"<<endl; 
				//~ //oldState = state; 
				//~ //ledDetection();
				//~ break;
			//~ case SHUTDOWN_RPi: 	
				//~ cout<<"STATE: shutdown"<<endl;
				//~ //oldState = state; 
				//~ break;
			//~ default: 
				//~ //cout<<"STATE: not defined"<<endl;
				//~ break;
		//~ }
	//}
	}
		
	serialClose(fd);
	cout<<"end"<<endl;
	return 0;
}
