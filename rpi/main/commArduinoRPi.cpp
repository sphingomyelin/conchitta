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

const int NBLOOP = 1;
const int NBLOOP_RECEPTION = 30;

const char BOTTLE_DETECTION = 'a';
const char GO_HOME = 'b';
const char SHUTDOWN_RPi = 'c';

const int SEC = 1;
const int FIVESEC = 5;

const int X = 0;
const int Y = 1;

char state;
char oldState;

int main(){
 
	int fd;
	int valueAvailable = 0;
		
	int centaine =0;
	int dizaine = 0;
	int unite = 0;
	
	int compteur = 0;
	
	//~ char xcharvalue[] = "x567.";
	//~ char ycharvalue[] = "y89 .";
	//à remplir
	char charvalue[] = "0000.";
	
	
	oldState = 'a';
	int value = 1;
	bool letter = 0;

	
	///xvalue=position_objects[index][0]
	///yvalue=position_objects[index][1]
	if(letter == X)
		charvalue[0]='x';
	else if (letter == Y)
		charvalue[0]='y';
	//~ else 
		//~ cout<<"prob"<<endl;
	
	if(value<10){
		charvalue[1]=(char)value+'0';
		charvalue[2]='.';
		charvalue[3]='.';
	}
	else if((value>=10)&&(value<100)){
		dizaine = value/10;
		charvalue[1]=(char)dizaine+'0';
		unite = value - (dizaine*10);
		charvalue[2]=(char)unite+'0';
		charvalue[3]='.';
	}
	else if(value>=100){
		centaine = value/100;
		charvalue[1]=(char)centaine+'0';
		dizaine = (value-(centaine*100))/10;
		charvalue[2]=(char)dizaine+'0';
		unite = value - (centaine*100) -(dizaine*10);
		charvalue[3]=(char)unite+'0';
	}
	//~ else
		//~ cout<<"prob2"<<endl;
		

	for(int i = 0; i<5;i++){
		cout<<"charvalue "<<charvalue[i]<<endl;
	}
	sleep(SEC);
	
/*****ouverture communication******/
	if((fd = serialOpen(device,baud))<0){
		cout<<"trouble opening"<<endl;
		//exit(-1);
	}
	else cout<<"serialOpen OK"<<endl;
	//sleep(SEC); //TEST


	if(wiringPiSetup() == -1){
		cout<<"unable to start wiringPi"<<endl;
		//exit(-1);
	}
	else cout<<"wiringPiSetup OK"<<endl;
	//sleep(SEC); //TEST
	
	cout<<"wait 5s..."<<endl; //TEST
	sleep(FIVESEC);//TEST
	
	while(compteur<NBLOOP){	
	/*****SEND DATA******/		
		serialPuts(fd,charvalue);
		//~ serialPuts(fd,ycharvalue);
		cout<<"data "<<compteur<<" send"<<endl;
		compteur++;
	}
	
	compteur = 0;
	while(1){ 
	/*****RECEPTION DATA******/
		if(valueAvailable = serialDataAvail(fd)){
			cout<<"value of available: "<< valueAvailable<<endl;
			state = serialGetchar (fd);
			cout<<"received: "<<state<<endl;
			//sleep(SEC); //TEST
		}
		//else {cout<<"NO reception"<<endl;}
		
		//cout<<xcharvalue[0]<<","<<xcharvalue[1]<<endl; //RETOURNE x5
		
		compteur++;
	
		if(state != oldState){
		switch(state){
			case BOTTLE_DETECTION: 
				cout<< "STATE: bottle"<<endl; 
				oldState = state; 
				//bottleDetection();
				break;
			case GO_HOME: 
				cout<<"STATE: go home"<<endl; 
				oldState = state; 
				//ledDetection();
				break;
			case SHUTDOWN_RPi: 	
				cout<<"STATE: shutdown"<<endl;
				//oldState = state; 
				break;
			default: 
				//cout<<"STATE: not defined"<<endl;
				break;
		}
	}
	}
		
	serialClose(fd);
	cout<<"end"<<endl;
	return 0;
}


/*********************************************************************/
//~ (ledDetect LedDetectionRPi.cpp)   ledDetect
//~ (project2 project.cpp)  project2
//~ (bottleDetection essaiBottleDetection_RPi.cpp)  bottleDetection
/*********************************************************************/


