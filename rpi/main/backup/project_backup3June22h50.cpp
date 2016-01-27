//include system librarys
#include <stdio.h> //for printf
#include <stdint.h> //uint8_t definitions
#include <stdlib.h> //for exit(int);
#include <string.h> //for errno
#include <errno.h> //error output
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>//fct sleep()

//opencv librarys
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <raspicam/raspicam_cv.h>

//wiring Pi
#include <wiringPi.h>
#include <wiringSerial.h>

using namespace std;
using namespace cv;


/*******constant STATE*******/
const char BOTTLE_DETECTION = 'a';
const char GO_HOME = 'b';
const char SHUTDOWN_RPI = 'c';

/*******constant bottle detection*******/
const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;

const int FRAME_CUT = FRAME_HEIGHT/3; //80

const int HUEVALUE = 0;
const int SATURATIONVALUE = 1;
const int BRIGHTVALUE = 2;
//Hue threshold range:
const int RANGEMIN = 85;//70;
const int RANGEMAX = 114;

const int MAX_OBJ = 15;
const int MIN_OBJECT_AREA = 10*10;
const int MAX_OBJECT_AREA = 1000;

raspicam::RaspiCam_Cv Camera;
int position_objects[MAX_OBJ][2]; //constante globale

/*******constant camera init*******/
const float EXPOSURE_TIME_CHOOSEN = 0.5;
const float GAIN_CHOOSEN = 0.1;

/*******constant LED detection*******/
const int WINDOWWIDTH = 320;
const int WINDOWHEIGHT = 30;
const int ONELINEHEIGHT = 1;

const int NBPIX = 5; //IMPAIR!

const int MINTHRES_Y = 10;
const int MAXTHRES_Y = 30;

const int MINTHRES_G = 35; //60;
const int MAXTHRES_G = 60; //85;

///blue neer: 100-110
const int MINTHRES_B = 85;
const int MAXTHRES_B = 130;

const int BIGTHRES_R = 130; //95
const int SMALLTHRES_R = 10;

char color;
int ledPosition;

const int THRESHOLD_BRIGHTNESS = 84;

/*******constant Communication******/
char state;
//char oldState;

const char device[] = "/dev/ttyACM0";
const unsigned long baud = 9600;

const int SEC = 1;
//const int FIVESEC = 5;

int fd;

/*******constant intToChar******/

const int X = 0;
const int Y = 1;
const int COLOR = 2;
const int NB_CHAR = 2; //charvalue[3][5] <-----------------------

const char NO_COLOR = 'n';

char charvalue[NB_CHAR][5]; 

/*********************** PROGRAMME *********************/

int initComm(){
	
  if((fd = serialOpen(device,baud))<0){
    cout<<"trouble opening"<<endl;
    return (-1);
  }
  else cout<<"serialOpen OK"<<endl;

  if(wiringPiSetup() == -1){
    cout<<"unable to start wiringPi"<<endl;
    return(-1);
  }
  else cout<<"wiringPiSetup OK"<<endl;
  sleep(SEC);//---------valeur TEST à ENLEVER! ---------

  return 0;
}

int endComm(){
	
  serialClose(fd);
  cout<<"end"<<endl;
  return 0;
}

int commArduinoReceive(){

  int valueAvailable = 0;
  char temp = 0;

  /***RECEIVE DATA***/		
  if(valueAvailable = serialDataAvail(fd)){
    //~ cout<<"value of available: "<< valueAvailable<<endl;
    temp = serialGetchar (fd);
    cout<<"received: "<<temp<<endl;
  }
  else cout<<"NO value available"<<endl;
  
  if((temp == BOTTLE_DETECTION)||(temp == GO_HOME)||(temp == SHUTDOWN_RPI)){
	  state = temp;
  }
  
  serialFlush(fd);

  return 0;
}

int intToChar(){

  int centaine = 0;
  int dizaine = 0;
  int unite = 0;

  for(int i=0; i<NB_CHAR;i++){
    for(int j=0;j<5;j++){
      charvalue[i][j] = 0;
    }
  }

  int maxYvalue = -2;
  int obj_index = 0;

  int value = 0;
  int letter = X;
  int object = 0;

  ///maximum of position_object
  for(int k=0;k<MAX_OBJ;k++){
    if(maxYvalue < position_objects[k][Y]){
      maxYvalue = position_objects[k][Y]; //VERIFIER QUE C'EST PAS SUR X QUIL FAUT REGARDER
      obj_index = k;
    }
  }
  //~ if(state == GO_HOME){ <--------------------------------
		  //~ letter = COLOR;
	  //~ }
	if(maxYvalue<0){
		return -1;
	}	
  /***int->char***/
  for(int j=0;j<2;j++){
	if(letter == X){
	  charvalue[X][0]='x';
	  object = X;
	  value = position_objects[obj_index][X];
	  letter=Y;
	}
	else if (letter == Y){
	  charvalue[Y][0]='y';
	  object = Y;
	  value = maxYvalue;
	}
    /** <---------------------------------------------------------
      else if(letter == COLOR){ //changer la valeur de letter qqe part
	      charvalue[COLOR][0]=color;
	      object = COLOR;
	      if(color == NO_COLOR){
			 value = 999;
	      }else
		      value = ledPosition;
       }
     **/

    if(value<10){
      charvalue[object][1]=(char)value+'0';
      charvalue[object][2]=' ';
      charvalue[object][3]=' ';
    }
    else if((value>=10)&&(value<100)){
      dizaine = value/10;
      charvalue[object][1]=(char)dizaine+'0';
      unite = value - (dizaine*10);
      charvalue[object][2]=(char)unite+'0';
      charvalue[object][3]=' ';
    }
    else if(value>=100){
      centaine = value/100;
      charvalue[object][1]=(char)centaine+'0';
      dizaine = (value-(centaine*100))/10;
      charvalue[object][2]=(char)dizaine+'0';
      unite = value - (centaine*100) -(dizaine*10);
      charvalue[object][3]=(char)unite+'0';
    }
    charvalue[object][4]=0;
  }
  for(int i=0; i<2;i++){
    cout<<"charvalue XY"<<i+1<<": "<<charvalue[i]<<endl;
  }
  
  return 0;
}

int commArduinoSend(){

  if(intToChar()<0){
	  cout<<"no data to send"<<endl;
	  return -1;
  }

  /*****SEND DATA******/		
  for(int i=0; i<2;i++){
    serialPuts(fd,charvalue[i]);
    cout<<"data send"<<endl;
  }
  return 0;
}



void findColor(int meanpixValue){
  if(meanpixValue> MINTHRES_Y && meanpixValue < MAXTHRES_Y){
    cout<<"yellow"<<endl;
    color ='j';
  }
  else if(meanpixValue> BIGTHRES_R || meanpixValue < SMALLTHRES_R){
    cout<<"red"<<endl;
    color= 'r';
  }
  else if(meanpixValue> MINTHRES_G && meanpixValue < MAXTHRES_G){
    cout<<"green"<<endl;
    color= 'g';
  }
  else if(meanpixValue> MINTHRES_B && meanpixValue < MAXTHRES_B){
    cout<<"blue"<<endl;
    color = 'b';
  }
  else{
    cout<<"color: range not defined"<<endl;
    color = 'n';
  }
}



int ledDetection(){

  Mat cameraFeed2;
  Mat hsvImage_led;
  vector<Mat> hsvChannels;

  Mat brightness, saturation, teinte;
  Mat cameraWindowV = Mat::zeros(WINDOWHEIGHT,WINDOWWIDTH,CV_8U);
  Mat cameraWindowH = Mat::zeros(WINDOWHEIGHT,WINDOWWIDTH,CV_8U);
  Mat meanOneLineV = Mat::zeros(ONELINEHEIGHT,WINDOWWIDTH,CV_8U);
  Mat meanOneLineH = Mat::zeros(ONELINEHEIGHT,WINDOWWIDTH,CV_8U);

  int pixV = 0, pixmiddleV = 0;
  int pixH = 0, pixmiddleH = 0;
  double maxVal = 0;
  int maxIdx[2]={0};
  int ledColor = 0;
  int pixval=0;
  int i = 0, j = 0;
  double t4 = 0, t5 = 0;


  //~ cout<<"begin: LED detection"<<endl;
  t4=(double)getTickCount();
  Camera.grab();
  Camera.retrieve(cameraFeed2);		

  cvtColor(cameraFeed2,hsvImage_led,COLOR_BGR2HSV);
  split(hsvImage_led,hsvChannels);
  brightness = hsvChannels[BRIGHTVALUE];
  teinte = hsvChannels[HUEVALUE];


  for(i=0;i<WINDOWWIDTH-1;i++){
    for(j=0;j<WINDOWHEIGHT-1;j++){
      ///somme les pixel par colonne
      pixV = pixV +  (int)brightness.at<uchar>(Point(i,j));
      pixH = pixH +  (int)teinte.at<uchar>(Point(i,j));
    }
    pixmiddleV = pixV/WINDOWHEIGHT;
    pixmiddleH = pixH/WINDOWHEIGHT;
    ///ligne avec les valeurs moyennes par colonne de la fenetre
    meanOneLineV.at<uchar>(Point(i,0)) = (unsigned char) pixmiddleV;
    meanOneLineH.at<uchar>(Point(i,0)) = (unsigned char) pixmiddleH;
    pixV = 0; pixmiddleV = 0;
    pixH = 0; pixmiddleH = 0;
  }
  ///pix de max intensite (V) - brightness
  minMaxIdx(meanOneLineV,0,&maxVal,0,maxIdx);
  ledPosition = maxIdx[1]; 
  //~ cout<<"brightness value: "<<maxVal<<endl;

  if(maxVal>THRESHOLD_BRIGHTNESS){
    ///couleur (H) 
    for(i=0;i<NBPIX;i++){
      ledColor = ledColor + (int)meanOneLineH.at<uchar>(Point(i+maxIdx[1]-((NBPIX-1)/2),0));
    }
    ledColor = ledColor/NBPIX;
    //~ cout<<"Color/Hvalue "<<ledColor<<endl;
    findColor(ledColor);
  }
  else{
    cout<<"brightness too low - no color"<<endl;
    color = 'n';
  }

  ledColor= 0;

  line(cameraFeed2,Point(maxIdx[1],0),Point(maxIdx[1],WINDOWHEIGHT),Scalar(200, 200, 200),1,8,0);
  //~ cout<<"light position (x): "<<maxIdx[1]<<endl;
  //~ imshow("cam",cameraFeed2);

  t5=((double)getTickCount()-t4)/getTickFrequency();
  //~ cout<<"end: time of process: "<< t5<<endl;

  //~ waitKey(10);    
  return 0;
}



int bottleDetection(){

  Mat cameraFeed, hsvImage;
  Mat morphImage2;
  Mat thresholdImage;

  double t = 0;
  double t1 = 0;
  double t2 =0;
  double t3 = 0;
  double t6 = 0;

  int erosion_size = 10;
  int erosion_size1 = 6;
  int erosion_size2 = 9;
  int erosion_size3 = 7;


  Mat element = getStructuringElement(MORPH_ELLIPSE,Size( 2*erosion_size + 1, 2*erosion_size+1 ),Point( erosion_size, erosion_size ) );
  Mat element1 = getStructuringElement(MORPH_ELLIPSE,Size( 2*erosion_size1 + 1, 2*erosion_size1+1 ),Point( erosion_size1, erosion_size1 ) );
  Mat element2 = getStructuringElement(MORPH_ELLIPSE,Size( 2*erosion_size2 + 1, 2*erosion_size2+1 ),Point( erosion_size2, erosion_size2 ) );
  Mat element3 = getStructuringElement(MORPH_ELLIPSE,Size(2*erosion_size3 +1,2*erosion_size3+1),Point(erosion_size3,erosion_size3));


  //cout<<"begin: BOTTLE detection"<<endl;
  t2=(double)getTickCount();
  //t1=(double)getTickCount();
  Camera.grab();
  Camera.retrieve(cameraFeed);
  //t=((double)getTickCount()-t1)/getTickFrequency();
  //cout<<"grab pic "<<t<<endl;


  ///IMAGE PROCESSING:
  ///cut hupper part of image
  //t1=(double)getTickCount();
  cvtColor(cameraFeed,cameraFeed,COLOR_BGR2GRAY); //3 Channels
  Mat cutImage = cameraFeed(Range(FRAME_CUT,FRAME_HEIGHT),Range::all());
  //t=((double)getTickCount()-t1)/getTickFrequency();
  //cout<<"cutImage "<<t<<endl;

  ///Morphing image processing
  //t1=(double)getTickCount();
  dilate(cutImage,morphImage2,element);
  erode(morphImage2,morphImage2,element1);
  dilate(morphImage2,morphImage2,element2);
  erode(morphImage2,morphImage2,element3);
  //t=((double)getTickCount()-t1)/getTickFrequency();
  //cout<<"dilate+erode "<<t<<endl;

  ///Threshold on lighter part
  //t1=(double)getTickCount();
  inRange(morphImage2,RANGEMIN,RANGEMAX,thresholdImage);
  //t=((double)getTickCount()-t1)/getTickFrequency();
  //cout<<"threshold "<<t<<endl;

  ///find objects and point(x,y)
  vector< vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(thresholdImage,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

  for(int i= 0; i< MAX_OBJ; i++){
    for(int j=0;j<2;j++){
      position_objects[i][j] = (-1);
    }
  }

  //t1=(double)getTickCount();
  if (hierarchy.size() > 0) {
    int numObjects = hierarchy.size();
    if(numObjects<MAX_OBJ){
      for (int index = 0; index >= 0; index = hierarchy[index][0]){
        Moments moment = moments((cv::Mat)contours[index]);
        double area = moment.m00;

        if(area > (MIN_OBJECT_AREA) && area < (MAX_OBJECT_AREA)){
          position_objects[index][X] = moment.m10/area;
          position_objects[index][Y] = moment.m01/area;
          cout<<"x,y: " <<position_objects[index][X]<<","<<position_objects[index][Y]<<endl;
          circle(cutImage,Point(position_objects[index][X],position_objects[index][Y]),7,Scalar(200,200,200),2);
        }
      }
      //~ for(int k=0; k<MAX_OBJ;k++){
        //~ cout<<"obj "<<k+1<<"-- x,y "<<position_objects[k][X]<<","<<position_objects[k][Y]<<endl;
      //~ }
    }
  }
  //t=((double)getTickCount()-t1)/getTickFrequency();
  //cout<<"point centre mass "<<t<<endl;

  //t3=((double)getTickCount()-t2)/getTickFrequency();
  //cout<<"end: time of process:  "<<t3<<endl;


  //~ imshow("morph",morphImage2);
  //~ //imshow("thresh",thresholdImage);
  //~ waitKey(10);
  //~ 
  //~ cout<<"je suis ici"<<endl;
  //~ namedWindow("cut",WINDOW_AUTOSIZE);
  //~ imshow("cut",cutImage);
  //~ waitKey(10);
  //~ cout<<"je suis la"<<endl;


  t6=((double)getTickCount()-t2)/getTickFrequency();
  cout<<"TotTime "<<t6<<endl;
  return 0;
}



int init_camera(){

  ///setting camera
  Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
  Camera.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
  Camera.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
  Camera.set(CV_CAP_PROP_EXPOSURE,EXPOSURE_TIME_CHOOSEN);
  Camera.set(CV_CAP_PROP_GAIN,GAIN_CHOOSEN);

  if (!Camera.open()) {
    cerr<<"Error opening the camera"<<endl;
    return -1;
  }
  cout<<"camera init OK"<<endl;
  sleep(SEC);//---------valeur TEST à ENLEVER! ---------
  return 0;

}



int main(){
  //state = BOTTLE_DETECTION;
  state = 0;
    
  while(initComm() < 0){}
  
  while(init_camera()<0){}

  while(1){
    cout<<"programme START"<<endl;
    commArduinoReceive();

    switch(state){
      case BOTTLE_DETECTION: 
        cout<< "STATE: bottle"<<endl; 
        bottleDetection();
        //ledDetection();
        commArduinoSend();
        break;
      case GO_HOME: 
        cout<<"STATE: go home"<<endl; 
        ledDetection();
        //commArduinoSend(); //->AJOUTER dans intToChar()
        break;
      case SHUTDOWN_RPI: 	
        cout<<"STATE: shutdown"<<endl;
        //~ Camera.release();
        //~ endComm();
        //~ system("sudo shutdown -h now"); 
        break;
      default: 
        cout<<"STATE: default - statevalue: "<<state<<endl;
        break;
    }
    cout<<"programme END"<<endl;
    cout<<"-------------"<<endl;
  }
  return 0;
}
