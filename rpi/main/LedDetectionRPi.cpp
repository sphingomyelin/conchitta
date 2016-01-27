#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <raspicam/raspicam_cv.h> 

using namespace cv;
using namespace std;

const float EXPOSURE_TRUC = 0.2;

const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;

const int WINDOWWIDTH = 320;
const int WINDOWHEIGHT = 30;
const int ONELINEHEIGHT = 1;

const int HUEVALUE = 0;
const int SATURATIONVALUE = 1;
const int BRIGHTVALUE = 2;

const int NBPIX = 5; //IMPAIR

const int MINTHRES_Y = 10;
const int MAXTHRES_Y = 30;

const int MINTHRES_G = 35; //60;
const int MAXTHRES_G = 60; //85;

///blue neer: 100-110
const int MINTHRES_B = 85;
const int MAXTHRES_B = 130;

const int BIGTHRES_R = 130; //95
const int SMALLTHRES_R = 10;

const int THRESHOLD_BRIGHTNESS =25; //84;


void findColor(int meanpixValue){
    if(meanpixValue> MINTHRES_Y && meanpixValue < MAXTHRES_Y){
        cout<<"yellow"<<endl;
    }
    else if(meanpixValue> BIGTHRES_R || meanpixValue < SMALLTHRES_R){ 
        cout<<"red"<<endl;
    }
    else if(meanpixValue> MINTHRES_G && meanpixValue < MAXTHRES_G){
        cout<<"green"<<endl;
    }
    else if(meanpixValue> MINTHRES_B && meanpixValue < MAXTHRES_B){
        cout<<"blue"<<endl;
    }
    else{
        cout<<"----"<<endl;
    }
}


int main(){
	raspicam::RaspiCam_Cv Camera;
	
    Mat cameraFeed2;
	Mat hsvImage_led;
    vector<Mat> hsvChannels;
    int i = 0, j = 0;
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
    
    double t4 = 0, t5 = 0;


    Camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
	Camera.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
    Camera.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
	Camera.set(CV_CAP_PROP_EXPOSURE,EXPOSURE_TRUC);
	Camera.set(CV_CAP_PROP_GAIN,0.1);
	
    cout<<"Opening Camera..."<<endl;
    if (!Camera.open()) {
		cerr<<"Error opening the camera"<<endl;return -1;
	}
	


    while(1){
	
		Camera.grab();
		Camera.retrieve(cameraFeed2);
		
		t4=(double)getTickCount();
		
		cvtColor(cameraFeed2,hsvImage_led,COLOR_BGR2HSV);
		split(hsvImage_led,hsvChannels);
        brightness = hsvChannels[BRIGHTVALUE];
        teinte = hsvChannels[HUEVALUE];
        //~ saturation = hsvChannels[SATURATIONVALUE];


        //cout<<"line: ";
        for(i=0;i<WINDOWWIDTH-1;i++){
            for(j=0;j<WINDOWHEIGHT-1;j++){
                //~ cameraWindowV.at<uchar>(Point(i,j)) = brightness.at<uchar>(Point(i,j));
                //~ cameraWindowH.at<uchar>(Point(i,j)) = teinte.at<uchar>(Point(i,j));
                pixV = pixV +  (int)brightness.at<uchar>(Point(i,j));
                pixH = pixH +  (int)teinte.at<uchar>(Point(i,j));
            }
            pixmiddleV = pixV/WINDOWHEIGHT;
            pixmiddleH = pixH/WINDOWHEIGHT;
            ///ligne avec les valeurs moyennes par colonne de la fenetre
            meanOneLineV.at<uchar>(Point(i,0)) = (unsigned char) pixmiddleV;
            meanOneLineH.at<uchar>(Point(i,0)) = (unsigned char) pixmiddleH;
            //cout<<pixmiddleH<<",";
            pixV = 0; pixmiddleV = 0;
            pixH = 0; pixmiddleH = 0;
        }
        //cout<<"--"<<endl;

        minMaxIdx(meanOneLineV,0,&maxVal,0,maxIdx);
        cout<<"maxBrightValue: "<<maxVal<<endl;

        //~ for(i=0;i<NBPIX;i++){
            //~ ledColor = ledColor + (int)meanOneLineH.at<uchar>(Point(i+maxIdx[1]-((NBPIX-1)/2),0));
        //~ }
        //~ ledColor = ledColor/NBPIX;
        //~ cout<<"Color "<<ledColor<<endl;
        //~ findColor(ledColor);


	  if(maxVal>THRESHOLD_BRIGHTNESS){
	    ///couleur (H) 
	    for(i=0;i<NBPIX;i++){
	      ledColor = ledColor + (int)meanOneLineH.at<uchar>(Point(i+maxIdx[1]-((NBPIX-1)/2),0));
	    }
	    ledColor = ledColor/NBPIX;
	    findColor(ledColor);
	  }
	  else{
	    cout<<"brightness too low - no color"<<endl;//TEST
	  }
	cout<<"color: "<<ledColor<<endl;
	  ledColor= 0;


        line(cameraFeed2,Point(maxIdx[1],0),Point(maxIdx[1],WINDOWHEIGHT),Scalar(255, 0, 0),1,8,0);

        //imshow("windows Hue",cameraWindowH);
        imshow("cam",cameraFeed2);

		t5=((double)getTickCount()-t4)/getTickFrequency();
		cout<<"time of process: "<< t5<<endl;

        waitKey(30);
    }
	cout<<"Stop camera..."<<endl;
    Camera.release();
    return 0;
}
