#ifndef DAQ_H
#define DAQ_H

using namespace std;
using namespace FlyCapture2;
using namespace cv;

#define STEP_SIZE 1

class Daq
{
private:
	TaskHandle	taskHandleX;
	TaskHandle	taskHandleY;

	//float thetax, thetay;

	float64     dataX[STEP_SIZE];
	float64     dataY[STEP_SIZE];

public:
	float thetax, thetay;

	Daq();
	void reset();
	void configure();
	void start();
	void write();
};

#endif