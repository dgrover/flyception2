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
	TaskHandle  taskHandleDig;

	float thetax, thetay;

	float64     dataX[STEP_SIZE];
	float64     dataY[STEP_SIZE];
	uInt8       dataDig[8];
	uInt8		ifB0;
	uInt8		ifB1;
	uInt8		ifB2;

public:
	Daq();
	void reset();
	void configure();
	void start();
	void startTrigger();
	void stopTrigger();
	void lensCommand(int cmd);
	void write();
	void flashHigh();
	void flashLow();
	//void ConvertPtToDeg(Point2f pt);
	void ConvertPixelToDeg(float x, float y);
	Point2f ConvertDegToPt();
	Point2f GetGalvoAngles();
	void SetGalvoAngles(Point2f angle);
	void MoveLeft();
	void MoveRight();
	void MoveUp();
	void MoveDown();
	void PollLensPosition();
};

#endif