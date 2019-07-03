#include "stdafx.h"
#include "daq.h"

Daq::Daq()
{
	taskHandleX = 0;
	taskHandleY = 0;
	taskHandleDig = 0;

	thetax = 0.0;
	thetay = 0.0;

	dataX[0] = 0.0;
	dataY[0] = 0.0;

	for (int i = 0; i < 8; i++)
		dataDig[i] = 0;

	ifB0 = 3;
	ifB1 = 4;
	ifB2 = 5;


}

void Daq::configure()
{
	// DAQmx Configure Code
	DAQmxCreateTask("", &taskHandleX);
	DAQmxCreateTask("", &taskHandleY);
	DAQmxCreateTask("", &taskHandleDig);

	DAQmxCreateAOVoltageChan(taskHandleX, "Dev1/ao0", "", -10.0, 10.0, DAQmx_Val_Volts, "");
	DAQmxCreateAOVoltageChan(taskHandleY, "Dev1/ao1", "", -10.0, 10.0, DAQmx_Val_Volts, "");
	DAQmxCreateDOChan(taskHandleDig, "Dev1/port0/line0:7", "", DAQmx_Val_ChanForAllLines);

}

void Daq::start()
{
	// DAQmx Start Code
	DAQmxStartTask(taskHandleX);
	DAQmxStartTask(taskHandleY);
	DAQmxStartTask(taskHandleDig);
}

void Daq::startTrigger()
{
	// Enable Triggers
	dataDig[0] = 1;
	DAQmxWriteDigitalLines(taskHandleDig, 1, 1, 10.0, DAQmx_Val_GroupByChannel, dataDig, NULL, NULL);


}

void Daq::stopTrigger()
{
	// Disable Triggers
	dataDig[0] = 0;
	DAQmxWriteDigitalLines(taskHandleDig, 1, 1, 10.0, DAQmx_Val_GroupByChannel, dataDig, NULL, NULL);
}

void Daq::flashHigh()
{

	//Trigger Flash
	dataDig[1] = 1;
	DAQmxWriteDigitalLines(taskHandleDig, 1, 1, 10.0, DAQmx_Val_GroupByChannel, dataDig, NULL, NULL);
}

void Daq::flashLow()
{
	dataDig[1] = 0;
	DAQmxWriteDigitalLines(taskHandleDig, 1, 1, 10.0, DAQmx_Val_GroupByChannel, dataDig, NULL, NULL);
}

/*
* Lens Interface
  DAq  ->  Arduino  Pin
  -----------------------
	3  ->  5		Bit 0
	4  ->  6		Bit 1
	5  ->  7		Bit 2

* Lens Commands
	Code	-		Command
	0		-		Move down coarse
	1		-		Move up coarse
	2		-		Move down fine
	3		-		Move up	fine
	4		-		Move down to min
	5		-		Move up to max
	6		-		Query lens position 

*/
void Daq::lensCommand(int cmd)
{
	switch (cmd) 
	{

	// Move -Z_STEP_COARSE Steps
	case 1:
		dataDig[ifB2] = 0;
		dataDig[ifB1] = 0;
		dataDig[ifB0] = 1;
		break;

	// Move +Z_STEP_COARSE Steps
	case 2:
		dataDig[ifB2] = 0;
		dataDig[ifB1] = 1;
		dataDig[ifB0] = 0;
		break;

	// Move -Z_STEP_FINE Steps
	case 3:
		dataDig[ifB2] = 0;
		dataDig[ifB1] = 1;
		dataDig[ifB0] = 1;
		break;

	// Move +Z_STEP_FINE Steps
	case 4:
		dataDig[ifB2] = 1;
		dataDig[ifB1] = 0;
		dataDig[ifB0] = 0;
		break;

	// Move to Min
	case 5:
		dataDig[ifB2] = 1;
		dataDig[ifB1] = 0;
		dataDig[ifB0] = 1;
		break;

	// Move to Max
	case 6:
		dataDig[ifB2] = 1;
		dataDig[ifB1] = 1;
		dataDig[ifB0] = 0;
		break;

	// Query lens position over serial
	case 7:
		dataDig[ifB2] = 1;
		dataDig[ifB1] = 1;
		dataDig[ifB0] = 1;
		break;
	}


	dataDig[2] = !dataDig[2];
	DAQmxWriteDigitalLines(taskHandleDig, 1, 1, 10.0, DAQmx_Val_GroupByChannel, dataDig, NULL, NULL);
}

void Daq::write()
{

	dataX[0] = XOFFSET + (thetax * XVOLTPERDEGREE);
	dataY[0] = YOFFSET + (thetay * YVOLTPERDEGREE);
	
	// DAQmx Write Code
	DAQmxWriteAnalogF64(taskHandleX, STEP_SIZE, 0, 10.0, DAQmx_Val_GroupByChannel, dataX, NULL, NULL);
	DAQmxWriteAnalogF64(taskHandleY, STEP_SIZE, 0, 10.0, DAQmx_Val_GroupByChannel, dataY, NULL, NULL);

	//function for scalar value
	//DAQmxWriteAnalogScalarF64(taskHandleX, 0, 10.0, dataX, NULL);
}

void Daq::ConvertPtToDeg(Point2f pt)
{
	// move mirror by half the angle
	thetax = atan(pt.x / (GALVO_Y_HEIGHT + GALVO_XY_DIST)) * 180 / (CV_PI * 2);
	thetay = atan(pt.y / ( GALVO_Y_HEIGHT / cos(thetax * 2 * CV_PI / 180 ) )  ) * 180 / (CV_PI * 2);
}

void Daq::ConvertPixelToDeg(float x, float y)
{
	thetax = thetax + x;
	thetay = thetay + y;
}

Point2f Daq::ConvertDegToPt()
{
	Point2f pt;

	//final point would be at double the mirror angle
	pt.x = (GALVO_Y_HEIGHT + GALVO_XY_DIST) * tan(thetax * 2 * CV_PI / 180);
	pt.y = (GALVO_Y_HEIGHT / cos(thetax * 2 * CV_PI / 180)) * tan(thetay * 2 * CV_PI / 180);

	return pt;
}

void Daq::reset()
{
	thetax = 0.0;
	thetay = 0.0;
}

Point2f Daq::GetGalvoAngles()
{
	Point2f angle;

	angle.x = thetax;
	angle.y = thetay;

	return angle;
}

void Daq::SetGalvoAngles(Point2f angle)
{
	thetax = angle.x;
	thetay = angle.y;
}

void Daq::MoveLeft()
{
	//thetax += 0.1;
	thetax += GALVO_STEP_SIZE;
}

void Daq::MoveRight()
{
	//thetax -= 0.1;
	thetax -= GALVO_STEP_SIZE;
}

void Daq::MoveUp()
{
	//thetay += 0.1;
	thetay += GALVO_STEP_SIZE;
}

void Daq::MoveDown()
{
	//thetay -= 0.1;
	thetay -= GALVO_STEP_SIZE;
}