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

void Daq::lensCommand(char cmd)
{
	switch (cmd) {

		// Move -10 Steps
	case '1':
		dataDig[4] = 0;
		dataDig[3] = 0;
		break;
		// Move +10 Steps
	case '2':
		dataDig[4] = 0;
		dataDig[3] = 1;
		break;
		// Move to Min
	case '4':
		dataDig[4] = 1;
		dataDig[3] = 0;
		break;
		// Move to Max
	case '5':
		dataDig[4] = 1;
		dataDig[3] = 1;
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