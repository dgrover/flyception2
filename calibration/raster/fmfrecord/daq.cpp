#include "stdafx.h"
#include "daq.h"

Daq::Daq()
{
	taskHandleX = 0;
	taskHandleY = 0;

	thetax = MIN_ANGLE;
	thetay = MIN_ANGLE;

	dataX[0] = 0.0;
	dataY[0] = 0.0;
}

void Daq::configure()
{
	// DAQmx Configure Code
	DAQmxCreateTask("", &taskHandleX);
	DAQmxCreateTask("", &taskHandleY);
	DAQmxCreateAOVoltageChan(taskHandleX, "Dev1/ao0", "", -10.0, 10.0, DAQmx_Val_Volts, "");
	DAQmxCreateAOVoltageChan(taskHandleY, "Dev1/ao1", "", -10.0, 10.0, DAQmx_Val_Volts, "");
}

void Daq::start()
{
	// DAQmx Start Code
	DAQmxStartTask(taskHandleX);
	DAQmxStartTask(taskHandleY);
}

void Daq::write()
{

	dataX[0] = thetax * XVOLTPERDEGREE;
	dataY[0] = thetay * YVOLTPERDEGREE;
	
	// DAQmx Write Code
	DAQmxWriteAnalogF64(taskHandleX, STEP_SIZE, 0, 10.0, DAQmx_Val_GroupByChannel, dataX, NULL, NULL);
	DAQmxWriteAnalogF64(taskHandleY, STEP_SIZE, 0, 10.0, DAQmx_Val_GroupByChannel, dataY, NULL, NULL);

	//function for scalar value
	//DAQmxWriteAnalogScalarF64(taskHandleX, 0, 10.0, dataX, NULL);
}

void Daq::reset()
{
	thetax = MIN_ANGLE;
	thetay = MIN_ANGLE;
}
