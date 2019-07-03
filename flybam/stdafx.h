// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <windows.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <tchar.h>
#include <vector>
#include <omp.h>
#include <queue>
#include <algorithm>
#include <numeric>
#include "conio.h"
#include <mmsystem.h>
#include <concurrent_queue.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "FlyCapture2.h"
#include <NIDAQmx.h>

#include "pgrcam.h"
#include "fvfmfwriter.h"
#include "avfmfwriter.h"
#include "daq.h"
#include "utility.h"
#include "arduino.h"

#include "sapclassbasic.h"
#include "ExampleUtils.h"

#define GALVO_Y_HEIGHT 68.167			//in mm
#define GALVO_XY_DIST 15.174			//in mm
#define GALVO_X_MIRROR_ANGLE 15			//in degrees
#define GALVO_STEP_SIZE 0.0000075		// doubled the step size to maintain similar manual speed control (due to spectre/meltdown processor slowdown)

#define XVOLTPERDEGREE 0.55
#define YVOLTPERDEGREE 0.525

#define XOFFSET -0.25		// x-offset for centering galvo to target (in volts)
#define YOFFSET -0.315		// y-offset for centering galvo to target (in volts)

#define SCALEX 0.0008
#define SCALEY 0.0008

#define NFLIES 1
#define NLOSTFRAMES 5

#define MAXFVRECFRAMES 1000*100
#define MAXAVRECFRAMES 50*100
#define FLASHFRAMES 5

#define MIN_MARKER_SIZE 10
#define MAX_MARKER_SIZE 500
#define NBEADS 3

#define Z_PERIOD 100
#define Z_EPSILON 5.0
#define Z_STEP_COARSE 10.0
#define Z_STEP_FINE 5.0


// TODO: reference additional headers your program requires here
