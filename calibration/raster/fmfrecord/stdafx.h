// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <windows.h>
#include <stdio.h>
#include <tchar.h>
#include <vector>
#include <algorithm>

#include <omp.h>
#include <queue>


#include "FlyCapture2.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>

#include "pgrcam.h"
#include "readerwriterqueue.h"
#include <NIDAQmx.h>
#include "daq.h"

#define XVOLTPERDEGREE 0.55
#define YVOLTPERDEGREE 0.525

#define MIN_ANGLE -7.5
#define MAX_ANGLE 7.5
#define ANGLE_STEP_SIZE 0.1

// TODO: reference additional headers your program requires here
