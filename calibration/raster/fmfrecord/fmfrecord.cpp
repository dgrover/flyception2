// fmfrecord.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;
using namespace moodycamel;

bool stream = true;
bool record = false;

int imageWidth = 512, imageHeight = 512;
int imageLeft = 384, imageTop = 256;

//Point center(263, 219);
//int maj_axis = 236, min_axis = 138;
//int angle = 178;

ReaderWriterQueue<Image> q(30);
ReaderWriterQueue<Mat> disp_frame(1), disp_mask(1);

int last = 0, fps = 0;

int ConvertTimeToFPS(int ctime, int ltime)
{
	int dtime;

	if (ctime < ltime)
		dtime = ctime + (8000 - ltime);
	else
		dtime = ctime - ltime;

	if (dtime > 0)
		dtime = 8000 / dtime;
	else
		dtime = 0;

	return dtime;
}

void OnImageGrabbed(Image* pImage, const void* pCallbackData)
{
	Image img;
	
	fps = ConvertTimeToFPS(pImage->GetTimeStamp().cycleCount, last);
	last = pImage->GetTimeStamp().cycleCount;

	img.DeepCopy(pImage);
	
	if (stream)
		q.enqueue(img);
	
	return;
}

vector<vector<Point>> findLargestContour(Mat mask, int &j, float &max_size, int &contour_count, Point &pt)
{
	vector<vector<Point>> contours;
	findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	int nrow = mask.rows;
	int ncol = mask.cols;

	mask = Mat::zeros(Size(ncol, nrow), CV_8UC1);
	contour_count = 0;

	if (contours.size() > 0)
	{
		max_size = 0;

		// Get the moments and mass centers
		vector<Moments> mu(contours.size());
		vector<Point2f> mc(contours.size());

		for (int i = 0; i < contours.size(); i++)
		{
			//drawContours(mask, fly_contours, i, Scalar(255, 255, 255), FILLED, 1);
			mu[i] = moments(contours[i], false);
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);

			double csize = contourArea(contours[i]);

			if (csize > 50)
				contour_count++;

			if (csize > max_size)
			{
				j = i;
				max_size = csize;
			}

		}

		if (contour_count > 0)
		{
			drawContours(mask, contours, j, Scalar(255, 255, 255), FILLED, 0);
			pt = mc[j];
		}
	}

	return contours;
}

int _tmain(int argc, _TCHAR* argv[])
{
	FILE *fp = NULL;

	//configure and start NIDAQ
	Daq ndq;
	ndq.configure();
	ndq.start();
	ndq.write();
	
	PGRcam cam;

	BusManager busMgr;
	unsigned int numCameras;
	PGRGuid guid;

	FlyCapture2::Error error;

	error = busMgr.GetNumOfCameras(&numCameras);
	printf("Number of cameras detected: %u\n", numCameras);

	if (numCameras < 1)
	{
		printf("Camera not connected\n");
		return -1;
	}

	error = busMgr.GetCameraFromIndex(0, &guid);

	error = cam.Connect(guid);

	error = cam.SetCameraParameters(imageLeft, imageTop, imageWidth, imageHeight);
	
	error = cam.SetTrigger();
	//error = cam.SetProperty(FRAME_RATE, 100);
	error = cam.SetProperty(SHUTTER, 3.002);
	error = cam.SetProperty(GAIN, 0.0);
	error = cam.cam.StartCapture(OnImageGrabbed);

	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return -1;
	}

	int thresh = 100;
	int erd = 0;
	int dlt = 0;

	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));

	#pragma omp parallel sections num_threads(3)
	{
		#pragma omp section
		{
			Image img;
			Mat frame, mask;

			while (true)
			{
				if (q.try_dequeue(img))
				{
					unsigned int rowBytes = (double)img.GetReceivedDataSize() / (double)img.GetRows();
					Mat tframe = Mat(img.GetRows(), img.GetCols(), CV_8UC1, img.GetData(), rowBytes);

					frame = tframe.clone();

					threshold(frame, mask, thresh, 255, THRESH_BINARY);

					erode(mask, mask, element, Point(-1, -1), erd);
					dilate(mask, mask, element, Point(-1, -1), dlt);

					float max_size;
					int j;
					int contour_count = 0;

					Point contour_center;

					vector<vector<Point>> contours = findLargestContour(mask, j, max_size, contour_count, contour_center);

					circle(frame, contour_center, 1, Scalar(0, 0, 0), FILLED, 1);

					putText(frame, to_string(fps), Point((imageWidth - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
					putText(frame, to_string(q.size_approx()), Point((imageWidth - 50), 20), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

					disp_frame.try_enqueue(frame.clone());
					disp_mask.try_enqueue(mask.clone());

					if (record)
					{
						fprintf(fp, "%.2f %.2f %d %d\n", ndq.thetax, ndq.thetay, contour_center.x, contour_center.y);
						
						if (ndq.thetax < MAX_ANGLE)
						{
							ndq.thetax += ANGLE_STEP_SIZE;
							ndq.write();
						}
						else
						{
							ndq.thetax = MIN_ANGLE;
							
							if (ndq.thetay < MAX_ANGLE)
							{
								ndq.thetay += ANGLE_STEP_SIZE;
								ndq.write();
							}
							else
							{
								ndq.reset();
								ndq.write();
								record = false;

								fclose(fp);
								fp = NULL;
							}
						}
					}

				}

				if (!stream)
					break;
			}
		}

		#pragma omp section
		{
			//namedWindow("parameters", WINDOW_AUTOSIZE);
			//createTrackbar("center x", "parameters", &center.x, imageWidth);
			//createTrackbar("center y", "parameters", &center.y, imageHeight);
			//createTrackbar("major axis", "parameters", &maj_axis, imageWidth / 2);
			//createTrackbar("minor axis", "parameters", &min_axis, imageHeight / 2);
			//createTrackbar("angle", "parameters", &angle, 180);
			
			namedWindow("control", WINDOW_AUTOSIZE);
			createTrackbar("thresh", "control", &thresh, 255);
			createTrackbar("erode", "control", &erd, 5);
			createTrackbar("dilate", "control", &dlt, 5);

			Mat tframe, tmask;

			while (true)
			{

				if (disp_frame.try_dequeue(tframe))
				{
					//ellipse(tframe, center, Size(maj_axis, min_axis), angle, 0, 360, Scalar(255, 255, 255));
					imshow("camera", tframe);
				}

				if (disp_mask.try_dequeue(tmask))
					imshow("mask", tmask);

				waitKey(1);

				if (!stream)
				{
					destroyWindow("controls");
					destroyWindow("camera");
					destroyWindow("mask");
					break;
				}

			}
		}

		#pragma omp section
		{
			//int mask_key_state = 0;
			int record_key_state = 0;

			while (true)
			{
				//if (GetAsyncKeyState(VK_F1))
				//{
				//	if (!mask_key_state)
				//	{
				//		Mat mask = Mat::zeros(Size(imageWidth, imageHeight), CV_8UC1);
				//		ellipse(mask, Point(imageWidth / 2, imageHeight / 2), Size(maj_axis, min_axis), angle, 0, 360, Scalar(255, 255, 255), FILLED);
				//		imwrite("mask.bmp", mask);
				//	}

				//	mask_key_state = 1;
				//}
				//else
				//	mask_key_state = 0;

				if (GetAsyncKeyState(VK_F2))
				{
					if (!record_key_state)
					{
						record = !record;

						if (record)
						{
							fp = new FILE;
							fopen_s(&fp, "map.txt", "w");

							if (fp == NULL)
							{
								printf("\nError creating file. Exiting.");
								break;
							}
						}
						else
						{
							fclose(fp);
							fp = NULL;
						}
					}
					record_key_state = 1;
				}
				else
					record_key_state = 0;

				if (GetAsyncKeyState(VK_ESCAPE))
				{
					stream = false;
					break;
				}
			}
		}
	}

	cam.Stop();

	return 0;
}


