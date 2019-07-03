// flybam.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "readerwriterqueue.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;
using namespace moodycamel;
using namespace concurrency;

bool stream = true;

bool flyview_track = false;
bool manual_track = false;

bool flyview_record = false;
bool arenaview_record = false;
bool flashPressed = false;

struct fvwritedata
{
	Mat img;
	int stamp;
	Point2f laser;
	Point2f head;
	Point2f galvo_angle;
	int lens_pos;
};

struct avwritedata
{
	Mat img;
	TimeStamp stamp;
	vector<Point2f> pts;
	vector<double> szs;
};

concurrent_queue<Mat> q;
concurrent_queue<int> sq;
concurrent_queue<Image> aq;

ReaderWriterQueue<Mat> arenaDispStream(1), arenaMaskStream(1), flyDispStream(1), flyMaskStream(1);

concurrent_queue<fvwritedata> fvwdata;
concurrent_queue<avwritedata> avwdata;

Mat frame;
int stamp;

int lens_pos = 0;
char serial_buffer[2] = { 0 };

static void AcqCallback(SapXferCallbackInfo *pInfo)
{
	SapView *pView = (SapView *)pInfo->GetContext();
	SapBuffer *pBuffer = pView->GetBuffer();

	pBuffer->GetCounterStamp(&stamp);

	int width = pBuffer->GetWidth();
	int height = pBuffer->GetHeight();
	int depth = pBuffer->GetPixelDepth();

	void *pData = NULL;
	pBuffer->GetAddress(&pData);

	Mat tframe(height, width, CV_8U, (void*)pData);
	frame = tframe.clone();

	sq.push(stamp);
	q.push(frame);
}

void OnImageGrabbed(Image* pImage, const void* pCallbackData)
{
	Image img;

	img.DeepCopy(pImage);
	
	aq.push(img);
	
	return;
}

int _tmain(int argc, _TCHAR* argv[])
{
	

	// initialize camera link gazelle camera
	SapAcquisition	*Acq	 = NULL;
	SapBuffer		*Buffers = NULL;
	SapView			*View	 = NULL;
	SapTransfer		*Xfer	 = NULL;
	
	UINT32   acqDeviceNumber;
	char*    acqServerName = new char[CORSERVER_MAX_STRLEN];
	char*    configFilename = new char[MAX_PATH];

	acqServerName = "Xcelera-CL_PX4_1";
	acqDeviceNumber = 0;
	
	//configure and start NIDAQ
	Daq ndq;
	ndq.configure();
	ndq.start();
	ndq.write();

	ndq.startTrigger();

	// init arduino for camera lens control
	Serial* SP = new Serial("COM4");    // adjust as needed

	if (SP->IsConnected())
	{
		printf("Connecting lens controller arduino [OK]\n");

		// Query for lens position
		ndq.lensCommand(7);

		printf("Reading initial lens position from Arduino...\n");
		Sleep(2000);

		if (SP->ReadData(serial_buffer, 2))
		{
			lens_pos = (int16)*serial_buffer;
			printf("Inital Lens Position: %d\n", lens_pos);
		}
		else
			printf("Read failed\n");

		// Shutdown Serial
		//printf("Closing serial connection...\n");
		SP->~Serial();
	}
	else
		printf("Failed connecting to arduino over serial\n");


	configFilename = "..\\ccf\\P_GZL-CL-20C5M_Gazelle_240x240.ccf";
	//configFilename = "..\\ccf\\P_GZL-CL-20C5M_Gazelle_256x256.ccf";
	
	printf("Initializing camera link fly view camera ");

	SapLocation loc(acqServerName, acqDeviceNumber);

	if (SapManager::GetResourceCount(acqServerName, SapManager::ResourceAcq) > 0)
	{
		Acq = new SapAcquisition(loc, configFilename);
		
		//Buffers = new SapBufferWithTrash(10, Acq);
		Buffers = new SapBuffer(10, Acq);

		View = new SapView(Buffers, SapHwndAutomatic);
		Xfer = new SapAcqToBuf(Acq, Buffers, AcqCallback, View);

		// Create acquisition object
		if (Acq && !*Acq && !Acq->Create())
			return -1;

	}

	// Create buffer object
	if (Buffers && !*Buffers && !Buffers->Create())
		return -1;

	// Create transfer object
	if (Xfer && !*Xfer && !Xfer->Create())
		return -1;

	// Start continous grab
	Xfer->Grab();

	printf("[OK]\n");
	
	int arena_image_width = 512, arena_image_height = 512;
	int arena_image_left = 384, arena_image_top = 256;

	int fly_image_width = 240, fly_image_height = 240;
	//int fly_image_width = 256, fly_image_height = 256;

	Point el_center(260, 220);
	int el_maj_axis = 237, el_min_axis = 133;
	int el_angle = 178;

	PGRcam arena_cam;
	BusManager busMgr;
	unsigned int numCameras;
	PGRGuid guid;
	FlyCapture2::Error error;

	fstream map("..\\calibration\\raster\\fmfrecord\\map.txt", ios_base::in);

	vector<Point2f> raster_pts;
	vector<Point2f> raster_angles;
	
	Point2f raster_pt;
	Point2f raster_ang;

	while (map >> raster_ang.x >> raster_ang.y >> raster_pt.x >> raster_pt.y)
	{
		if (raster_pt.x != 0 && raster_pt.y != 0)
		{
			raster_pts.push_back(raster_pt);
			raster_angles.push_back(raster_ang);
		}
	}

	FVFmfWriter fvfout;
	AVFmfWriter avfout;

	vector<Point2f> arena_pt(NFLIES);

	fvwritedata fvin;
	avwritedata avin;
	
	error = busMgr.GetNumOfCameras(&numCameras);
	printf("Number of point grey cameras detected: %u\n", numCameras);

	if (numCameras < 1)
	{
		printf("Insufficient number of cameras... exiting\n");
		return -1;
	}

	printf("Initializing arena view camera ");
	error = busMgr.GetCameraFromIndex(0, &guid);
	error = arena_cam.Connect(guid);
	error = arena_cam.SetCameraParameters(arena_image_left, arena_image_top, arena_image_width, arena_image_height);
	error = arena_cam.SetTrigger();
	
	//shutter setting for 0.1A power
	error = arena_cam.SetProperty(SHUTTER, 3.002);
	
	//shutter setting for 0.75A power
	//error = arena_cam.SetProperty(SHUTTER, 0.249);
	
	error = arena_cam.SetProperty(GAIN, 0.0);
	error = arena_cam.cam.StartCapture(OnImageGrabbed);

	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return -1;
	}
	printf("[OK]\n");

	//create arena mask
	Mat outer_mask = Mat::zeros(Size(arena_image_width, arena_image_height), CV_8UC1);
	ellipse(outer_mask, el_center, Size(el_maj_axis, el_min_axis), el_angle, 0, 360, Scalar(255, 255, 255), FILLED);

	Mat arena_img, arena_frame, arena_mask, arena_bg;
	Mat fly_img, fly_frame, fly_fg, fly_mask;

	Mat arena_mean = Mat::zeros(arena_image_width, arena_image_height, CV_32F);

	// thresholds for fly-view image at 0.1A power
	int arena_thresh = 65;
	int fly_thresh = 150;

	// thresholds for fly-view image at 0.75A power
	//int arena_thresh = 45;
	//int fly_thresh = 175;

	int fly_erode = 0;
	int fly_dilate = 3;

	int arena_erode = 1;
	int arena_dilate = 2;

	int focal_fly = 0;
	bool comp_bg = true;

	Mat fly_element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));
	Mat arena_element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));

	int fvrcount = 0;
	int avrcount = 0;
	int flashcount = 0;

	int lost = 0;

	vector<Point2f> bpt(NBEADS);
	float d_01, d_02, d_12;
	bool three_point_tracking = false;

	//Press [F1] to start/stop fly-view tracking, [F2] to start/stop recording, [ESC] to exit.
	#pragma omp parallel sections num_threads(7)
	{
		#pragma omp section
		{
			int fly_last = 0, fly_now = 0;
			float fly_fps;
			
			Mat tframe;

			while (true)
			{
				Point2f pt2d, wpt, galvo_mirror_angle;

				if (q.try_pop(tframe))
				{
					sq.try_pop(fly_now);

					int duration = fly_now - fly_last;

					if (duration != 0)
					{
						fly_fps = (1.0 / duration) * 1000 * 1000;

						fly_frame = tframe.clone();
						fly_img = tframe.clone();

						fly_last = fly_now;

						threshold(fly_frame, fly_fg, fly_thresh, 255, THRESH_BINARY);

						erode(fly_fg, fly_mask, fly_element, Point(-1, -1), fly_erode);
						dilate(fly_mask, fly_mask, fly_element, Point(-1, -1), fly_dilate);

						//double fm = varianceOfLaplacian(fly_frame);

						if (flyview_track)
						{
							float total_x = 0;
							float total_y = 0;
							int contour_count = 0;

							vector<vector<Point>> fly_contours;
							findContours(fly_mask, fly_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

							if (fly_contours.size() > 0)
							{
								// Get the moments and mass centers
								vector<Moments> fly_mu(fly_contours.size());
								vector<Point2f> fly_mc(fly_contours.size());

								vector<Point2f> bead_ctr_pts;
								vector<Point2f> bead_pt(NBEADS);

								for (int i = 0; i < fly_contours.size(); i++)
								{
									fly_mu[i] = moments(fly_contours[i], false);
									fly_mc[i] = Point2f(fly_mu[i].m10 / fly_mu[i].m00, fly_mu[i].m01 / fly_mu[i].m00);

									double csize = contourArea(fly_contours[i]);

									if (csize > MIN_MARKER_SIZE && csize < MAX_MARKER_SIZE)
									{
										drawContours(fly_mask, fly_contours, i, Scalar(255, 255, 255), FILLED, 1);
										bead_ctr_pts.push_back(fly_mc[i]);
									}
								}

								if (bead_ctr_pts.size() >= NBEADS)
								{
									for (int i = 0; i < NBEADS; i++)
									{
										int j = findClosestPoint(bpt[i], bead_ctr_pts);

										bead_pt[i] = bead_ctr_pts[j];

										putText(fly_frame, to_string(i), bead_pt[i], FONT_HERSHEY_COMPLEX, 0.4, Scalar(0, 0, 0));

										bead_ctr_pts.erase(bead_ctr_pts.begin() + j);
									}

									d_01 = dist(bead_pt[0], bead_pt[1]);
									d_02 = dist(bead_pt[0], bead_pt[2]);
									d_12 = dist(bead_pt[1], bead_pt[2]);

									three_point_tracking = true;

								}
								else if (bead_ctr_pts.size() < NBEADS)
								{
									vector<Point2f> last_bead_pt = bpt;
									vector<int> bead_pt_ind;

									for (int i = 0; i < NBEADS; i++)
										bead_pt_ind.push_back(i);

									for (int i = 0; i < bead_ctr_pts.size(); i++)
									{
										int j = findClosestPoint(bead_ctr_pts[i], last_bead_pt);

										bead_pt[bead_pt_ind[j]] = bead_ctr_pts[i];

										putText(fly_frame, to_string(bead_pt_ind[j]), bead_pt[bead_pt_ind[j]], FONT_HERSHEY_COMPLEX, 0.4, Scalar(0, 0, 0));

										last_bead_pt.erase(last_bead_pt.begin() + j);
										bead_pt_ind.erase(bead_pt_ind.begin() + j);
									}

									if (three_point_tracking && bead_ctr_pts.size() == (NBEADS - 1))
									{
										if (bead_pt[0].x == 0 && bead_pt[0].y == 0)
										{
											float theta = atan2(bead_pt[2].y - bead_pt[1].y, bead_pt[2].x - bead_pt[1].x);

											float x_pp = (pow(d_12, 2) + pow(d_01, 2) - pow(d_02, 2)) / (2 * d_12);

											float y_pp_p = sqrt((d_12 + d_01 + d_02) * (d_12 + d_01 - d_02) * (d_12 - d_01 + d_02) * (-d_12 + d_01 + d_02)) / (2 * d_12);
											float y_pp_n = -sqrt((d_12 + d_01 + d_02) * (d_12 + d_01 - d_02) * (d_12 - d_01 + d_02) * (-d_12 + d_01 + d_02)) / (2 * d_12);

											float x_p_p = cos(theta)*x_pp - sin(theta)*y_pp_p;
											float y_p_p = sin(theta)*x_pp + cos(theta)*y_pp_p;

											float x_p_n = cos(theta)*x_pp - sin(theta)*y_pp_n;
											float y_p_n = sin(theta)*x_pp + cos(theta)*y_pp_n;

											float x_p = x_p_p + bead_pt[1].x;
											float y_p = y_p_p + bead_pt[1].y;

											float x_n = x_p_n + bead_pt[1].x;
											float y_n = y_p_n + bead_pt[1].y;

											if (dist(bpt[0], Point2f(x_p, y_p)) < dist(bpt[0], Point2f(x_n, y_n)))
											{
												bead_pt[0] = Point2f(x_p, y_p);
												drawMarker(fly_frame, Point2f(x_p, y_p), Scalar(0, 0, 0), MARKER_TILTED_CROSS, 5, 1);
											}
											else
											{
												bead_pt[0] = Point2f(x_n, y_n);
												drawMarker(fly_frame, Point2f(x_n, y_n), Scalar(0, 0, 0), MARKER_TILTED_CROSS, 5, 1);
											}
										}


										if (bead_pt[1].x == 0 && bead_pt[1].y == 0)
										{
											float theta = atan2(bead_pt[2].y - bead_pt[0].y, bead_pt[2].x - bead_pt[0].x);

											float x_pp = (pow(d_02, 2) + pow(d_01, 2) - pow(d_12, 2)) / (2 * d_02);

											float y_pp_p = sqrt((d_02 + d_01 + d_12) * (d_02 + d_01 - d_12) * (d_02 - d_01 + d_12) * (-d_02 + d_01 + d_12)) / (2 * d_02);
											float y_pp_n = -sqrt((d_02 + d_01 + d_12) * (d_02 + d_01 - d_12) * (d_02 - d_01 + d_12) * (-d_02 + d_01 + d_12)) / (2 * d_02);

											float x_p_p = cos(theta)*x_pp - sin(theta)*y_pp_p;
											float y_p_p = sin(theta)*x_pp + cos(theta)*y_pp_p;

											float x_p_n = cos(theta)*x_pp - sin(theta)*y_pp_n;
											float y_p_n = sin(theta)*x_pp + cos(theta)*y_pp_n;

											float x_p = x_p_p + bead_pt[0].x;
											float y_p = y_p_p + bead_pt[0].y;

											float x_n = x_p_n + bead_pt[0].x;
											float y_n = y_p_n + bead_pt[0].y;

											if (dist(bpt[1], Point2f(x_p, y_p)) < dist(bpt[1], Point2f(x_n, y_n)))
											{
												bead_pt[1] = Point2f(x_p, y_p);
												drawMarker(fly_frame, Point2f(x_p, y_p), Scalar(0, 0, 0), MARKER_TILTED_CROSS, 5, 1);
											}
											else
											{
												bead_pt[1] = Point2f(x_n, y_n);
												drawMarker(fly_frame, Point2f(x_n, y_n), Scalar(0, 0, 0), MARKER_TILTED_CROSS, 5, 1);
											}
										}


										if (bead_pt[2].x == 0 && bead_pt[2].y == 0)
										{
											float theta = atan2(bead_pt[1].y - bead_pt[0].y, bead_pt[1].x - bead_pt[0].x);

											float x_pp = (pow(d_01, 2) + pow(d_02, 2) - pow(d_12, 2)) / (2 * d_01);

											float y_pp_p = sqrt((d_01 + d_02 + d_12) * (d_01 + d_02 - d_12) * (d_01 - d_02 + d_12) * (-d_01 + d_02 + d_12)) / (2 * d_01);
											float y_pp_n = -sqrt((d_01 + d_02 + d_12) * (d_01 + d_02 - d_12) * (d_01 - d_02 + d_12) * (-d_01 + d_02 + d_12)) / (2 * d_01);

											float x_p_p = cos(theta)*x_pp - sin(theta)*y_pp_p;
											float y_p_p = sin(theta)*x_pp + cos(theta)*y_pp_p;

											float x_p_n = cos(theta)*x_pp - sin(theta)*y_pp_n;
											float y_p_n = sin(theta)*x_pp + cos(theta)*y_pp_n;

											float x_p = x_p_p + bead_pt[0].x;
											float y_p = y_p_p + bead_pt[0].y;

											float x_n = x_p_n + bead_pt[0].x;
											float y_n = y_p_n + bead_pt[0].y;

											if (dist(bpt[2], Point2f(x_p, y_p)) < dist(bpt[2], Point2f(x_n, y_n)))
											{
												bead_pt[2] = Point2f(x_p, y_p);
												drawMarker(fly_frame, Point2f(x_p, y_p), Scalar(0, 0, 0), MARKER_TILTED_CROSS, 5, 1);
											}
											else
											{
												bead_pt[2] = Point2f(x_n, y_n);
												drawMarker(fly_frame, Point2f(x_n, y_n), Scalar(0, 0, 0), MARKER_TILTED_CROSS, 5, 1);
											}
										}

										three_point_tracking = true;
									}
									else
										three_point_tracking = false;
								}

								for (int i = 0; i < NBEADS; i++)
								{
									if (bead_pt[i].x != 0 && bead_pt[i].y != 0)
									{
										total_x += bead_pt[i].x;
										total_y += bead_pt[i].y;
										contour_count++;
									}
								}

								bpt = bead_pt;

							}

							if (contour_count > 0)
							{
								lost = 0;

								pt2d.x = total_x / contour_count;
								pt2d.y = total_y / contour_count;

								//drawContours(fly_frame, fly_contours, j, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
								//drawContours(fly_mask, fly_contours, j, Scalar(255, 255, 255), FILLED, 1);
								//drawContours(fly_frame, hull, j, Scalar::all(255), 1, 8, vector<Vec4i>(), 0, Point());

								drawMarker(fly_frame, pt2d, Scalar(255, 255, 255), MARKER_TILTED_CROSS, 5, 1);

								Point2f rotpt = rotateFlyCenter(pt2d, fly_image_width, fly_image_height);

								float diffx = rotpt.x - (fly_image_width / 2);
								float diffy = rotpt.y - (fly_image_height / 2);

								ndq.ConvertPixelToDeg(diffx*SCALEX, diffy*SCALEY);
								wpt = ndq.ConvertDegToPt();
								galvo_mirror_angle = ndq.GetGalvoAngles();
								ndq.write();

							}
							else
							{
								lost++;

								if (lost > NLOSTFRAMES)
								{
									flyview_track = false;
									manual_track = false;
									three_point_tracking = false;
									ndq.reset();
								}
							}
						}

						putText(fly_frame, to_string(lens_pos), Point(0, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

						putText(fly_frame, to_string((int)fly_fps), Point((fly_image_width - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
						putText(fly_frame, to_string(q.unsafe_size()), Point((fly_image_width - 50), 20), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
						
						if (manual_track)
							drawMarker(fly_frame, Point2f(fly_image_width/2, fly_image_height/2), Scalar(255, 255, 255), MARKER_CROSS, 20, 1);

						if (flyview_record)
							putText(fly_frame, to_string(fvrcount), Point(0, 20), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

						flyDispStream.try_enqueue(fly_frame.clone());
						flyMaskStream.try_enqueue(fly_mask.clone());

						if (flyview_record)
						{
							fvin.img = fly_img;
							fvin.stamp = fly_now;
							fvin.head = pt2d;
							fvin.laser = wpt;
							fvin.galvo_angle = galvo_mirror_angle;
							fvin.lens_pos = lens_pos;

							fvwdata.push(fvin);
							fvrcount++;
						}

						if (flashPressed)
							flashcount++;
					}
				}

				if (!stream)
					break;
			}
		}

		#pragma omp section
		{
			Image img;
			int arena_last = 0, arena_fps = 0;
			
			while (true)
			{
				if (aq.try_pop(img))
				{
					vector<Point2f> fly_pt(NFLIES);
					vector<double> fly_sz(NFLIES);
					
					arena_fps = ConvertTimeToFPS(img.GetTimeStamp().cycleCount, arena_last);
					arena_last = img.GetTimeStamp().cycleCount;

					unsigned int rowBytes = (double)img.GetReceivedDataSize() / (double)img.GetRows();
					Mat tframe = Mat(img.GetRows(), img.GetCols(), CV_8UC1, img.GetData(), rowBytes);

					arena_img = tframe.clone();
					arena_frame = tframe.clone();
					
					if (comp_bg)
					{
						arena_bg = tframe.clone();
						comp_bg = false;
					}

					subtract(arena_bg, arena_frame, arena_mask);
					threshold(arena_mask, arena_mask, arena_thresh, 255, THRESH_BINARY);

					outer_mask = Mat::zeros(Size(arena_image_width, arena_image_height), CV_8UC1);
					ellipse(outer_mask, el_center, Size(el_maj_axis, el_min_axis), el_angle, 0, 360, Scalar(255, 255, 255), FILLED);
					arena_mask &= outer_mask;

					erode(arena_mask, arena_mask, arena_element, Point(-1, -1), arena_erode);
					dilate(arena_mask, arena_mask, arena_element, Point(-1, -1), arena_dilate);

					vector<vector<Point>> arena_contours;

					findContours(arena_mask, arena_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

					if (arena_contours.size() > 0)
					{
						// Get the moments and mass centers
						vector<Moments> arena_mu(arena_contours.size());
						vector<Point2f> arena_mc(arena_contours.size());
						vector<double> arena_sz(arena_contours.size());

						vector<Point2f> arena_ctr_pts;
						vector<double> arena_ctr_sz;

						for (int i = 0; i < arena_contours.size(); i++)
						{
							//drawContours(arena_mask, arena_contours, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
							arena_mu[i] = moments(arena_contours[i], false);
							arena_mc[i] = Point2f(arena_mu[i].m10 / arena_mu[i].m00, arena_mu[i].m01 / arena_mu[i].m00);

							arena_sz[i] = contourArea(arena_contours[i]);

							if (arena_sz[i] > 10)
							{
								//drawContours(arena_frame, arena_contours, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
								drawContours(arena_mask, arena_contours, i, Scalar(255, 255, 255), FILLED, 1);
								arena_ctr_pts.push_back(arena_mc[i]);
								arena_ctr_sz.push_back(arena_sz[i]);
							}
						}

						if (arena_ctr_pts.size() >= NFLIES)
						{
							for (int i = 0; i < NFLIES; i++)
							{
								int j = findClosestPoint(arena_pt[i], arena_ctr_pts);

								arena_pt[i] = arena_ctr_pts[j];
								
								fly_pt[i] = arena_pt[i];
								fly_sz[i] = arena_ctr_sz[j];

								putText(arena_frame, to_string(i), arena_pt[i], FONT_HERSHEY_COMPLEX, 0.2, Scalar(255, 255, 255));

								arena_ctr_pts.erase(arena_ctr_pts.begin() + j);
								arena_ctr_sz.erase(arena_ctr_sz.begin() + j);
							}
						}
						else if (arena_ctr_pts.size() < NFLIES)
						{
							vector<Point2f> last_arena_pt = arena_pt;
							vector<int> arena_pt_ind;

							for (int i = 0; i < NFLIES; i++)
								arena_pt_ind.push_back(i);

							for (int i = 0; i < arena_ctr_pts.size(); i++)
							{
								int j = findClosestPoint(arena_ctr_pts[i], last_arena_pt);

								arena_pt[arena_pt_ind[j]] = arena_ctr_pts[i];
								
								fly_pt[arena_pt_ind[j]] = arena_pt[arena_pt_ind[j]];
								fly_sz[arena_pt_ind[j]] = arena_ctr_sz[i];


								putText(arena_frame, to_string(arena_pt_ind[j]), arena_pt[arena_pt_ind[j]], FONT_HERSHEY_COMPLEX, 0.2, Scalar(255, 255, 255));

								last_arena_pt.erase(last_arena_pt.begin() + j);
								arena_pt_ind.erase(arena_pt_ind.begin() + j);
							}
						}

						if (!flyview_track && !manual_track)
						{
							int j = findClosestPoint(arena_pt[focal_fly], raster_pts);
							ndq.SetGalvoAngles(raster_angles[j]);
							ndq.write();
						}
					}

					putText(arena_frame, to_string(arena_fps), Point((arena_image_width - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
					putText(arena_frame, to_string(aq.unsafe_size()), Point((arena_image_width - 50), 20), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

					if (arenaview_record)
						putText(arena_frame, to_string(avrcount), Point(0, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

					arenaDispStream.try_enqueue(arena_frame.clone());
					arenaMaskStream.try_enqueue(arena_mask.clone());

					if (arenaview_record)
					{
						avin.img = arena_img;
						avin.stamp = img.GetTimeStamp();
						avin.pts = fly_pt;
						avin.szs = fly_sz;

						avwdata.push(avin);

						avrcount++;
					}
				}

				if (!stream)
					break;

			}

		}

		#pragma omp section
		{
			fvwritedata out;

			while (true)
			{
				if (fvwdata.try_pop(out))
				{
					if (!fvfout.IsOpen())
					{
						fvfout.Open();
						fvfout.InitHeader(fly_image_width, fly_image_height);
						fvfout.WriteHeader();
						printf("Recording ");
					}

					fvfout.WriteFrame(out.img);
					fvfout.WriteLog(out.stamp);
					fvfout.WriteTraj(out.laser, out.head, out.galvo_angle, out.lens_pos);
					
					fvfout.nframes++;
				}
				else
				{
					if (!flyview_record && fvfout.IsOpen())
					{
						fvfout.Close();
						printf("[OK]\n");
					}

					if (!stream)
						break;
				}
			}
		}


		#pragma omp section
		{
			avwritedata out;

			while (true)
			{
				if (avwdata.try_pop(out))
				{
					if (!avfout.IsOpen())
					{
						avfout.Open();
						avfout.InitHeader(arena_image_width, arena_image_height);
						avfout.WriteHeader();
						avfout.WriteBG(arena_bg);
					}

					avfout.WriteFrame(out.img);
					avfout.WriteLog(out.stamp);
					avfout.WriteTraj(out.pts, out.szs);
					avfout.nframes++;
				}
				else
				{
					if (!arenaview_record && avfout.IsOpen())
						avfout.Close();

					if (!stream)
						break;
				}
			}
		}

		#pragma omp section
		{
			namedWindow("controls", WINDOW_AUTOSIZE);
			createTrackbar("arena thresh", "controls", &arena_thresh, 255);
			createTrackbar("fly thresh", "controls", &fly_thresh, 255);
			createTrackbar("fly erode", "controls", &fly_erode, 5);
			createTrackbar("fly dilate", "controls", &fly_dilate, 5);
			createTrackbar("arena erode", "controls", &arena_erode, 5);
			createTrackbar("arena dilate", "controls", &arena_dilate, 5);

			namedWindow("parameters", WINDOW_AUTOSIZE);
			createTrackbar("center x", "parameters", &el_center.x, arena_image_width);
			createTrackbar("center y", "parameters", &el_center.y, arena_image_height);
			createTrackbar("major axis", "parameters", &el_maj_axis, arena_image_width / 2);
			createTrackbar("minor axis", "parameters", &el_min_axis, arena_image_height / 2);
			createTrackbar("angle", "parameters", &el_angle, 180);
			
			Mat tframe, tmask;

			while (true)
			{
				if (arenaDispStream.try_dequeue(tframe))
				{
					ellipse(tframe, el_center, Size(el_maj_axis, el_min_axis), el_angle, 0, 360, Scalar(255, 255, 255));
					imshow("arena image", tframe);
				}

				if (arenaMaskStream.try_dequeue(tmask))
					imshow("arena mask", tmask);
				
				waitKey(1);

				if (!stream)
				{
					destroyWindow("controls");
					destroyWindow("parameters");
					destroyWindow("arena image");
					destroyWindow("arena mask");
					break;
				}
			}
		}


		#pragma omp section
		{
			Mat tframe, tmask;
			while (true)
			{
				if (flyDispStream.try_dequeue(tframe))
					imshow("fly image", tframe);

				if (flyMaskStream.try_dequeue(tmask))
					imshow("fly mask", tmask);

				waitKey(1);

				if (!stream)
				{
					destroyWindow("fly image");
					destroyWindow("fly mask");
					break;
				}
			}
		}

		#pragma omp section
		{
			int record_key_state = 0;
			int track_key_state = 0;
			int flash_key_state = 0;
			int bg_key_state = 0;
			int fly_key_state = 0;

			int inc_foc_state = 0;
			int dec_foc_state = 0;

			int max_foc_state = 0;
			int min_foc_state = 0;
			
			int reset_galvo_state = 0;

			while (true)
			{
				if (GetAsyncKeyState(VK_NUMPAD1))
				{
					if (!inc_foc_state)
					{
						if ((lens_pos - Z_STEP) >= 0)
						{
							lens_pos -= Z_STEP;
							ndq.lensCommand(1); // 1: Move step (coarse) toward max lens position -> move focal plane toward camera
						}
					}

					inc_foc_state = 1;
				}
				else
					inc_foc_state = 0;

				if (GetAsyncKeyState(VK_NUMPAD2))
				{
					if (!dec_foc_state) 
					{
						ndq.lensCommand(2); // 2: Move step (coarse) toward min lens position -> move focal plane toward backlight
						lens_pos += Z_STEP;
					}
					
					dec_foc_state = 1;
				}
				else
					dec_foc_state = 0;

				// Max lens position -> min focal toward camera (-z up)
				if (GetAsyncKeyState(VK_NUMPAD4))
				{
					if (!max_foc_state)
					{
						lens_pos = 0;
						ndq.lensCommand(6);
					}
					
					max_foc_state = 1;
				}
				else
					max_foc_state = 0;

				// Min lens position -> max focal toward backlight (+z down)
				if (GetAsyncKeyState(VK_NUMPAD5))
				{
					if (!min_foc_state)
					{
						lens_pos = 9999;
						ndq.lensCommand(5);
					}

					min_foc_state = 1;
				}
				else
					min_foc_state = 0;

				
				if (GetAsyncKeyState(VK_TAB))
				{
					if (!fly_key_state)
					{
						int tfocal = focal_fly + 1;

						if (tfocal == NFLIES)
							focal_fly = 0;
						else
							focal_fly = tfocal;

						manual_track = false;
						flyview_track = false;
					}

					fly_key_state = 1;
				}
				else
					fly_key_state = 0;

				SHORT leftKeyState = GetAsyncKeyState(VK_LEFT);

				if ((1 << 15) & leftKeyState)
				{
					flyview_track = false;
					manual_track = true;
					ndq.MoveLeft();
					ndq.write();
				}

				SHORT rightKeyState = GetAsyncKeyState(VK_RIGHT);

				if ((1 << 15) & rightKeyState)
				{
					flyview_track = false;
					manual_track = true;
					ndq.MoveRight();
					ndq.write();
				}

				SHORT upKeyState = GetAsyncKeyState(VK_UP);

				if ((1 << 15) & upKeyState)
				{
					flyview_track = false;
					manual_track = true;
					ndq.MoveUp();
					ndq.write();
				}

				SHORT downKeyState = GetAsyncKeyState(VK_DOWN);

				if ((1 << 15) & downKeyState)
				{
					flyview_track = false;
					manual_track = true;
					ndq.MoveDown();
					ndq.write();
				}

				if (GetAsyncKeyState(VK_F1))
				{
					if (!track_key_state)
					{
						flyview_track = !flyview_track;

						manual_track = false;
					}

					track_key_state = 1;
				}
				else
					track_key_state = 0;

				if (GetAsyncKeyState(VK_F2))
				{
					if (!record_key_state)
					{
						flyview_record = !flyview_record;
						arenaview_record = !arenaview_record;
						
						fvrcount = 0;
						avrcount = 0;
					}

					record_key_state = 1;
				}
				else
					record_key_state = 0;

				if (GetAsyncKeyState(VK_F3))
				{
					if (!bg_key_state)
						comp_bg = true;

					bg_key_state = 1;
				}
				else
					bg_key_state = 0;

				if (GetAsyncKeyState(VK_ESCAPE))
				{
					stream = false;
					break;
				}

				if (flyview_record)
				{
					if (fvrcount == MAXFVRECFRAMES)
					{
						fvrcount = 0;
						flyview_record = false;
					}
				}

				if (arenaview_record)
				{
					if (avrcount == MAXAVRECFRAMES)
					{
						avrcount = 0;
						arenaview_record = false;
					}
				}

				if (flashPressed)
				{
					if (flashcount == FLASHFRAMES)
					{
						flashPressed = false;
						flashcount = 0;

						ndq.flashLow();
					}
				}

				if (GetAsyncKeyState(VK_HOME))
				{
					if (!reset_galvo_state)
					{
						flyview_track = false;
						manual_track = true;
						ndq.reset();
						ndq.write();
					}

					reset_galvo_state = 1;
				}
				else
					reset_galvo_state = 0;

				if (GetAsyncKeyState(VK_F4))
				{
					if (!flash_key_state)
					{
						ndq.flashHigh();
						flashPressed = true;
					}

					flash_key_state = 1;
				}
				else
					flash_key_state = 0;
			}
		}
	}

	if (fvfout.IsOpen())
	{
		fvfout.Close();
		printf("[OK]\n");
	}

	if (avfout.IsOpen())
		avfout.Close();

	arena_cam.Stop();
	
	// Stop grab
	Xfer->Freeze();
	if (!Xfer->Wait(5000))
		printf("Grab could not stop properly.\n");

	// Destroy transfer object
	if (Xfer && *Xfer && !Xfer->Destroy()) return FALSE;

	// Destroy buffer object
	if (Buffers && *Buffers && !Buffers->Destroy()) return FALSE;

	// Destroy view object
	if (View && *View && !View->Destroy()) return FALSE;

	// Destroy acquisition object
	if (Acq && *Acq && !Acq->Destroy()) return FALSE;

	// Delete all objects
	if (Xfer)		delete Xfer;
	if (Buffers)	delete Buffers;
	if (View)		delete View;
	if (Acq)		delete Acq;
	
	printf("\n\nCentering galvo ");
	
	ndq.reset();
	ndq.write();
	ndq.stopTrigger();

	printf("[OK]\n");

	return 0;
}
