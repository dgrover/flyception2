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

//bool odor_present = false;

struct {
	bool operator() (cv::Vec4i pt1, cv::Vec4i pt2) { return (pt1[3] > pt2[3]); }
} mycomp_dsize;

struct fvwritedata
{
	Mat img;
	int stamp;
	Point2f laser;
	Point2f head;
	//Point2f edge;
	Point2f galvo_angle;
	//int odor;
	//float body_angle;
};

struct avwritedata
{
	Mat img;
	TimeStamp stamp;
	vector<Point2f> pts;
};

concurrent_queue<Mat> q;
concurrent_queue<int> sq;
concurrent_queue<Image> aq;

ReaderWriterQueue<Mat> arenaDispStream(1), arenaMaskStream(1), flyDispStream(1), flyMaskStream(1);

concurrent_queue<fvwritedata> fvwdata;
concurrent_queue<avwritedata> avwdata;

Mat frame;
int stamp;

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

//vector<vector<Point>> findLargestContour(Mat mask, int &j, float &max_size, int &contour_count, Point2f &pt)
//{
//	vector<vector<Point>> fly_contours;
//	findContours(mask, fly_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//
//	int nrow = mask.rows;
//	int ncol = mask.cols;
//
//	mask = Mat::zeros(Size(ncol, nrow), CV_8UC1);
//	contour_count = 0;
//
//	if (fly_contours.size() > 0)
//	{
//		max_size = 0;
//
//		// Get the moments and mass centers
//		vector<Moments> fly_mu(fly_contours.size());
//		vector<Point2f> fly_mc(fly_contours.size());
//
//		for (int i = 0; i < fly_contours.size(); i++)
//		{
//			//drawContours(mask, fly_contours, i, Scalar(255, 255, 255), FILLED, 1);
//			fly_mu[i] = moments(fly_contours[i], false);
//			fly_mc[i] = Point2f(fly_mu[i].m10 / fly_mu[i].m00, fly_mu[i].m01 / fly_mu[i].m00);
//
//			double csize = contourArea(fly_contours[i]);
//
//			if (csize > 5)
//				contour_count++;
//
//			if (csize > max_size)
//			{
//				j = i;
//				max_size = csize;
//			}
//
//		}
//
//		drawContours(mask, fly_contours, j, Scalar(255, 255, 255), FILLED, 0);
//
//		pt = fly_mc[j];
//	}
//
//	return fly_contours;
//}


int _tmain(int argc, _TCHAR* argv[])
{
	SapAcquisition		*Acq = NULL;
	SapBuffer			*Buffers = NULL;
	SapView				*View = NULL;
	SapTransfer			*Xfer = NULL;
	
	UINT32   acqDeviceNumber;
	char*    acqServerName = new char[CORSERVER_MAX_STRLEN];
	char*    configFilename = new char[MAX_PATH];

	acqServerName = "Xcelera-CL_PX4_1";
	acqDeviceNumber = 0;
	//configFilename = "C:\\Program Files\\Teledyne DALSA\\Sapera\\CamFiles\\User\\P_GZL-CL-20C5M_Gazelle_240x240.ccf";
	configFilename = "..\\ccf\\P_GZL-CL-20C5M_Gazelle_240x240.ccf";
	
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

	Point el_center(261, 219);
	int el_maj_axis = 236, el_min_axis = 138;
	int el_angle = 178;

	//int edge_min = 1;
	//int edge_max = fly_image_width - 2;

	PGRcam arena_cam;
	BusManager busMgr;
	unsigned int numCameras;
	PGRGuid guid;
	FlyCapture2::Error error;

	//string filename = "..\\calibration\\arena\\camera_projection_data.xml";

	//Mat cameraMatrix, distCoeffs;
	//Mat rvec(1, 3, cv::DataType<double>::type);
	//Mat tvec(1, 3, cv::DataType<double>::type);
	//Mat rotationMatrix(3, 3, cv::DataType<double>::type);

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

			//printf("%f %f %f %f\n", raster_pt.x, raster_pt.y, raster_ang.x, raster_ang.y);
		}
	}





	FVFmfWriter fvfout;
	AVFmfWriter avfout;

	//vector<Tracker> tkf(NFLIES);
	vector<Point2f> pt(NFLIES);
	vector<Point2f> arena_pt(NFLIES);
	//vector<vector<Point2f>> arena_pt_vec(NFLIES);

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
	//arena_cam.GetImageSize(arena_image_width, arena_image_height);

	error = arena_cam.SetTrigger();
	error = arena_cam.SetProperty(SHUTTER, 3.002);
	error = arena_cam.SetProperty(GAIN, 0.0);

	//error = arena_cam.Start();
	error = arena_cam.cam.StartCapture(OnImageGrabbed);

	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return -1;
	}
	printf("[OK]\n");

	//FileStorage fs(filename, FileStorage::READ);
	//fs["camera_matrix"] >> cameraMatrix;
	//fs["distortion_coefficients"] >> distCoeffs;
	//fs["rvec"] >> rvec;
	//fs["tvec"] >> tvec;
	//fs["rotation_matrix"] >> rotationMatrix;
	//fs.release();

	//calculating galvo center position in pixel coordinates
	//Point3f galvo_center_3d(0, 0, (BASE_HEIGHT - sqrt((GALVO_Y_HEIGHT * GALVO_Y_HEIGHT) - (ARENA_RADIUS * ARENA_RADIUS))));
	//Point2f galvo_center_2d = project3d2d(Point2f(0, 0), cameraMatrix, distCoeffs, rvec, tvec);

	//initializing galvo to arena center in pixel coordinates
	//for (int i = 0; i < NFLIES; i++)
	//{
	//	tkf[i].Init(Point2f(0,0));
		//tkf[i].Init(galvo_center_2d);
	//}

	//Serial* SP = new Serial("COM6");    // adjust as needed

	//if (SP->IsConnected())
	//	printf("Connecting arduino [OK]\n");

	//Serial* SPA = new Serial("COM7");    // adjust as needed

	//if (SPA->IsConnected())
	//	printf("Connecting odor delivery arduino [OK]\n");

	//SPA->WriteData("1", 1);

	//configure and start NIDAQ
	Daq ndq;
	ndq.configure();
	ndq.start();

	//create arena mask
	//Mat outer_mask = Mat::zeros(Size(arena_image_width, arena_image_height), CV_8UC1);
	//RotatedRect arenaMask = createArenaMask(ARENA_X_RADIUS, ARENA_Y_RADIUS, cameraMatrix, distCoeffs, rvec, tvec);
	//RotatedRect arenaMask = createArenaMask(ARENA_X_RADIUS + 2.0, ARENA_Y_RADIUS + 2.0, cameraMatrix, distCoeffs, rvec, tvec);
	//ellipse(outer_mask, arenaMask, Scalar(255, 255, 255), FILLED);

	Mat outer_mask = Mat::zeros(Size(arena_image_width, arena_image_height), CV_8UC1);
	ellipse(outer_mask, el_center, Size(el_maj_axis, el_min_axis), el_angle, 0, 360, Scalar(255, 255, 255), FILLED);

	Mat arena_img, arena_frame, arena_mask;
	Mat fly_img, fly_frame, fly_fg, fly_mask;

	int arena_thresh = 95;
	int fly_thresh = 50;

	int fly_erode = 0;
	int fly_dilate = 2;

	int arena_erode = 1;
	int arena_dilate = 2;

	int focal_fly = 0;

	//Mat fly_element = getStructuringElement(MORPH_RECT, Size(9, 9), Point(4, 4));
	Mat fly_element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));
	Mat arena_element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));

	int fvrcount = 0;
	int avrcount = 0;

	int lost = 0;
	
	//Press [F1] to start/stop tracking, [F2] to start/stop recording, [ESC] to exit.
	#pragma omp parallel sections num_threads(7)
	{
		#pragma omp section
		{
			int fly_last = 0, fly_now = 0;
			float fly_fps;
			
			Mat tframe;

			while (true)
			{
				//Point2f pt2d, wpt, edgept, galvo_mirror_angle;
				Point2f pt2d, wpt, galvo_mirror_angle;
				//float fly_body_angle = 0.0;

				//for (int i = 0; i < NFLIES; i++)
				//	pt[i] = tkf[i].Predict();

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
						
						if (flyview_track)
						{
							//int contour_count = 0;
							//float max_size;
							//int j;
							
							//Point2f fly_contour_center;
							//vector<vector<Point>> fly_contours = findLargestContour(fly_mask, j, max_size, contour_count, fly_contour_center);

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

								for (int i = 0; i < fly_contours.size(); i++)
								{
									//drawContours(fly_mask, fly_contours, i, Scalar(255, 255, 255), FILLED, 1);
									fly_mu[i] = moments(fly_contours[i], false);
									fly_mc[i] = Point2f(fly_mu[i].m10 / fly_mu[i].m00, fly_mu[i].m01 / fly_mu[i].m00);

									double csize = contourArea(fly_contours[i]);

									if (csize > MIN_MARKER_SIZE && csize < MAX_MARKER_SIZE)
									{
										drawContours(fly_mask, fly_contours, i, Scalar(255, 255, 255), FILLED, 1);

										total_x += fly_mc[i].x;
										total_y += fly_mc[i].y;
										contour_count++;
									}
								}
							}

							if (contour_count > 0)
							{
								lost = 0;

								pt2d.x = total_x / contour_count;
								pt2d.y = total_y / contour_count;

								//drawContours(fly_frame, fly_contours, j, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
								
								//drawContours(fly_mask, fly_contours, j, Scalar(255, 255, 255), FILLED, 1);
								//drawContours(fly_frame, hull, j, Scalar::all(255), 1, 8, vector<Vec4i>(), 0, Point());

								circle(fly_frame, pt2d, 1, Scalar(0, 0, 0), FILLED, 1);
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
								//if (contour_count == 0)
								//{
								lost++;

								if (lost > NLOSTFRAMES)
								{
									flyview_track = false;
										
									// Enable the next two lines to stop recording when flyview tracking is lost
									//flyview_record = false;
									//arenaview_record = false;
										
									manual_track = false;
									ndq.reset();

									//pt2d = Point2f(0, 0);
									//wpt = Point2f(0, 0);
									//galvo_mirror_angle = Point2f(0, 0);
									//fly_body_angle = 0.0;

									//for (int i = 0; i < NFLIES; i++)
									//	tkf[i].Init();
								}
								//}
							}
						}
						//else
						//{
						//	last_contour_count = 0;
						//	last_contour_size = 0;
						//}

						putText(fly_frame, to_string((int)fly_fps), Point((fly_image_width - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
						
						//putText(fly_frame, to_string(q.size_approx()), Point((fly_image_width - 50), 20), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
						putText(fly_frame, to_string(q.unsafe_size()), Point((fly_image_width - 50), 20), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));


						if (flyview_record)
							putText(fly_frame, to_string(fvrcount), Point(0, 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));

						flyDispStream.try_enqueue(fly_frame.clone());
						flyMaskStream.try_enqueue(fly_mask.clone());

						if (flyview_record)
						{
							//if (odor_present)
							//{
							//	fvin.odor = 1;
							//	odor_present = false;
							//}
							//else
							//	fvin.odor = 0;

							fvin.img = fly_img;
							fvin.stamp = fly_now;
							fvin.head = pt2d;
							//fvin.edge = edgept;
							fvin.laser = wpt;
							fvin.galvo_angle = galvo_mirror_angle;
							//fvin.body_angle = fly_body_angle;

							//wdata.enqueue(in);
							fvwdata.push(fvin);

							//laser_pt.enqueue(wpt);
							//fly_pt.enqueue(pt2d);
							//body_angle.enqueue(fly_body_angle);

							//flyTimeStamps.enqueue(fly_now);
							//flyImageStream.enqueue(fly_img);

							fvrcount++;
						}
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
			
			//int arena_last_radius = arena_radius;

			while (true)
			{
				//for (int i = 0; i < NFLIES; i++)
				//{
				//	arena_pt[i] = tkf[i].Predict();
				//	pt[i] = backProject(arena_pt[i], cameraMatrix, rotationMatrix, tvec, BASE_HEIGHT);
				//}

				//if (aq.try_dequeue(img))
				if (aq.try_pop(img))
				{
					arena_fps = ConvertTimeToFPS(img.GetTimeStamp().cycleCount, arena_last);
					arena_last = img.GetTimeStamp().cycleCount;

					unsigned int rowBytes = (double)img.GetReceivedDataSize() / (double)img.GetRows();
					Mat tframe = Mat(img.GetRows(), img.GetCols(), CV_8UC1, img.GetData(), rowBytes);

					arena_img = tframe.clone();
					arena_frame = tframe.clone();
					
					//Mat arena_tframe = arena_cam.convertImagetoMat(arena_img);
					//undistort(arena_tframe, arena_frame, cameraMatrix, distCoeffs);

					threshold(arena_frame, arena_mask, arena_thresh, 255, THRESH_BINARY_INV);
					
					//if (arena_last_radius != arena_radius)
					//{
					//	//create arena mask
					//	outer_mask = Mat::zeros(Size(arena_image_width, arena_image_height), CV_8UC1);
					//	arenaMask = createArenaMask(arena_radius, cameraMatrix, distCoeffs, rvec, tvec);
					//	ellipse(outer_mask, arenaMask, Scalar(255, 255, 255), FILLED);

					//	arena_last_radius = arena_radius;
					//}
					//else
					
					arena_mask &= outer_mask;

					//morphologyEx(arena_mask, arena_mask, MORPH_OPEN, arena_element);
					erode(arena_mask, arena_mask, arena_element, Point(-1, -1), arena_erode);
					dilate(arena_mask, arena_mask, arena_element, Point(-1, -1), arena_dilate);

					//cv::Mat dist;
					//cv::distanceTransform(arena_mask, dist, CV_DIST_L2, 3);
					//cv::normalize(dist, dist, 0, 1., cv::NORM_MINMAX);
					//cv::threshold(dist, dist, .5, 1., CV_THRESH_BINARY);
					//dist.convertTo(arena_mask, CV_8U);

					vector<vector<Point>> arena_contours;

					findContours(arena_mask, arena_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

					if (arena_contours.size() > 0)
					{
						// Get the moments and mass centers
						vector<Moments> arena_mu(arena_contours.size());
						vector<Point2f> arena_mc(arena_contours.size());

						vector<Point2f> arena_ctr_pts;

						for (int i = 0; i < arena_contours.size(); i++)
						{
							//drawContours(arena_mask, arena_contours, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
							arena_mu[i] = moments(arena_contours[i], false);
							arena_mc[i] = Point2f(arena_mu[i].m10 / arena_mu[i].m00, arena_mu[i].m01 / arena_mu[i].m00);

							double csize = contourArea(arena_contours[i]);

							if (csize > 5 && csize < 500)
							{
								//drawContours(arena_frame, arena_contours, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
								drawContours(arena_mask, arena_contours, i, Scalar(255, 255, 255), FILLED, 1);
								arena_ctr_pts.push_back(arena_mc[i]);
							}
						}

						if (arena_ctr_pts.size() >= NFLIES)
						{
							for (int i = 0; i < NFLIES; i++)
							{
								int j = findClosestPoint(arena_pt[i], arena_ctr_pts);

								//Point2f est = tkf[i].Correct(arena_ctr_pts[j]);
								//pt[i] = backProject(est, cameraMatrix, rotationMatrix, tvec, BASE_HEIGHT);
								//arena_pt_vec[i].push_back(est);

								//putText(arena_frame, to_string(i), est, FONT_HERSHEY_COMPLEX, 0.2, Scalar(255, 255, 255));

								arena_pt[i] = arena_ctr_pts[j];
								pt[i] = arena_pt[i];

								// HERE convert arena pixel coordinate to galvo angle 
								//pt[i] = backProject(arena_pt[i], cameraMatrix, rotationMatrix, tvec, BASE_HEIGHT);
								
								//arena_pt_vec[i].push_back(arena_pt[i]);

								putText(arena_frame, to_string(i), arena_pt[i], FONT_HERSHEY_COMPLEX, 0.2, Scalar(255, 255, 255));

								//for (int k = 0; k < arena_pt_vec[i].size() - 1; k++)
								//	line(arena_frame, arena_pt_vec[i][k], arena_pt_vec[i][k + 1], Scalar(255, 255, 0), 1);

								//if (arena_pt_vec[i].size() > TAIL_LENGTH)
								//	arena_pt_vec[i].erase(arena_pt_vec[i].begin());

								arena_ctr_pts.erase(arena_ctr_pts.begin() + j);
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

								//Point2f est = tkf[arena_pt_ind[j]].Correct(arena_ctr_pts[i]);
								//pt[arena_pt_ind[j]] = backProject(est, cameraMatrix, rotationMatrix, tvec, BASE_HEIGHT);
								//arena_pt_vec[j].push_back(est);

								//putText(arena_frame, to_string(arena_pt_ind[j]), est, FONT_HERSHEY_COMPLEX, 0.2, Scalar(255, 255, 255));

								arena_pt[arena_pt_ind[j]] = arena_ctr_pts[i];
								pt[arena_pt_ind[j]] = arena_pt[arena_pt_ind[j]];
								
								// HERE convert arena pixel coordinate to galvo angle 
								//pt[arena_pt_ind[j]] = backProject(arena_pt[arena_pt_ind[j]], cameraMatrix, rotationMatrix, tvec, BASE_HEIGHT);

								putText(arena_frame, to_string(arena_pt_ind[j]), arena_pt[arena_pt_ind[j]], FONT_HERSHEY_COMPLEX, 0.2, Scalar(255, 255, 255));

								//arena_pt_vec[arena_pt_ind[j]].push_back(arena_pt[arena_pt_ind[j]]);

								//for (int k = 0; k < arena_pt_vec[arena_pt_ind[j]].size() - 1; k++)
								//	line(arena_frame, arena_pt_vec[arena_pt_ind[j]][k], arena_pt_vec[arena_pt_ind[j]][k + 1], Scalar(255, 255, 0), 1);

								//if (arena_pt_vec[arena_pt_ind[j]].size() > TAIL_LENGTH)
								//	arena_pt_vec[arena_pt_ind[j]].erase(arena_pt_vec[arena_pt_ind[j]].begin());

								last_arena_pt.erase(last_arena_pt.begin() + j);
								arena_pt_ind.erase(arena_pt_ind.begin() + j);
							}
						}

						if (!flyview_track && !manual_track)
						{
							int j = findClosestPoint(pt[focal_fly], raster_pts);
							ndq.SetGalvoAngles(raster_angles[j]);
							ndq.write();

							//ndq.ConvertPtToDeg(pt[focal_fly]);
							//ndq.SetGalvoAngles();
							//ndq.write();
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
						avin.pts = pt;

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
					
					//fvfout.WriteTraj(out.laser, out.head, out.edge, out.galvo_angle, out.odor);
					//fvfout.WriteTraj(out.laser, out.head, out.edge, out.galvo_angle);
					fvfout.WriteTraj(out.laser, out.head, out.galvo_angle);
					
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
					}

					avfout.WriteFrame(out.img);
					avfout.WriteLog(out.stamp);
					avfout.WriteTraj(out.pts);
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
			
			if (NFLIES > 1)
				createTrackbar("focal fly", "controls", &focal_fly, NFLIES-1);
			
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
			int arena_track_key_state = 0;
			
			int left_key_state = 0;
			int right_key_state = 0;
			int up_key_state = 0;
			int down_key_state = 0;

			while (true)
			{
				if (GetAsyncKeyState(VK_HOME))
				{
					if (!arena_track_key_state)
						manual_track = !manual_track;

					arena_track_key_state = 1;
				}
				else
					arena_track_key_state = 0;


				if (GetAsyncKeyState(VK_LEFT))
				{
					if (!left_key_state)
					{
						manual_track = true;
						ndq.MoveLeft();
						ndq.write();
					}

					left_key_state = 1;
				}
				else
					left_key_state = 0;

				if (GetAsyncKeyState(VK_RIGHT))
				{
					if (!right_key_state)
					{
						manual_track = true;
						ndq.MoveRight();
						ndq.write();
					}

					right_key_state = 1;
				}
				else
					right_key_state = 0;

				if (GetAsyncKeyState(VK_UP))
				{
					if (!up_key_state)
					{
						manual_track = true;
						ndq.MoveUp();
						ndq.write();
					}

					up_key_state = 1;
				}
				else
					up_key_state = 0;

				if (GetAsyncKeyState(VK_DOWN))
				{
					if (!down_key_state)
					{
						manual_track = true;
						ndq.MoveDown();
						ndq.write();
					}

					down_key_state = 1;
				}
				else
					down_key_state = 0;
								

				if (GetAsyncKeyState(VK_F1))
				{
					if (!track_key_state)
					{
						flyview_track = !flyview_track;

						manual_track = false;
						
						//Enable next few lines to stop recording when flyview loses track of fly
						//if (flyview_record)
						//	flyview_record = !flyview_record;

						//if (arenaview_record)
						//	arenaview_record = !arenaview_record;
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
	ndq.ConvertPtToDeg(Point2f(0, 0));
	ndq.write();
	printf("[OK]\n");

	return 0;
}
