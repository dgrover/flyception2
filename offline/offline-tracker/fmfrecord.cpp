// fmfrecord.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace cv;

#define NFLIES 2

float dist(Point2f p1, Point2f p2)
{
	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;
	return(sqrt(dx*dx + dy*dy));
}

int findClosestPoint(Point2f pt, vector<Point2f> nbor)
{
	int fly_index = 0;
	if (nbor.size() == 1)
		return fly_index;
	else
	{
		float fly_dist = dist(pt, nbor[0]);

		for (int i = 1; i < nbor.size(); i++)
		{
			float res = dist(pt, nbor[i]);
			if (res < fly_dist)
			{
				fly_dist = res;
				fly_index = i;                //Store the index of nearest point
			}
		}

		return fly_index;
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	FmfReader fin;
	string fmfname, bgname;
	bool compute_bg = false;

	int imageWidth = 512, imageHeight = 512;

	if (argc > 1)
	{
		fmfname = argv[1];
		
		if (argc == 2)
			compute_bg = true;
		else
			bgname = argv[2];
	}
	else
	{
		OPENFILENAME ofn;
		char szFileName[MAX_PATH] = "";

		ZeroMemory(&ofn, sizeof(ofn));

		ofn.lStructSize = sizeof(ofn); // SEE NOTE BELOW
		ofn.lpstrFilter = "FMF Files (*.fmf)\0*.fmf\0";
		ofn.lpstrFile = szFileName;
		ofn.nMaxFile = MAX_PATH;
		ofn.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
		ofn.lpstrDefExt = "fmf";

		if (GetOpenFileName(&ofn))
			fmfname = szFileName;
		else
			return -1;


		ZeroMemory(&ofn, sizeof(ofn));

		ofn.lStructSize = sizeof(ofn); // SEE NOTE BELOW
		ofn.lpstrFilter = "BMP Files (*.bmp)\0*.bmp\0";
		ofn.lpstrFile = szFileName;
		ofn.nMaxFile = MAX_PATH;
		ofn.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
		ofn.lpstrDefExt = "bmp";

		if (GetOpenFileName(&ofn))
			bgname = szFileName;
		else
			compute_bg = true;
	}

	fin.Open((_TCHAR *)fmfname.c_str());
	fin.ReadHeader();
	fin.GetImageSize(imageWidth, imageHeight);
	int nframes = fin.GetFrameCount();

	Mat arena_frame, arena_mask, arena_bg;

	if (compute_bg)
	{
		printf("Compute Background ");

		Mat accimg = Mat::zeros(imageWidth, imageHeight, CV_32F);

		for (int imageCount = 0; imageCount < nframes; imageCount++)
		{
			arena_frame = fin.ReadFrame(imageCount);
			accumulate(arena_frame, accimg);
		}

		arena_bg = accimg / nframes;
		arena_bg.convertTo(arena_bg, CV_8UC1);

		string fbg = fmfname.substr(0, fmfname.length() - 4);
		fbg.insert(fbg.length() - 15, "bg-");
		imwrite(fbg.append(".bmp"), arena_bg);

		printf("[OK]\n");
	}
	else
	{
		arena_bg = imread(bgname, 0);
		arena_bg.convertTo(arena_bg, CV_8UC1);
	}

	FILE *fout = new FILE;
	string ftraj = fmfname.substr(0, fmfname.length() - 4);
	ftraj.insert(ftraj.length() - 15, "traj-");
	printf("%s\n", ftraj.c_str());

	fopen_s(&fout, ftraj.append(".txt").c_str(), "w");

	if (fout == NULL)
	{
		printf("\nError creating trajectory file. Exiting...");
		return -1;
	}

	FILE* fw = new FILE;
	string fwe = fmfname.substr(0, fmfname.length() - 4);
	fwe.insert(fwe.length() - 15, "we-");
	fopen_s(&fw, fwe.append(".txt").c_str(), "w");

	if (fw == NULL)
	{
		printf("\nError creating wing extension file. Exiting...");
		return -1;
	}

	int arena_thresh = 10;
	int arena_erode = 1;
	int arena_dilate = 1;

	Mat arena_element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));

	vector<vector<Point2f>> all_pts;
	vector<vector<double>> all_szs;
	
	vector<bool> swap(nframes);
	vector<bool> lost(nframes);
	vector<bool> merge(nframes);

	vector<int> lw(nframes);
	vector<int> rw(nframes);

	all_pts.resize(nframes, vector<Point2f>(NFLIES));
	all_szs.resize(nframes, vector<double>(NFLIES));

	//vector<Point2f> arena_pt(NFLIES);
	
	// old videos before laser re-alignment
	//Point el_center(260, 227);
	//int el_maj_axis = 237, el_min_axis = 133;
	//int el_angle = 179;
	
	//videos after laser re-alignment
	Point el_center(258, 218);
	int el_maj_axis = 240, el_min_axis = 133;
	int el_angle = 179;

	float fly_dist = 0;

	Mat outer_mask = Mat::zeros(Size(imageWidth, imageWidth), CV_8UC1);
	ellipse(outer_mask, el_center, Size(el_maj_axis, el_min_axis), el_angle, 0, 360, Scalar(255, 255, 255), FILLED);

	bool stream = true;
	int delay = stream;
	int imageCount = 0;

	while (true)
	{
		vector<Point2f> fly_pt(NFLIES);
		vector<double> fly_sz(NFLIES);

		vector<Point2f> arena_pt(NFLIES);
		
		bool found = false;
		int rewind = 1;

		while ((imageCount - rewind) >= 0 && !found)
		{
			int xysum = 0;

			arena_pt = all_pts[imageCount - rewind];

			for (int i = 0; i < NFLIES; i++)
				xysum += (arena_pt[i].x + arena_pt[i].y);

			if (xysum != 0)
				found = true;
			else
				rewind++;
		}

		//if (imageCount > 0)
		//	arena_pt = all_pts[imageCount - 1];


		arena_frame = fin.ReadFrame(imageCount);

		subtract(arena_bg, arena_frame, arena_mask);
		threshold(arena_mask, arena_mask, arena_thresh, 255, THRESH_BINARY);

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

			Mat all_ctr_pts;

			for (int i = 0; i < arena_contours.size(); i++)
			{
				//drawContours(arena_mask, arena_contours, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
				arena_mu[i] = moments(arena_contours[i], false);
				arena_mc[i] = Point2f(arena_mu[i].m10 / arena_mu[i].m00, arena_mu[i].m01 / arena_mu[i].m00);

				arena_sz[i] = contourArea(arena_contours[i]);

				if (arena_sz[i] > MIN_FLY_SIZE)
				{
					//drawContours(arena_frame, arena_contours, i, Scalar(255, 255, 255), 1, 8, vector<Vec4i>(), 0, Point());
					drawContours(arena_mask, arena_contours, i, Scalar(255, 255, 255), FILLED, 1);
					arena_ctr_pts.push_back(arena_mc[i]);
					arena_ctr_sz.push_back(arena_sz[i]);

					for (int j = 0; j < arena_contours[i].size(); j++)
						all_ctr_pts.push_back(arena_contours[i].at(j));

				}
			}

			
			if (arena_ctr_pts.size() > 0)
			{
				//printf("%f %d\n", fly_dist, fly_lost);

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
					if (fly_dist > FLY_MERGE_MAX_DIST && arena_ctr_pts.size() < NFLIES)
					{
						if (!lost[imageCount] && !merge[imageCount])
						{
							lost[imageCount] = true;
							merge[imageCount] = false;
						}
					}
					else
					{
						if (!lost[imageCount] && !merge[imageCount])
						{
							lost[imageCount] = false;
							merge[imageCount] = true;
						}
					}


					if (lost[imageCount])
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


							//putText(arena_frame, to_string(arena_pt_ind[j]), arena_pt[arena_pt_ind[j]], FONT_HERSHEY_COMPLEX, 0.2, Scalar(255, 255, 255));

							last_arena_pt.erase(last_arena_pt.begin() + j);
							arena_pt_ind.erase(arena_pt_ind.begin() + j);
						}
					}
					else
					{
						Mat labels, centers;
						vector<Point2f> cluster_ctr_pts;
						vector<double> cluster_ctr_sz;

						all_ctr_pts = all_ctr_pts.reshape(1, all_ctr_pts.cols / 2); // n rows a 2 cols
						all_ctr_pts.convertTo(all_ctr_pts, CV_32F);

						kmeans(all_ctr_pts, NFLIES, labels, TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1.0), 10, KMEANS_PP_CENTERS, centers);

						for (int i = 0; i < NFLIES; i++)
						{
							cluster_ctr_pts.push_back(Point2f(centers.at<float>(i, 0), centers.at<float>(i, 1)));

							vector<Point2f> cluster_raw_ctr_pts;

							for (int j = 0; j < labels.rows; j++)
							{
								int idx = labels.at<int>(j);

								if (idx == i)
									cluster_raw_ctr_pts.push_back(Point2f(all_ctr_pts.at<float>(j, 0), all_ctr_pts.at<float>(j, 1)));
							}

							cluster_ctr_sz.push_back(contourArea(cluster_raw_ctr_pts));
						}

						for (int i = 0; i < NFLIES; i++)
						{
							int j = findClosestPoint(arena_pt[i], cluster_ctr_pts);

							arena_pt[i] = cluster_ctr_pts[j];

							fly_pt[i] = arena_pt[i];
							fly_sz[i] = cluster_ctr_sz[j];

							//putText(arena_frame, to_string(i), arena_pt[i], FONT_HERSHEY_COMPLEX, 0.2, Scalar(255, 255, 255));

							cluster_ctr_pts.erase(cluster_ctr_pts.begin() + j);
							cluster_ctr_sz.erase(cluster_ctr_sz.begin() + j);
						}
					}
				}

				if (swap[imageCount])
				{
					if (NFLIES == 2)
					{
						Point2f tpt = fly_pt[0];
						fly_pt[0] = fly_pt[1];
						fly_pt[1] = tpt;

						double tsz = fly_sz[0];
						fly_sz[0] = fly_sz[1];
						fly_sz[1] = tsz;
					}
				}

				if (NFLIES == 2)
					fly_dist = dist(fly_pt[0], fly_pt[1]);

			}
		}

		for (int i = 0; i < NFLIES; i++)
		{
			putText(arena_frame, to_string(i), fly_pt[i], FONT_HERSHEY_COMPLEX, 0.2, Scalar(255, 255, 255));

			all_pts[imageCount][i] = fly_pt[i];
			all_szs[imageCount][i] = fly_sz[i];

			printf("%f %f %f ", fly_pt[i].x, fly_pt[i].y, fly_sz[i]);
		}

		printf("%d %d\n", lw[imageCount], rw[imageCount]);

		//printf("\n");

		putText(arena_frame, to_string(imageCount), Point((imageWidth - 50), 10), FONT_HERSHEY_COMPLEX, 0.4, Scalar(255, 255, 255));
		ellipse(arena_frame, el_center, Size(el_maj_axis, el_min_axis), el_angle, 0, 360, Scalar(255, 255, 255));

		imshow("Raw", arena_frame);
		imshow("Mask", arena_mask);
		//imshow("Background", arena_bg);

		int ch = waitKey(delay);

		if (ch == 's')
		{
			if (NFLIES == 2)
			{
				if (swap[imageCount])
					swap[imageCount] = 0;
				else
					swap[imageCount] = 1;
			}
		}

		if (ch == 'm')
		{
			lost[imageCount] = !lost[imageCount];
			merge[imageCount] = !lost[imageCount];
		}

		if (ch == 'l')
			lw[imageCount] = !lw[imageCount];

		if (ch == 'r')
			rw[imageCount] = !rw[imageCount];

		if (ch == 32)
		{
			stream = !stream;
			delay = stream;
		}

		if (ch == 'a')
		{
			stream = false;
			delay = 0;

			if (imageCount > 0)
				imageCount--;
		}

		if (ch == 'd')
		{
			stream = false;
			delay = 0;
			
			if (imageCount < (nframes - 1))
			{
				imageCount++;
				lw[imageCount] = lw[imageCount - 1];
				rw[imageCount] = rw[imageCount - 1];
			}
		}

		if (ch == 27)
			break;

		if (stream)
		{
			if (imageCount < (nframes - 1))
			{
				imageCount++;
				lw[imageCount] = lw[imageCount - 1];
				rw[imageCount] = rw[imageCount - 1];
			}
			else
				stream = false;
		}
	}


	for (imageCount = 0; imageCount < nframes; imageCount++)
	{
		fprintf(fout, "%d ", imageCount);
		fprintf(fw, "%d %d %d\n", imageCount, lw[imageCount], rw[imageCount]);

		for (int i = 0; i < NFLIES; i++)
		{
			fprintf(fout, "%f %f %f ", all_pts[imageCount][i].x, all_pts[imageCount][i].y, all_szs[imageCount][i]);
			//printf("%f %f %f ", fly_pt[i].x, fly_pt[i].y, fly_sz[i]);
		}

		fprintf(fout, "\n");
		//printf("\n");
	}

	fin.Close();
	fclose(fout);
	fclose(fw);
	
	return 0;
}


