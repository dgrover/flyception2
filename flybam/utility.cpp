#include "stdafx.h"
#include "utility.h"

Point2f rotateFlyCenter(Point2f p, int image_width, int image_height)
{
	Point2f temp, refPt;

	//move point to origin and rotate by 15 degrees due to the tilt of the galvo x-mirror
	temp.x = (cos(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.x - image_width / 2) - sin(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.y - image_height / 2));
	temp.y = (sin(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.x - image_width / 2) + cos(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.y - image_height / 2));

	refPt.x = (image_width / 2) + temp.x;
	refPt.y = (image_height / 2) + temp.y;

	//printf("[%f %f]\n", pt.at<double>(0, 0), pt.at<double>(1, 0));
	//printf("[%f %f]\n", refPt.at<double>(0, 0), refPt.at<double>(1, 0));

	return refPt;
}

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
