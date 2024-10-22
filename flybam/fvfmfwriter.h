#ifndef FVFMFWRITER_H
#define FVFMFWRITER_H

using namespace std;
using namespace FlyCapture2;
using namespace cv;

class FVFmfWriter
{
	private:
		FILE *fp;
		FILE *flog;
		FILE *ftraj;

		char fname[100];
		char flogname[100];
		char ftrajname[100];

		unsigned __int32 fmfVersion, SizeY, SizeX;
		unsigned __int64 bytesPerChunk;
		char *buf;

	public:
		unsigned __int64 nframes;

		FVFmfWriter();

		int Open();
		int Close();

		void InitHeader(unsigned __int32 x, unsigned __int32 y);
		void WriteHeader();
		//void WriteFrame(TimeStamp st, Image img);
		void WriteFrame(Image img);
		void WriteFrame(Mat img);
		//void WriteLog(TimeStamp st);
		void WriteLog(int st);
		void WriteTraj(Point2f world, Point2f head, Point2f galvo_angle, int z_track, int lens_pos, double var_lap);
		
		int IsOpen();
		
};

#endif