// fmfrecord.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

using namespace std;
using namespace cv;

int _tmain(int argc, _TCHAR* argv[])
{
	FmfReader fin;
	string fmfname, bgname;
	bool compute_bg = true;

	int imageWidth = 512, imageHeight = 512;

	if (argc > 1)
		fmfname = argv[1];
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
	}

	fin.Open((_TCHAR *)fmfname.c_str());
	fin.ReadHeader();
	fin.GetImageSize(imageWidth, imageHeight);
	int nframes = fin.GetFrameCount();

	Mat arena_frame, arena_bg;

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
	imwrite(fbg.append("-bg.bmp"), arena_bg);

	printf("[OK]\n");

	fin.Close();

	return 0;
}


