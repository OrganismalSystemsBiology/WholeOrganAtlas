#pragma once
#include <stdio.h>
#include <vector>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <sstream>
#include <algorithm>
#include <fstream>
#include <sys/stat.h>
#include <math.h>
#include <chrono>
#include <time.h>

#define BIT unsigned short

using namespace std;
size_t GetFileSize(string fp)
{
	struct stat buf;
	//cout << "GetFileSize: " << fp << endl

	if (stat(fp.c_str(), &buf) == 0)
	{
		//cout << fp << ": " << buf.st_size / 1000000 << "Mbyte\n";
		return buf.st_size;
	}
	else
	{
		printf("not file exist: %s\n", fp.c_str());
	}
	return 0;
}
inline void Rotation(BIT *src, BIT *buf, int &height, int &width)
{
	int i = 0;
	int j = 0;
	size_t p = 0;
	for (i = 0; i < height; ++i)
	{
		for (j = 0; j < width; ++j, ++p)
		{
			buf[(width - j - 1) * height + i] = src[p];
		}
	}

}
inline void Scale_convert(BIT *src, BIT *dst, int scale, int y, int x, int &dst_y, int &dst_x)
{
	dst_x = x / scale;
	dst_y = y / scale;
	size_t ave = 0;
	for (auto i = 0; i < dst_x * dst_y; ++i)
	{
		dst[i] = 25530;
	}

	for (int i = 0; i < dst_y; ++i)
	{
		for (int j = 0; j < dst_x; ++j)
		{
			ave = 0;
			int conv = 0;
			for (int k = 0; k < scale; ++k)
			{
				for (int l = 0; l < scale; ++l)
				{
					if ((i * scale + k > y) || (j * scale + l > x))
					{
						continue;
					}
					else
					{
						//cout << i << "/" << dst_y<<": " <<j <<"/" <<dst_x<< endl;

						ave += src[((i * scale + k) * x + j * scale + l)];
						++conv;
					}
				}
			}
			if (conv == 0)
			{
				dst[i * dst_x + j] = 0;
				//  cout << "conv ==0: " << i << "/" << dst_y << ": " << j << "/" << dst_x << endl;
			}
			else
			{
				if (ave == 0)
				{
					// cout << "Ave ==0: " << i << "/" << dst_y << ": " << j << "/" << dst_x << endl;
				}
				dst[i * dst_x + j] = ave / conv;
			}
			//cout << i << ":" << j <<"\t"<< ave / conv << endl;
		}
	}
	//  cout << "return dst" << endl;
	// return dst;
}
inline void Importfile_bin(string fp, BIT *buf)
{
	FILE *fpr;
	size_t fsize = GetFileSize(fp);
	fpr = fopen(fp.c_str(), "rb");
	size_t rc = fread(buf, sizeof(char), fsize, fpr);

	fclose(fpr);
}
