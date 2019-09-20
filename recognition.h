#pragma once
#include "typeDef.h"
#include "utilities.h"

class Recognition
{
public:
	bool set_ObjectLibrary(std::vector< PointCloudPtr > &objectLib);

	bool findTheMatchingObject(PointCloudPtr scene);

private:
	std::vector< FeatureCloud > objectLibrary;

	FeatureCloud sourceCloud;
};

//希尔排序
template <typename T>
void shellsort(T A[], int l, int h, int* index)
{
	if (l < h)
	{
		int d;//增量
		int tmp;
		int j;
		int size = h - l + 1;
		for (d = size / 2; d >= 1; d /= 2)
		{
			//组内使用插入排序
			//d+l是第一组的第二个元素
			for (int i = d + l; i <= h; ++i)
			{
				tmp = A[i];
				j = i - d;//该组中，当前元素的前一个元素
						  //寻找要插入的位置
				while (j >= l && A[j] > tmp)
				{
					T temp = A[j];
					A[j] = A[j + d];
					A[j + d] = temp;

					int indexTemp = j;
					index[j] = index[j + d];
					index[j + d] = indexTemp;

					j -= d;
				}
				A[j + d] = tmp;
				index[j + d] = i;
			}
		}
	}
}

template <typename T>
void bubbleSort(T A[], int l, int h, int* index)
{
	if (l < h)
	{
		int size = h - l + 1;
		for (int i = 0; i < size; ++i)
		{
			for (int j = 0; j < size - i - 1; ++j)
			{
				if (A[j] > A[j + 1])
				{
					T temp = A[j];
					A[j] = A[j + 1];
					A[j + 1] = temp;

					int tempIndex = index[j];
					index[j] = index[j + 1];
					index[j + 1] = tempIndex;
				}
			}
		}		
	}
}