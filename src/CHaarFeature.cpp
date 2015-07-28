/*
 * CHaarFeature.cpp
 *
 *  Created on: Sep 22, 2011
 *      Author: David Fischinger
 */

#include <iostream>
#include "CHaarFeature.h"
#include <vector>
#include <opencv/cv.h>

using namespace std;

//constructor for features with 2 regions
CHaarFeature::CHaarFeature(int r1x1,int r1x2,int r1y1,int r1y2,int r2x1,int r2x2,int r2y1,int r2y2,double r1weight,double r2weight ){
	this->nr_reg = 2;
	this->weights = vector<double>(2);
	this->weights[0] = r1weight;
	this->weights[1] = r2weight;
	this->regions = vector<int>(8);
	this->regions[0] = r1x1;
	this->regions[1] = r1x2;
	this->regions[2] = r1y1;
	this->regions[3] = r1y2;
	this->regions[4] = r2x1;
	this->regions[5] = r2x2;
	this->regions[6] = r2y1;
	this->regions[7] = r2y2;
}

//constructor for features with 3 regions
CHaarFeature::CHaarFeature(int r1x1, int r1x2, int r1y1, int r1y2, int r2x1, int r2x2, int r2y1, int r2y2, int r3x1, int r3x2, int r3y1, int r3y2, double r1weight, double r2weight, double r3weight){
	this->nr_reg = 3;
	this->weights = vector<double>(3);
	this->weights[0] = r1weight;
	this->weights[1] = r2weight;
	this->weights[2] = r3weight;
	this->regions = vector<int>(12);
	this->regions[0] = r1x1;
	this->regions[1] = r1x2;
	this->regions[2] = r1y1;
	this->regions[3] = r1y2;
	this->regions[4] = r2x1;
	this->regions[5] = r2x2;
	this->regions[6] = r2y1;
	this->regions[7] = r2y2;
	this->regions[8] = r3x1;
	this->regions[9] = r3x2;
	this->regions[10] = r3y1;
	this->regions[11] = r3y2;
}
//constructor for features with 4 regions
CHaarFeature::CHaarFeature(int r1x1, int r1x2, int r1y1, int r1y2, int r2x1, int r2x2, int r2y1, int r2y2, int r3x1, int r3x2, int r3y1, int r3y2, int r4x1, int r4x2, int r4y1, int r4y2, \
			 double r1weight, double r2weight, double r3weight, double r4weight){
	this->nr_reg = 4;
	this->weights = vector<double>(4);
	this->weights[0] = r1weight;
	this->weights[1] = r2weight;
	this->weights[2] = r3weight;
	this->regions = vector<int>(16);
	this->regions[0] = r1x1;
	this->regions[1] = r1x2;
	this->regions[2] = r1y1;
	this->regions[3] = r1y2;
	this->regions[4] = r2x1;
	this->regions[5] = r2x2;
	this->regions[6] = r2y1;
	this->regions[7] = r2y2;
	this->regions[8] = r3x1;
	this->regions[9] = r3x2;
	this->regions[10] = r3y1;
	this->regions[11] = r3y2;
	this->regions[12] = r4x1;
	this->regions[13] = r4x2;
	this->regions[14] = r4y1;
	this->regions[15] = r4y2;
}


//calculates the Feature value for the feature using the given integral image and the position posx and posy
double CHaarFeature::calcFval(cv::Mat heightsIntegral, int pos_x, int pos_y)
{
	double retval = 0;
	double retval_old = 0;
	for (int reg = 0; reg < this->nr_reg; reg++){	//for each region calculate weighted value
		double weight = this->weights[reg];
		int colx1 = pos_x + this->regions[4*reg];	//x1-value for integral image pixel (of moving window) in region reg
		int colx2 = pos_x + this->regions[4*reg+1];	//x2-value for integral image pixel (of moving window) in region reg
		int rowy1 = pos_y + this->regions[4*reg+2]; //y1-value for integral image pixel (of moving window) in region reg
		int rowy2 = pos_y + this->regions[4*reg+3]; //y2-value for integral image pixel (of moving window) in region reg
		retval_old = retval;
		retval = retval + weight*(
				((double*)(heightsIntegral.ptr() + heightsIntegral.step*(rowy2)))[colx2]		//x2y2 biggest rect. positiv
			+	((double*)(heightsIntegral.ptr() + heightsIntegral.step*(rowy1-1)))[colx1-1]	//x1y1 smallest rect. positiv
			-	((double*)(heightsIntegral.ptr() + heightsIntegral.step*(rowy2)))[colx1-1]		//x1y2  rectangle left of region (to subtract)
			-	((double*)(heightsIntegral.ptr() + heightsIntegral.step*(rowy1-1)))[colx2]		//x2y1 rectangle above region (to subtract)
			);
		//cout << "\n sum of region: " << reg+1 << "  " << retval - retval_old << endl;
	}
	return retval;
}

