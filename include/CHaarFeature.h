/*
 * CHaarFeature.h
 *
 *  Created on: Sep 22, 2011
 *      Author: David Fischinger
 */

#ifndef CHAARFEATURE_H_
#define CHAARFEATURE_H_

#include <vector>
#include <opencv/cv.h>
using namespace std;

class CHaarFeature
{
public:
	int nr_reg;	//number of regions
	vector<double> weights;
	vector<int> regions;	//region points are defined w.r.t. the center point for which the feautre value is calculated

	//constructor for features with 2 regions
	CHaarFeature(int r1x1, int r1x2, int r1y1, int r1y2, int r2x1, int r2x2, int r2y1, int r2y2, double r1weight, double r2weight );
	//constructor for features with 3 regions
	CHaarFeature(int r1x1, int r1x2, int r1y1, int r1y2, int r2x1, int r2x2,int r2y1, int r2y2, int r3x1, int r3x2, int r3y1, int r3y2, double r1weight, double r2weight, double r3weight);
	//constructor for features with 4 regions
	CHaarFeature(int r1x1, int r1x2, int r1y1, int r1y2,
				 int r2x1, int r2x2, int r2y1, int r2y2,
				 int r3x1, int r3x2, int r3y1, int r3y2,
				 int r4x1, int r4x2, int r4y1, int r4y2,
				 double r1weight, double r2weight, double r3weight, double r4weight);


	double calcFval(cv::Mat heightsIntegral, int pos_x, int pos_y);
};

#endif /* CHAARFEATURE_H_ */
