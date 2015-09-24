/*
 * David Fischinger -TUW
 * 18.11.2011
 *
 * CLASS for calculating Feature Vector from Integral Images
 *
 * OUTDATED!!!!
 *
 * USAGE:
 * 	- generate ListOfIIFilenames.txt in correct folder
 * 	- change all Parameters
 * 	- execute
 *
 *
 * input:
 *   Filenames (generated with: "ls pcd* -1 > ListOfIIFilenames.txt"
 *   executed in IntegralImages folder)
 *   with integral images and filename with features
 *
 * output:
 *   One file for each integral images including all feature values (1 value for each feature) is saved in the given output folder.
 *
 *   PARAMETERS:
 *
 *    HEIGHT 15 (14+1)
 *    WIDTH 15 (14+1)
 *    goodgps		indicates if features for good or bad GPs are calculated => label +1/-1 in output .txt
 */

#include <CIntImage_to_Featurevec.h>
#include <CHaarFeature.h>
#include <iostream>
#include <iomanip>

#define HEIGHT 15
#define WIDTH 15

using namespace std;
using namespace cv;


CIntImage_to_Featurevec::CIntImage_to_Featurevec(){
	currentfeature = new CHaarFeature(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1.1,1.0,1.0,2.0);
}


void CIntImage_to_Featurevec::read_features(string featurespath)
{
	//PARAMETERS
	string path = featurespath;

 	ifstream file_features;
 	file_features.open(path.c_str());	//file_features: file with all features
	if (!file_features){
		cout << "\n PROBLEM opening feature file: " << path.c_str() << endl;
	}

	string line;
	int id_x = -1;
	getline(file_features, line);
	while (file_features.good())
	{
		int start = 0, end = 0;
		int reg_c[16];	//region_corners
		float reg_w[4];	//region_weights
		id_x++;
		for (int i = 0; i < 16; i++){
			end = line.find("\t", start);
			reg_c[i] = atoi(line.substr(start,end-start).c_str()) ;
			start = end+1;
		}
		for (int j = 0; j < 4; j++){
			end = line.find("\t", start);
			reg_w[j] = atof(line.substr(start,end-start).c_str()) ;
			start = end+1;
		}

		CHaarFeature * tmp_feature = new CHaarFeature(reg_c[0],reg_c[1],reg_c[2],reg_c[3], reg_c[4],reg_c[5],reg_c[6],reg_c[7],reg_c[8],reg_c[9],reg_c[10],reg_c[11],reg_c[12],reg_c[13],reg_c[14],reg_c[15],reg_w[0],reg_w[1],reg_w[2],reg_w[3]);
		this->allfeatures.push_back(*tmp_feature);

		getline(file_features, line);
	}
	file_features.close();
}


void CIntImage_to_Featurevec::print_features()
{

	for (int i = 0; i < this->allfeatures.size(); i++)
	{
		CHaarFeature curfeature = this->allfeatures.at(i);
		for (int j = 0; j < 16; j++){
			int reg_num = curfeature.regions.at(j);
			cout << reg_num << "\t";
		}
		for (int j = 0; j < 4; j++){
					float reg_w = curfeature.weights.at(j);
					cout << reg_w << "\t";
				}
		cout << i << "\n" ;
	}
}


void CIntImage_to_Featurevec::print_heights()
{
    //print heights matrix
	cout << "print integral image matrix:" << endl;
	for (int i = 0; i < HEIGHT; i++){  //rows
		for (int j = 0; j < WIDTH; j++){  //cols
			cout << this->intimagemat[i][j] << "\t";
		}
		cout << "\n";
	}
	cout << "\n david: ";
	cout << this->currentfeature->nr_reg << endl;
}



void CIntImage_to_Featurevec::write_featurevector(string outputpath, int nr_features_without_shaf)
{
    //open output file
	ofstream output_fv_file(outputpath.c_str(), fstream::app);
	if (this->goodgps)
		output_fv_file << "+1";
	else
		output_fv_file << "-1";
	//for all features
	for (int nr_feat = 0; nr_feat < this->allfeatures.size(); nr_feat++){
		float featureval = calc_featurevalue(nr_feat, nr_features_without_shaf);
		output_fv_file << " " << nr_feat+1 << ":" << setprecision(4) << featureval;
	}
	output_fv_file << "\n";
	output_fv_file.close();
}



float CIntImage_to_Featurevec::calc_featurevalue(int nr_feat, int nr_features_without_shaf)
{
	float returnval = 0;
	this->currentfeature = &this->allfeatures.at(nr_feat);
	if (nr_feat < nr_features_without_shaf)
	{
		for (int nr_reg = 0; nr_reg <4; nr_reg++){
			//corners of region
			int x1 = currentfeature->regions[nr_reg*4];
			int x2 = currentfeature->regions[nr_reg*4+1];
			int y1 = currentfeature->regions[nr_reg*4+2];
			int y2 = currentfeature->regions[nr_reg*4+3];
			float wgt = currentfeature->weights[nr_reg];	//weight of region

			if (( wgt == 0.0 ) or		//region not used (weight equal 0)
				( x2 < x1 ) or			//r.x2 < r.x1
				( y2 < y1 ) or			//r.y2 < r.y1
				(x2 == 0 and y2 == 0)) 	//region corners are (0,0,0,0)
				continue;

			returnval += wgt*(this->intimagemat[x2+1][y2+1] - this->intimagemat[x1][y2+1] -
						  this->intimagemat[x2+1][y1] + this->intimagemat[x1][y1]);
		}
	} else {//new (symmetry) features
		//new features to check if top and bottom of grasp area are symmetric
		float r[3];	//r[0]: top field, r[1] middle, r[2]: bottom/down
		r[0] = r[1] = r[2] = 0;
		for (int nr_reg = 0; nr_reg <3; nr_reg++){ //assuming 3 regions
			bool grasp_ok = true;
			//corners of region
			int x1 = currentfeature->regions[nr_reg*4];
			int x2 = currentfeature->regions[nr_reg*4+1];
			int y1 = currentfeature->regions[nr_reg*4+2];
			int y2 = currentfeature->regions[nr_reg*4+3];
			float wgt = currentfeature->weights[nr_reg];	//weight of region

			if (( wgt == 0.0 ) or		//region not used (weight equal 0)
				( x2 < x1 ) or			//r.x2 < r.x1
				( y2 < y1 ) or			//r.y2 < r.y1
				(x2 == 0 and y2 == 0)) 	//region corners are (0,0,0,0)
				continue;

			r[nr_reg] = wgt*(this->intimagemat[x2+1][y2+1] - this->intimagemat[x1][y2+1] -
							     this->intimagemat[x2+1][y1] + this->intimagemat[x1][y1]);
		}

		if (r[1] > r[0] and r[1] > r[2]){
			returnval = min(r[1]-r[0], r[1]-r[2]);  //must be possitiv then
		} else {
			returnval = -1.0;
		}
	}
	int cnt = 0;
	if (false /*nr_feat == 1*/){
		this->print_heights();
		cout << "cnt, feature, returnval: " << cnt++ << "\t"<< nr_feat << "  " << returnval << endl;
	}
	return returnval;
}


