#include "../LSS/OpenCVcfg3.1.0.hh"
#include "Train.hh"

#include <fstream>
#include <ml.hpp>

using std::cerr;
using std::cout;
using std::clog;
using std::endl;
using std::ifstream;
using cv::ml::SVM;
using cv::ml::TrainData;

Mat get_hogdescriptor_visu(const Mat& color_origImg, vector<float>& descriptorValues, const Size & size);

void convert_to_ml(const std::vector< cv::Mat > & train_samples, cv::Mat& trainData);

void load_images(const string & prefix, const string & filename, vector< Mat > & img_lst)
{
	string line;
	ifstream file;

	file.open((prefix + filename).c_str());
	if (!file.is_open())
	{
		cerr << "Unable to open the list of images from " << filename << " filename." << endl;
		exit(-1);
	}

	bool end_of_parsing = false;
	while (!end_of_parsing)
	{
		getline(file, line);
		if (line.empty()) // no more file to read
		{
			end_of_parsing = true;
			break;
		}
		Mat img = imread((prefix + line).c_str(), IMREAD_GRAYSCALE); // load the image
		if (img.empty()) // invalid image, just skip it.
			continue;
#ifdef _DEBUG
		imshow("image", img);
		waitKey(10);
#endif
		img_lst.push_back(img.clone());
	}
}

void compute_hog(const vector< Mat > & img_lst, vector< Mat > & gradient_lst, const Size & _winSize, const Size & _blockSize, const Size & _blockStride, const Size & _cellSize, const int _nbins)
{
	HOGDescriptor hog(_winSize, _blockSize, _blockStride, _cellSize, _nbins);
	Mat gray;
	vector< Point > location;
	vector< float > descriptors;

	vector< Mat >::const_iterator img = img_lst.begin();
	vector< Mat >::const_iterator end = img_lst.end();
	for (; img != end; ++img)
	{
		if (3 == (*img).channels())
		{
			cvtColor(*img, gray, COLOR_BGR2GRAY);
		}
		else if (1 == (*img).channels())
		{
			gray = *img;
		}
		else
		{
			CV_Assert("error - img channels");
		}
		hog.compute(gray, descriptors, _cellSize, Size(0, 0), location);
		gradient_lst.push_back(Mat(descriptors).clone());
#ifdef _DEBUG
		imshow("gradient", get_hogdescriptor_visu(img->clone(), descriptors, _winSize));
		waitKey(10);
#endif
	}
}


void train_svm(const vector< Mat > & gradient_lst, const vector< int > & labels, const string &savename)
{
	CV_Assert(!gradient_lst.empty());

	Mat train_data;
	convert_to_ml(gradient_lst, train_data);

	Ptr<SVM> svm = SVM::create();
	clog << "Start training... trainAuto() \n";
	//non-auto train way
/*	svm->setCoef0(0.0);
	svm->setDegree(3);
	svm->setTermCriteria(TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 1000, 1e-3));
	svm->setGamma(0);
	svm->setKernel(SVM::LINEAR);
	svm->setNu(0.5);
	svm->setP(0.1); // for EPSILON_SVR, epsilon in loss function?
	svm->setC(0.01); // From paper, soft classifier
	svm->train(train_data, ml::ROW_SAMPLE, Mat(labels));*/

	Ptr<TrainData> trainData_Auto = TrainData::create(train_data, ml::ROW_SAMPLE, labels);;

	svm->setKernel(SVM::LINEAR);
	svm->trainAuto(trainData_Auto);

	clog << "...[done]" << endl;

	svm->save(savename);
}

// From http://www.juergenwiki.de/work/wiki/doku.php?id=public:hog_descriptor_computation_and_visualization
Mat get_hogdescriptor_visu(const Mat& color_origImg, vector<float>& descriptorValues, const Size & size)
{
	const int DIMX = size.width;
	const int DIMY = size.height;
	float zoomFac = 3;
	Mat visu;
	resize(color_origImg, visu, Size((int)(color_origImg.cols*zoomFac), (int)(color_origImg.rows*zoomFac)));

	int cellSize = 8;
	int gradientBinSize = 9;
	float radRangeForOneBin = (float)(CV_PI / (float)gradientBinSize); // dividing 180 into 9 bins, how large (in rad) is one bin?

	// prepare data structure: 9 orientation / gradient strenghts for each cell
	int cells_in_x_dir = DIMX / cellSize;
	int cells_in_y_dir = DIMY / cellSize;
	float*** gradientStrengths = new float**[cells_in_y_dir];
	int** cellUpdateCounter = new int*[cells_in_y_dir];
	for (int y = 0; y < cells_in_y_dir; y++)
	{
		gradientStrengths[y] = new float*[cells_in_x_dir];
		cellUpdateCounter[y] = new int[cells_in_x_dir];
		for (int x = 0; x < cells_in_x_dir; x++)
		{
			gradientStrengths[y][x] = new float[gradientBinSize];
			cellUpdateCounter[y][x] = 0;

			for (int bin = 0; bin < gradientBinSize; bin++)
				gradientStrengths[y][x][bin] = 0.0;
		}
	}

	// nr of blocks = nr of cells - 1
	// since there is a new block on each cell (overlapping blocks!) but the last one
	int blocks_in_x_dir = cells_in_x_dir - 1;
	int blocks_in_y_dir = cells_in_y_dir - 1;

	// compute gradient strengths per cell
	int descriptorDataIdx = 0;
	int cellx = 0;
	int celly = 0;

	for (int blockx = 0; blockx < blocks_in_x_dir; blockx++)
	{
		for (int blocky = 0; blocky < blocks_in_y_dir; blocky++)
		{
			// 4 cells per block ...
			for (int cellNr = 0; cellNr < 4; cellNr++)
			{
				// compute corresponding cell nr
				cellx = blockx;
				celly = blocky;
				if (cellNr == 1) celly++;
				if (cellNr == 2) cellx++;
				if (cellNr == 3)
				{
					cellx++;
					celly++;
				}

				for (int bin = 0; bin < gradientBinSize; bin++)
				{
					float gradientStrength = descriptorValues[descriptorDataIdx];
					descriptorDataIdx++;

					gradientStrengths[celly][cellx][bin] += gradientStrength;

				} // for (all bins)


				// note: overlapping blocks lead to multiple updates of this sum!
				// we therefore keep track how often a cell was updated,
				// to compute average gradient strengths
				cellUpdateCounter[celly][cellx]++;

			} // for (all cells)


		} // for (all block x pos)
	} // for (all block y pos)


	// compute average gradient strengths
	for (celly = 0; celly < cells_in_y_dir; celly++)
	{
		for (cellx = 0; cellx < cells_in_x_dir; cellx++)
		{

			float NrUpdatesForThisCell = (float)cellUpdateCounter[celly][cellx];

			// compute average gradient strenghts for each gradient bin direction
			for (int bin = 0; bin < gradientBinSize; bin++)
			{
				gradientStrengths[celly][cellx][bin] /= NrUpdatesForThisCell;
			}
		}
	}

	// draw cells
	for (celly = 0; celly < cells_in_y_dir; celly++)
	{
		for (cellx = 0; cellx < cells_in_x_dir; cellx++)
		{
			int drawX = cellx * cellSize;
			int drawY = celly * cellSize;

			int mx = drawX + cellSize / 2;
			int my = drawY + cellSize / 2;

			rectangle(visu, Point((int)(drawX*zoomFac), (int)(drawY*zoomFac)), Point((int)((drawX + cellSize)*zoomFac), (int)((drawY + cellSize)*zoomFac)), Scalar(100, 100, 100), 1);

			// draw in each cell all 9 gradient strengths
			for (int bin = 0; bin < gradientBinSize; bin++)
			{
				float currentGradStrength = gradientStrengths[celly][cellx][bin];

				// no line to draw?
				if (currentGradStrength == 0)
					continue;

				float currRad = bin * radRangeForOneBin + radRangeForOneBin / 2;

				float dirVecX = cos(currRad);
				float dirVecY = sin(currRad);
				float maxVecLen = (float)(cellSize / 2.f);
				float scale = 2.5; // just a visualization scale, to see the lines better

				// compute line coordinates
				float x1 = mx - dirVecX * currentGradStrength * maxVecLen * scale;
				float y1 = my - dirVecY * currentGradStrength * maxVecLen * scale;
				float x2 = mx + dirVecX * currentGradStrength * maxVecLen * scale;
				float y2 = my + dirVecY * currentGradStrength * maxVecLen * scale;

				// draw gradient visualization
				line(visu, Point((int)(x1*zoomFac), (int)(y1*zoomFac)), Point((int)(x2*zoomFac), (int)(y2*zoomFac)), Scalar(0, 255, 0), 1);

			} // for (all bins)

		} // for (cellx)
	} // for (celly)


	// don't forget to free memory allocated by helper data structures!
	for (int y = 0; y < cells_in_y_dir; y++)
	{
		for (int x = 0; x < cells_in_x_dir; x++)
		{
			delete[] gradientStrengths[y][x];
		}
		delete[] gradientStrengths[y];
		delete[] cellUpdateCounter[y];
	}
	delete[] gradientStrengths;
	delete[] cellUpdateCounter;

	return visu;

} // get_hogdescriptor_visu

/*
* Convert training/testing set to be used by OpenCV Machine Learning algorithms.
* TrainData is a matrix of size (#samples x max(#cols,#rows) per samples), in 32FC1.
* Transposition of samples are made if needed.
*/
void convert_to_ml(const std::vector< cv::Mat > & train_samples, cv::Mat& trainData)
{
	//--Convert data
	const int rows = (int)train_samples.size();
	const int cols = (int)std::max(train_samples[0].cols, train_samples[0].rows);
	cv::Mat tmp(1, cols, CV_32FC1); //< used for transposition if needed
	trainData = cv::Mat(rows, cols, CV_32FC1);
	vector< Mat >::const_iterator itr = train_samples.begin();
	vector< Mat >::const_iterator end = train_samples.end();
	for (int i = 0; itr != end; ++itr, ++i)
	{
		CV_Assert(itr->cols == 1 ||
			itr->rows == 1);
		if (itr->cols == 1)
		{
			transpose(*(itr), tmp);
			tmp.copyTo(trainData.row(i));
		}
		else if (itr->rows == 1)
		{
			itr->copyTo(trainData.row(i));
		}
	}
}


void get_svm_detector(const Ptr<SVM>& svm, vector< float > & hog_detector)
{
	// get the support vectors
	Mat sv = svm->getSupportVectors();
	const int sv_total = sv.rows;
	// get the decision function
	Mat alpha, svidx;
	double rho = svm->getDecisionFunction(0, alpha, svidx);

	CV_Assert(alpha.total() == 1 && svidx.total() == 1 && sv_total == 1);
	CV_Assert((alpha.type() == CV_64F && alpha.at<double>(0) == 1.) ||
		(alpha.type() == CV_32F && alpha.at<float>(0) == 1.f));
	CV_Assert(sv.type() == CV_32F);
	hog_detector.clear();

	hog_detector.resize(sv.cols + 1);
	memcpy(&hog_detector[0], sv.ptr(), sv.cols*sizeof(hog_detector[0]));
	hog_detector[sv.cols] = (float)-rho;
}


void draw_locations(Mat & img, const vector< Rect > & locations, const Scalar & color)
{
	if (!locations.empty())
	{
		vector< Rect >::const_iterator loc = locations.begin();
		vector< Rect >::const_iterator end = locations.end();
		for (; loc != end; ++loc)
		{
			rectangle(img, *loc, color, 2);
		}
	}
}


void test_it(const Size & _winSize, const Size & _blockSize, const Size & _blockStride, const Size & _cellSize, const int _nbins)
{
	char key = 27;
	Scalar reference(0, 255, 0);
	Scalar trained(0, 0, 255);
	Mat img, draw;
	Ptr<SVM> svm;
	HOGDescriptor my_hog(_winSize, _blockSize, _blockStride, _cellSize, _nbins);
	vector< Rect > locations;

	// Load the trained SVM.
	svm = ml::StatModel::load<SVM>("SVM_DATA.xml");
	// Set the trained svm to my_hog
// 	vector< float > hog_detector;
// 	get_svm_detector(svm, hog_detector);
// 	my_hog.setSVMDetector(hog_detector);


	img = imread("D:/TihawFiles/code/DMcodeBin/longDistance_DM_barcode/longDistance_DM_barcode/DMset-rectified/test/132.bmp");
	Mat img2 = imread("D:/TihawFiles/code/DMcodeBin/longDistance_DM_barcode/longDistance_DM_barcode/DMset-rectified/test/137.bmp");
	Mat img3 = imread("D:/TihawFiles/code/DMcodeBin/longDistance_DM_barcode/longDistance_DM_barcode/DMset-rectified/test/142.bmp");
	Mat img4 = imread("D:/TihawFiles/code/DMcodeBin/longDistance_DM_barcode/longDistance_DM_barcode/DMset-rectified/test/147.bmp");

		if (img.empty())
			return;

		draw = img.clone();
		

		locations.clear();

		vector<float> hogfeature;
		my_hog.compute(img, hogfeature);

		float a = svm->predict(hogfeature);
		cout << a << endl;

		hogfeature.clear();
		my_hog.compute(img2, hogfeature);
		a = svm->predict(hogfeature);
		cout << a << endl;


		hogfeature.clear();
		my_hog.compute(img3, hogfeature);
		a = svm->predict(hogfeature);
		cout << a << endl;

		hogfeature.clear();
		my_hog.compute(img4, hogfeature);
		a = svm->predict(hogfeature);
		cout << a << endl;

		imshow("test", draw);
		waitKey(0);
}

