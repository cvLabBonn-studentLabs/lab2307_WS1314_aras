/*
 * classifer_svm.cpp
 *
 *  Created on: Mar 20, 2014
 *      Author: neek
 */


#include "classifier_svm.h"

namespace classifier {

float kConfidenceThreshold = 0.85f;
char const * BodyPartName[] = {"HEAD", "LEFT_HAND", "RIGHT_HAND", "LEFT_FOOT", "RIGHT_FOOT"};

ClassifierSVM::ClassifierSVM(int num_attr) {

	features_ = (struct svm_node *) malloc((num_attr + 1) * sizeof(struct svm_node));

}

ClassifierSVM::~ClassifierSVM() { free(features_); }

bool ClassifierSVM::read_model(const std::string file) {
	if ((model_ = svm_load_model(file.c_str()))==0) {
		std::cerr << "Can't open model file " << file << std::endl;
		return false;
	}

	return true;
}

int ClassifierSVM::predict(const cv::Mat image) {
	create_feature_vector(image);

	double predict_label;

	predict_label = svm_predict(model_, features_);
	return predict_label;
}

void ClassifierSVM::create_feature_vector(const cv::Mat image) {
	assert(image.depth() == CV_32F);

	int index = 0;
	for(int row = 0; row < image.rows; ++row) {
	    const float* p = image.ptr<float>(row);
	    for(int col = 0; col < image.cols; ++col) {
	    	features_[index].value = static_cast<double>(*p++);
	    	features_[index].index = index;
	    	index++;
	    }
	}

	// For libSVM prediction to find the end of the feature vector
	features_[index].index = -1;
}

void ClassifierSVM::predict_probability(const cv::Mat image, BodyPart* body_part) {
	create_feature_vector(image);

	static int labels[] = {1,2,3,4,5,0};

	double *prob_estimates = (double *) malloc(model_->nr_class*sizeof(double));
	double predict_label;

	predict_label = svm_predict_probability(model_, features_, prob_estimates);
	float max_probabilty = -1.f;
	int label = 0;

//	std::cerr << "Prediction: " << predict_label << std::endl;
	for(int j = 0; j< model_->nr_class; j++) {
//		std::cerr << "class " << j << ": " << prob_estimates[j] << std::endl;
		if (labels[j] == 0) continue;

		if (prob_estimates[j] > kConfidenceThreshold
				&& max_probabilty < prob_estimates[j]) {
			max_probabilty = prob_estimates[j];
			label = labels[j];
		}
	}

	*body_part = static_cast<BodyPart>(label);
	//std::cerr << std::endl;
//	*confidence = (label != 0) ? max_probabilty : prob_estimates[0];
}


}

