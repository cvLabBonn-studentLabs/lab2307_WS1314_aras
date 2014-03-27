/*
 * classifer_svm.cpp
 *
 *  Created on: Mar 20, 2014
 *      Author: neek
 */


#include "classifier_svm.h"

namespace classifier {

ClassifierSVM::ClassifierSVM(int num_attr) {

	features_ = (struct svm_node *) malloc(num_attr * sizeof(struct svm_node));

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

void ClassifierSVM::predict_probability(const cv::Mat image, BodyPart* body_part, float* confidence) {
	create_feature_vector(image);

	double *prob_estimates = (double *) malloc(model_->nr_class*sizeof(double));
	double predict_label;

	predict_label = svm_predict_probability(model_, features_, prob_estimates);
	float max_probabilty;
	for(int j = 0; j< model_->nr_class; j++) {
		if (max_probabilty < prob_estimates[j])
			max_probabilty = prob_estimates[j];
	}

	*body_part = static_cast<BodyPart>(predict_label);
	*confidence = max_probabilty;
}


}

