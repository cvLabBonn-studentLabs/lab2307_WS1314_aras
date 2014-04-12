/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Computer Science Institute III, University of Bonn
 *  Author: Nikita Araslanov, 20.03.2014
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Bonn, Computer Science Institute
 *     III nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
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

	for(int j = 0; j< model_->nr_class; j++) {
		if (labels[j] == 0) continue;

		if (prob_estimates[j] > kConfidenceThreshold
				&& max_probabilty < prob_estimates[j]) {
			max_probabilty = prob_estimates[j];
			label = labels[j];
		}
	}

	*body_part = static_cast<BodyPart>(label);
}


}

