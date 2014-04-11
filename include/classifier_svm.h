/*
 * classifier_svm.h
 *
 *  Created on: Mar 20, 2014
 *      Author: neek
 */

#ifndef CLASSIFIER_SVM_H_
#define CLASSIFIER_SVM_H_

#include <opencv2/opencv.hpp>
#include "svm.h"
#include "constants.h"


namespace classifier {

enum BodyPart { ZERO, HEAD, LEFT_HAND, RIGHT_HAND, LEFT_FOOT, RIGHT_FOOT };
extern char const * BodyPartName[];

class ClassifierSVM {
public:
	ClassifierSVM(int num_attr);
	~ClassifierSVM();
	bool read_model(const std::string file);
	int predict(const cv::Mat image);
	void predict_probability(const cv::Mat image, BodyPart *body_part);
private:
	void create_feature_vector(const cv::Mat image);
	struct svm_model* model_;
	struct svm_node* features_;
};

}


#endif /* CLASSIFIER_SVM_H_ */
