/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Computer Science Institute III, University of Bonn
 *  Author: Nikita Araslanov, 12.04.2014
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


#include "data_io.h"
#include "mesher.h"
#include "constants.h"
#include "keyframe.h"
#include "classifier_svm.h"
#include "timer.h"
#include "opflow.h"
#include "opflow_tensor.h"
#include "ldof.h"
#include "CFilter.h"

//#define BENCHMARK 1
//#define DEBUG 1

/////////////////////////////// Constants and Globals /////////////////////////

const float infinity = 99999.9f;
static cv::Mat depth_image;
static mesher::Mesh mesh(pose::kMeshDistanceThreshold, pose::kNumInterestPoints);

// Setting the frame range
const int start_from_frame = 260;	// For StanfordEval: 1700
const int stop_at_frame = 600;		// For StanfordEval: 2400

const int num_attr = pose::kDescriptorSize*pose::kDescriptorSize;
const int num_classes = 5;

///////////////////////////////////////////////////////////////////////////////

#ifdef BENCHMARK
timer::Timer t;
#endif

// FIXME: consider removal

void mark_body_part(cv::Mat& image, cv::Point centre, classifier::BodyPart body_part) {

//	std::cout << "Marking BP: " << body_part << " : " << centre << std::endl;

	cv::Scalar colour;
	bool negative = false;

	switch(body_part) {
	case classifier::HEAD:
		colour = cv::Scalar(0, 0, 255);
		break;
	case classifier::LEFT_HAND:
	case classifier::RIGHT_HAND:
		colour = cv::Scalar(0, 255, 0);
		break;
	case classifier::LEFT_FOOT:
	case classifier::RIGHT_FOOT:
		colour = cv::Scalar(255, 0, 0);
		break;
	case classifier::ZERO:
		negative = true;
		//colour = cv::Scalar(0, 255, 255);
		break;
	default:
		negative = true;
		break;
	}

	if (!negative) {
		cv::circle(image,
					centre,
					 3.0,
					 colour,
					 -1,
					 8);
	}
}

#ifdef DEBUG
static void onMouse( int event, int x, int y, int, void* )
{
    if( event != cv::EVENT_LBUTTONDOWN )
        return;

    std::cout << "(" << x << ", " << y << ")" << "= " << depth_image.at<float>(y, x) << std::endl;
}
#endif



float l2norm(cv::Point a, cv::Point b) {
	return sqrt((a.x - b.x)*(a.x - b.x)
					+ (a.y - b.y)*(a.y - b.y)
					+ (depth_image.at<float>(a.y, a.x) - depth_image.at<float>(b.y, b.x))*(depth_image.at<float>(a.y, a.x) - depth_image.at<float>(b.y, b.x)));
}
float precision(float tp, float fp) {
	return tp/(tp + fp);
}

float recall(float tp, float fn) {
	return tp/(tp + fn);
}

int compute() {
	int tps[] = {0, 0, 0, 0, 0};
	int fps[] = {0, 0, 0, 0, 0};
	int fns[] = {0, 0, 0, 0, 0};

	int confusion_matrix[num_classes][num_classes];
	for (int i = 0; i < num_classes; i++)
		for (int j = 0; j < num_classes; j++)
			confusion_matrix[i][j] = 0;

	std::vector<std::vector<float> > rms;
	for (int i = 0; i < num_classes; i++) {
		rms.push_back(std::vector<float>());
	}

	io::DataIO data(io::kCurrentDataset);
	classifier::ClassifierSVM svm(num_attr);
	svm.read_model("data/bonn/41x41/data.model"); // For StanfordEval: "data/stanford/training/data.model"

	opflow::OFTensor<float> aImage1, aImage2;
	opflow::OFTensor<float> aForward, aBackward;

	cv::Mat depth_image_prev, rgb_image;

	int frame_idx = 0;
	while(true) {

		/********** Reading in new data ***********/
#ifdef BENCHMARK
t.start();
#endif
		bool is_successful = data.read(depth_image);
#ifdef BENCHMARK
t.stop();
std::cout << "(BENCHMARKING) Reading data: " << t.duration() << "ms" << std::endl;
#endif
		/******************************************/

		if (!is_successful) break;

		if (frame_idx < start_from_frame) {
			frame_idx++;
			continue;
		}

		if (frame_idx > stop_at_frame) {
			break;
		}

		bool detected[] = {false, false, false, false, false};
		float min_dist[] = {99999.9f, 99999.9f, 99999.9f, 99999.9f, 99999.9f};

		/************ Ground truth: for evaluation purposes **************/
		cv::Point centre_head_gt,
					centre_lhand_gt, centre_rhand_gt,
					centre_lfoot_gt, centre_rfoot_gt;


		data.get_ground_truth(centre_head_gt, centre_lhand_gt, centre_rhand_gt,
								centre_lfoot_gt, centre_rfoot_gt);

		std::vector<cv::Point> centres_gt;
		centres_gt.push_back(centre_head_gt);
		centres_gt.push_back(centre_lhand_gt);
		centres_gt.push_back(centre_rhand_gt);
		centres_gt.push_back(centre_lfoot_gt);
		centres_gt.push_back(centre_rfoot_gt);

		int half_descriptor = pose::kDescriptorSize/2;
		std::vector<cv::Point> toplefts;
		for (std::vector<cv::Point>::iterator it = centres_gt.begin();
				it != centres_gt.end(); it++) {
			cv::Point topleft(it->x - half_descriptor,
								it->y - half_descriptor);

			toplefts.push_back(topleft);
		}
		/******************************************************************/

		// To remove: converting the image for display purposes
		cv::Mat depth_image_flow, depth_image_display;
		cv::normalize(depth_image, depth_image_flow, 0.0, 255.0, cv::NORM_MINMAX);
		depth_image_flow.convertTo(depth_image_display, CV_8U);
		cv::cvtColor(depth_image_display, depth_image_display, CV_GRAY2RGB);

		/************** Optical flow **************/
		cv::Mat image_of(depth_image.size(), CV_32F, 0.f);

		if (opflow::ON) {
			aImage2.readFromMat(depth_image_flow);

			NFilter::recursiveSmoothX(aImage2,0.8f);
			NFilter::recursiveSmoothY(aImage2,0.8f);

			if (frame_idx > start_from_frame) {
#ifdef BENCHMARK
t.start();
#endif
ldof(aImage1, aImage2, aForward, aBackward);
#ifdef BENCHMARK
t.stop();
std::cout << "(BENCHMARKING) Optical Flow: " << t.duration() << "ms" << std::endl;
#endif
				cv::Mat image_f(cv::Size(aForward.xSize(), aForward.ySize()), CV_32F);
				cv::Mat image_b(cv::Size(aBackward.xSize(), aBackward.ySize()), CV_32F);

				aForward.writeToMat(image_f);
				aBackward.writeToMat(image_b);
				opflow::merge_flows(image_f, image_b, image_of);
			}

			aImage1 = aImage2;
#ifdef DEBUG
			cv::imshow("Optical flow", image_of);
#endif
		}
		/******************************************/


		/************* Creating a mesh ************/
#ifdef BENCHMARK
t.start();
#endif
 	 	mesh = mesher::Mesh(pose::kMeshDistanceThreshold, pose::kNumInterestPoints);
		mesh.compute(depth_image, data.getScaleZ(), image_of);
#ifdef BENCHMARK
t.stop();
std::cout << "(BENCHMARKING) Creating a mesh: " << t.duration() << "ms" << std::endl;
#endif
		/******************************************/

#ifdef DEBUG
		cv::Mat segments;
		depth_image_display.convertTo(segments, CV_8UC3);
		mesh.colour_mat(segments);
#endif

		std::vector<cv::Point> keypoints, key_orientations;
		mesh.get_interest_points(keypoints, key_orientations);

#ifdef DEBUG
		// Marking the centroids
		mesh.mark_centroids(segments, 1.0);
#endif

		keyframe::Keyframe kframe(depth_image);
		for (int i = 0; i < keypoints.size(); i++) {
			cv::Vec4i orientation;
			cv::Point centre;
			cv::Mat patch;

			centre.x = keypoints[i].x;
			centre.y = keypoints[i].y;

			// Apply crude outlier filter for image boundaries
			if (centre.x < 5 || centre.x > depth_image.cols - 5
					|| centre.y < 3 || centre.y > depth_image.rows - 3) continue;

			orientation[0] = key_orientations[i].x;
			orientation[1] = key_orientations[i].y;
			orientation[2] = keypoints[i].x;
			orientation[3] = keypoints[i].y;

			if (!kframe.extract_part(patch, centre, orientation, pose::kDescriptorSize)) {
				// Patch extraction was unsuccessful
				continue;
			}

			classifier::BodyPart body_part;

			/************* SVM prediction *************/
#ifdef BENCHMARK
t.start();
#endif
			svm.predict_probability(patch, &body_part);
#ifdef BENCHMARK
t.stop();
std::cout << "(BENCHMARKING) SVM prediction: " << t.duration() << "ms" << std::endl;
#endif
			/******************************************/

			mark_body_part(depth_image_display, centre, body_part);

#ifdef DEBUG
			cv::imshow("Interest Point", patch);
			// Mark orienations if needed
			// mark_body_part(depth_image_display, cv::Point(key_orientations[i].x, key_orientations[i].y), body_part);

			cv::waitKey(0);
#endif

			cv::Point topleft;
			topleft.x = centre.x - half_descriptor;
			topleft.y = centre.y - half_descriptor;

			int index = body_part - 1;
			if (index > -1) {
				std::string bp_name(classifier::BodyPartName[index]);

				if (centres_gt[index].x > 0) {
					float dist = l2norm(centre, centres_gt[index]);

					min_dist[index] = std::min(min_dist[index], dist);
					if (io::DataIOBackend::intersection_ratio(topleft, toplefts[index]) > 0.5 && !detected[index]) {
						// true positive
						tps[index]++;
						detected[index] = true;
					} else {
						fps[index]++;

						for (int j = 0; j < num_classes; j++) {
							if (j == index) continue;
							if (io::DataIOBackend::intersection_ratio(topleft, toplefts[j]) > 0.5) {
								confusion_matrix[index][j]++;
								break;
							}
						}
					}
				}
			}
		}

		for (int j = 0; j < num_classes; j++) {
			if (centres_gt[j].x > 0 && !detected[j]) {
				fns[j]++;
			} else if (detected[j]) {
				rms[j].push_back(min_dist[j]);
			}
		}

		// Displaying the frames
		cv::imshow("Body parts", depth_image_display);
		cv::waitKey(1);

#ifdef DEBUG
		cv::imshow("Segments", segments);
		cv::setMouseCallback( "Segments", onMouse, 0 );
		cv::normalize(image_of, image_of, 0, 128, cv::NORM_MINMAX);
		image_of.convertTo(image_of, CV_8UC1);

		// Writing the files if needed
		// cv::imwrite("body_part.png", depth_image_display);
		// cv::imwrite("segments.png", segments);
		// cv::imwrite("of.png", image_of);
		cv::waitKey(0);
#endif
		depth_image_prev = depth_image;
		frame_idx++;
	}

	////////////////////////// Printing the results ///////////////////////////
	std::cout << "Total number of frames: " << frame_idx - start_from_frame << std::endl << std::endl;

	for (int i = 0; i < num_classes; i++) {
		std::cout << " * " << classifier::BodyPartName[i] << " * " << std::endl;
		std::cout << "RMS (" << rms[i].size() << ")" << std::endl;
		for (int j = 0; j < rms[i].size(); j++) std::cout << rms[i][j] << ", ";
		std::cout << std::endl;
		std::cout << "TP = " << tps[i] << " FP = " << fps[i] << " FN = " << fns[i] << std::endl;
		std::cout << "Precision: " << precision(tps[i], fps[i]) << std::endl;
		std::cout << "Recall: " << recall(tps[i], fns[i]) << std::endl << std::endl;
	}

	// HANDS
	int hands_tp = tps[1] + tps[2] + confusion_matrix[1][2] + confusion_matrix[2][1];
	int hands_fn = fns[1] + fns[2];
	int hands_fp = fps[1] + fps[2] - confusion_matrix[1][2] - confusion_matrix[2][1];
	std::cout << " * HANDS * " << std::endl;
	std::cout << "TP = " << hands_tp << " FP = " << hands_fp   << " FN = " << hands_fn << std::endl;
	std::cout << "Precision: " << precision(hands_tp, hands_fp) << std::endl;
	std::cout << "Recall: " << recall(hands_tp, hands_fn) << std::endl << std::endl;


	// FEET
	int feet_tp = tps[3] + tps[4] + confusion_matrix[3][4] + confusion_matrix[4][3];
	int feet_fn = fns[3] + fns[4];
	int feet_fp = fps[3] + fps[4] - confusion_matrix[3][4] - confusion_matrix[4][3];
	std::cout << " * FEET * " << std::endl;
	std::cout << "TP = " << feet_tp << " FP = " << feet_fp << " FN = " << feet_fn << std::endl;
	std::cout << "Precision: " << precision(feet_tp, feet_fp) << std::endl;
	std::cout << "Recall: " << recall(feet_tp, feet_fn) << std::endl << std::endl;

	std::cout << "Confusion matrix: " << std::endl;
	for (int i = 0; i < num_classes; i++) {
		for (int j = 0; j < num_classes; j++) {
			std::cout << confusion_matrix[i][j] << " ";
		}

		std::cout << std::endl;
	}
	std::cout << std::endl;

	return 0;
}

int main(int argc, char * argv[]) {
	for (float conf = 0.45; conf < 1.f; conf += .05f) {
		classifier::kConfidenceThreshold = conf;
		std::cout << "*** Classifier confidence value: " << conf << " ***" << std::endl;
		compute();
		std::cout << std::endl << std::endl;
	}
}
