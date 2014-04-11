/*
 * extract_keypoints.cpp
 *
 *  Created on: Mar 30, 2014
 *      Author: neek
 */

#include "data_io.h"

static std::vector<cv::Point> keypoints;
static const std::string window_title = "Annotation Tool";
static cv::Mat depth_image;
static int frame_step = 3;

static void onMouse( int event, int x, int y, int, void* )
{
	if (event == cv::EVENT_LBUTTONDOWN) {
		std::cout << "Event " << event << " x: " << x << " y: " << y << std::endl;
		keypoints.push_back(cv::Point(x, y));

		cv::circle(depth_image, cv::Point(x, y), 3.0, 255, -1, 8);
		cv::imshow(window_title, depth_image);
	} else if (event == cv::EVENT_RBUTTONDOWN) {
		std::cout << "Event " << event << " x: " << 0 << " y: " << 0 << std::endl;
		keypoints.push_back(cv::Point(x, y));
	}
}

void annotate(std::ostream& ofile, io::DataIO& data) {
	cv::namedWindow(window_title, cv::WINDOW_AUTOSIZE);
	cv::setMouseCallback(window_title, onMouse, 0);

	int step = frame_step;
	while(data.read(depth_image)) {
		if (step > 1) {
			step--;
			continue;
		}

		cv::imshow(window_title, depth_image);
		int c = cv::waitKey(0);

		if( (c & 255) == 27 )
		{
			std::cout << "Exiting ...\n";
			break;
		}

		for (int j = 0; j < frame_step; j++) {
			std::cout << "Saving " << keypoints.size() << " points." << std::endl;
			for (int i = 0; i < keypoints.size(); i++) {
				ofile << keypoints[i].x << " " << keypoints[i].y << " ";

			}
			ofile << "\n";
			step++;
		}

		while (keypoints.size() > 0) {
			keypoints.erase(keypoints.begin());
		}

		step--;
	}
}


int main(int argc, char * argv[]) {

	io::DataIO data(io::StanfordEval);
	std::string dir;
	if (argc > 1) {
		std::string dir = argv[1];
		std::cout << "Saving to " << dir << std::endl;

		// Creating directory structure
		boost::filesystem::path training_data_path_head(dir);
		boost::filesystem::create_directory(dir);

		std::ofstream ofile((dir + "/labels.txt").c_str());

		annotate(ofile, data);

		ofile.close();
	} else {
		std::cout << "Extracting training data" << std::endl;
		data.extract_data();
		std::cout << "Done" << std::endl;
	}

	return 0;
}
