Author:  Juergen Gall
Contact: gall@vision.ee.ethz.ch
Date: 18.05.2011
Computer Vision Laboratory, ETH Zurich

All data is only for research purposes. When using this data, please acknowledge the effort that went into data collection by referencing the corresponding paper:

Gall J., Fossati A., and van Gool L., Functional Categorization of Objects using Real-time Markerless Motion Capture, IEEE Conference on Computer Vision and Pattern Recognition (CVPR'11), 1969-1976, 2011. 

##########################################################################

Project is a simple visualization tool of the estimated poses. It shows how to read calibration data and pose data. The binary is compiled for Linux 64bit but the source code is also provided. Usage:
./Project config_example.txt outpath
outpath - path to store pose/joints projected to the amplitude image and the rgb image (check_XXXXX.png or checkRGB_XXXXX.png)
config_example.txt - config file

The config file has the following entries:
# file with image file names 
dataset_function_public/table-object11/data/s009.bmf
# path of images
dataset_function_public/table-object11
# start frame
55
# end frame
335
# calibration file for TOF
dataset_function_public/table-object11/data/calib_TOF.txt
# calibration file for RGB
calib_RGB.txt
# human model
dataset_function_public/table-object11/data/model.dat
# path where pose are estimated
table-object11/out009_1

For visualizing a sequence of a subject, one just has to fill in the corresponding data stored in the Matlab file SequenceParameters.mat. It contains the data for all 6 subjects as matrices (params_1,...,params_6) where each row contains four values. The first two are the start and end frame, the next value is for 'table-objectXX' and the last for 'sYYY.bmf' and 'outYYY_1'. The above example is taken from: 

>> params_1(10,:)
ans =
    55   335    11     9


##########################################################################

Juergen Gall
