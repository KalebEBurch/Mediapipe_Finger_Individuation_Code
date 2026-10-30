
# Instructions and workflow for using finger individuation pose estimation code

## Overview
This package can be used to measure finger individuation with digital videos. Python code is used to run Google Mediapipe pose estimation to track hand coordinates, and then Matlab code is used to clean the data, identify each finger tap repetition, and ultimately calculate individuation index.

## Required software
- Matlab (I used R2023b)
- Jupyter Notebook
- Python packages (time, mediapipe, cv2, numpy, os, pandas)

## Required scripts
### Mediapipe_TrackHandsAndBody.ipynb
This script is a Jupyter Notebook that uses Python code to run Google Mediapipe pose estimation on an input video. The script runs three-dimensional pose estimation for the body model and both hand models.

Inputs:
- path to your video file. The name of your video file will be used to define the variable [your_filename], which will be used to define output filenames.

Outputs:
- [your_filename]_tracked.avi: video file with overlaid pose estimation keypoints
- [your_filename]_body_outputs.csv: This file contains the 3D pose estimation coordinates (in camera image-scaled coordinates) for body keypoints.
- [your_filename]_hand_outputs.csv: This file contains the 3D pose estimation coordinates for both left and right hand keypoints.

### gapfillKPs.m
This function will correct keypoint labels and gap-fill keypoint coordinates for both the body and hand models. It accepts the .csv files and outputs a .mat file with scructures that contain keypoint coordinates for the body and both hand models.
- Coordinates are stored as nested structured arrays (structs) for the body, left hand, and right hand.
- Each subfield of the structure is a unique keypoint. The 3D coordinates for each keypoint are stored as doubles of size (frame count) x 3.

### finger_indiv_pose_pipelines.m
 This function applies pipelines for finger individuation analysis. It accepts a structure for hand keypoint coordinates created by the gapfillKPs function. It outputs a stucture with finger individuation results for each finger.

## Steps
1. Install necessary libraries to run pose estimation. Use the following commands in terminal:
    - To install OpenCV: `pip install opencv-python`
    - To install Google Mediapipe: `pip install mediapipe`         
1. Run pose estimation with Mediapipe in Jupyter using Mediapipe_TrackHandsAndBody.ipynb. This script will create the .csv file with 3D coordinates for the Mediapipe hand model.
1. Run the gapfillKPs.m function in Matlab to fill gaps for keypoint coordinates.
1. Run finger_indiv_pose_pipelines to run finger individuation pipelines on keypoint coordinates for the hand of interest (left or right) from hand keypoint structure
    - This function accepts a structure which contains hand keypoint data. The structure, loaded from XXX_correctedData.mat, will be called either "HAND_L" or "HAND_R"
    - This function returns a structure called "RESULTS". RESULTS contains sub-structures corresponding to each finger (index, middle, ring, and pinky). Each finger has individation index (II) results as well as other data.

## Video Recording Recommendations
- It is best to record videos from the **frontal plane** since Mediapipe tracks finger motions the best from this angle and individuation measurements from this angle exhibit smaller errors relative to motion capture.
- Tracking is best when the hand is upright with the palm facing directly towards the camera

## Example Data
An example video title "fingerIndiv.MOV" is include with this package. If this video is successfully processed, it will return two files, fingerIndiv_body_outputs.csv and fingerIndiv_hand_outputs.csv. Examples of these files are provided in the exampleResults folder. A data dictionary (data_dictionary.xlsx) is provided which defines all variables in these CSV files. If these CSV files are processed correctly with "finger_indiv_pose_pipelines.m", then it will create a RESULTS structure. An example of this structure is also stored in the exampleResults folder in "results.mat".

## Funding Sources
1. The Johns Hopkins University Claude D. Pepper Older Americans Independence Center, funded by the National Institute on Aging and National Institutes of Health (grant P30AG021334)
1. The Translational Aging Research Training Program (NIA Grant T32AG058527)
1. The Precise Center, funded by the NIH (grant P50HD118624)
