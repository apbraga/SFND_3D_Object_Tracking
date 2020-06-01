
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains

void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
   
    //Add all matches on current frame to a given bounding box if is enclosed
    
   double accumulator = 0;
   int counter = 0;
   for(auto match : kptMatches){
       if(boundingBox.roi.contains(kptsCurr[match.trainIdx].pt)){
           boundingBox.kptMatches.push_back(match);
           counter++;
           accumulator += cv::norm(kptsCurr[match.trainIdx].pt - kptsPrev[match.queryIdx].pt);
       }
   }
   double mean_distance = accumulator/counter;
   boundingBox.kptMatches.erase(std::remove_if(boundingBox.kptMatches.begin(),
          boundingBox.kptMatches.end(),
          [&mean_distance](cv::DMatch& match) { return match.distance < 0.5 * mean_distance; }),
      boundingBox.kptMatches.end());
   
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    /*
    Loops over all matches for the current and previous frame and calculate all the distances between points and their ratios, use media ratio to calculate TTC
    */
   std::vector<double> ratios;
   for(auto match_out: kptMatches) {
        cv::KeyPoint prevFrame_point_out = kptsPrev[match_out.queryIdx];
        cv::KeyPoint currFrame_point_out = kptsCurr[match_out.trainIdx];
        for(auto match_in : kptMatches) {
            cv::KeyPoint prevFrame_point_in = kptsPrev[match_in.queryIdx];
            cv::KeyPoint currFrame_point_in = kptsCurr[match_in.trainIdx];

            vector<double> distances = {cv::norm(currFrame_point_out.pt-currFrame_point_in.pt), cv::norm(prevFrame_point_out.pt-prevFrame_point_in.pt)};

            if(distances[1] > std::numeric_limits<double>::epsilon() && distances[0] >= 100.0){
                ratios.push_back(distances[0]/distances[1]);
            }
        }
   }

    if(ratios.size() == 0){
        TTC = NAN;
        return;
    }

   std::sort(ratios.begin(), ratios.end());
   //std::cout << "test:" << ratios.size() <<std::endl;
   TTC = (-1.0 / frameRate) / ( 1 - ratios[ratios.size()/2]);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    /*
    sort prev and curr lidar points based on their x component and use the median to calculate the TTC
    */
   
   std::sort(lidarPointsPrev.begin(), lidarPointsPrev.end(), [](LidarPoint a, LidarPoint b) {return a.x < b.x;});
   std::sort(lidarPointsCurr.begin(), lidarPointsCurr.end(), [](LidarPoint a, LidarPoint b) {return a.x < b.x;});
   TTC = lidarPointsCurr[lidarPointsCurr.size()/2].x*(1.0/frameRate)/(lidarPointsPrev[lidarPointsPrev.size()/2].x - lidarPointsCurr[lidarPointsCurr.size()/2].x);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    /*
    Loops over all matches to check in which bounding box its cointained on prev and current images
    */
    
    std::vector<std::vector<int>> pt_box_match(prevFrame.boundingBoxes.size(), std::vector<int>(currFrame.boundingBoxes.size(),0));

    for(auto match: matches) {
        cv::KeyPoint prevFrame_point = prevFrame.keypoints[match.queryIdx];
        cv::KeyPoint currFrame_point = currFrame.keypoints[match.trainIdx];
        std::vector<int> prev_boxes_ids, curr_boxes_ids;
        
        for(auto box : prevFrame.boundingBoxes){
            if(box.roi.contains(prevFrame_point.pt)){
                prev_boxes_ids.push_back(box.boxID);
            }

        }

        for(auto box : currFrame.boundingBoxes){
            if(box.roi.contains(currFrame_point.pt)){
                curr_boxes_ids.push_back(box.boxID);
            }
            
        }

        if((prev_boxes_ids.size() > 0)  &&  (curr_boxes_ids.size() > 0)){
            for(auto prev_id : prev_boxes_ids){
                for(auto curr_id : curr_boxes_ids){
                    pt_box_match[prev_id][curr_id] += 1 ;
                }
            }  
        }
    }
    /*
    Rank up the combination of boxes with highest hits on prev and current
    set the top 1 as matching box 
    */
    for(auto prev_id : prevFrame.boundingBoxes){
        int count = 0;
        int match_id = 0;
        for(auto curr_id : currFrame.boundingBoxes){
            if(pt_box_match[prev_id.boxID][curr_id.boxID] > count){
                count = pt_box_match[prev_id.boxID][curr_id.boxID];
                match_id = curr_id.boxID;
            }
        bbBestMatches[prev_id.boxID] = match_id;
        }
    }
}
