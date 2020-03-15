#include "two_view_geometry.h"
#include <algorithm>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace TwoViewGeometry {

void FindFundamental(const vector<cv::Point2f> &vPts1,
                     const vector<cv::Point2f> &vPts2,
                     const vector<cv::DMatch> &vMatches12,
                     vector<bool> &vbMatchesInliers, cv::Mat &F21, float &score,
                     float sigma, int maxIterations) {
  // TODO homework
}

bool ReconstructF(const vector<cv::Point2f> &vPts1,
                  const vector<cv::Point2f> &vPts2,
                  const vector<cv::DMatch> &vMatches12,
                  vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                  cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D,
                  vector<bool> &vbTriangulated, float minParallax,
                  int minTriangulated, float sigma) {
  // TODO homework
  return false;
}

}