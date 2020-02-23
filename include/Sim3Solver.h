/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "KeyFrame.h"



namespace ORB_SLAM2
{

class Sim3Solver
{
public:

    /**
     * @brieif pKF1+pKF2+vpMatched12 provide 3d vs 3d correspondence 
     * @param pKF1
     * @param pKF2
     * @param vpMatched12
     * @param bFixScale: true: se3, false: sim3
     */
    Sim3Solver(KeyFrame* pKF1, KeyFrame* pKF2, const std::vector<MapPoint*> &vpMatched12, const bool bFixScale = true);

    /**
     * @brief 设置ransac参数
     * @param probability
     * @param minInliers
     * @param maxIterations
     */
    void SetRansacParameters(double probability = 0.99, int minInliers = 6 , int maxIterations = 300);

    cv::Mat find(std::vector<bool> &vbInliers12, int &nInliers);

    /**
     * @brief 核心函数
     * @param nIterations
     * @param bNoMore
     * @param vbInliers
     * @param nInliers
     * @return
     */
    cv::Mat iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);

    cv::Mat GetEstimatedRotation();
    cv::Mat GetEstimatedTranslation();
    float GetEstimatedScale();


protected:

    void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);

    void ComputeSim3(cv::Mat &P1, cv::Mat &P2);

    void CheckInliers();

    void Project(const std::vector<cv::Mat> &vP3Dw, std::vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K);
    void FromCameraToImage(const std::vector<cv::Mat> &vP3Dc, std::vector<cv::Mat> &vP2D, cv::Mat K);


protected:

    // KeyFrames and matches
    KeyFrame* mpKF1;
    KeyFrame* mpKF2;
    
    // 保存MapPoint在KF1和KF2相机系下的3d点坐标
    std::vector<cv::Mat> mvX3Dc1;
    std::vector<cv::Mat> mvX3Dc2;
    // KF1和KF2一一对应的3d vs 3d correspondences
    std::vector<MapPoint*> mvpMapPoints1;
    std::vector<MapPoint*> mvpMapPoints2;
    // correspondence的一种索引方式(vector索引是KF1中的MapPoint的序号，对应的MapPoint*是KF2中的MapPoint指针)
    std::vector<MapPoint*> mvpMatches12;
    // 保存correspondence在KF1上的索引
    std::vector<size_t> mvnIndices1;
    // correspondence在KF1和KF2的KeyPoint提取时所在level的scale^2(1.2^(2*octave))
    std::vector<size_t> mvSigmaSquare1;
    std::vector<size_t> mvSigmaSquare2;
    // 根据KeyPoint提取时所在的level设置ransac阈值
    std::vector<size_t> mvnMaxError1;
    std::vector<size_t> mvnMaxError2;

    // valid correspondence number
    int N;
    // correspondences number
    int mN1;

    // Current Estimation
    cv::Mat mR12i;
    cv::Mat mt12i;
    float ms12i;
    cv::Mat mT12i;
    cv::Mat mT21i;
    std::vector<bool> mvbInliersi;
    int mnInliersi;

    // Current Ransac State
    int mnIterations;
    std::vector<bool> mvbBestInliers;
    int mnBestInliers;
    cv::Mat mBestT12;
    cv::Mat mBestRotation;
    cv::Mat mBestTranslation;
    float mBestScale;

    // Scale is fixed to 1 in the stereo/RGBD case
    bool mbFixScale;

    // Indices for random selection
    std::vector<size_t> mvAllIndices;

    // Projections: correspondences在KF1和KF2上的图像坐标
    std::vector<cv::Mat> mvP1im1;
    std::vector<cv::Mat> mvP2im2;

    // RANSAC probability
    // 连续若干次随机采样，至少有一次采样的所有点都是inlier的概率
    double mRansacProb;

    // RANSAC min inliers
    // 人为设置的数据集中最少的内点数目(经验值)
    int mRansacMinInliers;

    // RANSAC max iterations
    // (综合人工设置的最大迭代次数和mRansacProb&mRansacMinInliers计算出的最大迭代次数)
    int mRansacMaxIts;

    // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
    float mTh;
    float mSigma2;

    // Calibration
    cv::Mat mK1;
    cv::Mat mK2;

};

} //namespace ORB_SLAM

#endif // SIM3SOLVER_H
