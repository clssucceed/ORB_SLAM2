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


#include "Sim3Solver.h"

#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>

#include "KeyFrame.h"
#include "ORBmatcher.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace ORB_SLAM2
{


Sim3Solver::Sim3Solver(KeyFrame *pKF1, KeyFrame *pKF2, const vector<MapPoint *> &vpMatched12, const bool bFixScale):
    mnIterations(0), mnBestInliers(0), mbFixScale(bFixScale)
{
    mpKF1 = pKF1;
    mpKF2 = pKF2;

    vector<MapPoint*> vpKeyFrameMP1 = pKF1->GetMapPointMatches();

    mN1 = vpMatched12.size();

    mvpMapPoints1.reserve(mN1);
    mvpMapPoints2.reserve(mN1);
    mvpMatches12 = vpMatched12;
    mvnIndices1.reserve(mN1);
    mvX3Dc1.reserve(mN1);
    mvX3Dc2.reserve(mN1);

    cv::Mat Rcw1 = pKF1->GetRotation();
    cv::Mat tcw1 = pKF1->GetTranslation();
    cv::Mat Rcw2 = pKF2->GetRotation();
    cv::Mat tcw2 = pKF2->GetTranslation();

    mvAllIndices.reserve(mN1);

    size_t idx=0;
    for(int i1=0; i1<mN1; i1++)
    {
        if(vpMatched12[i1])
        {
            // a 3d vs 3d correspondence
            MapPoint* pMP1 = vpKeyFrameMP1[i1];
            MapPoint* pMP2 = vpMatched12[i1];

            // validity check
            if(!pMP1)
                continue;

            if(pMP1->isBad() || pMP2->isBad())
                continue;

            // 做ICP，为什么要获取2d点的信息: inilier检查是通过重投影误差检查的
            // 为什么ICP的inlier检查使用重投影误差来做: 因为不同点的重投影误差可以通过同一个阈值来检查，
            // 使用反投的3d点之间的误差对于近处和远处点的阈值就不能够一样，具体还需要看下ComputeSim3提到的那篇论文
            // local feature id of this MapPoint in both kfs
            int indexKF1 = pMP1->GetIndexInKeyFrame(pKF1);
            int indexKF2 = pMP2->GetIndexInKeyFrame(pKF2);

            // validity check
            if(indexKF1<0 || indexKF2<0)
                continue;

            // KeyPoint of this MapPoint in both kfs
            const cv::KeyPoint &kp1 = pKF1->mvKeysUn[indexKF1];
            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[indexKF2];

            // 获取KeyPoint提取时所在level的scale^2(1.2^(2*octave))
            const float sigmaSquare1 = pKF1->mvLevelSigma2[kp1.octave];
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];

            // 1. 根据KeyPoint提取时所在的level设置ransac阈值
            // 2. 由于不同KF的KeyPoint所处的level不一样，所以一个pixel对应的3d尺度是不一样的,
            // 为了使得不同KF上的ransac阈值对应的3d空间尺度是一样的需要根据KeyPoint提取时所在的level设置ransac阈值.
            // 3. 2d图像投影都是在level0上的，所以如果假设KeyPoint在其检测level上的误差阈值是sqrt(9.210),
            // 则level0上的误差阈值则应该乘以sqrt(sigmaSquare)
            // 9.210这个神奇的数是哪里来的: 卡方分布k=2,P=0.01时的数值
            mvnMaxError1.push_back(9.210*sigmaSquare1);
            mvnMaxError2.push_back(9.210*sigmaSquare2);

            // 转存3d vs 3d correspondence的索引关系，方便下面的ransac算法使用
            mvpMapPoints1.push_back(pMP1);
            mvpMapPoints2.push_back(pMP2);
            // 保存correspondence在KF1上的索引
            mvnIndices1.push_back(i1);

            // 保存MapPoint在KF1和KF2相机系下的3d点坐标
            cv::Mat X3D1w = pMP1->GetWorldPos();
            mvX3Dc1.push_back(Rcw1*X3D1w+tcw1);

            cv::Mat X3D2w = pMP2->GetWorldPos();
            mvX3Dc2.push_back(Rcw2*X3D2w+tcw2);

            mvAllIndices.push_back(idx);
            idx++;
        }
    }

    mK1 = pKF1->mK;
    mK2 = pKF2->mK;

    // project point_in_camera into image
    FromCameraToImage(mvX3Dc1,mvP1im1,mK1);
    FromCameraToImage(mvX3Dc2,mvP2im2,mK2);

    SetRansacParameters();
}

/**
 * @brief 根据下面传入的参数设置ransac最大迭代次数
 * @param probability: 连续若干次随机采样，至少有一次采样的所有点都是inlier的概率.default: 0.99
 * @param minInliers: 人为设置的数据集中最少的内点数目(经验值).default: 6
 * @param maxIterations: 人工设置最大迭代次数. default: 300
 */
void Sim3Solver::SetRansacParameters(double probability, int minInliers, int maxIterations)
{
    // 连续若干次随机采样，至少有一次采样的所有点都是inlier的概率
    mRansacProb = probability;
    // 人为设置的数据集中最少的内点数目(经验值)
    mRansacMinInliers = minInliers;
    // 人工设置最大迭代次数
    mRansacMaxIts = maxIterations;    

    N = mvpMapPoints1.size(); // number of correspondences

    mvbInliersi.resize(N);

    // Adjust Parameters according to number of correspondences
    // 点集中内点比例的最小值
    float epsilon = (float)mRansacMinInliers/N;

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    int nIterations;

    // 根据probability和minInliers计算最大迭代次数
    // 计算原理大致如下:
    // 内点比例epsilon，每次采样需要采集3个点(因为ICP至少需要3个点)，则
    // 1. 一次采样都是内点的概率是epsilon^3，
    // 2. 一次采样存在外点的概率1-epsilon^3
    // 3. k次采样每一次都存在外点的概率(1-epsilon^3)^k
    // 4. k次采样至少有一次采样都是内点的概率1-(1-epsilon^3)^k
    // 如果k是最大迭代次数，则1-(1-epsilon^3)^k=p
    // --> k = log(1-p) / log(1-epsilon^3)
    if(mRansacMinInliers==N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(epsilon,3)));

    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

    mnIterations = 0;
}

// Ransac求解mvX3Dc1和mvX3Dc2之间Sim3，函数返回mvX3Dc2到mvX3Dc1的Sim3变换
cv::Mat Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;                        // 现在还没有达到最好的效果
    vbInliers = vector<bool>(mN1,false);    // 和最初传递给这个解算器的地图点向量是保持一致
    nInliers=0;                             // 存储迭代过程中得到的内点个数

    // 如果经过"处理"后的点的数目已经比那个要求的最小点的数据少了,那么就说明...我们已经没有更好的选择了
    if(N<mRansacMinInliers)
    {
        bNoMore = true;
        return cv::Mat();   // 表示求解失败
    }

    // 可以使用的点对的索引,为了避免重复使用
    vector<size_t> vAvailableIndices;

    // 来自于这两个帧的三对匹配点
    cv::Mat P3Dc1i(3,3,CV_32F);
    cv::Mat P3Dc2i(3,3,CV_32F);

    // 这个函数中迭代的次数
    int nCurrentIterations = 0;
    // 条件1: 还没有超过限制的最大迭代次数
    // 条件2: nCurrentIterations  还没有超过给定的迭代次数
    while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;// 这个函数中迭代的次数
        mnIterations++;      // 总的迭代次数，默认为最大为300

        vAvailableIndices = mvAllIndices;

        // Get min set of points
        // STEP 1：任意取三组点算Sim矩阵
        for(short i = 0; i < 3; ++i)
        {
            // DBoW3中的随机数生成函数
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            // P3Dc1i和P3Dc2i中点的排列顺序：
            // x1 x2 x3 ...
            // y1 y2 y3 ...
            // z1 z2 z3 ...
            mvX3Dc1[idx].copyTo(P3Dc1i.col(i));
            mvX3Dc2[idx].copyTo(P3Dc2i.col(i));

            // 从"可用索引列表"中删除这个点
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        // STEP 2：根据两组匹配的3D点，计算之间的Sim3变换
        ComputeSim3(P3Dc1i,P3Dc2i);

        // STEP 3：通过投影误差进行inlier检测
        CheckInliers();

        // 更新最多的内点数目
        if(mnInliersi>=mnBestInliers)
        {
            mvbBestInliers = mvbInliersi;
            mnBestInliers = mnInliersi;
            mBestT12 = mT12i.clone();
            mBestRotation = mR12i.clone();
            mBestTranslation = mt12i.clone();
            mBestScale = ms12i;

            if(mnInliersi>mRansacMinInliers)// 只要计算得到一次合格的Sim变换，就直接返回
            {
                // 返回值,告知得到的内点数目
                nInliers = mnInliersi;
                for(int i=0; i<N; i++)
                    if(mvbInliersi[i])
                        // 用这种方式将"处理后的地图点向量坐标"转换成为"处理前的地图点向量坐标"
                        vbInliers[mvnIndices1[i]] = true;
                return mBestT12;
            } // 如果当前次迭代已经合格了,直接返回
        } // 更新最多的内点数目
    } // 迭代循环

    // 如果已经达到了设计的最大迭代次数,就no more 了
    if(mnIterations>=mRansacMaxIts)
        bNoMore=true;

    return cv::Mat();   // no more的时候返回的是一个空矩阵
}

// 在"进行迭代计算"函数 iterate 的基础上套了一层壳,使用默认参数. 不过目前好像没有被使用到
cv::Mat Sim3Solver::find(vector<bool> &vbInliers12, int &nInliers)
{
    bool bFlag;
    return iterate(mRansacMaxIts,bFlag,vbInliers12,nInliers);
}

// 给出三个点,计算它们的质心以及去质心之后的坐标
void Sim3Solver::ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
{
    // 这两句可以使用CV_REDUCE_AVG选项来搞定
    cv::reduce(P,C,1,CV_REDUCE_SUM);// 矩阵P每一行求和
    C = C/P.cols;// 求平均

    for(int i=0; i<P.cols; i++)
    {
        Pr.col(i)=P.col(i)-C;//减去质心
    }
}

// 根据两组匹配的3D点,计算之间的Sim3变换
// 三对匹配点,每个点的坐标都是列向量形式,三个点组成了3x3的矩阵,三对点组成了两个3x3矩阵P1,P2
void Sim3Solver::ComputeSim3(cv::Mat &P1, cv::Mat &P2)
{
    // ！！！！！！！这段代码一定要看这篇论文！！！！！！！！！！！
    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

    // Step 1: Centroid and relative coordinates

    cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
    cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
    cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
    cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

    // O1和O2分别为P1和P2矩阵中3D点的质心
    // Pr1和Pr2为减去质心后的3D点
    ComputeCentroid(P1,Pr1,O1);
    ComputeCentroid(P2,Pr2,O2);

    // Step 2: Compute M matrix
    // 这里是按照"大于三组匹配点"中的M矩阵来计算的;形式和论文中略有不同,但是本质上是一样的
    cv::Mat M = Pr2*Pr1.t();

    // Step 3: Compute N matrix

    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

    cv::Mat N(4,4,P1.type());

    N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
    N12 = M.at<float>(1,2)-M.at<float>(2,1);
    N13 = M.at<float>(2,0)-M.at<float>(0,2);
    N14 = M.at<float>(0,1)-M.at<float>(1,0);
    N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
    N23 = M.at<float>(0,1)+M.at<float>(1,0);
    N24 = M.at<float>(2,0)+M.at<float>(0,2);
    N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
    N34 = M.at<float>(1,2)+M.at<float>(2,1);
    N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

    N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                 N12, N22, N23, N24,
                                 N13, N23, N33, N34,
                                 N14, N24, N34, N44);


    // Step 4: Eigenvector of the highest eigenvalue

    cv::Mat eval, evec;  // val vec

    cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

    /**
     * \n 补充一下这里计算四元数的时候用到的一些技巧:
     * \n 对于四元数 p=(p0,p1i,p2j,p3k), 其中的三个虚部和空间中的三个轴相对应:
     * \n \f$ p=\cos(\theta/2)+\mathbf{n}\sin(\theta/2) \f$
     * \n 可以非常方便地写成旋转向量的表达方式,上式中的旋转向量就是n;一般地旋转向量模的大小表示了旋转的角度theta的大小,但是这里的n只能够表示旋转的方向
     * \n so我们只需要得到了旋转向量即可恢复出欧拉角.其中 
     * \n \f$ \mathbf{n}\sin(\theta/2)
     * 
     */

    // N矩阵最大特征值（第一个特征值）对应特征向量就是要求的四元数（q0 q1 q2 q3） (第0行,其中q0就是我们日常说的w)
    // 将(q1 q2 q3)放入vec行向量，vec就是四元数旋转轴乘以sin(angle/2)
    cv::Mat vec(1,3,evec.type());
    (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

    // Rotation angle. sin is the norm of the imaginary part, cos is the real part
    // 这里是这样来的:将四元数转换成为复数的形式,计算虚部(模长)和实部的夹角
    // 这里的 norm(vec)=sin(theta/2), evec.at<float>(0,0)=q0=cos(theta/2)
    // ? 为什么不 arccos(w)=angle/2呢???? 
    double ang=atan2(norm(vec),evec.at<float>(0,0));

    // 虚部表示旋转向量,归一化得到归一化后的旋转向量,然后乘上角度得到包含了旋转轴和旋转角信息的旋转向量.
    vec = 2*ang*vec/norm(vec); //Angle-axis x. quaternion angle is the half

    mR12i.create(3,3,P1.type());

    cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis

    // Step 5: Rotate set 2
    // 要放到同一个坐标系下进行计算
    cv::Mat P3 = mR12i*Pr2;

    // Question: 为什么scale计算没有使用ppt中推荐的方法
    // Step 6: Scale

    if(!mbFixScale)
    {
        // 论文中还有一个求尺度的公式，p632右中的位置，那个公式不用考虑旋转
        // 这个公式对应着论文中p632左中位置的那个,r对应着Pr1,l对应着P3(经过坐标系转换的Pr2),剩下的就和论文中都一样了
        double nom = Pr1.dot(P3);
        // 准备计算分母
        cv::Mat aux_P3(P3.size(),P3.type());
        aux_P3=P3;
        // 先得到平方
        cv::pow(P3,2,aux_P3);
        double den = 0;

        // 然后再累加
        for(int i=0; i<aux_P3.rows; i++)
        {
            for(int j=0; j<aux_P3.cols; j++)
            {
                den+=aux_P3.at<float>(i,j);
            }
        }

        ms12i = nom/den;
    }
    else
        ms12i = 1.0f;

    // Step 7: Translation

    mt12i.create(1,3,P1.type());
    // 论文中公式
    mt12i = O1 - ms12i*mR12i*O2;

    // Step 8: Transformation
    // 计算双向的位姿变换,目的是在下面的检查的过程中能够进行双向的投影操作

    // Step 8.1 T12
    mT12i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sR = ms12i*mR12i;

    //         |sR t|
    // mT12i = | 0 1|
    sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
    mt12i.copyTo(mT12i.rowRange(0,3).col(3));

    // Step 8.2 T21

    mT21i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sRinv = (1.0/ms12i)*mR12i.t();

    sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
    cv::Mat tinv = -sRinv*mt12i;
    tinv.copyTo(mT21i.rowRange(0,3).col(3));
}

// 通过投影误差进行内点检测
void Sim3Solver::CheckInliers()
{
    vector<cv::Mat> vP1im2, vP2im1;
    Project(mvX3Dc2,vP2im1,mT12i,mK1);// 把2系中的3D经过Sim3变换(mT12i)到1系中计算重投影坐标
    Project(mvX3Dc1,vP1im2,mT21i,mK2);// 把1系中的3D经过Sim3变换(mT21i)到2系中计算重投影坐标

    mnInliersi=0;

    // 对于两帧的每一个匹配点
    for(size_t i=0; i<mvP1im1.size(); i++)
    {
        // 对于这对匹配关系,在两帧上的投影点距离都要进行计算
        cv::Mat dist1 = mvP1im1[i]-vP2im1[i];
        cv::Mat dist2 = vP1im2[i]-mvP2im2[i];

        // 取距离的平方作为误差
        const float err1 = dist1.dot(dist1);
        const float err2 = dist2.dot(dist2);

        // 根据之前确定的这个最大容许误差来确定这对匹配点是否是外点
        if(err1<mvnMaxError1[i] && err2<mvnMaxError2[i])
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
            mvbInliersi[i]=false;
    }// 遍历其中的每一对匹配点
}

// 得到计算的旋转矩阵
cv::Mat Sim3Solver::GetEstimatedRotation()
{
    return mBestRotation.clone();
}

// 得到计算的平移向量
cv::Mat Sim3Solver::GetEstimatedTranslation()
{
    return mBestTranslation.clone();
}
// 得到估计的从候选帧到当前帧的变换尺度
float Sim3Solver::GetEstimatedScale()
{
    return mBestScale;
}

// 按照给定的Sim3变换进行投影操作,得到三维点的2D投影点
void Sim3Solver::Project(const vector<cv::Mat> &vP3Dw, vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K)
{
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dw.size());

    // 对每个3D地图点进行投影操作
    for(size_t i=0, iend=vP3Dw.size(); i<iend; i++)
    {
        // 首先将对方关键帧的地图点坐标转换到这个关键帧的相机坐标系下
        cv::Mat P3Dc = Rcw*vP3Dw[i]+tcw;
        // 投影
        const float invz = 1/(P3Dc.at<float>(2));
        const float x = P3Dc.at<float>(0)*invz;
        const float y = P3Dc.at<float>(1)*invz;

        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
    }
}

// 将相机系下的3d点投影到图像上
void Sim3Solver::FromCameraToImage(const vector<cv::Mat> &vP3Dc, vector<cv::Mat> &vP2D, cv::Mat K)
{
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dc.size());

    for(size_t i=0, iend=vP3Dc.size(); i<iend; i++)
    {
        const float invz = 1/(vP3Dc[i].at<float>(2));
        const float x = vP3Dc[i].at<float>(0)*invz;
        const float y = vP3Dc[i].at<float>(1)*invz;

        vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
    }
}

} //namespace ORB_SLAM
