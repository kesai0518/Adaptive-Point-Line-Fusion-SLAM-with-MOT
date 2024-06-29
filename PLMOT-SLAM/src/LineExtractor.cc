/**
* This file is part of ORB-LINE-SLAM
*
* Copyright (C) 2020-2021 John Alamanos, National Technical University of Athens.
* Copyright (C) 2016-2018, Ruben Gomez-Ojeda, University of Malaga.
* Copyright (C) 2016-2018, David Zuñiga-Noël, University of Malaga.         
* Copyright (C) 2016-2018, MAPIR group, University of Malaga.    
*
* ORB-LINE-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-LINE-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-LINE-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <chrono>

#include "LineExtractor.h"

using namespace cv;
using namespace line_descriptor;
using namespace std;

namespace ORB_SLAM3
{

Lineextractor::Lineextractor(int _lsd_nfeatures, int _lsd_refine, float _lsd_scale, int _nlevels, float _scale, int _extractor)
    :lsd_nfeatures(_lsd_nfeatures), lsd_refine(_lsd_refine), lsd_scale(_lsd_scale), nlevels(_nlevels), scale(_scale), extractor(_extractor)
{

}

void Lineextractor::operator()( const cv::Mat& img, const cv::Mat& mask, 
            std::vector<cv::line_descriptor::KeyLine>& keylines, cv::Mat& descriptors_line, const std::vector<string> &labels, const std::vector<uint8_t> &masks)
{
    // Line Length Threshold
    min_line_length = 0.025;
    Mat image = img.clone();
    if(extractor==0) // LSD Extractor
    {
        // Detect line features
        Ptr<BinaryDescriptor>   lbd = BinaryDescriptor::createBinaryDescriptor();
        Ptr<line_descriptor::LSDDetectorC> lsd = line_descriptor::LSDDetectorC::createLSDDetectorC();
        keylines.clear();
        // lsd parameters
        lsd_sigma_scale = 0.6;
        lsd_quant = 2.0;
        lsd_ang_th = 22.5;
        lsd_log_eps = 1.0;
        lsd_density_th = 0.6;
        lsd_n_bins = 1024;
        line_descriptor::LSDDetectorC::LSDOptions opts;
        opts.refine       = lsd_refine;
        opts.scale        = lsd_scale;
        opts.sigma_scale  = lsd_sigma_scale;
        opts.quant        = lsd_quant;
        opts.ang_th       = lsd_ang_th;
        opts.log_eps      = lsd_log_eps;
        opts.density_th   = lsd_density_th;
        opts.n_bins       = lsd_n_bins;
        opts.min_length   = min_line_length*(std::min(img.cols,img.rows));
//        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        lsd->detect( img, keylines, scale, nlevels, opts);
//        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//        double LineExtratrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
//        cout<<"Real Extracting Lines use : "<<LineExtratrack<<" s"<<endl;
        // filter keyline


        //TODO 新增内容，目的是剔除环境中的动态特征点
        Mat mskMat;
        int nums = labels.size();
        int height = image.size().height;
        int width = image.size().width;
        Mat dst = Mat(image.size(),CV_8U,Scalar(0,0,0));
        for (int i = 0; i <nums ; ++i)
        {
            if(labels[i] == "person")
            {
                vector<uint8_t >::const_iterator first1 = masks.begin()+i*height*width;
                vector<uint8_t >::const_iterator first2 = masks.begin()+(i+1)*height*width;
                vector<uint8_t > mask(first1,first2);
                Mat msk = Mat(mask);
                Mat MSK = msk.reshape(1,height);
                Mat dst_temp = Mat(image.size(),CV_8U,Scalar(0,0,0));
                for (int j = 0; j <height ; ++j)
                {
                    for (int k = 0; k <width ; ++k)
                    {
                        if(MSK.at<uint8_t>(j,k) == 1)
                        {
                            dst_temp.at<uchar>(j,k) = 255;
                        }
                    }
                }
                dst += dst_temp;
            }

        }

        vector<vector<Point>> contours;
        findContours(dst,contours,RETR_EXTERNAL,CHAIN_APPROX_NONE);


        //cout<<"before removal -------------"<<keylines.size()<<" 个"<<endl;
        //vector<cv::line_descriptor::KeyLine> &KeyLines = keylines;
        for(vector<cv::line_descriptor::KeyLine>::iterator keyline = keylines.begin(), keylineEnd = keylines.end(); keyline != keylineEnd; ++keyline)
        {
            cv::Point lineS = keyline->getStartPoint();
            cv::Point lineE = keyline->getEndPoint();
            for(int i = 0; i<contours.size(); ++i)
            {
                double dstS = pointPolygonTest(contours[i], lineS, true);
                double dstE = pointPolygonTest(contours[i], lineE, true);
                if(dstS >=-15 || dstE >= -15)
                {
                    //cout<<"-------！！！！"<<endl;
                    keylines.erase(keyline);
                    keyline--;
                    keylineEnd--;
                    break;
                }
            }
        }

        //cout<<"after removal -------------------"<<keylines.size()<<" 个"<<endl;
        if( int(keylines.size())>lsd_nfeatures && lsd_nfeatures!=0  )
        {
            //cout<<"filter keylines ~~~~~~~~"<<endl;
            // sort keylines by their response or by their length
            sort( keylines.begin(), keylines.end(), sort_lines_by_response() );
            //sort( keylines.begin(), keylines.end(), sort_lines_by_length() );
            keylines.resize(lsd_nfeatures);
            // reassign index
            for( int i = 0; i < lsd_nfeatures; i++  )
                keylines[i].class_id = i;
        }

        nlevels_l=nlevels;
        mvLevelSigma2_l.resize(nlevels);
        mvLevelSigma2_l[0]=1.0f;
        mvInvLevelSigma2_l.resize(nlevels);
        for (int i = 0; i < nlevels; i++) 
        {
            mvImagePyramid_l.push_back(lsd->gaussianPyrs[i]);
            mvScaleFactor_l.push_back(lsd->mvScaleFactor[i]);
            mvInvScaleFactor_l.push_back(lsd->mvInvScaleFactor[i]);
            if (i>0)  
                mvLevelSigma2_l[i]=mvScaleFactor_l[i]*mvScaleFactor_l[i];
            mvInvLevelSigma2_l[i]=1.0f/mvLevelSigma2_l[i];
        }
//        std::chrono::steady_clock::time_point tt1 = std::chrono::steady_clock::now();
        lbd->compute( img, keylines, descriptors_line);
//        std::chrono::steady_clock::time_point tt2 = std::chrono::steady_clock::now();
//        double LBDCULCU= std::chrono::duration_cast<std::chrono::duration<double> >(tt2 - tt1).count();
//        cout<<"LBD use : "<<LBDCULCU<<" s"<<endl;

    }
    else if(extractor==1) // ED Extractor
    {
        // Detect line features
        Ptr<BinaryDescriptor>   lbd = BinaryDescriptor::createBinaryDescriptor();
        Ptr<line_descriptor::LSDDetectorC> lsd = line_descriptor::LSDDetectorC::createLSDDetectorC();
        keylines.clear();
        double min_length = min_line_length*(std::min(img.cols,img.rows)); 
        lsd->detect_ED( img, keylines, scale, nlevels, min_length);
        // filter keyline
        if( int(keylines.size())>lsd_nfeatures && lsd_nfeatures!=0  )
        {
            // sort keylines by their response or by their length
            sort( keylines.begin(), keylines.end(), sort_lines_by_response() );
            //sort( keylines.begin(), keylines.end(), sort_lines_by_length() );
            keylines.resize(lsd_nfeatures);
            // reassign index
            for( int i = 0; i < lsd_nfeatures; i++  )
                    keylines[i].class_id = i;
        }

        nlevels_l=nlevels;
        mvLevelSigma2_l.resize(nlevels);
        mvLevelSigma2_l[0]=1.0f;
        mvInvLevelSigma2_l.resize(nlevels);
        for (int i = 0; i < nlevels; i++) 
        {
            mvImagePyramid_l.push_back(lsd->gaussianPyrs[i]);
            mvScaleFactor_l.push_back(lsd->mvScaleFactor[i]);
            mvInvScaleFactor_l.push_back(lsd->mvInvScaleFactor[i]);
            if (i>0)  
                mvLevelSigma2_l[i]=mvScaleFactor_l[i]*mvScaleFactor_l[i];
            mvInvLevelSigma2_l[i]=1.0f/mvLevelSigma2_l[i];
        } 

        lbd->compute( img, keylines, descriptors_line);        
    }
}

} //namespace ORB_SLAM
