/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 * $Id$
 *
 */

/**
\author Marius Muja
**/

#ifndef SERIALIZATION_SUPPORT_H_
#define SERIALIZATION_SUPPORT_H_


#include <opencv_workbench/recognition_pipeline/algorithms/types.h>

#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/split_free.hpp>

#include <opencv2/core/core.hpp>

// ------------- cv::Mat
namespace boost { namespace serialization {

template<class Archive>
void save(Archive & ar, const cv::Mat& mat, const unsigned int version)
{
    cv::Mat mat_;
    mat_ = mat;
	if (!mat.isContinuous()) {
        mat_ = mat.clone();
//		throw recognition_pipeline::Exception("Only serialising continuous cv::Mat objects");
    }
	int type = mat_.type();
    ar & mat_.rows;
    ar & mat_.cols;
    ar & type;
    ar & boost::serialization::make_binary_object(mat_.data, mat_.step*mat_.rows);
}

template<class Archive>
void load(Archive & ar, cv::Mat& mat, const unsigned int version)
{
	int rows, cols, type;
    ar & rows;
    ar & cols;
    ar & type;
    mat.create(rows, cols, type);
    ar & boost::serialization::make_binary_object(mat.data, mat.step*mat.rows);
}

}}
BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat);


namespace boost { namespace serialization {

// ------------- cv::Point_
template<class Archive, class T>
void serialize(Archive & ar, cv::Point_<T>& point, const unsigned int version)
{
	ar & point.x;
	ar & point.y;
}

// ------------- cv::Point3_
template<class Archive, class T>
void serialize(Archive & ar, cv::Point3_<T>& point, const unsigned int version)
{
	ar & point.x;
	ar & point.y;
	ar & point.z;
}

// ------------- cv::Size_

template<class Archive, class T>
void serialize(Archive & ar, cv::Size_<T>& size, const unsigned int version)
{
	ar & size.width;
	ar & size.height;
}

// ------------- cv::Rect_
template<class Archive, class T>
void serialize(Archive & ar, cv::Rect_<T>& rect, const unsigned int version)
{
	ar & rect.x;
	ar & rect.y;
	ar & rect.width;
	ar & rect.height;
}

// ------------- cv::RotatedRect
template<class Archive>
void serialize(Archive & ar, cv::RotatedRect& rrect, const unsigned int version)
{
	ar & rrect.center;
	ar & rrect.size;
	ar & rrect.angle;
}

// ------------- cv::Vec
template<class Archive, class T, int n>
void serialize(Archive & ar, cv::Vec<T,n>& vec, const unsigned int version)
{
	ar & vec.val;
}

// ------------- cv::Scalar_
template<class Archive, class T>
void serialize(Archive & ar, cv::Scalar_<T>& scalar, const unsigned int version)
{
	ar & scalar.val;
}


// ------------- cv::Range
template<class Archive>
void serialize(Archive & ar, cv::Range& range, const unsigned int version)
{
	ar & range.start;
	ar & range.end;
}
}}

#endif /* SERIALIZATION_SUPPORT_H_ */
