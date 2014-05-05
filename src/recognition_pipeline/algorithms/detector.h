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

#ifndef DETECTOR_H_
#define DETECTOR_H_

#include <opencv_workbench/recognition_pipeline/algorithms/types.h>
#include <opencv_workbench/recognition_pipeline/algorithms/model_storage.h>

//#include <opencv_workbench/recognition_pipeline/DetectionArray.h>
//#include <opencv_workbench/recognition_pipeline/MaskArray.h>
//#include <sensor_msgs/PointCloud2.h>

#include <boost/make_shared.hpp>

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

namespace recognition_pipeline {

class Detector
{
public:

	Detector() {};

	Detector(ModelStoragePtr model_storage) : model_storage_(model_storage) {};

	/**
	 * \brief Set input image for the detector.
	 * @param image The input image
	 */
	inline void setImage(const cv::Mat& image)
	{
		image_ = image;
	}


	/**
	 * Sets the regions-of-interest (ROI/masks) where the detector should look.
	 * @param ros_list Array of rectangles representing the regions of interest. If empty
	 * the entire image is used.
	 */
	//inline void setMasks(const MaskArrayConstPtr& masks)
	//{
	//	masks_ = masks;
	//}

	/**
	 * Set list of detections. This is useful when the detector works like a filter, for chaining detectors
	 * @param ros_list Array of detections.
	 */
	inline void setInputdetections(const DetectionArrayConstPtr& input_detections)
	{
		input_detections_ = input_detections;
	}

	/**
	 * Sets the point cloud to be used in the detection.
	 * @param point_cloud
	 */
	//inline void setPointCloud(const sensor_msgs::PointCloud2ConstPtr point_cloud)
	//{
	//	point_cloud_ = point_cloud;
	//}


	/**
	 * \brief Run the object detector. The detection results are stored in
	 * class member detections_.
	 */
	virtual void detect() = 0;

	/**
	 * Each detector will have an unique name. This returns the detector name.
	 * @return name of the detector
	 */
	virtual std::string getName() = 0;

	/**
	 * Loads pre-trained models for a list of objects.
	 * @param models list of objects to load
	 */
	virtual void loadModels(const std::vector<std::string>& models) = 0;

	/**
	 * Loads all available pre-trained models
	 */
	void loadAllModels()
	{
		if (model_storage_ != NULL) {
			std::vector<std::string> models;
			model_storage_->getModelList(getName(),models);
			loadModels(models);
		}
	}


	/**
	 * This returns the list of resulting detection after detect() is called.
	 * @return detections array
	 */
	//inline DetectionArray getDetections() const
	//{
	//	return detections_;
	//}

protected:

	/**
	 * The current image
	 */
	cv::Mat image_;

	/**
	 * Input ROIs
	 */
	//MaskArrayConstPtr masks_;

	/**
	 * Input ROIs
	 */
	//DetectionArrayConstPtr input_detections_;

	/**
	 * PointCloud
	 */
	//sensor_msgs::PointCloud2ConstPtr point_cloud_;


	/**
	 * Detection results
	 */
	//DetectionArray detections_;


	/**
	 * Model storage object
	 */
	ModelStoragePtr model_storage_;

};
typedef boost::shared_ptr<Detector> DetectorPtr;
typedef boost::shared_ptr<Detector const> DetectorConstPtr;

}


#endif /* DETECTOR_H_ */
