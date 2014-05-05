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

#ifndef TRAINER_H_
#define TRAINER_H_

#include <string>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

namespace recognition_pipeline {

struct TrainingData
{
	// the image to train on
	cv::Mat image;
	// region of interest in the image
	cv::Rect roi;
	// mask (same size as roi)
	cv::Mat mask;
	// is this image a background image
	bool background;
};

/**
 * Interface for an algorithm supporting capable of training new models
 */
class Trainable
{
public:
	/**
	 * Starts training for a new object category model. It may allocate/initialize
	 * data structures needed for training a new category.
	 * @param name
	 */
	virtual void startTraining(const std::string& name) = 0;

	/**
	 * Trains the model on a new data instance.
	 * @param name The name of the model
	 * @param data Training data instance
	 */
	virtual void trainInstance(const std::string& name, const recognition_pipeline::TrainingData& data) = 0;

	/**
	 * Saves a trained model.
	 * @param name model name
	 */
	virtual void endTraining(const std::string& name) = 0;

};
typedef boost::shared_ptr<Trainable> TrainablePtr;
typedef boost::shared_ptr<Trainable const> TrainableConstPtr;

}

#endif /* TRAINER_H_ */
