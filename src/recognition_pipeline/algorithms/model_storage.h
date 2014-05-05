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

#ifndef MODEL_STORAGE_H_
#define MODEL_STORAGE_H_


#include <opencv_workbench/recognition_pipeline/algorithms/serialization_support.h>

#include <string>
#include <vector>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>


namespace recognition_pipeline
{

/**
 * Interface for a model storage
 */
class ModelStorage
{
public:
	/**
	 * Saves a model blob to a storage space, for example a model database.
	 * @param[in] name Name of the model
	 * @param[in] detector Name of the detector that uses this model
	 * @param[in] model_blob The model blob. It's the detector's job to serialize/deserialize the model.
	 */
	virtual void saveModel(const std::string& name, const std::string& detector, const std::string& model_blob) = 0;

	/**
	 * Loads a model blob from a storage space (a model database for example)
	 * @param[in] name Name of the model
	 * @param[in] detector Name of the detector using the model
	 * @param[out] model_blob The model blob.
	 * @return true if model is found in database
	 */
	virtual bool loadModel(const std::string& name, const std::string& detector, std::string& model_blob) = 0;

	/**
	 * Returns the list f all models available for a particular detector
	 * @param[in] detector detector name
	 * @param[out] model_list list of models available for the detector
	 * @return true if any models are found in database
	 */
	virtual bool getModelList(const std::string& detector, std::vector<std::string>& model_list) = 0;


	/**
	 * Save boost-serializable object
	 * @param name
	 * @param detector
	 * @param model
	 */
	template <typename ModelType>
	void save(const std::string& name, const std::string& detector, const ModelType& model)
	{
		std::ostringstream ostream;
		boost::archive::binary_oarchive oa(ostream);
		oa << model;
		saveModel(name, detector, ostream.str());
	}

	/**
	 * Load boost-serializable object
	 * @param name
	 * @param detector
	 * @param model
	 */
	template <typename ModelType>
	bool load(const std::string& name, const std::string& detector, ModelType& model)
	{
		std::string model_blob;
		if (!loadModel(name, detector, model_blob)) {
			return false;
		}
		std::istringstream istream(model_blob);
		boost::archive::binary_iarchive ia(istream);
		ia >> model;
		return true;
	}
};

typedef boost::shared_ptr<ModelStorage> ModelStoragePtr;
typedef boost::shared_ptr<ModelStorage const> ModelStorageConstPtr;

}

#endif /* MODEL_STORAGE_H_ */
