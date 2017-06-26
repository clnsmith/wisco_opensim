/* -------------------------------------------------------------------------- *
*                          OpenSim:  WISCO_VTPFileAdapter.h                        *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2015 Stanford University and the Authors                *
*                                                                            *
* Licensed under the Apache License, Version 2.0 (the "License"); you may    *
* not use this file except in compliance with the License. You may obtain a  *
* copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
*                                                                            *
* Unless required by applicable law or agreed to in writing, software        *
* distributed under the License is distributed on an "AS IS" BASIS,          *
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
* See the License for the specific language governing permissions and        *
* limitations under the License.                                             *
* -------------------------------------------------------------------------- */

#ifndef OPENSIM_VTP_FILE_ADAPTER_H_
#define OPENSIM_VTP_FILE_ADAPTER_H_

#include "OpenSim/Common/FileAdapter.h"
#include "SimTKmath.h"
#include "osimPluginDLL.h"

namespace OpenSim {

    class OSIMPLUGIN_API WISCO_VTPFileAdapter : public FileAdapter {
    //class WISCO_VTPFileAdapter : public FileAdapter {
    public:

       WISCO_VTPFileAdapter();
       WISCO_VTPFileAdapter(const WISCO_VTPFileAdapter&) = default;
       WISCO_VTPFileAdapter(WISCO_VTPFileAdapter&&) = default;
       WISCO_VTPFileAdapter& operator=(const WISCO_VTPFileAdapter&) = default;
       WISCO_VTPFileAdapter& operator=(WISCO_VTPFileAdapter&&) = default;
       ~WISCO_VTPFileAdapter() = default;

       WISCO_VTPFileAdapter* clone() const override;


	   /**
	   @param[in] vertices
			A matrix [nVertices x nTimeSteps] containing Vec3 with the locations
			of the contact mesh vertices in space.

		@param[in] faces
			A matrix [nFaces x nVerticesPerFace]


	   */
	   void write(const std::string& fileName, const std::string& filePath,
		   const SimTK::Matrix_<SimTK::Vec3>& vertices, const SimTK::Matrix& faces,
		   const int nTimeSteps) const;

	   void write(const std::string& fileName, const std::string& filePath,
		   const SimTK::PolygonalMesh& mesh, const int nTimeSteps) const;


	   /**
	   @param[in] aFaceDataNames
			Cannot include spaces

	   @param[in] aFaceData
			A vector [nDataField] of SimTK::Matrix [nTimeStep x nFaces]
			containing data values for each face in the contact mesh
	   @param[in] aFaceDataTypes
			Options:
			Int8, UInt8, Int16, UInt16, Int32,
			UInt32, Int64, UInt64, Float32, Float64

	   */
	   void setFaceData(std::vector<std::string> aFaceDataNames, std::vector<SimTK::Matrix> aFaceData)
	   {
		   _faceDataNames = aFaceDataNames;
		   _faceData = aFaceData;
	   };

	   void setPointData(std::vector<std::string> aPointDataNames, std::vector<SimTK::Matrix> aPointData)
	   {
		   _pointDataNames = aPointDataNames;
		   _pointData = aPointData;

	   };

	   void setDataFormat(SimTK::String format) {
		   _data_format = format.toLower();
	   }

    protected:
        OutputTables extendRead(const std::string& fileName) const override;

        void extendWrite(const InputTables& tables, const std::string& fileName) const override;
    private:
		bool isLittleEndian() const;
		std::string encodeFloatDataVTPBase64(std::vector<float>& data) const;
		std::string encodeIntDataVTPBase64(std::vector<uint32_t>& data) const;

	//Data
	private:
		std::vector<std::string> _faceDataNames;
		std::vector<SimTK::Matrix>_faceData;

		std::vector<std::string> _pointDataNames;
		std::vector<SimTK::Matrix> _pointData;

		SimTK::String _data_format;
    };

} // namespace OpenSim

#endif // OPENSIM_VTP_FILE_ADAPTER_H_
