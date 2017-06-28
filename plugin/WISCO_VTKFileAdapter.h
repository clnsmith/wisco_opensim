/* -------------------------------------------------------------------------- *
*                          OpenSim:  WISCO_VTKFileAdapter.h                        *
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

#ifndef OPENSIM_VTK_FILE_ADAPTER_H_
#define OPENSIM_VTK_FILE_ADAPTER_H_

#include "OpenSim/Common/FileAdapter.h"
//#include "OpenSim\OpenSim.h"
#include "SimTKmath.h"
#include "osimPluginDLL.h"

namespace OpenSim {

    class OSIMPLUGIN_API WISCO_VTKFileAdapter : public FileAdapter {
    //class WISCO_VTKFileAdapter : public FileAdapter {
    public:

       WISCO_VTKFileAdapter();
       WISCO_VTKFileAdapter(const WISCO_VTKFileAdapter&) = default;
       WISCO_VTKFileAdapter(WISCO_VTKFileAdapter&&) = default;
       WISCO_VTKFileAdapter& operator=(const WISCO_VTKFileAdapter&) = default;
       WISCO_VTKFileAdapter& operator=(WISCO_VTKFileAdapter&&) = default;
       ~WISCO_VTKFileAdapter() = default;

       WISCO_VTKFileAdapter* clone() const override;


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

	   @param[in] aFaceData
			A vector [nDataField] of SimTK::Matrix [nTimeStep x nFaces]
			containing data values for each face in the contact mesh

	   */
	   void setFaceData(std::vector<std::string> aFaceDataNames, std::vector<SimTK::Matrix> aFaceData)
	   {
		   faceDataNames = aFaceDataNames;
		   faceData = aFaceData;
	   };

	   void setPointData(std::vector<std::string> aPointDataNames, std::vector<SimTK::Matrix> aPointData)
	   {
		   pointDataNames = aPointDataNames;
		   pointData = aPointData;
	   };
    protected:
        OutputTables extendRead(const std::string& fileName) const override;

        void extendWrite(const InputTables& tables, const std::string& fileName) const override;
    private:
        void write_vertex_positions(std::ofstream& out_stream, const SimTK::Matrix_<SimTK::Vec3>& vertices, int nTimeStep) const;
        void write_face_connectivity(std::ofstream& out_stream, const SimTK::Matrix& faces) const;

	//Data
	private:
		std::vector<std::string> faceDataNames;
		std::vector<SimTK::Matrix> faceData;
		std::vector<std::string> pointDataNames;
		std::vector<SimTK::Matrix> pointData;
    };

} // namespace OpenSim

#endif // OPENSIM_VTK_FILE_ADAPTER_H_
