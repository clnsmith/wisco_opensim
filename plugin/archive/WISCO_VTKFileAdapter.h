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

#include "OpenSim\Common\FileAdapter.h"
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


       //void write(const std::string& fileName, const std::string& filePath, const SimTK::PolygonalMesh& mesh, const TimeSeriesTable& intable1) const;
	   
	   void write(const std::string& fileName, const std::string& filePath, const SimTK::PolygonalMesh& mesh, 
		   std::vector<SimTK::Matrix>& vecOutputMatrix, std::vector<std::string>& OutputNames) const;
    protected:
        OutputTables extendRead(const std::string& fileName) const override;
        
        void extendWrite(const InputTables& tables, const std::string& fileName) const override;
    private:
        void write_vertex_positions(std::ofstream& out_stream, const SimTK::PolygonalMesh& mesh) const;
        void write_face_connectivity(std::ofstream& out_stream,const SimTK::PolygonalMesh& mesh) const;

    };

} // namespace OpenSim

#endif // OPENSIM_VTK_FILE_ADAPTER_H_
