/* -------------------------------------------------------------------------- *
*                          OpenSim:  WISCO_KeywordFileAdapter.h                        *
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

#ifndef OPENSIM_KEYWORD_FILE_ADAPTER_H_
#define OPENSIM_KEYWORD_FILE_ADAPTER_H_

#include "OpenSim\Common\FileAdapter.h"
#include "SimTKmath.h"
//#include "osimPluginDLL.h"

namespace OpenSim {

    //class OSIMPLUGIN_API PostViewDataFileAdapter : public DelimFileAdapter {
    class WISCO_KeywordFileAdapter : public FileAdapter {
    public:
               
       WISCO_KeywordFileAdapter();
       WISCO_KeywordFileAdapter(const WISCO_KeywordFileAdapter&) = default;
       WISCO_KeywordFileAdapter(WISCO_KeywordFileAdapter&&) = default;
       WISCO_KeywordFileAdapter& operator=(const WISCO_KeywordFileAdapter&) = default;
       WISCO_KeywordFileAdapter& operator=(WISCO_KeywordFileAdapter&&) = default;
       ~WISCO_KeywordFileAdapter() = default;

       WISCO_KeywordFileAdapter* clone() const override;


       void write(const std::string& fileName,
                const SimTK::PolygonalMesh& mesh1b, const SimTK::PolygonalMesh& mesh1c,
                const SimTK::PolygonalMesh& mesh2b, const SimTK::PolygonalMesh& mesh2c ) const;
    protected:
        OutputTables extendRead(const std::string& fileName) const override;
        
        void extendWrite(const InputTables& tables, const std::string& fileName) const override;
    private:
        void write_vertex_positions(std::ofstream& out_stream, const SimTK::PolygonalMesh mesh, const int ver_start_ind) const;
        void write_face_connectivity(std::ofstream& out_stream,const SimTK::PolygonalMesh mesh, 
            const int part_ind, const int ver_start_ind, const int tri_start_ind) const;

    };

} // namespace OpenSim

#endif // OPENSIM_KEYWORD_FILE_ADAPTER_H_
