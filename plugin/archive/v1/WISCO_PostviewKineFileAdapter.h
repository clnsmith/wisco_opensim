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

#ifndef OPENSIM_POSTVIEW_KINE_FILE_ADAPTER_H_
#define OPENSIM_POSTVIEW_KINE_FILE_ADAPTER_H_

#include "OpenSim\Common\FileAdapter.h"
#include "SimTKmath.h"
#include "OpenSim\Common\TimeSeriesTable.h"
//#include "osimPluginDLL.h"

namespace OpenSim {

    //class OSIMPLUGIN_API PostViewDataFileAdapter : public DelimFileAdapter {
    class WISCO_PostViewKineFileAdapter : public FileAdapter {
    public:

        WISCO_PostViewKineFileAdapter();
        WISCO_PostViewKineFileAdapter(const WISCO_PostViewKineFileAdapter&) = default;
        WISCO_PostViewKineFileAdapter(WISCO_PostViewKineFileAdapter&&) = default;
        WISCO_PostViewKineFileAdapter& operator=(const WISCO_PostViewKineFileAdapter&) = default;
        WISCO_PostViewKineFileAdapter& operator=(WISCO_PostViewKineFileAdapter&&) = default;
        ~WISCO_PostViewKineFileAdapter() = default;

        WISCO_PostViewKineFileAdapter* clone() const override;


        static
            void write(const TimeSeriesTable& intable1, const TimeSeriesTable& intable2, const std::string& fileName);
    protected:
        OutputTables extendRead(const std::string& fileName) const override;

        void extendWrite(const InputTables& tables, const std::string& fileName) const override;
    };

} // namespace OpenSim

#endif // OPENSIM_POSTVIEW_KINE_FILE_ADAPTER_H_
