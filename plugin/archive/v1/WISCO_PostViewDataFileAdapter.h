/* -------------------------------------------------------------------------- *
 *                          OpenSim:  WISCO_PostViewDataFileAdapter.h                        *
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

#ifndef OPENSIM_PostViewData_FILE_ADAPTER_H_
#define OPENSIM_PostViewData_FILE_ADAPTER_H_

#include "OpenSim\Common\DelimFileAdapter.h"
//#include <OpenSim\OpenSim.h>
#include "SimTKmath.h"
//#include "osimPluginDLL.h"

namespace OpenSim {

/** CSVFileAdapter is a DelimFileAdapter that presets the delimiters 
appropriately for CSV files.                                                  */
//class OSIMPLUGIN_API WISCO_PostViewDataFileAdapter : public DelimFileAdapter {
class WISCO_PostViewDataFileAdapter : public DelimFileAdapter {
public:
	WISCO_PostViewDataFileAdapter();
	WISCO_PostViewDataFileAdapter(const WISCO_PostViewDataFileAdapter&)            = default;
	WISCO_PostViewDataFileAdapter(WISCO_PostViewDataFileAdapter&&)                 = default;
	WISCO_PostViewDataFileAdapter& operator=(const WISCO_PostViewDataFileAdapter&) = default;
	WISCO_PostViewDataFileAdapter& operator=(WISCO_PostViewDataFileAdapter&&)      = default;
    ~WISCO_PostViewDataFileAdapter()                                         = default;

	WISCO_PostViewDataFileAdapter* clone() const override;

    /** Read a CSV file.                                                      */
    //static
    //TimeSeriesTable read(const std::string& fileName);

    /** Write a CSV file.                                                     */
    static
    void write(const TimeSeriesTable& table, const std::string& fileName, const SimTK::PolygonalMesh mesh, const int ver_start_ind);
};

}

#endif // OPENSIM_CSV_FILE_ADAPTER_H_
