/* -------------------------------------------------------------------------- *
*                    WISCO_OpenSim:  WISCO_H5FileAdapter.h                   *
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

#ifndef OPENSIM_H5_FILE_ADAPTER_H_
#define OPENSIM_H5_FILE_ADAPTER_H_

#include "OpenSim\Common\FileAdapter.h"
#include "SimTKmath.h"
#include "H5Cpp.h"
#include "hdf5_hl.h"
#include "osimPluginDLL.h"
#include "OpenSim\Common\TimeSeriesTable.h"

namespace OpenSim {

    class OSIMPLUGIN_API WISCO_H5FileAdapter : public FileAdapter {
    //class WISCO_H5FileAdapter : public FileAdapter {
    public:
	   WISCO_H5FileAdapter();
       WISCO_H5FileAdapter(const WISCO_H5FileAdapter&) = default;
       WISCO_H5FileAdapter(WISCO_H5FileAdapter&&) = default;
       WISCO_H5FileAdapter& operator=(const WISCO_H5FileAdapter&) = default;
       WISCO_H5FileAdapter& operator=(WISCO_H5FileAdapter&&) = default;
       ~WISCO_H5FileAdapter() = default;

       WISCO_H5FileAdapter* clone() const override;
	   
	   
	   void open(const std::string& file_name);
	   void close();

	   void createGroup(const std::string& new_group);
	   void writeDataSet(const TimeSeriesTable& table, const std::string group_path);
	   void writeDataSetVec3(const TimeSeriesTableVec3& table, const std::string group_path);
	   void writeDataSetVector(const TimeSeriesTable& table, const std::string group_path);
	   void WISCO_H5FileAdapter::writeDataSetSimTKVector(const SimTK::Vector& data_vector, const std::string dataset_path);
	   void writeTimeDataSet(const TimeSeriesTable& table);
	   void writeStatesDataSet(const TimeSeriesTable& table);
	   void writeKinematicsDataSet(const TimeSeriesTable& pos_table, const TimeSeriesTable& vel_table, const TimeSeriesTable& acc_table);
	   void writeMuscleDataSet(std::vector<SimTK::Matrix>& msl_table, std::vector<std::string> msl_names, 
		   std::vector<std::string> param_names, const TimeSeriesTable& states_table);
	   void writeLigamentDataSet(const TimeSeriesTable& table);
    protected:
        OutputTables extendRead(const std::string& fileName) const override;
        
        void extendWrite(const InputTables& tables, const std::string& fileName) const override;
    private:


	//Data
	private:
		H5::H5File _file;
		bool _time_is_empty;

    };

} // namespace OpenSim

#endif // OPENSIM_H5_FILE_ADAPTER_H_
