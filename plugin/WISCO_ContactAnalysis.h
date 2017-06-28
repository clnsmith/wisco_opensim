#ifndef _WISCO_CONTACT_ANALYSIS_h_
#define _WISCO_CONTACT_ANALYSIS_h_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Kinematics.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/Analysis.h>
#include "WISCO_ElasticFoundationForce.h"
#include "WISCO_H5FileAdapter.h"
#include "osimPluginDLL.h"
#include "H5Cpp.h"
#include "hdf5_hl.h"

//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * A class for recording the kinematics of the generalized coordinates
 * of a model during a simulation.
 *
 * @author Colin Smith
 * @version 1.0
 */
class OSIMPLUGIN_API WISCO_ContactAnalysis : public Analysis {
//class WISCO_ContactAnalysis : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(WISCO_ContactAnalysis, Analysis);

//=============================================================================
// PROPERTIES
//=============================================================================
protected:

    OpenSim_DECLARE_LIST_PROPERTY(contact_names, std::string, 
        "Names of WISCO_ElasticFoundation contacts to be recorded.");
	OpenSim_DECLARE_PROPERTY(time_interval,double,
		"Step step for output data. Use 0 to use the input data interval.")
	OpenSim_DECLARE_PROPERTY(output_pressure, bool,
		"Output pressure values");
	OpenSim_DECLARE_PROPERTY(output_proximity, bool,
		"Output proximity values");
	OpenSim_DECLARE_PROPERTY(output_data_mesh_format, std::string,
		"Format of output data 'face', 'vertex' or 'both'.")
	OpenSim_DECLARE_PROPERTY(write_h5_file, bool,
		"Write binary .h5 file")
	OpenSim_DECLARE_PROPERTY(h5_raw_contact_data, bool,
		"Write detailed contact data to .h5 file")
	OpenSim_DECLARE_PROPERTY(h5_summary_contact_data, bool,
		"Write summary statistics of contact to .h5 file.")
	OpenSim_DECLARE_PROPERTY(h5_medial_lateral_summary, bool,
		"Write medial-lateral regional summary statistics to .h5 file")
	OpenSim_DECLARE_PROPERTY(h5_states_data, bool,
		"Write states data to .h5 file")
	OpenSim_DECLARE_PROPERTY(h5_kinematics_data, bool,
			"Write kinematics data to .h5 file")
	OpenSim_DECLARE_PROPERTY(h5_muscle_data,bool,
		"Write muscle analysis data to .h5 file")
	OpenSim_DECLARE_PROPERTY(h5_ligament_data, bool,
		"Write WISCO_ligament data to h5 file")
	OpenSim_DECLARE_PROPERTY(write_static_vtk_files, bool,
		"Write .vtk files with meshes fixed in space (in local frame).")
	OpenSim_DECLARE_PROPERTY(write_dynamic_vtk_files,bool,
		"Write .vtk files with meshes moving in space.")
	OpenSim_DECLARE_PROPERTY(dynamic_output_frame,std::string,
		"Model reference frame for mesh motion.")
	OpenSim_DECLARE_PROPERTY(write_variable_property_vtk, std::string,
		"Add cartilage thickness and material property maps to vtk files."
		"Options: 'thickness','elastic modulus','poisson ratio' or 'all'");


//=============================================================================
// METHODS
//=============================================================================
public:
	WISCO_ContactAnalysis();
	WISCO_ContactAnalysis(Model *aModel);

	void setModel(Model& aModel) override;

	int begin(SimTK::State& s) override;
	int step(const SimTK::State& s, int setNumber) override;
	int end(SimTK::State& s) override;
	int printResults(const std::string &aBaseName, const std::string &aDir, double aDT,
		const std::string &aExtension);
private:
    void setNull();
    void constructProperties();


	int record(const SimTK::State& s);
	
	void collectMeshData(const std::string& mesh_name,
		const std::vector<std::string>& contact_names,
		std::vector<SimTK::Matrix>& faceData, std::vector<std::string>& faceDataNames,
		std::vector<SimTK::Matrix>& pointData, std::vector<std::string>& pointDataNames);

	void writeVTKFile(const std::string& file_path,
		const std::string& base_name, const std::string& mesh_name,
		const std::vector<std::string>& contact_names, bool isDynamic);
	void writeH5File(const std::string &aBaseName, const std::string &aDir);
	
	void addContactReportsToH5File(WISCO_H5FileAdapter& h5_adapt, 
		const std::string& group_name, const std::string& contact_name);
	
	void decomposeReportName(const std::string& name,
		std::string& contact_name, std::string& mesh_name,
		std::string& data_type, std::string& data_name);
	
	void renameReportLabelsToH5Path(std::vector<std::string>& labels, std::string mesh_path);
//=============================================================================
// DATA
//=============================================================================
private:
	std::vector<std::string> _contact_force_names;
	std::vector<std::string> _contact_mesh_names;
	mutable std::vector<SimTK::Matrix_<SimTK::Vec3>> _mesh_vertex_locations;

//=============================================================================
};  // END of class WISCO_ContactAnalysis

}; //namespace



#endif // #ifndef __WISCO_CONTACT_ANALYSIS_h__
