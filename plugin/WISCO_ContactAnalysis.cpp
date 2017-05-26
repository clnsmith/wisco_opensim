/* -------------------------------------------------------------------------- *
 *                          OpenSim:  WISCO_ContactAnalysis.cpp               *
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
#include <OpenSim/Simulation/Model/Model.h>
#include "WISCO_ContactAnalysis.h"
#include <OpenSim\Common\Reporter.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include "WISCO_VTKFileAdapter.h"
//#include <OpenSim\OpenSim.h>
#include "H5Cpp.h"
#include "hdf5_hl.h"
#include "WISCO_ElasticFoundationForce.h"
using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================

WISCO_ContactAnalysis::WISCO_ContactAnalysis() : Analysis()
{
	setNull();
	constructProperties();
}

WISCO_ContactAnalysis::WISCO_ContactAnalysis(Model *aModel) :
    Analysis(aModel)
{
    // NULL
    setNull();

    // CHECK MODEL
    if(aModel==NULL) return;
    setModel(*aModel);

}


//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void WISCO_ContactAnalysis::setNull()
{
    //setName("WISCO_ContactAnalysis");

}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void WISCO_ContactAnalysis::constructProperties()
{
    Array<std::string> defaultContactNames;
	defaultContactNames.append("all");

    constructProperty_contact_names(defaultContactNames);
	constructProperty_output_pressure(true);
	constructProperty_output_proximity(true);
	constructProperty_output_data_mesh_format("face");
	constructProperty_h5_raw_contact_data(true);
	constructProperty_h5_summary_contact_data(true);
	constructProperty_h5_medial_lateral_summary(true);
	constructProperty_write_h5_file(true);
	constructProperty_h5_write_states_data(true);
	constructProperty_write_static_vtk_files(true);
	constructProperty_write_dynamic_vtk_files(true);
	constructProperty_dynamic_output_frame("ground");
	constructProperty_write_variable_property_vtk(false);

}


//_____________________________________________________________________________
/**
 * Set the model to use when performing analysis. 
 */

void WISCO_ContactAnalysis::setModel(Model& aModel)
{
	// BASE CLASS
	Super::setModel(aModel);

}




//=============================================================================
// ANALYSIS
//=============================================================================


int WISCO_ContactAnalysis::record(const SimTK::State& s)
{
	_model->realizeReport(s);
	
	//Store mesh vertex locations
	if (get_write_dynamic_vtk_files()) {
		std::string frame = get_dynamic_output_frame();

		

		for (int i = 0; i < _contact_force_names.size(); ++i) {
			//Target Mesh
			int mesh1_nRow = _mesh1_vertex_locations[i].nrow();
			int mesh1_nCol = _mesh1_vertex_locations[i].ncol();

			_mesh1_vertex_locations[i].resizeKeep(mesh1_nRow, mesh1_nCol + 1);

			SimTK::Vector_<SimTK::Vec3> mesh1_ver = _model->getComponent<WISCO_ElasticFoundationForce>
				(_contact_force_names[i]).getMeshVerticesInFrame(s, "target_mesh", frame);

			double test = mesh1_ver(0)(0);

			for (int j = 0; j < mesh1_nRow; ++j){
				_mesh1_vertex_locations[i](j, mesh1_nCol) = mesh1_ver(j);
			}
			
			//Mesh2
			int mesh2_nRow = _mesh2_vertex_locations[i].nrow();
			int mesh2_nCol = _mesh2_vertex_locations[i].ncol();

			_mesh2_vertex_locations[i].resizeKeep(mesh2_nRow, mesh2_nCol + 1);

			SimTK::Vector_<SimTK::Vec3> mesh2_ver = _model->getComponent<WISCO_ElasticFoundationForce>
				(_contact_force_names[i]).getMeshVerticesInFrame(s, "casting_mesh", frame);

			for (int j = 0; j < mesh2_nRow; ++j) {
				_mesh2_vertex_locations[i](j, mesh2_nCol) = mesh2_ver(j);
			}
		}

	}
    return(0);
}

void WISCO_ContactAnalysis::addContactReportersToModel(WISCO_ElasticFoundationForce& contactForce)
{
	SimTK::String mesh_output_format = 
		SimTK::String::toLower(get_output_data_mesh_format());
	
	std::string mesh1_name = contactForce.getConnectee<WISCO_ContactMesh>("target_mesh").getName();
	std::string mesh2_name = contactForce.getConnectee<WISCO_ContactMesh>("casting_mesh").getName();
	std::string force_name = contactForce.getName();

	//Raw Data
	if (get_h5_raw_contact_data()) {
		//Pressure Reporters
		if (get_output_pressure()) {

			//Triangle Pressure
			if (mesh_output_format == "face" || mesh_output_format == "both") {
				addContactReporter(contactForce, "target_mesh_tri_pressure", "vector");
				addContactReporter(contactForce, "casting_mesh_tri_pressure", "vector");
			}

			//Vertex Pressure
			if (mesh_output_format == "vertex" || mesh_output_format == "both") {
				addContactReporter(contactForce, "target_mesh_vertex_pressure", "vector");
				addContactReporter(contactForce, "casting_mesh_vertex_pressure", "vector");

			}
		}

		//Proximity Reporters
		if (get_output_proximity()) {
			//Triangle Proximity
			if (mesh_output_format == "face" || mesh_output_format == "both") {
				addContactReporter(contactForce, "target_mesh_tri_proximity", "vector");
				addContactReporter(contactForce, "casting_mesh_tri_proximity", "vector");
			}
			//Vertex Proximity
			if (mesh_output_format == "vertex" || mesh_output_format == "both") {
				addContactReporter(contactForce, "target_mesh_vertex_proximity","vector");
				addContactReporter(contactForce, "casting_mesh_vertex_proximity","vector");
			}
		}
	}
	//Summary Stats
	if (get_h5_summary_contact_data()) {
		addContactReporter(contactForce, "target_mesh_mean_pressure","real");
		addContactReporter(contactForce, "target_mesh_max_pressure", "real");
		addContactReporter(contactForce, "target_mesh_mean_proximity", "real");
		addContactReporter(contactForce, "target_mesh_max_proximity", "real");
		addContactReporter(contactForce, "target_mesh_contact_area", "real");
		addContactReporter(contactForce, "target_mesh_cop", "vec3");
		addContactReporter(contactForce, "target_mesh_contact_force", "vec3");

		addContactReporter(contactForce, "casting_mesh_mean_pressure", "real");
		addContactReporter(contactForce, "casting_mesh_max_pressure", "real");
		addContactReporter(contactForce, "casting_mesh_mean_proximity", "real");
		addContactReporter(contactForce, "casting_mesh_max_proximity", "real");
		addContactReporter(contactForce, "casting_mesh_contact_area", "real");
		addContactReporter(contactForce, "casting_mesh_cop", "vec3");
		addContactReporter(contactForce, "casting_mesh_contact_force", "vec3");

		if (get_h5_medial_lateral_summary()) {
			addContactReporter(contactForce, "target_mesh_mean_pressure_medial", "real");
			addContactReporter(contactForce, "target_mesh_max_pressure_medial", "real");
			addContactReporter(contactForce, "target_mesh_mean_proximity_medial", "real");
			addContactReporter(contactForce, "target_mesh_max_proximity_medial", "real");
			addContactReporter(contactForce, "target_mesh_contact_area_medial", "real");
			addContactReporter(contactForce, "target_mesh_cop_medial", "vec3");
			addContactReporter(contactForce, "target_mesh_contact_force_medial", "vec3");

			addContactReporter(contactForce, "casting_mesh_mean_pressure_medial", "real");
			addContactReporter(contactForce, "casting_mesh_max_pressure_medial", "real");
			addContactReporter(contactForce, "casting_mesh_mean_proximity_medial", "real");
			addContactReporter(contactForce, "casting_mesh_max_proximity_medial", "real");
			addContactReporter(contactForce, "casting_mesh_contact_area_medial", "real");
			addContactReporter(contactForce, "casting_mesh_cop_medial", "vec3");
			addContactReporter(contactForce, "casting_mesh_contact_force_medial", "vec3");

			addContactReporter(contactForce, "target_mesh_mean_pressure_lateral", "real");
			addContactReporter(contactForce, "target_mesh_max_pressure_lateral", "real");
			addContactReporter(contactForce, "target_mesh_mean_proximity_lateral", "real");
			addContactReporter(contactForce, "target_mesh_max_proximity_lateral", "real");
			addContactReporter(contactForce, "target_mesh_contact_area_lateral", "real");
			addContactReporter(contactForce, "target_mesh_cop_lateral", "vec3");
			addContactReporter(contactForce, "target_mesh_contact_force_lateral", "vec3");

			addContactReporter(contactForce, "casting_mesh_mean_pressure_lateral", "real");
			addContactReporter(contactForce, "casting_mesh_max_pressure_lateral", "real");
			addContactReporter(contactForce, "casting_mesh_mean_proximity_lateral", "real");
			addContactReporter(contactForce, "casting_mesh_max_proximity_lateral", "real");
			addContactReporter(contactForce, "casting_mesh_contact_area_lateral", "real");
			addContactReporter(contactForce, "casting_mesh_cop_lateral", "vec3");
			addContactReporter(contactForce, "casting_mesh_contact_force_lateral", "vec3");
		}
	}
}
/**
reporter_type = "vector" or "
*/
void WISCO_ContactAnalysis::addContactReporter(WISCO_ElasticFoundationForce& contactForce,
	const SimTK::String& output_name, const SimTK::String& reporter_type) {

	SimTK::String target_id = "target_mesh";
	SimTK::String casting_id = "casting_mesh";

	SimTK::String mesh_name, data_name;
	

	if (output_name.compare(0, target_id.size(), target_id)==0) {
		mesh_name = contactForce.getConnectee<WISCO_ContactMesh>("target_mesh").getName();
		data_name = output_name.substr(target_id.size());
	}

	if (output_name.compare(0, casting_id.size(), casting_id)==0) {
		mesh_name = contactForce.getConnectee<WISCO_ContactMesh>("casting_mesh").getName();
		data_name = output_name.substr(casting_id.size());
	}

	SimTK::String force_name = contactForce.getName();

	if (reporter_type == "vector") {
		TableReporterVector* mesh1_reporter = new TableReporterVector();
		mesh1_reporter->setName(force_name + "_" + mesh_name + data_name);
		mesh1_reporter->set_report_time_interval(0);
		mesh1_reporter->addToReport(contactForce.getOutput(output_name));
		_model->addComponent(mesh1_reporter);
	}

	if (reporter_type == "real") {
		TableReporter* mesh1_reporter = new TableReporter();
		mesh1_reporter->setName(force_name + "_" + mesh_name + data_name);
		mesh1_reporter->set_report_time_interval(0);
		mesh1_reporter->addToReport(contactForce.getOutput(output_name));
		_model->addComponent(mesh1_reporter);
	}

	if (reporter_type == "vec3") {
		TableReporterVec3* mesh1_reporter = new TableReporterVec3();
		mesh1_reporter->setName(force_name + "_" + mesh_name + data_name);
		mesh1_reporter->set_report_time_interval(0);
		mesh1_reporter->addToReport(contactForce.getOutput(output_name));
		_model->addComponent(mesh1_reporter);
	}
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the beginning of an integration 
 *
 * @param s current state of System
 *
 * @return -1 on error, 0 otherwise.
 */
int WISCO_ContactAnalysis::begin( SimTK::State& s )
{
    if(!proceed()) return(0);
	

	//Find contact pairs for analysis
	const ForceSet& forceSet = _model->getForceSet();

	if (get_contact_names(0) == "all") {
		for (const WISCO_ElasticFoundationForce& contactForce : _model->getComponentList<WISCO_ElasticFoundationForce>()){
			_contact_force_names.push_back(contactForce.getName());			
		}
	}
	else {
		for (int i = 0; i < getProperty_contact_names().size(); ++i) {
			const WISCO_ElasticFoundationForce& contactForce = _model->getComponent<WISCO_ElasticFoundationForce>(get_contact_names(i));
			_contact_force_names.push_back(contactForce.getName());
		}
	}

	//Add Reporters
	for (int i = 0; i < _contact_force_names.size(); ++i) {
		
		WISCO_ElasticFoundationForce& contactForce = _model->updComponent
			<WISCO_ElasticFoundationForce>(_contact_force_names[i]);

		addContactReportersToModel(contactForce);
	}

	//States Reporter
	StatesTrajectoryReporter* states_reporter = new StatesTrajectoryReporter();
	states_reporter->setName("states_reporter");
	states_reporter->set_report_time_interval(0);
	_model->addComponent(states_reporter);

	_model->initSystem();

	//Setup WISCO_ElasticFoundationForces in model for contact analysis
	for (int i = 0; i < _contact_force_names.size(); ++i) {

		WISCO_ElasticFoundationForce& contactForce = _model->updComponent
			<WISCO_ElasticFoundationForce>(_contact_force_names[i]);

		contactForce.setModelingOption(s, "contact_analysis", 1);
		
		SimTK::String mesh_output_format =
			SimTK::String::toLower(get_output_data_mesh_format());

		if (mesh_output_format == "vertex" || mesh_output_format == "both") {
			contactForce.setModelingOption(s, "interpolate_vertex_data", 1);
		}

		if (get_h5_summary_contact_data()) {
			contactForce.setModelingOption(s, "contact_stats", 1);

			if (get_h5_medial_lateral_summary()) {
				contactForce.setModelingOption(s, "contact_stats_medial_lateral", 1);
			}
		}
	}
	
	//Setup Vertex location storage (dynamic output only)
	if (get_write_dynamic_vtk_files()) {
		_mesh1_vertex_locations.resize(_contact_force_names.size());
		_mesh2_vertex_locations.resize(_contact_force_names.size());

		for (int i = 0; i < _contact_force_names.size(); ++i) {

			int mesh1_nVer = _model->getComponent<WISCO_ElasticFoundationForce>
				(_contact_force_names[i]).getConnectee<WISCO_ContactMesh>
				("target_mesh").getPolygonalMesh().getNumVertices();

			_mesh1_vertex_locations[i].resize(mesh1_nVer, 0);

			int mesh2_nVer = _model->getComponent<WISCO_ElasticFoundationForce>
				(_contact_force_names[i]).getConnectee<WISCO_ContactMesh>
				("casting_mesh").getPolygonalMesh().getNumVertices();

			_mesh2_vertex_locations[i].resize(mesh2_nVer, 0);

		}
	}
	//record(s);
    return(0);
}


//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called 
 *
 * This method should be overridden in derived classes.  It is
 * included here so that the derived class will not have to implement it if
 * it is not necessary.
 *
 * @param s current state of system
 *
 * @return -1 on error, 0 otherwise.
 */
int WISCO_ContactAnalysis::step(const SimTK::State& s, int stepNumber )
{
	if (!proceed(stepNumber)) return(0);

	record(s);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration 
 *
 * This method should be overridden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param s current state of System
 *
 * @return -1 on error, 0 otherwise.
 */
int WISCO_ContactAnalysis::end( SimTK::State& s )
{
    if (!proceed()) return 0;

    record(s);

    return(0);
}




//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print results.
 * 
 * The file names are constructed as
 * aDir + "/" + aBaseName + "_" + ComponentName + aExtension
 *
 * @param aDir Directory in which the results reside.
 * @param aBaseName Base file name.
 * @param aDT Desired time interval between adjacent storage vectors.  Linear
 * interpolation is used to print the data out at the desired interval.
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int WISCO_ContactAnalysis::printResults(const std::string &aBaseName,const std::string &aDir,double aDT,
                 const std::string &aExtension)
{
	for (int i = 0; i < _contact_force_names.size(); ++i) {

		std::string contact_name = _contact_force_names[i];

		std::vector<SimTK::Matrix> mesh1FaceData, mesh1PointData;
		std::vector<std::string> mesh1FaceDataNames, mesh1PointDataNames;

		std::vector<SimTK::Matrix> mesh2FaceData, mesh2PointData;
		std::vector<std::string> mesh2FaceDataNames, mesh2PointDataNames;

		collectMeshOutputs(contact_name, 
			mesh1FaceData, mesh1FaceDataNames, mesh1PointData, mesh1PointDataNames,
			mesh2FaceData, mesh2FaceDataNames, mesh2PointData, mesh2PointDataNames);

		//Write Static VTK Files
		if (get_write_static_vtk_files()) {
			
			SimTK::PolygonalMesh mesh1 = _model->getComponent<WISCO_ElasticFoundationForce>(contact_name).
				getConnectee<WISCO_ContactMesh>("target_mesh").getPolygonalMesh();
			SimTK::PolygonalMesh mesh2 = _model->getComponent<WISCO_ElasticFoundationForce>(contact_name).
				getConnectee<WISCO_ContactMesh>("casting_mesh").getPolygonalMesh();

			std::string mesh1_name = _model->getComponent<WISCO_ElasticFoundationForce>(contact_name).
				getConnectee<WISCO_ContactMesh>("target_mesh").getName();
			std::string mesh2_name = _model->getComponent<WISCO_ElasticFoundationForce>(contact_name).
				getConnectee<WISCO_ContactMesh>("casting_mesh").getName();

			WISCO_VTKFileAdapter* mesh1_vtk = new WISCO_VTKFileAdapter();
			WISCO_VTKFileAdapter* mesh2_vtk = new WISCO_VTKFileAdapter();

			SimTK::String mesh_output_format =
				SimTK::String::toLower(get_output_data_mesh_format());

			int nTimeStep = 1;

			if (mesh_output_format == "face" || mesh_output_format == "both") {
				mesh1_vtk->setFaceData(mesh1FaceDataNames, mesh1FaceData);
				mesh2_vtk->setFaceData(mesh2FaceDataNames, mesh2FaceData);
				nTimeStep = mesh1FaceData[0].nrow();
			}

			if (mesh_output_format == "vertex" || mesh_output_format == "both") {
				mesh1_vtk->setPointData(mesh1PointDataNames, mesh1PointData);
				mesh2_vtk->setPointData(mesh2PointDataNames, mesh2PointData);
				nTimeStep = mesh1PointData[0].nrow();
			}

			mesh1_vtk->write(aBaseName + "_" + contact_name + "_" + mesh1_name + "_static", aDir +"/",
				mesh1,nTimeStep);
			mesh2_vtk->write(aBaseName + "_" + contact_name + "_" + mesh1_name + "_static", aDir + "/",
				mesh2, nTimeStep);
		}
		

		//Write Dynamic VTK Files
		if (get_write_dynamic_vtk_files()) {

			//Mesh1 Vertex Positions
			SimTK::PolygonalMesh mesh1 = _model->getComponent<WISCO_ElasticFoundationForce>(contact_name).
				getConnectee<WISCO_ContactMesh>("target_mesh").getPolygonalMesh();

			SimTK::Matrix mesh1_faces(mesh1.getNumFaces(), mesh1.getNumVerticesForFace(0));

			for (int j = 0; j < mesh1.getNumFaces(); ++j) {
				for (int k = 0; k < mesh1.getNumVerticesForFace(0); ++k) {
					mesh1_faces(j, k) = mesh1.getFaceVertex(j, k);
				}				
			}
			
			//Mesh2 Vertex Positions
			SimTK::PolygonalMesh mesh2 = _model->getComponent<WISCO_ElasticFoundationForce>(contact_name).
				getConnectee<WISCO_ContactMesh>("casting_mesh").getPolygonalMesh();

			SimTK::Matrix mesh2_faces(mesh2.getNumFaces(), mesh2.getNumVerticesForFace(0));

			for (int j = 0; j < mesh2.getNumFaces(); ++j) {
				for (int k = 0; k < mesh2.getNumVerticesForFace(0); ++k) {
					mesh2_faces(j, k) = mesh2.getFaceVertex(j, k);
				}
			}
			
			WISCO_VTKFileAdapter* mesh1_vtk = new WISCO_VTKFileAdapter();
			WISCO_VTKFileAdapter* mesh2_vtk = new WISCO_VTKFileAdapter();
			
			SimTK::String mesh_output_format =
				SimTK::String::toLower(get_output_data_mesh_format());

			int nTimeStep = 1;

			if (mesh_output_format == "face" || mesh_output_format == "both") {
				mesh1_vtk->setFaceData(mesh1FaceDataNames, mesh1FaceData);
				mesh2_vtk->setFaceData(mesh2FaceDataNames, mesh2FaceData);
				nTimeStep = mesh1FaceData[0].nrow();
			}

			if (mesh_output_format == "vertex" || mesh_output_format == "both") {
				mesh1_vtk->setPointData(mesh1PointDataNames, mesh1PointData);
				mesh2_vtk->setPointData(mesh2PointDataNames, mesh2PointData);
				nTimeStep = mesh1PointData[0].nrow();
			}

			std::string mesh1_name = _model->getComponent<WISCO_ElasticFoundationForce>(contact_name).
				getConnectee<WISCO_ContactMesh>("target_mesh").getName();
			std::string mesh2_name = _model->getComponent<WISCO_ElasticFoundationForce>(contact_name).
				getConnectee<WISCO_ContactMesh>("casting_mesh").getName();

			mesh1_vtk->write(aBaseName + "_" + contact_name + "_"+ mesh1_name + 
				"_dynamic_" + get_dynamic_output_frame(), aDir + "/",
				_mesh1_vertex_locations[i], mesh1_faces, nTimeStep);

			mesh2_vtk->write(aBaseName + "_" + contact_name + "_" + mesh2_name +
				"_dynamic_" + get_dynamic_output_frame(), aDir + "/",
				_mesh2_vertex_locations[i], mesh2_faces, nTimeStep);
		}
	}

	//Write h5 file
	if (get_write_h5_file()) {
		writeH5File(aBaseName, aDir);
	}
    return(0);
}

void WISCO_ContactAnalysis::collectMeshOutputs(const std::string& contact_name,
	std::vector<SimTK::Matrix>& mesh1FaceData, std::vector<std::string>& mesh1FaceDataNames,
	std::vector<SimTK::Matrix>& mesh1PointData, std::vector<std::string>& mesh1PointDataNames,
	std::vector<SimTK::Matrix>& mesh2FaceData, std::vector<std::string>& mesh2FaceDataNames,
	std::vector<SimTK::Matrix>& mesh2PointData, std::vector<std::string>& mesh2PointDataNames)
{

	std::string mesh1_name =  _model->getComponent<WISCO_ElasticFoundationForce>
		(contact_name).getConnectee<WISCO_ContactMesh>("target_mesh").getName();

	std::string mesh2_name = _model->getComponent<WISCO_ElasticFoundationForce>
		(contact_name).getConnectee<WISCO_ContactMesh>("casting_mesh").getName();

	for (TableReporterVector report : _model->getComponentList<TableReporterVector>()) {

		//Target Mesh
		std::string target_tri_name = contact_name + "_"+ mesh1_name + "_tri";
 		if (report.getName().compare(0, target_tri_name.size(), target_tri_name) == 0) {
			mesh1FaceDataNames.push_back(report.getName());
			mesh1FaceData.push_back(report.getTable().getMatrix().getAsMatrix());
		}

		std::string target_ver_name = contact_name + "_" + mesh1_name + "_vertex";
		if (report.getName().compare(0, target_ver_name.size(), target_ver_name) == 0) {
			mesh1PointDataNames.push_back(report.getName());
			mesh1PointData.push_back(report.getTable().getMatrix().getAsMatrix());
		}

		//Casting Mesh
		std::string casting_tri_name = contact_name + "_" + mesh2_name + "_tri";
		if (report.getName().compare(0, casting_tri_name.size(), casting_tri_name) == 0) {
			mesh2FaceDataNames.push_back(report.getName());
			mesh2FaceData.push_back(report.getTable().getMatrix().getAsMatrix());
		}

		std::string casting_ver_name = contact_name + "_" + mesh2_name + "_vertex";
		if (report.getName().compare(0, casting_ver_name.size(), casting_ver_name) == 0) {
			mesh2PointDataNames.push_back(report.getName());
			mesh2PointData.push_back(report.getTable().getMatrix().getAsMatrix());
		}
	}

}

void WISCO_ContactAnalysis::collectMeshContactSummary(const std::string& contact_name, 
	std::vector<SimTK::Vector>& mesh1DoubleData, std::vector<std::string>& mesh1DoubleNames,
	std::vector<SimTK::Matrix_<SimTK::Vec3>>& mesh1Vec3Data, std::vector<std::string>& mesh1Vec3Names,
	std::vector<SimTK::Vector>& mesh2DoubleData, std::vector<std::string>& mesh2DoubleNames,
	std::vector<SimTK::Matrix_<SimTK::Vec3>>& mesh2Vec3Data, std::vector<std::string>& mesh2Vec3Names) {
	
	std::string mesh1_name = _model->getComponent<WISCO_ElasticFoundationForce>
		(contact_name).getConnectee<WISCO_ContactMesh>("target_mesh").getName();

	std::string mesh2_name = _model->getComponent<WISCO_ElasticFoundationForce>
		(contact_name).getConnectee<WISCO_ContactMesh>("casting_mesh").getName();
	
	std::vector<std::string> double_val_names;

	double_val_names.push_back("_mean_pressure");
	double_val_names.push_back("_max_pressure");
	double_val_names.push_back("_mean_proximity");
	double_val_names.push_back("_max_proximity");
	double_val_names.push_back("_contact_area");

	std::vector<std::string> vec3_val_names;
	
	vec3_val_names.push_back("_cop");
	vec3_val_names.push_back("_contact_force");

	for (TableReporter report : _model->getComponentList<TableReporter>()) {
		
		//Target Mesh
		for (int i = 0; i < double_val_names.size(); ++i) {
			std::string base_name = contact_name + "_" + mesh1_name + double_val_names[i];
			if (report.getName().compare(0, base_name.size(), base_name) == 0) {
				mesh1DoubleNames.push_back(report.getName());
				mesh1DoubleData.push_back(report.getTable().getMatrix().getAsVector());
				SimTK::Matrix test = report.getTable().getMatrix().getAsMatrix();
				std::cout << test << std::endl;
			}
		}
		//Casting Mesh
		for (int i = 0; i < double_val_names.size(); ++i) {
			std::string base_name = contact_name + "_" + mesh2_name + double_val_names[i];
			if (report.getName().compare(0, base_name.size(), base_name) == 0) {
				mesh2DoubleNames.push_back(report.getName());
				mesh2DoubleData.push_back(report.getTable().getMatrix().getAsVector());
			}
		}
	}

	for (TableReporterVec3 report : _model->getComponentList<TableReporterVec3>()) {
		//Target Mesh
		for (int i = 0; i < vec3_val_names.size(); ++i) {
			std::string base_name = contact_name + "_" + mesh1_name + vec3_val_names[i];
			if (report.getName().compare(0, base_name.size(), base_name) == 0) {
				mesh1Vec3Names.push_back(report.getName());
				mesh1Vec3Data.push_back(report.getTable().getMatrix().getAsMatrix());
			}
		}
		//Casting Mesh
		for (int i = 0; i < vec3_val_names.size(); ++i) {
			std::string base_name = contact_name + "_" + mesh2_name + vec3_val_names[i];
			if (report.getName().compare(0, base_name.size(), base_name) == 0) {
				mesh2Vec3Names.push_back(report.getName());
				mesh2Vec3Data.push_back(report.getTable().getMatrix().getAsMatrix());
			}
		}
	}
}

void WISCO_ContactAnalysis::writeH5File(
	const std::string &aBaseName, const std::string &aDir)
{
	WISCO_H5FileAdapter h5_adapter;

	const std::string h5_file{ aDir + "/" + aBaseName + ".h5" };
	h5_adapter.open(h5_file);

	//Write States Data
	if (get_h5_write_states_data()) {
		TimeSeriesTable states_table = _model->getComponent<StatesTrajectoryReporter>("states_reporter").getStates().exportToTable(*_model);
		h5_adapter.writeStatesDataSet(states_table);
	}

	//Write Contact Data
	
	std::string cnt_group_name{ "/WISCO_ElasticFoundationForce" };
	h5_adapter.createGroup(cnt_group_name);

	for (int i = 0; i < _contact_force_names.size(); ++i) {

		std::string contact_name = _contact_force_names[i];
		
		//Write Mesh Contact Data
		addContactReportsToH5File(h5_adapter, cnt_group_name, contact_name);
	}
	h5_adapter.close();

}

void WISCO_ContactAnalysis::addContactReportsToH5File(
	WISCO_H5FileAdapter h5_adapt, 	const std::string& group_name, 
	const std::string& contact_name) 
{
	std::vector<std::string> mesh_names;
	
	mesh_names.push_back(_model->getComponent<WISCO_ElasticFoundationForce>
		(contact_name).getConnectee<WISCO_ContactMesh>("target_mesh").getName());

	mesh_names.push_back(_model->getComponent<WISCO_ElasticFoundationForce>
		(contact_name).getConnectee<WISCO_ContactMesh>("casting_mesh").getName());

	std::vector<std::string> real_names;
		
	real_names.push_back("mean_pressure");
	real_names.push_back("max_pressure");
	real_names.push_back("mean_proximity");
	real_names.push_back("max_proximity");
	real_names.push_back("contact_area");

	std::vector<std::string> vec3_names;

	vec3_names.push_back("cop");
	vec3_names.push_back("contact_force");

	std::vector<std::string> vector_names;

	vector_names.push_back("tri_pressure");
	vector_names.push_back("tri_proximity");
	vector_names.push_back("vertex_pressure");
	vector_names.push_back("vertex_proximity");

	//Create Groups in H5 File
	h5_adapt.createGroup(group_name + "/" + contact_name);


	for (std::string mesh_name : mesh_names) {

		std::string mesh_path = group_name + "/" + contact_name + "/" + mesh_name;
		h5_adapt.createGroup(mesh_path);

		for (TableReporter report : _model->getComponentList<TableReporter>()) {
			std::string report_name = report.getName();

			for (std::string real_name : real_names) {
				std::string base_name = contact_name + "_" + mesh_name + "_" + real_name;
				if (report_name.compare(0, base_name.size(), base_name) == 0) {
					std::string data_path = mesh_path + "/" + report_name.substr(report_name.find(real_name));
					h5_adapt.writeDataSet(report.getTable(), data_path);
				}
			}
		}

		for (TableReporterVec3 report : _model->getComponentList<TableReporterVec3>()) {
			std::string report_name = report.getName();
			
			for (std::string vec3_name : vec3_names) {
				std::string base_name = contact_name + "_" + mesh_name + "_" + vec3_name;
				if (report_name.compare(0, base_name.size(), base_name) == 0) {
					std::string data_path = mesh_path + "/" + report_name.substr(report_name.find(vec3_name));
					h5_adapt.writeDataSetVec3(report.getTable(), data_path);
				}
			}
		}

		for (TableReporterVector report : _model->getComponentList<TableReporterVector>()) {
			std::string report_name = report.getName();
			
			for (std::string vector_name : vector_names) {
				std::string base_name = contact_name + "_" + mesh_name + "_" + vector_name;
				if (report_name.compare(0, base_name.size(), base_name) == 0) {
					std::string data_path = mesh_path + "/" + report_name.substr(report_name.find(vector_name));
					h5_adapt.writeDataSetVector(report.getTable(), data_path);
				}
			}
		}
	}
}
