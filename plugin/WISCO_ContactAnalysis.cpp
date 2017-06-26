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
#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include "WISCO_VTPFileAdapter.h"
#include "H5Cpp.h"
#include "hdf5_hl.h"
#include "WISCO_ElasticFoundationForce.h"
#include "WISCO_HelperFunctions.h"
#include "WISCO_LigamentReporter.h"
#include <OpenSim/Analyses/ForceReporter.h>
#include <OpenSim/Analyses/StatesReporter.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Common/STOFileAdapter.h>
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
	setAuthors("Colin Smith");

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
	constructProperty_time_interval(0);
	constructProperty_output_pressure(true);
	constructProperty_output_proximity(true);
	constructProperty_output_data_mesh_format("face");
	constructProperty_h5_raw_contact_data(true);
	constructProperty_h5_summary_contact_data(true);
	constructProperty_h5_medial_lateral_summary(true);
	constructProperty_write_h5_file(true);
	constructProperty_h5_states_data(true);
	constructProperty_h5_kinematics_data(true);
	constructProperty_h5_muscle_data(true);
	constructProperty_h5_ligament_data(true);
	constructProperty_write_static_vtk_files(true);
	constructProperty_write_dynamic_vtk_files(true);
	constructProperty_dynamic_output_frame("ground");
	constructProperty_write_variable_property_vtk("none");

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
int WISCO_ContactAnalysis::begin(SimTK::State& s)
{
	if (!proceed()) return(0);

	//Save initial pose
	SimTK::Vector initial_Q = s.getQ();
	SimTK::Vector initial_U = s.getU();

	//Find contact pairs for analysis
	const ForceSet& forceSet = _model->getForceSet();

	if (get_contact_names(0) == "all") {
		for (const WISCO_ElasticFoundationForce& contactForce : _model->getComponentList<WISCO_ElasticFoundationForce>()) {
			_contact_force_names.push_back(contactForce.getName());

			std::string casting_mesh_name = contactForce.getConnectee<WISCO_ContactMesh>("casting_mesh").getName();
			std::string target_mesh_name = contactForce.getConnectee<WISCO_ContactMesh>("target_mesh").getName();

			if (!contains_string(_contact_mesh_names, casting_mesh_name)) {
				_contact_mesh_names.push_back(casting_mesh_name);
			}
			if (!contains_string(_contact_mesh_names, target_mesh_name)) {
				_contact_mesh_names.push_back(target_mesh_name);
			}
		}
	}
	else {
		for (int i = 0; i < getProperty_contact_names().size(); ++i) {
			const WISCO_ElasticFoundationForce& contactForce = _model->getComponent<WISCO_ElasticFoundationForce>(get_contact_names(i));
			_contact_force_names.push_back(contactForce.getName());

			std::string casting_mesh_name = contactForce.getConnectee<WISCO_ContactMesh>("casting_mesh").getName();
			std::string target_mesh_name = contactForce.getConnectee<WISCO_ContactMesh>("target_mesh").getName();

			if (!contains_string(_contact_mesh_names, casting_mesh_name)) {
				_contact_mesh_names.push_back(casting_mesh_name);
			}
			if (!contains_string(_contact_mesh_names, target_mesh_name)) {
				_contact_mesh_names.push_back(target_mesh_name);
			}
		}
	}

	//Add Contact Reporters
	for (int i = 0; i < _contact_force_names.size(); ++i) {

		WISCO_ElasticFoundationForce& contactForce = _model->updComponent
			<WISCO_ElasticFoundationForce>(_contact_force_names[i]);

		addContactReportersToModel(contactForce);
	}

	//States
	if (get_h5_states_data() || get_h5_muscle_data()) {
		StatesReporter* states_rep = new StatesReporter();
		states_rep->setName("states_analysis");
		states_rep->setStepInterval(getStepInterval());
		states_rep->setPrintResultFiles(false);
		//states_rep->setInDegrees(true);
		_model->addAnalysis(states_rep);
	}

	//Kinematics
	if(get_h5_kinematics_data()){
		Kinematics* kin_rep = new Kinematics();
		kin_rep->setName("kinematics_analysis");
		kin_rep->setStepInterval(getStepInterval());
		kin_rep->setPrintResultFiles(false);
		//kin_rep->setInDegrees(true);
		_model->addAnalysis(kin_rep);
	}

	//Muscle
	if (get_h5_muscle_data()) {
		MuscleAnalysis* msl_rep = new MuscleAnalysis();
		msl_rep->setName("muscle_analysis");
		msl_rep->setStepInterval(getStepInterval());
		msl_rep->setComputeMoments(false);
		msl_rep->setPrintResultFiles(false);
		_model->addAnalysis(msl_rep);
	}
	//Ligament
	if (get_h5_ligament_data()) {
		WISCO_LigamentReporter* lig_rep = new WISCO_LigamentReporter();
		lig_rep->setName("ligament_reporter");
		_model->addComponent(lig_rep);
	}

	s = _model->initSystem();

	//Setup WISCO_ElasticFoundationForces in model for contact analysis
	for (int i = 0; i < _contact_force_names.size(); ++i) {

		WISCO_ElasticFoundationForce& contactForce = _model->updComponent
			<WISCO_ElasticFoundationForce>(_contact_force_names[i]);

		contactForce.setModelingOption(s, "flip_meshes", 1);

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
		_mesh_vertex_locations.resize(_contact_mesh_names.size());

		for (int i = 0; i < _contact_mesh_names.size(); ++i) {

			int mesh_nVer = _model->getComponent<WISCO_ContactMesh>
				(_contact_mesh_names[i]).getPolygonalMesh().getNumVertices();

			_mesh_vertex_locations[i].resize(mesh_nVer, 0);
		}
	}


	//Repose state
	//s.setQ(initial_Q);
	//s.setU(initial_U);

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
int WISCO_ContactAnalysis::step(const SimTK::State& s, int stepNumber)
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
int WISCO_ContactAnalysis::end(SimTK::State& s)
{
	if (!proceed()) return 0;

	record(s);

	return(0);
}

int WISCO_ContactAnalysis::record(const SimTK::State& s)
{
	_model->realizeReport(s);

	//Store mesh vertex locations
	if (get_write_dynamic_vtk_files()) {
		std::string frame = get_dynamic_output_frame();

		for (int i = 0; i < _contact_mesh_names.size(); ++i) {

			//Target Mesh
			int nRow = _mesh_vertex_locations[i].nrow();
			int nCol = _mesh_vertex_locations[i].ncol();

			_mesh_vertex_locations[i].resizeKeep(nRow, nCol + 1);

			SimTK::Vector_<SimTK::Vec3> ver = _model->getComponent<WISCO_ContactMesh>
				(_contact_mesh_names[i]).getVertexLocationsInFrame(s, frame);

			double test = ver(0)(0);

			for (int j = 0; j < nRow; ++j){
				_mesh_vertex_locations[i](j, nCol) = ver(j);
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
	if (get_h5_raw_contact_data() || get_write_static_vtk_files() || get_write_dynamic_vtk_files()) {
		//Pressure Reporters
		if (get_output_pressure()) {

			//Triangle Pressure
			if (mesh_output_format == "face" || mesh_output_format == "both") {
				addContactReporter(contactForce, "target_mesh", mesh1_name, "tri","pressure","vector");
				addContactReporter(contactForce, "casting_mesh", mesh2_name, "tri","pressure","vector");
			}

			//Vertex Pressure
			if (mesh_output_format == "vertex" || mesh_output_format == "both") {
				addContactReporter(contactForce, "target_mesh", mesh1_name, "vertex","pressure", "vector");
				addContactReporter(contactForce, "casting_mesh", mesh2_name, "vertex","pressure","vector");

			}
		}

		//Proximity Reporters
		if (get_output_proximity()) {
			//Triangle Proximity
			if (mesh_output_format == "face" || mesh_output_format == "both") {
				addContactReporter(contactForce, "target_mesh", mesh1_name, "tri","proximity","vector");
				addContactReporter(contactForce, "casting_mesh", mesh2_name, "tri","proximity","vector");
			}
			//Vertex Proximity
			if (mesh_output_format == "vertex" || mesh_output_format == "both") {
				addContactReporter(contactForce, "target_mesh", mesh1_name,"vertex","proximity","vector");
				addContactReporter(contactForce, "casting_mesh", mesh2_name, "vertex","proximity","vector");
			}
		}
	}
	//Summary Stats
	if (get_h5_summary_contact_data()) {
		std::vector<std::string> stat_names;
		std::vector<std::string> stat_types;

		stat_names.push_back("mean_pressure");
		stat_types.push_back("real");
		stat_names.push_back("max_pressure");
		stat_types.push_back("real");
		stat_names.push_back("mean_proximity");
		stat_types.push_back("real");
		stat_names.push_back("max_proximity");
		stat_types.push_back("real");
		stat_names.push_back("contact_area");
		stat_types.push_back("real");
		stat_names.push_back("cop");
		stat_types.push_back("vec3");
		stat_names.push_back("contact_force");
		stat_types.push_back("vec3");

		for (int i = 0; i < stat_names.size(); ++i) {
				addContactReporter(contactForce, "target_mesh", mesh1_name, "total", stat_names[i], stat_types[i]);
				addContactReporter(contactForce, "casting_mesh", mesh2_name, "total", stat_names[i], stat_types[i]);
		}

		if (get_h5_medial_lateral_summary()) {
			for (int i = 0; i < stat_names.size(); ++i) {
				addContactReporter(contactForce, "target_mesh", mesh1_name, "medial", stat_names[i], stat_types[i]);
				addContactReporter(contactForce, "casting_mesh", mesh2_name, "medial", stat_names[i], stat_types[i]);

				addContactReporter(contactForce, "target_mesh", mesh1_name, "lateral", stat_names[i], stat_types[i]);
				addContactReporter(contactForce, "casting_mesh", mesh2_name, "lateral", stat_names[i], stat_types[i]);
			}
		}
	}
}

void WISCO_ContactAnalysis::addContactReporter(
	WISCO_ElasticFoundationForce& contactForce,
	const std::string& mesh_type, const std::string& mesh_name,
	const std::string& data_type, const std::string& data_name,
	const std::string& reporter_type) {

	std::string force_name = contactForce.getName();
	std::string reporter_name = force_name + "|" + mesh_name + "|" + data_type + "|" + data_name;
	std::string output_name = mesh_type + "_" + data_type + "_" + data_name;
	double time_interval = get_time_interval();

	if (reporter_type == "vector") {
		TableReporterVector* mesh1_reporter = new TableReporterVector();
		mesh1_reporter->setName(reporter_name);
		mesh1_reporter->set_report_time_interval(0);
		mesh1_reporter->addToReport(contactForce.getOutput(output_name));
		_model->addComponent(mesh1_reporter);
	}

	if (reporter_type == "real") {
		TableReporter* mesh1_reporter = new TableReporter();
		mesh1_reporter->setName(reporter_name);
		mesh1_reporter->set_report_time_interval(0);
		mesh1_reporter->addToReport(contactForce.getOutput(output_name));
		_model->addComponent(mesh1_reporter);
	}

	if (reporter_type == "vec3") {
		TableReporterVec3* mesh1_reporter = new TableReporterVec3();
		mesh1_reporter->setName(reporter_name);
		mesh1_reporter->set_report_time_interval(0);
		mesh1_reporter->addToReport(contactForce.getOutput(output_name));
		_model->addComponent(mesh1_reporter);
	}
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
	for (std::string mesh_name : _contact_mesh_names) {

		//Write Static VTK Files
		if (get_write_static_vtk_files()) {
			writeVTKFile(aDir, aBaseName, mesh_name, _contact_force_names, false);
		}

		//Write Dynamic VTK Files
		if (get_write_dynamic_vtk_files()) {
			writeVTKFile(aDir, aBaseName, mesh_name, _contact_force_names, true);
		}
	}

	//Write h5 file
	if (get_write_h5_file()) {
		writeH5File(aBaseName, aDir);
	}
    return(0);
}

void WISCO_ContactAnalysis::writeVTKFile(const std::string& file_path,
	const std::string& base_name, const std::string& mesh_name,
	const std::vector<std::string>& contact_names, bool isDynamic) {

	//Collect data
	std::vector<SimTK::Matrix> faceData, pointData;
	std::vector<std::string> faceDataNames, pointDataNames;

	collectMeshData(mesh_name,contact_names,
		faceData, faceDataNames, pointData, pointDataNames);

	//Mesh face connectivity
	SimTK::PolygonalMesh mesh = _model->getComponent<WISCO_ContactMesh>
		(mesh_name).getPolygonalMesh();

	SimTK::Matrix mesh_faces(mesh.getNumFaces(), mesh.getNumVerticesForFace(0));

	for (int j = 0; j < mesh.getNumFaces(); ++j) {
		for (int k = 0; k < mesh.getNumVerticesForFace(0); ++k) {
			mesh_faces(j, k) = mesh.getFaceVertex(j, k);
		}
	}

	//Write file
	WISCO_VTPFileAdapter* mesh_vtp = new WISCO_VTPFileAdapter();
	mesh_vtp->setDataFormat("binary");

	SimTK::String mesh_output_format =
		SimTK::String::toLower(get_output_data_mesh_format());

	int nTimeStep = 1;

	if (mesh_output_format == "face" || mesh_output_format == "both") {
		mesh_vtp->setFaceData(faceDataNames, faceData);
		nTimeStep = faceData[0].nrow();
	}

	if (mesh_output_format == "vertex" || mesh_output_format == "both") {
		mesh_vtp->setPointData(pointDataNames, pointData);
		nTimeStep = pointData[0].nrow();
	}

	if (isDynamic) {
		int mesh_index;
		contains_string(_contact_mesh_names, mesh_name, mesh_index);

		mesh_vtp->write(base_name + "_" + mesh_name +
			"_dynamic_" + get_dynamic_output_frame(), file_path + "/",
			_mesh_vertex_locations[mesh_index], mesh_faces, nTimeStep);
	}
	else { //static
		SimTK::PolygonalMesh poly_mesh =
			_model->getComponent<WISCO_ContactMesh>(mesh_name).getPolygonalMesh();

		mesh_vtp->write(base_name + "_" + mesh_name +
			"_static_" + get_dynamic_output_frame(), file_path + "/",
			poly_mesh, nTimeStep);
	}
	delete mesh_vtp;

}

void WISCO_ContactAnalysis::collectMeshData(const std::string& mesh_name,
	const std::vector<std::string>& contact_names,
	std::vector<SimTK::Matrix>& faceData, std::vector<std::string>& faceDataNames,
	std::vector<SimTK::Matrix>& pointData, std::vector<std::string>& pointDataNames)
{
	//Point and Vertex Data
	for (TableReporterVector report : _model->getComponentList<TableReporterVector>()) {
		std::string r_contact_name, r_mesh_name, r_data_type, r_data_name;
		decomposeReportName(report.getName(),
			r_contact_name, r_mesh_name, r_data_type, r_data_name);

		if (r_mesh_name != mesh_name) continue;

		if (contains_string(contact_names, r_contact_name)) {

			if (r_data_type == "tri") {
				faceData.push_back(report.getTable().getMatrix().getAsMatrix());
				faceDataNames.push_back(r_data_type + r_data_name + r_contact_name);

				//combine contact forces
				int data_index;
				if (contains_string(faceDataNames, r_data_type + r_data_name, data_index)) {
					faceData[data_index] += report.getTable().getMatrix().getAsMatrix();
				}
				else {
					faceDataNames.push_back(r_data_type + r_data_name);
					faceData.push_back(report.getTable().getMatrix().getAsMatrix());
				}
			}
			else if (r_data_type == "vertex") {
				pointData.push_back(report.getTable().getMatrix().getAsMatrix());
				pointDataNames.push_back(r_data_type + r_data_name + r_contact_name);

				//combine contact forces
				int data_index;
				if (contains_string(pointDataNames, r_data_type + r_data_name, data_index)) {
					pointData[data_index] += report.getTable().getMatrix().getAsMatrix();
				}
				else {
					pointDataNames.push_back(r_data_type+r_data_name);
					pointData.push_back(report.getTable().getMatrix().getAsMatrix());
				}
			}
		}
	}

	//Variable Cartilage Properties
	if (get_write_variable_property_vtk() == "thickness" || get_write_variable_property_vtk() == "all") {
		SimTK::Vector face_thickness = _model->getComponent<WISCO_ContactMesh>(mesh_name).getTriangleThickness();
		faceDataNames.push_back("thickness");
		faceData.push_back(face_thickness);
	}
	if (get_write_variable_property_vtk() == "elastic modulus" || get_write_variable_property_vtk() == "all") {
		SimTK::Vector face_E = _model->getComponent<WISCO_ContactMesh>(mesh_name).getTriangleElasticModulus();
		faceDataNames.push_back("elastic modulus");
		faceData.push_back(face_E);
	}
	if (get_write_variable_property_vtk() == "poissons ratio" || get_write_variable_property_vtk() == "all") {
		SimTK::Vector face_v = _model->getComponent<WISCO_ContactMesh>(mesh_name).getTrianglePoissonsRatio();
		faceDataNames.push_back("poissons ratio");
		faceData.push_back(face_v);
	}
}

void WISCO_ContactAnalysis::writeH5File(
	const std::string &aBaseName, const std::string &aDir)
{
	WISCO_H5FileAdapter h5_adapter;

	const std::string h5_file{ aDir + "/" + aBaseName + ".h5" };
	h5_adapter.open(h5_file);

	//Write States Data
	if (get_h5_states_data()) {
		//TimeSeriesTable states_table = _model->getComponent<StatesTrajectoryReporter>("states_reporter").getStates().exportToTable(*_model);
		//h5_adapter.writeStatesDataSet(states_table);

		StatesReporter& states_analysis = dynamic_cast<StatesReporter&>(_model->updAnalysisSet().get("states_analysis"));
		const TimeSeriesTable& states_table = states_analysis.getStatesStorage().getAsTimeSeriesTable();
		h5_adapter.writeStatesDataSet(states_table);
	}

	if (get_h5_kinematics_data()) {
		Kinematics& kin_analysis = dynamic_cast<Kinematics&>(_model->updAnalysisSet().get("kinematics_analysis"));
		const TimeSeriesTable& pos_table = kin_analysis.getPositionStorage()->getAsTimeSeriesTable();
		const TimeSeriesTable& vel_table = kin_analysis.getVelocityStorage()->getAsTimeSeriesTable();
		const TimeSeriesTable& acc_table = kin_analysis.getAccelerationStorage()->getAsTimeSeriesTable();
		//_model->getSimbodyEngine().convertRadiansToDegrees(pos_table);
		//_model->getSimbodyEngine().convertRadiansToDegrees(vel_table);
		//_model->getSimbodyEngine().convertRadiansToDegrees(acc_table);
		h5_adapter.writeKinematicsDataSet(pos_table, vel_table, acc_table);
	}

	//Write Muscle Data
	if (get_h5_muscle_data()) {
		StatesReporter& states_analysis = dynamic_cast<StatesReporter&>(_model->updAnalysisSet().get("states_analysis"));
		const TimeSeriesTable& states_table = states_analysis.getStatesStorage().getAsTimeSeriesTable();

		MuscleAnalysis& msl_analysis = dynamic_cast<MuscleAnalysis&>(_model->updAnalysisSet().get("muscle_analysis"));
		const TimeSeriesTable& force_table = msl_analysis.getForceStorage()->getAsTimeSeriesTable();

		std::vector<std::string> msl_names = force_table.getColumnLabels();

		std::vector<SimTK::Matrix> msl_vec;
		msl_vec.push_back(force_table.getMatrix().getAsMatrix());

		std::vector<std::string> param_names;
		param_names.push_back("force");

		h5_adapter.writeMuscleDataSet(msl_vec, msl_names, param_names, states_table);
	}

	//Write Ligament Data
	if (get_h5_ligament_data()) {
		TimeSeriesTable lig_table = _model->getComponent<WISCO_LigamentReporter>("ligament_reporter").getTableReporter().getTable();
		h5_adapter.writeLigamentDataSet(lig_table);
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
	WISCO_H5FileAdapter& h5_adapt, 	const std::string& group_name,
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

	vector_names.push_back("pressure");
	vector_names.push_back("proximity");

	//Create Groups in H5 File
	h5_adapt.createGroup(group_name + "/" + contact_name);

	for (std::string mesh_name : mesh_names) {

		std::string mesh_path = group_name + "/" + contact_name + "/" + mesh_name;
		h5_adapt.createGroup(mesh_path);

		if (get_h5_medial_lateral_summary()) {
			h5_adapt.createGroup(mesh_path + "/total");
			h5_adapt.createGroup(mesh_path + "/medial");
			h5_adapt.createGroup(mesh_path + "/lateral");
		}

		std::string data_format = SimTK::String::toLower(get_output_data_mesh_format());
		if (data_format == "face" || data_format == "both") {
			h5_adapt.createGroup(mesh_path + "/tri");
		}
		if (data_format == "vertex" || data_format == "both") {
			h5_adapt.createGroup(mesh_path + "/vertex");
		}

		for (TableReporter report : _model->getComponentList<TableReporter>()) {
			std::string r_contact_name, r_mesh_name, r_data_type, r_data_name;
			decomposeReportName(report.getName(),
				r_contact_name, r_mesh_name, r_data_type, r_data_name);

			if (r_contact_name != contact_name) continue;
			if (r_mesh_name != mesh_name) continue;

			if (contains_string(real_names, r_data_name)) {
				std::string data_path = mesh_path + "/" + r_data_type + "/" + r_data_name;
				h5_adapt.writeDataSet(report.getTable(), data_path);
			}
		}

		for (TableReporterVec3 report : _model->getComponentList<TableReporterVec3>()) {
			std::string r_contact_name, r_mesh_name, r_data_type, r_data_name;
			decomposeReportName(report.getName(),
				r_contact_name, r_mesh_name, r_data_type, r_data_name);

			if (r_contact_name != contact_name) continue;
			if (r_mesh_name != mesh_name) continue;

			if (contains_string(vec3_names, r_data_name)) {
				std::string data_path = mesh_path + "/" + r_data_type + "/" + r_data_name;
				h5_adapt.writeDataSetVec3(report.getTable(), data_path);
			}
		}

		for (TableReporterVector report : _model->getComponentList<TableReporterVector>()) {
			std::string r_contact_name, r_mesh_name, r_data_type, r_data_name;
			decomposeReportName(report.getName(),
				r_contact_name, r_mesh_name, r_data_type, r_data_name);

			if (r_contact_name != contact_name) continue;
			if (r_mesh_name != mesh_name) continue;

			if (contains_string(vector_names, r_data_name)) {
				std::string data_path = mesh_path + "/" + r_data_type + "/" + r_data_name;
				h5_adapt.writeDataSetVector(report.getTable(), data_path);
			}
		}
	}
}

void WISCO_ContactAnalysis::decomposeReportName(const std::string& name,
	std::string& contact_name, std::string& mesh_name,
	std::string& data_type, std::string& data_name)
{
	std::vector<std::string> token = split_string(name,"|");

	if (token.size() != 4) {
		contact_name = "";
		mesh_name = "";
		data_type = "";
		data_name = "";
		return;
	}

	contact_name = token[0];
	mesh_name = token[1];
	data_type = token[2];
	data_name = token[3];
}
