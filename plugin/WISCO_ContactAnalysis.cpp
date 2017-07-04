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
#include "WISCO_ElasticFoundationForceReporter.h"
#include "WISCO_IdealMuscleReporter.h"
#include "WISCO_CoordinateReporter.h"
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
	constructProperty_vtk_include_attached_geometry(true);
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

	//Contact Reporter
	WISCO_ElasticFoundationForceReporter* cnt_rep = new WISCO_ElasticFoundationForceReporter();
	cnt_rep->setName("contact_reporter");
	_model->addComponent(cnt_rep);
	//cnt_rep->finalizeFromProperties();

	//States	
	if (get_h5_states_data()) {
		StatesReporter* states_rep = new StatesReporter();
		states_rep->setName("states_analysis");
		states_rep->setStepInterval(getStepInterval());
		states_rep->setPrintResultFiles(false);
		//states_rep->setInDegrees(true);
		_model->addAnalysis(states_rep);
	}

	//Kinematics
	if(get_h5_kinematics_data()){
		WISCO_CoordinateReporter* coord_rep = new WISCO_CoordinateReporter();
		coord_rep->setName("coordinate_reporter");
		_model->addComponent(coord_rep);
	}

	//Muscle
	if (get_h5_muscle_data()) {
		WISCO_IdealMuscleReporter* msl_rep = new WISCO_IdealMuscleReporter();
		msl_rep->setName("muscle_reporter");
		_model->addComponent(msl_rep);
	}
	//Ligament
	if (get_h5_ligament_data()) {
		WISCO_LigamentReporter* lig_rep = new WISCO_LigamentReporter();
		lig_rep->setName("ligament_reporter");
		_model->addComponent(lig_rep);
	}

	s = _model->initSystem();
	
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
	setupDynamicVertexLocationStorage();
	//Repose state
	//s.setQ(initial_Q);
	//s.setU(initial_U);

	record(s);
	return(0);
}

void WISCO_ContactAnalysis::setupDynamicVertexLocationStorage() {
	if (get_write_dynamic_vtk_files()) {
		_mesh_vertex_locations.resize(_contact_mesh_names.size());

		for (int i = 0; i < _contact_mesh_names.size(); ++i) {

			int mesh_nVer = _model->getComponent<WISCO_ContactMesh>
				(_contact_mesh_names[i]).getPolygonalMesh().getNumVertices();

			_mesh_vertex_locations[i].resize(mesh_nVer, 0);
		}
	}

	if (get_vtk_include_attached_geometry()) {
		std::string model_file = SimTK::Pathname::getAbsolutePathname(_model->getDocumentFileName());
		std::string model_dir, dummy1, dummy2;
		bool dummyBool;
		SimTK::Pathname::deconstructPathname(model_file, dummyBool, model_dir, dummy1, dummy2);
		
		for (std::string cnt_name : _contact_force_names) {
			WISCO_ElasticFoundationForce& contactForce = _model->updComponent
				<WISCO_ElasticFoundationForce>(cnt_name);

			const PhysicalFrame& CparentFrame = contactForce.getConnectee
				<WISCO_ContactMesh>("casting_mesh").get_mesh_frame().getParentFrame();

			int nCParentAttachGeo = CparentFrame.getProperty_attached_geometry().size();

			for (int i = 0; i < nCParentAttachGeo; ++i) {

				const Geometry& geo = CparentFrame.get_attached_geometry(i);

				if (geo.getConcreteClassName() != "Mesh") {
					continue;
				}

				if (contains_string(_attach_geo_names, geo.getName())) {
					continue;
				}


				
				Mesh* mesh = (Mesh*)&geo;
				std::string file = mesh->get_mesh_file();
				SimTK::PolygonalMesh ply_mesh;
				ply_mesh.loadFile(model_dir + file);

				_attach_geo_names.push_back(geo.getName());
				_attach_geo_frames.push_back(CparentFrame.getName());
				_attach_geo_meshes.push_back(ply_mesh);
				_attach_geo_vertex_locations.push_back(SimTK::Matrix_<SimTK::Vec3>(ply_mesh.getNumVertices(), 0));
			}

			const PhysicalFrame& TparentFrame = contactForce.getConnectee
				<WISCO_ContactMesh>("target_mesh").get_mesh_frame().getParentFrame();

			int nTParentAttachGeo = TparentFrame.getProperty_attached_geometry().size();

			for (int i = 0; i < nTParentAttachGeo; ++i) {

				const Geometry& geo = TparentFrame.get_attached_geometry(i);

				if (geo.getConcreteClassName() != "Mesh") {
					continue;
				}

				if (contains_string(_attach_geo_names, geo.getName())) {
					continue;
				}

				Mesh* mesh = (Mesh*)&geo;
				std::string file = mesh->get_mesh_file();
				SimTK::PolygonalMesh ply_mesh;
				ply_mesh.loadFile(model_dir + file);

				_attach_geo_names.push_back(geo.getName());
				_attach_geo_frames.push_back(TparentFrame.getName());
				_attach_geo_meshes.push_back(ply_mesh);
				_attach_geo_vertex_locations.push_back(SimTK::Matrix_<SimTK::Vec3>(ply_mesh.getNumVertices(), 0));
			}
		}
	}
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
	std::string frame = get_dynamic_output_frame();

	if (get_write_dynamic_vtk_files()) {
		for (int i = 0; i < _contact_mesh_names.size(); ++i) {
			int nRow = _mesh_vertex_locations[i].nrow();
			int nCol = _mesh_vertex_locations[i].ncol();

			_mesh_vertex_locations[i].resizeKeep(nRow, nCol + 1);

			SimTK::Vector_<SimTK::Vec3> ver = _model->getComponent<WISCO_ContactMesh>
				(_contact_mesh_names[i]).getVertexLocationsInFrame(s, frame);

			for (int j = 0; j < nRow; ++j){
				_mesh_vertex_locations[i](j, nCol) = ver(j);
			}
		}

	}

	if (get_vtk_include_attached_geometry()) {
		for (int i = 0; i < _attach_geo_names.size(); ++i) {
			int nRow = _attach_geo_vertex_locations[i].nrow();
			int nCol = _attach_geo_vertex_locations[i].ncol();

			const SimTK::PolygonalMesh& mesh = _attach_geo_meshes[i];

			_attach_geo_vertex_locations[i].resizeKeep(nRow, nCol + 1);

			std::string out_frame_name = get_dynamic_output_frame();
			const Frame& out_frame = _model->getComponent<Frame>(out_frame_name);
			SimTK::Transform trans = _model->getComponent<PhysicalFrame>(_attach_geo_frames[i]).findTransformBetween(s, out_frame);

			for (int j = 0; j < mesh.getNumVertices(); ++j) {
				_attach_geo_vertex_locations[i](j, nCol) = trans.shiftFrameStationToBase(mesh.getVertexPosition(j));
			}
		}
	}
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

	if (get_vtk_include_attached_geometry()) {
		if (get_write_static_vtk_files()) {
			writeAttachedGeometryVTKFiles(aDir, aBaseName, false);
		}
		if (get_write_dynamic_vtk_files()) {
			writeAttachedGeometryVTKFiles(aDir, aBaseName,true);
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

void WISCO_ContactAnalysis::writeAttachedGeometryVTKFiles(std::string file_path, std::string base_name, bool isDynamic) {
	for (int i = 0; i < _attach_geo_names.size(); ++i) {
		
		//Face Connectivity
		const SimTK::PolygonalMesh& mesh = _attach_geo_meshes[i];

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

		int nTimeStep = _attach_geo_vertex_locations[0].ncol();

		if (isDynamic) {
			mesh_vtp->write(base_name + "_" + _attach_geo_names[i] + "_dynamic_" + 
				get_dynamic_output_frame(), file_path + "/",
				_attach_geo_vertex_locations[i], mesh_faces, nTimeStep);
		}
		else { //static
			mesh_vtp->write(base_name + "_" + _attach_geo_names[i] + "_static_" +
				get_dynamic_output_frame(), file_path + "/",
				mesh, nTimeStep);
		}
		delete mesh_vtp;
	}
}

void WISCO_ContactAnalysis::collectMeshData(const std::string& mesh_name,
	const std::vector<std::string>& contact_names,
	std::vector<SimTK::Matrix>& faceData, std::vector<std::string>& faceDataNames,
	std::vector<SimTK::Matrix>& pointData, std::vector<std::string>& pointDataNames)
{
	//Point and Vertex Data


	WISCO_ElasticFoundationForceReporter& cnt_rep = _model->updComponent<WISCO_ElasticFoundationForceReporter>("contact_reporter");
	std::vector<std::string> names = cnt_rep.getReporterNames();
	
	ComponentList<const TableReporterVector> cnt_reporter_vectors = cnt_rep.getTableReportersVector();

	
	for (TableReporterVector report : cnt_reporter_vectors) {
		std::string r_contact_name, r_mesh_name, r_data_type, r_data_name;

		decomposeReportName(report.getName(), r_contact_name, r_mesh_name, r_data_type, r_data_name);

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
					pointDataNames.push_back(r_data_type + r_data_name);
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
		TimeSeriesTable coord_table = _model->getComponent<WISCO_CoordinateReporter>("coordinate_reporter").getTableReporter().getTable();
			h5_adapter.writeCoordinatesDataSet(coord_table);
	}

	//Write Muscle Data
	if (get_h5_muscle_data()) {
		TimeSeriesTable msl_table = _model->getComponent<WISCO_IdealMuscleReporter>("muscle_reporter").getTableReporter().getTable();
		h5_adapter.writeMuscleDataSet(msl_table);
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
	WISCO_ElasticFoundationForce& frc = _model->
		updComponent<WISCO_ElasticFoundationForce>(contact_name);

	WISCO_ElasticFoundationForceReporter& frc_rep = _model->
		updComponent<WISCO_ElasticFoundationForceReporter>("contact_reporter");


	std::vector<std::string> mesh_names;
	mesh_names.push_back(frc.getConnectee<WISCO_ContactMesh>("target_mesh").getName());
	mesh_names.push_back(frc.getConnectee<WISCO_ContactMesh>("casting_mesh").getName());



	ComponentList<const TableReporterVector> cnt_reporters_vector = frc_rep.getTableReportersVector();

	std::vector<std::string> real_names = frc.getContactStatNames();
	std::vector<std::string> vec3_names = frc.getContactStatNamesVec3();
	std::vector<std::string> vector_names = frc.getMeshDataNames();

	//Create Groups in H5 File
	h5_adapt.createGroup(group_name + "/" + contact_name);

	for (std::string mesh_name : mesh_names) {

		std::string mesh_path = group_name + "/" + contact_name + "/" + mesh_name;
		h5_adapt.createGroup(mesh_path);
		if (get_h5_summary_contact_data()) {
			h5_adapt.createGroup(mesh_path + "/total");
		}
		if (get_h5_medial_lateral_summary()) {			
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

		TableReporter report = frc_rep.getTableReporter(contact_name, mesh_name);
		std::vector<std::string> report_labels = report.getTable().getColumnLabels();
		renameReportLabelsToH5Path(report_labels, mesh_path);
		h5_adapt.writeDataSetSimTKMatrixColumns(report.getTable().getMatrix().getAsMatrix(), report_labels);

		TableReporterVec3 report_vec3 = frc_rep.getTableReporterVec3(contact_name, mesh_name);
		std::vector<std::string> report_vec3_labels = report_vec3.getTable().getColumnLabels();
		renameReportLabelsToH5Path(report_vec3_labels, mesh_path);
		h5_adapt.writeDataSetSimTKMatrixVec3Columns(report_vec3.getTable().getMatrix().getAsMatrix(), report_vec3_labels);

		for (TableReporterVector report : cnt_reporters_vector) {
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
	std::vector<std::string> token = split_string(name,".");

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

void WISCO_ContactAnalysis::renameReportLabelsToH5Path(std::vector<std::string>& labels, std::string mesh_path)
	
{
	for (std::string& label : labels) {
		std::vector<std::string> token = split_string(label, ".");
		std::string region = token[1];
		std::string data_type = token[2];

		label = mesh_path + "/" + region + "/" + data_type;
	}
}