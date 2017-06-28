/* -------------------------------------------------------------------------- *
*                       OpenSim:  ContactGeometry.cpp                        *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2016 Stanford University and the Authors                *
* Author(s): Peter Eastman                                                   *
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

#include "WISCO_ContactMesh.h"
#include <OpenSim/Common/ScaleSet.h>
#include "OpenSim/Common/Object.h"
#include "OpenSim/Simulation/SimbodyEngine/Body.h"
#include "OpenSim/Simulation/Model/Model.h"
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include "coldet.h"

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy
// assignment operator.

//_____________________________________________________________________________
// Default constructor.
WISCO_ContactMesh::WISCO_ContactMesh() : ModelComponent()
{
	setNull();
	constructProperties();
	mesh_is_cached = false;
}

//_____________________________________________________________________________
// Convenience constructor.
WISCO_ContactMesh::WISCO_ContactMesh(const std::string& name, const std::string& mesh_file, const PhysicalFrame& parent_frame) :
	WISCO_ContactMesh()
{
	set_file_name(mesh_file);
    updSocket<PhysicalFrame>("parent_frame").connect(parent_frame);
	setName(name);
	
	PhysicalOffsetFrame mesh_frame(parent_frame.getName()+ "_offset", parent_frame,SimTK::Transform());
	mesh_frame.setParentFrame(getConnectee<PhysicalFrame>("parent_frame"));
	set_mesh_frame(mesh_frame);
}

WISCO_ContactMesh::WISCO_ContactMesh(const std::string& name, const std::string& mesh_file, const PhysicalFrame& frame,
	const SimTK::Vec3& location, const SimTK::Vec3& orientation) : WISCO_ContactMesh(name,mesh_file,frame)
{
	upd_mesh_frame().set_orientation(orientation);
	upd_mesh_frame().set_translation(location);

}

WISCO_ContactMesh::WISCO_ContactMesh(const std::string& name, const std::string& mesh_file, const PhysicalFrame& frame,
	const SimTK::Vec3& location, const SimTK::Vec3& orientation,
	const std::string& mesh_back_file, double min_thickness, double max_thickness) : WISCO_ContactMesh(name, mesh_file, frame, location, orientation)
{
	set_mesh_back_file(mesh_back_file);
	set_min_thickness(min_thickness);
	set_max_thickness(max_thickness);

	
}

void WISCO_ContactMesh::setNull()
{
	setAuthors("Colin Smith");
}

void WISCO_ContactMesh::constructProperties()
{
	constructProperty_file_name("");
	constructProperty_mesh_frame(PhysicalOffsetFrame());
	constructProperty_display_preference(1);
	constructProperty_medial_lateral_axis("x");
	constructProperty_mesh_back_file("");
	constructProperty_material_properties_file("");
	constructProperty_min_thickness(0.0);
	constructProperty_max_thickness(0.01);


	Array<double> defaultColor(1.0, 3); //color default to 0, 1, 1
	defaultColor[0] = 0.0;
	constructProperty_color(defaultColor);
}

void WISCO_ContactMesh::extendFinalizeFromProperties() {
	Super::extendFinalizeFromProperties();
	if (!mesh_is_cached) {
		initializeMesh();
	}
	//computeVariableCartilageThickness();
}
/*
void WISCO_ContactMesh::extendSetPropertiesFromState(const SimTK::State& state) {
	Super::extendSetPropertiesFromState(state);
	initializeMesh();
	//computeVariableCartilageThickness();
}
*/
void WISCO_ContactMesh::extendAddToSystem(SimTK::MultibodySystem &system) const {
    Super::extendAddToSystem(system);
	CollisionModel3D* default_coldet_model = newCollisionModel3D();
    addCacheVariable("coldet_model", default_coldet_model, SimTK::Stage::LowestRuntime);
}

void WISCO_ContactMesh::extendInitStateFromProperties(SimTK::State& state) const {
    Super::extendInitStateFromProperties(state);
	CollisionModel3D* coldet_model = initColdetModel(mesh);


	setCacheVariableValue<CollisionModel3D*>(state, "coldet_model", coldet_model);
}

void WISCO_ContactMesh::extendRealizeReport(const SimTK::State &state) const {
	Super::extendRealizeReport(state);
}

CollisionModel3D* WISCO_ContactMesh::initColdetModel(const SimTK::PolygonalMesh& cnt_mesh) const {

    // Initialize Coldet model
    CollisionModel3D* coldet_model = newCollisionModel3D();

    // Add triangles to Coldet Model and compute properties
    //==================================================
    for (int i = 0; i < mesh.getNumFaces(); ++i) {

        // Get Triangle Vertice Positions
        int v1_i = mesh.getFaceVertex(i, 0);
        int v2_i = mesh.getFaceVertex(i, 1);
        int v3_i = mesh.getFaceVertex(i, 2);

        SimTK::Vec3 v1 = mesh.getVertexPosition(v1_i);
        SimTK::Vec3 v2 = mesh.getVertexPosition(v2_i);
        SimTK::Vec3 v3 = mesh.getVertexPosition(v3_i);

        // Add Triangle to COLDET Model
        coldet_model->addTriangle(v1[0], v1[1], v1[2], v2[0], v2[1], v2[2],
			v3[0], v3[1], v3[2]);
    }

    //Finalize COLDET model
    coldet_model->finalize();

	return coldet_model;
}

std::string WISCO_ContactMesh::findMeshFile(const std::string& file)
{
	/*This is a modified version of the code found in Geometry.cpp
	Mesh::extendFinalizeFromProperties to find geometry files in
	the same directory as the modelDir, modelDir/Geometry, or 
	the installDir: OPENSIM_HOME/Geometry

	It plays some games to figure out the modelDir, because the
	WISCO_ContactMesh can't call getModel() at this stage
	*/

	bool isAbsolutePath; 
	std::string directory, fileName, extension;
	
	SimTK::Pathname::deconstructPathname(file, isAbsolutePath, directory, fileName, extension);
	const std::string lowerExtension = SimTK::String::toLower(extension);
	
	//Check for correct extension
	if (lowerExtension != ".vtp" && lowerExtension != ".obj" && lowerExtension != ".stl") {
		std::cout << "WISCO_ContactMesh ERROR: '" << file << "'; only .vtp .stl and .obj files currently supported.\n";
		throw OpenSim::Exception("WISCO_ContactMesh: Bad file type.");
	}

	// Find OpenSim modelDir
	const Component* rootModel = nullptr;
	if (!hasOwner()) {
		std::cout << "Mesh " << file << " not connected to model..ignoring\n";
		return file;   // Orphan Mesh not part of a model yet
	}
	const Component* parent = &getOwner();
	while (parent != nullptr) {
		if (dynamic_cast<const Model*>(parent) != nullptr) {
			rootModel = parent;
			break;
		}
		if (parent->hasOwner())
			parent = &(parent->getOwner()); // traverse up Component tree
		else
			break; // can't traverse up.
	}

	if (rootModel == nullptr) {
		std::cout << "Mesh " << file << " not connected to model..ignoring\n";
		return file;   // Orphan Mesh not descendent of a model
	}
	//const Model& model = dynamic_cast<const Model&>(*rootModel);
	std::string osimFileName = rootModel->getDocumentFileName();
	

	//Find geometry file
	Model model;
	model.setInputFileName(osimFileName);
	
	SimTK::Array_<std::string> attempts;

	bool foundIt = ModelVisualizer::findGeometryFile(model, file, isAbsolutePath, attempts);

	if (!foundIt) {
		std::cout << "WISCO_ContactMesh couldn't find file '" << file << "'; tried\n";
		for (unsigned i = 0; i < attempts.size(); ++i)
			std::cout << "  " << attempts[i] << "\n";
		if (!isAbsolutePath && !SimTK::Pathname::environmentVariableExists("OPENSIM_HOME")) {
			std::cout << "Set environment variable OPENSIM_HOME " << "to search $OPENSIM_HOME/Geometry.\n";
		}
		throw OpenSim::Exception("WISCO_ContactMesh: File NOT found.");
	}

	return attempts.back();
}

void WISCO_ContactMesh::initializeMesh()
{
	mesh_is_cached = true;

	//Clear mesh in case of multiple calls
	//mesh.clear();

	// Initialize Reused Variables
	double mag;
	SimTK::Vec3 e1, e2, cross;

	// Load Mesh from file

	std::string file = findMeshFile(get_file_name());
	mesh.loadFile(file);

		
	tri_center.resize(mesh.getNumFaces());
	tri_normal.resize(mesh.getNumFaces());
	tri_area.resize(mesh.getNumFaces());
	tri_thickness.resize(mesh.getNumFaces());
	tri_elastic_modulus.resize(mesh.getNumFaces());
	tri_poissons_ratio.resize(mesh.getNumFaces());

	vertex_locations.resize(mesh.getNumVertices());
	face_vertex_locations.resize(mesh.getNumFaces(), 3);
		
	

	// Compute Mesh Properties
	//========================

	//setup medial lateral 
	SimTK::String ml_axis =
		SimTK::String::toLower(get_medial_lateral_axis());

	int ml_ind;
	if (ml_axis == "x") ml_ind = 0;
	else if (ml_axis == "y") ml_ind = 1;
	else if (ml_axis == "z") ml_ind = 2;

	lat_tri_ind.resize(mesh.getNumFaces());
	med_tri_ind.resize(mesh.getNumFaces());

	int nLat = 0;
	int nMed = 0;

	for (int i = 0; i < mesh.getNumFaces(); ++i) {

		// Get Triangle Vertice Positions
		int v1_i = mesh.getFaceVertex(i, 0);
		int v2_i = mesh.getFaceVertex(i, 1);
		int v3_i = mesh.getFaceVertex(i, 2);

		SimTK::Vec3 v1 = mesh.getVertexPosition(v1_i);
		SimTK::Vec3 v2 = mesh.getVertexPosition(v2_i);
		SimTK::Vec3 v3 = mesh.getVertexPosition(v3_i);

		// Compute Triangle Center
		tri_center(i)(0) = (v1(0) + v2(0) + v3(0)) / 3.0;
		tri_center(i)(1) = (v1(1) + v2(1) + v3(1)) / 3.0;
		tri_center(i)(2) = (v1(2) + v2(2) + v3(2)) / 3.0;

		// Compute Triangle Normal
		e1 = v3 - v1;
		e2 = v2 - v1;

		cross[0] = e1[1] * e2[2] - e1[2] * e2[1];
		cross[1] = e1[2] * e2[0] - e1[0] * e2[2];
		cross[2] = e1[0] * e2[1] - e1[1] * e2[0];

		mag = sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);//normal;

		for (int j = 0; j < 3; ++j) {
			tri_normal(i)(j) = -cross[j] / mag;
		}

		// Compute Triangle Area
		double s, s1, s2, s3;
		// Compute length of each side of the triangle
		s1 = s2 = s3 = 0.;
		for (int i = 0; i<3; i++) {
			s1 += (v2[i] - v1[i])*(v2[i] - v1[i]);
			s2 += (v3[i] - v2[i])*(v3[i] - v2[i]);
			s3 += (v1[i] - v3[i])*(v1[i] - v3[i]);
		}

		// Now employ Heron's formula
		s = (sqrt(s1) + sqrt(s2) + sqrt(s3)) / 2;
		tri_area[i] = sqrt(s*(s - s1)*(s - s2)*(s - s3));

		//Determine ML triangles
		if (tri_center(i)(ml_ind) > 0.0) {

			lat_tri_ind(nLat) = i;
			nLat++;
		}

		if (tri_center(i)(ml_ind) < 0.0) {
			med_tri_ind(nMed) = i;
			nMed++;
		}
	}

	_n_medial_tri = nMed;
	_n_lateral_tri = nLat;

	//Vertex Locations

	for (int i = 0; i < mesh.getNumVertices(); ++i) {
		vertex_locations(i) = mesh.getVertexPosition(i);
	}

	//Face Vertex Locations
	

    for (int i = 0; i < mesh.getNumFaces(); ++i) {
		for (int j = 0; j < 3; ++j) {
			int v_ind = mesh.getFaceVertex(i, j);
			face_vertex_locations(i,j) = mesh.getVertexPosition(v_ind);
		}
    }

	//Vertex Connectivity
	int max_ver_nTri = 20;
	
	SimTK::Matrix ver_tri_ind(mesh.getNumVertices(), max_ver_nTri);
	SimTK::Vector ver_nTri(mesh.getNumVertices());
	ver_tri_ind.setToZero();
	ver_nTri.setToZero();
	

	for (int i = 0; i < mesh.getNumFaces(); ++i) {
		for (int j = 0; j < 3; ++j) {
			int ver = mesh.getFaceVertex(i, j);
			ver_tri_ind(ver, ver_nTri(ver)) = i;
			ver_nTri(ver)++;

			if (ver_nTri(ver) == max_ver_nTri) {
				max_ver_nTri +=5;
				ver_tri_ind.resizeKeep(mesh.getNumVertices(), max_ver_nTri);
			}
		}
	}

	int max_nNeighbors = 20; // Will increase automatically if necessary

		tri_neighbors.resize(mesh.getNumFaces(), max_nNeighbors);
		tri_neighbors.setToZero();
		nTriNeighbors.resize(mesh.getNumFaces());
		nTriNeighbors.setToZero();
	
	for (int i = 0; i < mesh.getNumFaces(); ++i) {
		for (int j = 0; j < 3; ++j) {

			int ver = mesh.getFaceVertex(i, j);

			for (int k = 0; k < ver_nTri(ver); ++k) {
				bool repeat_tri = false;
				int tri = ver_tri_ind(ver, k);

				if (tri == i) {
					continue;
				}

				for (int l = 0; l < nTriNeighbors(i); ++l) {
					if (tri == tri_neighbors(i, l)){
						repeat_tri = true;
						break;
					}
				}

				if (repeat_tri)
					continue;

				tri_neighbors(i, nTriNeighbors(i)) = tri;
				nTriNeighbors(i)++;

				if (nTriNeighbors(i) == max_nNeighbors) {
					max_nNeighbors +=5;
					tri_neighbors.resizeKeep(mesh.getNumFaces(), max_nNeighbors);
				}
			}

		}
	}
}

void WISCO_ContactMesh::computeVariableCartilageThickness() {
		// Get Mesh Properties
		double min_thickness = get_min_thickness();
		double max_thickness = get_max_thickness();

		//Clear mesh in case of multiple calls
		mesh_back.clear();

		// Load Mesh from file
		std::string file = findMeshFile(get_mesh_back_file());
		mesh_back.loadFile(file);

		mesh_back.loadFile(get_mesh_back_file());

		//Construct Coldet Model for mesh_back
		CollisionModel3D* mb = initColdetModel(mesh_back);

		/*
		auto coldet_transform = ~GtoM1.toMat44();

		float m1_transform[16];
		int count = 0;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				m1_transform[count] = coldet_transform(i, j);
				count++;
			}
		}
		m1->setTransform(m1_transform);
		*/

		//Compute Thickness for each triangle in mesh
		//===========================================

		float origin[3], vector[3];

		//Loop through all triangles in mesh
		for (int i = 0; i < mesh.getNumFaces(); ++i) {
			double depth = 0.0;
			SimTK::Vec3 contact_pnt(0);

			//Go through the OBB hierarchy
			//----------------------------

			for (int j = 0; j < 3; ++j) {
				origin[j] = tri_center(i)(j);
				vector[j] = -tri_normal(i)(j);
			}

			if (mb->rayCollision(origin, vector, true, 0, max_thickness)) {
				//Colliding Triangles
				int tri1, tri2;
				mb->getCollidingTriangles(tri1, tri2);

				// Compute Proximity
				float cnt_point[3];
				mb->getCollisionPoint(cnt_point, false);

				for (int j = 0; j < 3; ++j) {
					contact_pnt(j) = cnt_point[j];
				}

				SimTK::Vec3 cen_cnt_point = contact_pnt - tri_center(i);

				depth = sqrt(pow(cen_cnt_point(0), 2) + pow(cen_cnt_point(1), 2) + pow(cen_cnt_point(2), 2));

				if (depth < min_thickness) {
					depth = min_thickness;
				}

				if (depth > max_thickness) {
					depth = max_thickness;
				}
			}
			else{ //Normal from mesh missed mesh back
				depth = (min_thickness + max_thickness) / 2;
			}

			tri_thickness(i) = depth;

		}
}

SimTK::Transform WISCO_ContactMesh::getTransformGroundToMesh(const SimTK::State& state) const
{
	getModel().realizePosition(state);
	return get_mesh_frame().getTransformInGround(state);
}

const SimTK::Vector_<SimTK::Vec3>& WISCO_ContactMesh::getTriangleCenters() const
{
	return tri_center;
}

const SimTK::Vector_<SimTK::Vec3>& WISCO_ContactMesh::getTriangleNormals() const
{
	return tri_normal;
}

SimTK::Vector_<SimTK::Vec3> WISCO_ContactMesh::getTriangleCentersInBody() const
{
	SimTK::Vector_<SimTK::Vec3> triCentersInBody(mesh.getNumFaces());
	SimTK::Transform MtoB = get_mesh_frame().getOffsetTransform();

	for (int i = 0; i < mesh.getNumFaces(); ++i) {
		triCentersInBody(i) = MtoB.shiftBaseStationToFrame(tri_center(i));

	}
	return triCentersInBody;
}

SimTK::Vector_<SimTK::Vec3> WISCO_ContactMesh::getTriangleCentersInGround(const SimTK::State& state) const
{
	getModel().realizePosition(state);
	SimTK::Vector_<SimTK::Vec3> triCentersInGround(mesh.getNumFaces());

	//Transform all of the faces to ground
	for (int i = 0; i < mesh.getNumFaces(); ++i) {
		triCentersInGround(i) = get_mesh_frame().findStationLocationInGround(state, tri_center(i));
	}

	return triCentersInGround;
}

SimTK::Vector_<SimTK::Vec3> WISCO_ContactMesh::getTriangleNormalsInGround(const SimTK::State& state) const
{
	getModel().realizePosition(state);
	SimTK::Vector_<SimTK::Vec3> triNormalsInGround(mesh.getNumFaces());

	//Transform all of the faces to ground
	for (int i = 0; i < mesh.getNumFaces(); ++i) {
		triNormalsInGround(i) = get_mesh_frame().findStationLocationInGround(state, tri_normal(i));
	}
	return triNormalsInGround;
}

const SimTK::Vector& WISCO_ContactMesh::getTriangleAreas() const
{
	return tri_area;
}

const SimTK::Vector_<SimTK::Vec3>& WISCO_ContactMesh::getVertexLocations() const
{
	return vertex_locations;
}

const SimTK::Vector_<SimTK::Vec3> WISCO_ContactMesh::getVertexLocationsInGround(const SimTK::State& state) const
{
	SimTK::Vector_<SimTK::Vec3> verLocInGround(mesh.getNumVertices());

	for (int i = 0; i < mesh.getNumVertices(); ++i) {
		verLocInGround(i) = get_mesh_frame().findStationLocationInGround(state, vertex_locations(i));
	}
	return verLocInGround;
}

const SimTK::Matrix_<SimTK::Vec3>& WISCO_ContactMesh::getFaceVertexLocations() const
{
	return face_vertex_locations;
}

const SimTK::Vector_<SimTK::Vec3> WISCO_ContactMesh::getVertexLocationsInFrame(
	const SimTK::State& state, const std::string& frame_name) const
{
	getModel().realizePosition(state);

	SimTK::Vector_<SimTK::Vec3> vertices = getVertexLocationsInGround(state);

	SimTK::Transform bodyInGround = getModel().getComponent<Frame>
		(frame_name).getTransformInGround(state);

	SimTK:: Vector_<SimTK::Vec3> verticesInFrame(vertices.size());

	for (int i = 0; i < vertices.nrow(); ++i) {

		verticesInFrame(i) = bodyInGround.shiftBaseStationToFrame(vertices(i));

	}

	return verticesInFrame;
}

SimTK::Matrix_<SimTK::Vec3> WISCO_ContactMesh::getFaceVertexLocationsInGround(const SimTK::State& state) const
{
	getModel().realizePosition(state);
	SimTK::Matrix_<SimTK::Vec3> verLocInGround(mesh.getNumFaces(),3);

	//Transform all of the faces to ground
	for (int i = 0; i < mesh.getNumFaces(); ++i) {
		for (int j = 0; j < 3; ++j) {
			verLocInGround(i,j) = get_mesh_frame().findStationLocationInGround(state, face_vertex_locations(i,j));
		}
	}

	return verLocInGround;
}


SimTK::Vector WISCO_ContactMesh::getNeighborTris(int tri, int& nNeighborTri) const
{
	nNeighborTri = nTriNeighbors(tri);

	SimTK::Vector neighbor_tri_list(nNeighborTri);

	for (int i = 0; i < nNeighborTri; ++i) {
		neighbor_tri_list(i) = tri_neighbors(tri,i);
	}

    return neighbor_tri_list;
}

const int WISCO_ContactMesh::getDisplayPreference()
{
	return get_display_preference();
}

void WISCO_ContactMesh::setDisplayPreference(const int dispPref)
{
	set_display_preference(dispPref);
}


void WISCO_ContactMesh::scale(const ScaleSet& aScaleSet)
{
	throw Exception("ContactGeometry::scale is not implemented");
}


