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
#include <OpenSim/OpenSim.h>
using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Rotation;

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
}

//_____________________________________________________________________________
// Convenience constructor.
WISCO_ContactMesh::WISCO_ContactMesh(const std::string mesh_file, const PhysicalFrame& frame, const std::string& name) :
	WISCO_ContactMesh()
{
	set_file_name(mesh_file);
    updSocket<PhysicalFrame>("frame").connect(frame);
	setName(name);
	initializeMesh();
}

WISCO_ContactMesh::WISCO_ContactMesh(const std::string mesh_file, const Vec3& location, const Vec3& orientation,
	const PhysicalFrame& frame, const std::string& name) : WISCO_ContactMesh(mesh_file,frame,name)
{
	set_location(location);
	set_orientation(orientation);
}

void WISCO_ContactMesh::setNull()
{
	setAuthors("Colin Smith");
}

void WISCO_ContactMesh::constructProperties()
{
	constructProperty_file_name("");
	constructProperty_location(Vec3(0));
	constructProperty_orientation(Vec3(0));
	constructProperty_display_preference(1);

	Array<double> defaultColor(1.0, 3); //color default to 0, 1, 1
	defaultColor[0] = 0.0;
	constructProperty_color(defaultColor);
}

void WISCO_ContactMesh::extendFinalizeFromProperties() {
	Super::extendFinalizeFromProperties();
	
	//initializeMesh();
    
	//attachMeshToOffsetFrame();
}

void WISCO_ContactMesh::extendAddToSystem(SimTK::MultibodySystem &system) const {
    Super::extendAddToSystem(system);
    CollisionModel3D* default_coldet_model = newCollisionModel3D();
    
    addCacheVariable("coldet_model", default_coldet_model, SimTK::Stage::LowestRuntime);
}

void WISCO_ContactMesh::extendInitStateFromProperties(SimTK::State &state) const {
    Super::extendInitStateFromProperties(state);
    initColdetModel(state);

}

void WISCO_ContactMesh::initColdetModel(SimTK::State &state) const {

    // Load Mesh
    //mesh.loadFile(get_file_name());

    // Initialize PQP model
    CollisionModel3D* coldet_model = newCollisionModel3D();

    // Add triangles to PQP Model and compute properties
    //==================================================
    for (int i = 0; i < mesh.getNumFaces(); ++i) {

        // Get Triangle Vertice Positions
        int v1_i = mesh.getFaceVertex(i, 0);
        int v2_i = mesh.getFaceVertex(i, 1);
        int v3_i = mesh.getFaceVertex(i, 2);

        Vec3 v1 = mesh.getVertexPosition(v1_i);
        Vec3 v2 = mesh.getVertexPosition(v2_i);
        Vec3 v3 = mesh.getVertexPosition(v3_i);

        // Add Triangle to COLDET Model
        coldet_model->addTriangle(v1[0], v1[1], v1[2], v2[0], v2[1], v2[2], v3[0], v3[1], v3[2]);
    }

    //Finalize COLDET model
    coldet_model->finalize();

    setCacheVariableValue<CollisionModel3D*>(state, "coldet_model", coldet_model);

}

void WISCO_ContactMesh::initializeMesh() {
	
	// Initialize Reused Variables

	double mag;
	Vec3 e1, e2, cross;

	// Load Mesh
	mesh.loadFile(get_file_name());

	// Initialize PQP model
	//CollisionModel3D* coldet_model = newCollisionModel3D();

	// Add triangles to PQP Model and compute properties
	//==================================================
	tri_center.resize(mesh.getNumFaces());
	tri_normal.resize(mesh.getNumFaces());
	tri_area.resize(mesh.getNumFaces());

	for (int i = 0; i < mesh.getNumFaces(); ++i) {

		// Get Triangle Vertice Positions
		int v1_i = mesh.getFaceVertex(i, 0);
		int v2_i = mesh.getFaceVertex(i, 1);
		int v3_i = mesh.getFaceVertex(i, 2);

		Vec3 v1 = mesh.getVertexPosition(v1_i);
		Vec3 v2 = mesh.getVertexPosition(v2_i);
		Vec3 v3 = mesh.getVertexPosition(v3_i);

		// Add Triangle to COLDET Model
		//coldet_model->addTriangle(v1[0], v1[1], v1[2], v2[0], v2[1], v2[2], v3[0], v3[1], v3[2]);

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
 
	}

	//Finalize COLDET model
	//coldet_model->finalize();
    //setCacheVariableValue<CollisionModel3D>(state,"coldet_model",coldet_model)
    
	//Vertex Connectivity
    vertex0.resize(mesh.getNumFaces());
    vertex1.resize(mesh.getNumFaces());
    vertex2.resize(mesh.getNumFaces());

    for (int i = 0; i < mesh.getNumFaces(); ++i) {
        int v0 = mesh.getFaceVertex(i, 0);
        vertex0(i) = mesh.getVertexPosition(v0);

        int v1 = mesh.getFaceVertex(i, 1);
        vertex1(i) = mesh.getVertexPosition(v1);

        int v2 = mesh.getFaceVertex(i, 2);
        vertex2(i) = mesh.getVertexPosition(v2);
    }

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

SimTK::Transform WISCO_ContactMesh::getTransformParentToMesh() const
{
	return SimTK::Transform(
		SimTK::Rotation(SimTK::BodyRotationSequence,
			get_orientation()[0], SimTK::XAxis,
			get_orientation()[1], SimTK::YAxis,
			get_orientation()[2], SimTK::ZAxis),
		get_location());
}

SimTK::Transform WISCO_ContactMesh::getTransformGroundToMesh(SimTK::State state) const
{
	SimTK::Transform PtoM = getTransformParentToMesh();
    getModel().realizePosition(state);
    const PhysicalFrame& frame = getConnectee<PhysicalFrame>("frame");

    SimTK::Transform GtoM = frame.getTransformInGround(state);
    
        //const SimTK::Transform& GtoM = getConnectee<PhysicalFrame>("frame").getTransformInGround(state);

	return GtoM;
}

int WISCO_ContactMesh::getNumFaces() const
{
	return mesh.getNumFaces();
}

/*
CollisionModel3D* WISCO_ContactMesh::updColdetModel() const 
{
	return coldet_model;
}
*/

SimTK::Vector_<SimTK::Vec3> WISCO_ContactMesh::getTriangleCenters() const
{
	return tri_center;
}

SimTK::Vector_<SimTK::Vec3> WISCO_ContactMesh::getTriangleNormals() const
{
	return tri_normal;
}
/*
SimTK::Vector_<SimTK::Vec3> WISCO_ContactMesh::getTriangleCentersInGround(SimTK::State state) const
{
	SimTK::Transform MtoG = getTransformMeshToGround(state);

	SimTK::Vector_<SimTK::Vec3> triCentersInGround(mesh.getNumFaces());

	//Transform all of the faces to ground
	for (int i = 0; i < mesh.getNumFaces(); ++i) {
		triCentersInGround(i) = MtoG.shiftBaseStationToFrame(tri_center(i));
		}
		
	return triCentersInGround;
}

SimTK::Vector_<SimTK::Vec3> WISCO_ContactMesh::getTriangleNormalsInGround(SimTK::State state) const
{
	
	SimTK::Transform MtoG = getTransformMeshToGround(state);
	SimTK::Vector_<SimTK::Vec3> triNormalsInGround(mesh.getNumFaces());

	//Transform all of the faces to ground
	for (int i = 0; i < mesh.getNumFaces(); ++i) {
		triNormalsInGround(i) = MtoG.xformBaseVecToFrame(tri_center(i));
	}
	return triNormalsInGround;
}
*/
SimTK::Vector WISCO_ContactMesh::getTriangleAreas() const
{
	return tri_area;
}

const SimTK::Vector_<SimTK::Vec3>& WISCO_ContactMesh::getVertex0() const
{
    return vertex0;
}

const SimTK::Vector_<SimTK::Vec3>& WISCO_ContactMesh::getVertex1() const 
{
    return vertex1;
}

const SimTK::Vector_<SimTK::Vec3>& WISCO_ContactMesh::getVertex2() const
{
    return vertex2;
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

void WISCO_ContactMesh::updateFromXMLNode(SimTK::Xml::Element& node,
	int versionNumber) {
	if (versionNumber < XMLDocument::getLatestVersion()) {
		if (versionNumber < 30505) {
			SimTK::Xml::element_iterator bodyElement =
				node.element_begin("body_name");
			std::string body_name("");
			// Element may not exist if body_name property had default value.
			if (bodyElement != node.element_end()) {
				bodyElement->getValueAs<std::string>(body_name);
			}
			XMLDocument::addConnector(node, "Connector_PhysicalFrame_",
				"frame", body_name);
		}
	}
	Super::updateFromXMLNode(node, versionNumber);
}

