/* -------------------------------------------------------------------------- *
 *                 WISCO_OpenSim:  ElasticFoundationForce.cpp                 *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

//==============================================================================
// INCLUDES
//==============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/GCVSpline.h>
#include "WISCO_ElasticFoundationForce.h"
#include "coldet.h"
#include "WISCO_ContactMesh.h"
#include <cctype>
#include <OpenSim/Common/Lmdif.h>
//#include "OpenSim/OpenSim.h"
//==============================================================================
// USING
//==============================================================================
using namespace OpenSim;
using namespace SimTK;

//==============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//==============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy
// assignment operator.

//_____________________________________________________________________________
/**
 * Default constructor.
 */
WISCO_ElasticFoundationForce::WISCO_ElasticFoundationForce() : Force()
{
    setNull();
    constructProperties();

}


/**
 * Convenience Constructor
 *
 */

OpenSim::WISCO_ElasticFoundationForce::WISCO_ElasticFoundationForce(
	WISCO_ContactMesh& target_mesh,
    WISCO_ElasticFoundationForce::ContactParameters target_mesh_params,
    WISCO_ContactMesh& casting_mesh,
    WISCO_ElasticFoundationForce::ContactParameters casting_mesh_params,
    int verbose)
{
	setNull();
	constructProperties();

    set_target_mesh_contact_params(target_mesh_params);
    set_casting_mesh_contact_params(casting_mesh_params);
	set_verbose(verbose);
    updSocket<WISCO_ContactMesh>("target_mesh").connect(target_mesh);
    updSocket<WISCO_ContactMesh>("casting_mesh").connect(casting_mesh);
}


void WISCO_ElasticFoundationForce::setNull()
{
    setAuthors("Colin Smith");

}

void WISCO_ElasticFoundationForce::constructProperties()
{
	constructProperty_min_proximity(0.00);
	constructProperty_max_proximity(0.02);
	constructProperty_elastic_foundation_formulation("nonlinear");
	constructProperty_use_smart_backside_contact(false);
	constructProperty_use_lumped_contact_model(false);
	constructProperty_verbose(0);
	constructProperty_target_mesh_contact_params(WISCO_ElasticFoundationForce::ContactParameters());
    constructProperty_casting_mesh_contact_params(WISCO_ElasticFoundationForce::ContactParameters());

}

void WISCO_ElasticFoundationForce::extendAddToSystem(MultibodySystem& system) const
{
	Super::extendAddToSystem(system);

	WISCO_ElasticFoundationForce::ContactParameters
		target_mesh_params = get_target_mesh_contact_params();
	WISCO_ElasticFoundationForce::ContactParameters
		casting_mesh_params = get_casting_mesh_contact_params();

	int target_mesh_nTri = getSocket<WISCO_ContactMesh>
		("target_mesh").getConnectee().getNumFaces();
	int casting_mesh_nTri = getSocket<WISCO_ContactMesh>
		("casting_mesh").getConnectee().getNumFaces();

	int target_mesh_nVertex = getSocket<WISCO_ContactMesh>
		("target_mesh").getConnectee().getNumVertices();
	int casting_mesh_nVertex = getSocket<WISCO_ContactMesh>
		("casting_mesh").getConnectee().getNumVertices();

	Vector target_mesh_def_vec(target_mesh_nTri);
	Vector casting_mesh_def_vec(casting_mesh_nTri);
	target_mesh_def_vec = 0;
	casting_mesh_def_vec = 0;


	addCacheVariable<Vector>("target_mesh_contacting_tri",
		target_mesh_def_vec, Stage::LowestRuntime);
	addCacheVariable<Vector>("casting_mesh_contacting_tri",
		casting_mesh_def_vec, Stage::LowestRuntime);

	addCacheVariable<Vector>("target_mesh.tri.pressure",
		target_mesh_def_vec, Stage::Dynamics);
	addCacheVariable<Vector>("casting_mesh.tri.pressure",
		casting_mesh_def_vec, Stage::Dynamics);

	addCacheVariable<Vector>("target_mesh.tri.proximity",
		target_mesh_def_vec, Stage::Dynamics);
	addCacheVariable<Vector>("casting_mesh.tri.proximity",
		casting_mesh_def_vec, Stage::Dynamics);

	addCacheVariable<int>("target_mesh_n_active_tri",
		0, Stage::Dynamics);
	addCacheVariable<int>("casting_mesh_n_active_tri",
		0, Stage::Dynamics);

	Vector target_vertex_def_vec(target_mesh_nVertex);
	Vector casting_vertex_def_vec(casting_mesh_nVertex);
	target_vertex_def_vec = 0;
	casting_vertex_def_vec = 0;

	addCacheVariable<Vector>("target_mesh.vertex.pressure",
		target_vertex_def_vec, Stage::Dynamics);
	addCacheVariable<Vector>("casting_mesh.vertex.pressure",
		casting_vertex_def_vec, Stage::Dynamics);

	addCacheVariable<Vector>("target_mesh.vertex.proximity",
		target_vertex_def_vec, Stage::Dynamics);
	addCacheVariable<Vector>("casting_mesh.vertex.proximity",
		casting_vertex_def_vec, Stage::Dynamics);

	addCacheVariable<Vec3>("force", Vec3(0), Stage::Dynamics);
	addCacheVariable<Vec3>("torque", Vec3(0), Stage::Dynamics);

	//Contact Stats
	std::vector<std::string> double_names, vec3_names;
	double_names.push_back("mean_pressure");
	double_names.push_back("max_pressure");
	double_names.push_back("mean_proximity");
	double_names.push_back("max_proximity");
	double_names.push_back("contact_area");
	vec3_names.push_back("cop");
	vec3_names.push_back("contact_force");

	for (std::string name : double_names) {
		addCacheVariable<double>("target_mesh.total." + name, 0.0, Stage::Dynamics);
		addCacheVariable<double>("casting_mesh.total." + name, 0.0, Stage::Dynamics);

		addCacheVariable<double>("target_mesh.medial." + name, 0.0, Stage::Dynamics);
		addCacheVariable<double>("casting_mesh.medial." + name, 0.0, Stage::Dynamics);
		addCacheVariable<double>("target_mesh.lateral." + name, 0.0, Stage::Dynamics);
		addCacheVariable<double>("casting_mesh.lateral." + name, 0.0, Stage::Dynamics);
	}

	for (std::string name : vec3_names) {
		addCacheVariable<SimTK::Vec3>("target_mesh.total." + name, Vec3(0.0), Stage::Dynamics);
		addCacheVariable<SimTK::Vec3>("casting_mesh.total." + name, Vec3(0.0), Stage::Dynamics);
		addCacheVariable<SimTK::Vec3>("target_mesh.medial." + name, Vec3(0.0), Stage::Dynamics);
		addCacheVariable<SimTK::Vec3>("casting_mesh.medial." + name, Vec3(0.0), Stage::Dynamics);
		addCacheVariable<SimTK::Vec3>("target_mesh.lateral." + name, Vec3(0.0), Stage::Dynamics);
		addCacheVariable<SimTK::Vec3>("casting_mesh.lateral." + name, Vec3(0.0), Stage::Dynamics);
	}

	//Modeling Options
	//----------------
	addModelingOption("flip_meshes", 1);
	addModelingOption("interpolate_vertex_data", 1);
	addModelingOption("contact_stats", 1);
	addModelingOption("contact_stats_medial_lateral", 1);
}

void WISCO_ElasticFoundationForce::extendFinalizeFromProperties()
{
	Super::extendFinalizeFromProperties();

	_mesh_data_names.clear();
	_region_names.clear();
	_stat_names.clear();
	_stat_names_vec3.clear();

	//Mesh Data
	_mesh_data_names.push_back("pressure");
	_mesh_data_names.push_back("proximity");

	//Contact Stats
	_region_names.push_back("medial");
	_region_names.push_back("lateral");

	_stat_names.push_back("mean_pressure");
	_stat_names.push_back("max_pressure");
	_stat_names.push_back("mean_proximity");
	_stat_names.push_back("max_proximity");
	_stat_names.push_back("contact_area");

	_stat_names_vec3.push_back("cop");
	_stat_names_vec3.push_back("contact_force");

	//Add channels to output list
	auto& Ctri_data = updOutput("casting_tri_data");
	auto& Ttri_data = updOutput("target_tri_data");

	for (std::string mesh_data : _mesh_data_names) {
		Ctri_data.addChannel("casting_mesh.tri." + mesh_data);
		Ttri_data.addChannel("target_mesh.tri." + mesh_data);
	}

	auto& Cver_data = updOutput("casting_vertex_data");
	auto& Tver_data = updOutput("target_vertex_data");

	for (std::string mesh_data : _mesh_data_names) {
		Cver_data.addChannel("casting_mesh.vertex." + mesh_data);
		Tver_data.addChannel("target_mesh.vertex." + mesh_data);
	}

	auto& Ccnt_tot_stat = updOutput("casting_contact_stats_total");
	auto& Tcnt_tot_stat = updOutput("target_contact_stats_total");

	auto& Ccnt_tot_stat_vec3 = updOutput("casting_contact_stats_total_vec3");
	auto& Tcnt_tot_stat_vec3 = updOutput("target_contact_stats_total_vec3");

	auto& Ccnt_ml_stat = updOutput("casting_contact_stats_medial_lateral");
	auto& Tcnt_ml_stat = updOutput("target_contact_stats_medial_lateral");

	auto& Ccnt_ml_stat_vec3 = updOutput("casting_contact_stats_medial_lateral_vec3");
	auto& Tcnt_ml_stat_vec3 = updOutput("target_contact_stats_medial_lateral_vec3");

	for (std::string stat : _stat_names) {		
		Ccnt_tot_stat.addChannel("casting_mesh.total." + stat);
		Tcnt_tot_stat.addChannel("target_mesh.total." + stat);

		Ccnt_ml_stat.addChannel("casting_mesh.medial." + stat);
		Ccnt_ml_stat.addChannel("casting_mesh.lateral." + stat);
		Tcnt_ml_stat.addChannel("target_mesh.medial." + stat);
		Tcnt_ml_stat.addChannel("target_mesh.lateral." + stat);
	}
	for (std::string stat_vec3 : _stat_names_vec3) {
		Ccnt_tot_stat_vec3.addChannel("casting_mesh.total." + stat_vec3);
		Tcnt_tot_stat_vec3.addChannel("target_mesh.total." + stat_vec3);

		Ccnt_ml_stat_vec3.addChannel("casting_mesh.medial." + stat_vec3);
		Tcnt_ml_stat_vec3.addChannel("target_mesh.medial." + stat_vec3);
		Ccnt_ml_stat_vec3.addChannel("casting_mesh.lateral." + stat_vec3);
		Tcnt_ml_stat_vec3.addChannel("target_mesh.lateral." + stat_vec3);
	}

}

void WISCO_ElasticFoundationForce::extendInitStateFromProperties(State & state)	const
{
    Super::extendInitStateFromProperties(state);

    int casting_mesh_nTri = getSocket<WISCO_ContactMesh>("casting_mesh").getConnectee().getNumFaces();
    Vector casting_default_vec(casting_mesh_nTri);
    casting_default_vec = -1;

    setCacheVariableValue<Vector>(state, "casting_mesh_contacting_tri", casting_default_vec);

	int target_mesh_nTri = getSocket<WISCO_ContactMesh>("target_mesh").getConnectee().getNumFaces();
	Vector target_default_vec(target_mesh_nTri);
	target_default_vec = -1;

	setCacheVariableValue<Vector>(state, "target_mesh_contacting_tri", target_default_vec);
}

void WISCO_ElasticFoundationForce::extendRealizeReport(const State & state)	const
{
	/*
	if (getModelingOption(state, "interpolate_vertex_data")) {
		computeVertexValues(state);
	}*/
	/*
	if (getModelingOption(state, "contact_stats")) {
		computeContactStats(state, false);

		if (getModelingOption(state, "flip_meshes")) {
			computeContactStats(state, true);
		}
	}*/

	if (get_verbose() > 0) {
		if (get_appliesForce()) {
			std::cout << std::endl;
			std::cout << "Time: " << state.getTime() << std::endl;
			std::cout << getName() << " casting_mesh Active Triangles: ";
			std::cout << getCacheVariableValue<int>(state, "casting_mesh_n_active_tri");
			std::cout << "/" << getConnectee<WISCO_ContactMesh>("casting_mesh").getNumFaces() << std::endl;
			std::cout << std::endl;
		}
	}
}

//-----------------------------------------------------------------------------
// FORCE METHODS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void WISCO_ElasticFoundationForce::computeForce(const State& state,
	Vector_<SpatialVec>& bodyForces,
	Vector& generalizedForces) const
{

	meshCollision(state, bodyForces, true, false);


	//For contact analysis, flip meshes to get pressure on target mesh
	if (getModelingOption(state, "flip_meshes")) {
		meshCollision(state, bodyForces, false, true);
	}
}

void WISCO_ElasticFoundationForce::meshCollision(
	const State& state, Vector_<SpatialVec>& bodyForces,
	bool applyContactForces, bool flipMeshes) const
{
	/*
	Convience for variable naming:
	T = target mesh
	C = casting mesh
	*/

	// Get Mesh Properties
    double min_proximity = get_min_proximity();
	double max_proximity = get_max_proximity();

	const WISCO_ContactMesh& targetMesh = (!flipMeshes) ?
		getConnectee<WISCO_ContactMesh>("target_mesh") :
		getConnectee<WISCO_ContactMesh>("casting_mesh");

	const WISCO_ContactMesh& castingMesh = (!flipMeshes) ?
		getConnectee<WISCO_ContactMesh>("casting_mesh") :
		getConnectee<WISCO_ContactMesh>("target_mesh");

    Vector_<SimTK::Vec3> tri_cenC = castingMesh.getTriangleCenters();
	Vector_<SimTK::Vec3> tri_cenC_ground = castingMesh.getTriangleCentersInGround(state);

	Vector_<SimTK::Vec3> tri_norT_ground = targetMesh.getTriangleNormalsInGround(state);
	Vector_<SimTK::Vec3> tri_norC_ground = castingMesh.getTriangleNormalsInGround(state);

	Matrix_<SimTK::Vec3> meshT_ver_loc_ground = targetMesh.getFaceVertexLocationsInGround(state);

    Vector tri_areaC = castingMesh.getTriangleAreas();

    // Get Mesh Transforms
    Transform GtoMeshT = targetMesh.getTransformGroundToMesh(state);
    Transform MeshCtoMeshT = getTransformCastingToTargetMesh(state);

    //Reset depth arrays
	Vector meshC_tri_pressure;
	Vector meshC_tri_proximity;

	if (!flipMeshes) {
		meshC_tri_pressure = updCacheVariableValue<Vector>
			(state, "casting_mesh.tri.pressure");

		meshC_tri_proximity = updCacheVariableValue<Vector>
			(state, "casting_mesh.tri.proximity");
	}
	else {
		meshC_tri_pressure = updCacheVariableValue<Vector>
			(state, "target_mesh.tri.pressure");

		meshC_tri_proximity = updCacheVariableValue<Vector>
			(state, "target_mesh.tri.proximity");
	}

    meshC_tri_pressure = 0;
	meshC_tri_proximity = 0;


	//Collision Detection
	//-------------------------------------------------------------------------

	//Get the collision object from mesh and set transform in ground
	CollisionModel3D* mT;

	if (!flipMeshes) {
		mT = getConnectee<WISCO_ContactMesh>("target_mesh").
			updCacheVariableValue<CollisionModel3D*>(state, "coldet_model");
	}
	else {
		mT = getConnectee<WISCO_ContactMesh>("casting_mesh").
			updCacheVariableValue<CollisionModel3D*>(state, "coldet_model");
	}

	auto coldet_transform = ~GtoMeshT.toMat44();

	float meshC_transform[16];
	int count = 0;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			meshC_transform[count] =coldet_transform(i, j);
			count++;
		}
	}
	mT->setTransform(meshC_transform);

	//Initialize contact variables
    int meshC_nActiveTri = 0;
    float origin[3], vector[3];

	Vector meshC_target_tri;
	if (!flipMeshes) {
		meshC_target_tri = getCacheVariableValue<Vector>
			(state, "casting_mesh_contacting_tri");
	}
	else {
		meshC_target_tri = getCacheVariableValue<Vector>
			(state, "target_mesh_contacting_tri");
	}

	SimTK::Vec3 force(0);
	SimTK::Vec3 moment(0);

	//Keep track of triangle collision type for debugging
    int same = 0;
    int diff = 0;
	int neighbor = 0;

	//Loop through all triangles in mesh2
    for (int i = 0; i < castingMesh.getNumFaces(); ++i) {
        bool contact_detected = false;
		double depth = 0.0;
		Vec3 contact_pnt_ground(0);

        Vec3 center = tri_cenC_ground(i);
        Vec3 normal = tri_norC_ground(i);

        Vec3 center_in_mT = MeshCtoMeshT.shiftBaseStationToFrame(center);

        //If triangle was in contact in previous timestep, recheck same contact triangle and neighbors
		if (meshC_target_tri(i) >= 0) {

			//Check same triangle as previous time step
			Vec3 vertex0 = meshT_ver_loc_ground(meshC_target_tri(i), 0);
			Vec3 vertex1 = meshT_ver_loc_ground(meshC_target_tri(i), 1);
			Vec3 vertex2 = meshT_ver_loc_ground(meshC_target_tri(i), 2);

			if (rayIntersectTriTest(center, -normal, vertex0, vertex1, vertex2, contact_pnt_ground, depth))
			{
				if (verifyTriContact(tri_norT_ground(meshC_target_tri(i)), tri_norC_ground(i), depth)) {
					meshC_tri_proximity(i) = depth;
					meshC_tri_pressure(i) = computeTriPressure(
						i, meshC_target_tri(i), depth, flipMeshes);

					meshC_nActiveTri++;
					same++;
					continue;
				}
			}


            //Check neighboring triangles
            int nNeighborTri;
            Vector neighborTri = targetMesh.getNeighborTris(meshC_target_tri(i), nNeighborTri);

	        for (int j = 0; j < nNeighborTri; ++j) {

                vertex0 = meshT_ver_loc_ground(neighborTri(j),0);
                vertex1 = meshT_ver_loc_ground(neighborTri(j), 1);
                vertex2 = meshT_ver_loc_ground(neighborTri(j), 2);

                if (rayIntersectTriTest(center, -normal, vertex0, vertex1, vertex2, contact_pnt_ground, depth))
				{
                    if (verifyTriContact(tri_norT_ground(neighborTri(j)), tri_norC_ground(i), depth)) {
						meshC_tri_proximity(i) = depth;
						meshC_tri_pressure(i) = computeTriPressure(
							i, neighborTri(j), depth, flipMeshes);

						meshC_target_tri(i) = neighborTri(j);

						meshC_nActiveTri++;
						neighbor++;

						contact_detected = true;
                        break;
                    }
                }
            }
            if (contact_detected)
                continue;

        }

        //Go through the OBB hierarchy
        for (int j = 0; j < 3; ++j) {
            origin[j] = tri_cenC_ground(i)(j);
            vector[j] = -tri_norC_ground(i)(j);
        }

        if (mT->rayCollision(origin, vector, true, min_proximity, max_proximity)) {
		    //Colliding Triangles
            int triT, triC;
            mT->getCollidingTriangles(triT, triC);

			// Compute Proximity
			Vec3 vertex0 = meshT_ver_loc_ground(triT, 0);
			Vec3 vertex1 = meshT_ver_loc_ground(triT, 1);
			Vec3 vertex2 = meshT_ver_loc_ground(triT, 2);

			rayIntersectTriTest(center, -normal, vertex0, vertex1, vertex2, contact_pnt_ground, depth);
			//Verify Collision
			if (verifyTriContact(tri_norT_ground(triT), tri_norC_ground(i),depth))
			{
				meshC_target_tri(i) = triT;
				meshC_tri_proximity(i) = depth;

				//Compute Pressure
				meshC_tri_pressure(i) = computeTriPressure(i,triT,depth,flipMeshes);

				meshC_nActiveTri++;
				diff++;
				continue;
			}
        }

        //Else - triangle is not in contact
        meshC_target_tri[i] = -1;

    }


    //Apply Point Forces
	//-------------------------------------------------------------------------
	if (applyContactForces) {
		const PhysicalFrame& frameT = (!flipMeshes) ?
			getConnectee<WISCO_ContactMesh>("target_mesh").get_mesh_frame() :
			getConnectee<WISCO_ContactMesh>("casting_mesh").get_mesh_frame();

		const PhysicalFrame& frameC = (!flipMeshes) ?
			getConnectee<WISCO_ContactMesh>("casting_mesh").get_mesh_frame() :
			getConnectee<WISCO_ContactMesh>("target_mesh").get_mesh_frame();

		for (int i = 0; i < castingMesh.getNumFaces(); ++i) {
			
			Vec3 prs_force = computeForceVector(meshC_tri_pressure(i), tri_areaC(i), -tri_norC_ground(i));
			applyForceToPoint(state, frameC, tri_cenC(i), prs_force, bodyForces);

			Vec3 point_in_mT = MeshCtoMeshT.shiftBaseStationToFrame(tri_cenC(i));
			applyForceToPoint(state, frameT, point_in_mT, -prs_force, bodyForces);
		}
	}


    //Store Contact Info
	//-------------------------------------------------------------------------
	if (!flipMeshes) {
		setCacheVariableValue(state, "casting_mesh.tri.pressure", meshC_tri_pressure);
		setCacheVariableValue(state, "casting_mesh.tri.proximity", meshC_tri_proximity);
		setCacheVariableValue<Vector>(state, "casting_mesh_contacting_tri", meshC_target_tri);
		setCacheVariableValue<int>(state, "casting_mesh_n_active_tri", meshC_nActiveTri);
		getConnectee<WISCO_ContactMesh>("target_mesh").markCacheVariableValid(state, "coldet_model");
	}
	else {
		setCacheVariableValue(state, "target_mesh.tri.pressure", meshC_tri_pressure);
		setCacheVariableValue(state, "target_mesh.tri.proximity", meshC_tri_proximity);
		setCacheVariableValue<Vector>(state, "target_mesh_contacting_tri", meshC_target_tri);
		setCacheVariableValue<int>(state, "target_mesh_n_active_tri", meshC_nActiveTri);
		getConnectee<WISCO_ContactMesh>("casting_mesh").markCacheVariableValid(state, "coldet_model");
	}

	//Compute Contact Stats
	if (getModelingOption(state, "contact_stats")) {
		computeContactStats(state, castingMesh, meshC_tri_pressure, meshC_tri_proximity, meshC_nActiveTri, flipMeshes);
	}
	
	//Interpolate vertex values
	if (getModelingOption(state, "interpolate_vertex_data")) {
		computeVertexValues(state, castingMesh, meshC_tri_pressure, meshC_tri_proximity, flipMeshes);
	}

	//Debugging Report to console
	//---------------------------
	if (get_verbose() > 1) {
		std::cout << "Integrator Time: " << state.getTime() << std::endl;

		int width = 10;

		std::cout << std::left << std::setw(width) << std::setfill(' ') << "Same";
		std::cout << std::left << std::setw(width) << std::setfill(' ') << "Neighbor";
		std::cout << std::left << std::setw(width) << std::setfill(' ') << "Diff";
		std::cout << std::endl;

		std::cout << std::left << std::setw(width) << std::setfill(' ') << same;
		std::cout << std::left << std::setw(width) << std::setfill(' ') << neighbor;
		std::cout << std::left << std::setw(width) << std::setfill(' ') << diff;
		std::cout << std::endl;
	}
}

bool WISCO_ElasticFoundationForce::rayIntersectTriTest(Vec3 p, Vec3 d, Vec3 v0, Vec3 v1, Vec3 v2, Vec3& intersection_pt, double& distance) const
{
    //double p[], double d[], double v0[], double v1[], double v2[], double intersection_pt[], double *distance
    // p - reference point (i.e. center of triangle from which ray is cast)
    // d - search direction (i.e. normal to triangle from which ray is cast)
    // v0, v1, v2 - are the vertices of the target triangel
    //
    // www.lighthouse3d.com/tutorials/maths/ray-triangle-intersection/
    // www.cs.virginia.edu/~gfx/Courses/2003/ImageSynthesis/papers/Acceleration/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
    //

    int i;
    Vec3 e1, e2, h, s, q;
    double a, f, u, v;

    // find triangle edges
    for (i = 0; i<3; i++) {
        e1[i] = v1[i] - v0[i];
        e2[i] = v2[i] - v0[i];
    }
    // cross-product h = d x e2
    h[0] = d[1] * e2[2] - d[2] * e2[1];
    h[1] = d[2] * e2[0] - d[0] * e2[2];
    h[2] = d[0] * e2[1] - d[1] * e2[0];
    // dot-product a = e1 dot h
    a = e1[0] * h[0] + e1[1] * h[1] + e1[2] * h[2];
    // First test to see if intersection occurs
    if (a > -0.00000001 && a < 0.00000001) //a > -0.00001 && a < 0.00001) this should be adjusted for precision, a=0 when e1 and h are pependicular
        return(false);

    // Else on to second test
    // find triangle edges
    f = 1 / a;
    for (i = 0; i<3; i++)
        s[i] = p[i] - v0[i];
    // dot product:	s_dot_h=f3s_dot(s,h);
    u = f*(s[0] * h[0] + s[1] * h[1] + s[2] * h[2]);
    if (u < 0 || u > 1.0)
        return(false);

    // cross-product f3s_cross(q,s,e1);
    q[0] = s[1] * e1[2] - s[2] * e1[1];
    q[1] = s[2] * e1[0] - s[0] * e1[2];
    q[2] = s[0] * e1[1] - s[1] * e1[0];
    // dot product and scaling f * f3s_dot(d,q);
    v = f*(d[0] * q[0] + d[1] * q[1] + d[2] * q[2]);
    if (v < 0.0 || u + v > 1.0)
        return(false);
    else {		// else there is a line intersection
                // at this stage we can compute the distance to the intersection point on the line
                //     point(t) = p + t * d
                //	where
                //		p is a point in the line
                //		d is a vector that provides the line's direction
                //		t is the distance
        distance = f*(e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2]);
        intersection_pt[0] = p[0] + distance*d[0];
        intersection_pt[1] = p[1] + distance*d[1];
        intersection_pt[2] = p[2] + distance*d[2];
        return(true);
    }
}

bool WISCO_ElasticFoundationForce::verifyTriContact(Vec3 tri_norT_ground, Vec3 tri_norC_ground, double depth) const
{
	//Check proximity in range
	double min_proximity = get_min_proximity();
	double max_proximity = get_max_proximity();

	if ((depth < min_proximity) || (depth > max_proximity))
		return false;

	// Check to make sure backside of mesh2 isn't causing contact
	if (get_use_smart_backside_contact()) {
		double dir_vec = dot(-tri_norC_ground, tri_norT_ground.normalize());

		if (dir_vec > 0)
			return true;
		else
			return false;
	}

	return true;
}

double WISCO_ElasticFoundationForce::computeTriProximity(Vec3 cnt_pnt_ground, Vec3 tri_cen_ground) const
{

	//Find Depth
		Vec3 cen_cnt_point;
		cen_cnt_point = cnt_pnt_ground - tri_cen_ground;

		double depth = sqrt(pow(cen_cnt_point(0), 2) + pow(cen_cnt_point(1), 2) + pow(cen_cnt_point(2), 2));

    return depth;
}

double WISCO_ElasticFoundationForce::computeTriPressure(
	int castingTri, int targetTri, double depth,
	bool flipMeshes) const
{
    //Get Contact Parameters
	//----------------------
	const WISCO_ContactMesh& tMesh = (!flipMeshes) ?
		getConnectee<WISCO_ContactMesh>("target_mesh") :
		getConnectee<WISCO_ContactMesh>("casting_mesh");

	const WISCO_ContactMesh& cMesh = (!flipMeshes) ?
		getConnectee<WISCO_ContactMesh>("casting_mesh") :
		getConnectee<WISCO_ContactMesh>("target_mesh");

	const WISCO_ElasticFoundationForce::ContactParameters& tContactParams =
		(!flipMeshes) ?
		get_target_mesh_contact_params() :
		get_casting_mesh_contact_params();

	const WISCO_ElasticFoundationForce::ContactParameters& cContactParams =
		(!flipMeshes) ?
		get_casting_mesh_contact_params() :
		get_target_mesh_contact_params();

	double hT, hC; //thickness
	double ET, EC; //elastic modulus
	double vT, vC; //poissons ratio

	if (tContactParams.get_use_variable_thickness())
		hT = tMesh.getTriangleThickness()(targetTri);
	else
		hT = tContactParams.get_thickness();

	if (tContactParams.get_use_variable_elastic_modulus())
		ET = tMesh.getTriangleElasticModulus()(targetTri);
	else
		ET = tContactParams.get_elastic_modulus();

	if (tContactParams.get_use_variable_poissons_ratio())
		vT = tMesh.getTrianglePoissonsRatio()(targetTri);
	else
		vT = tContactParams.get_poissons_ratio();

	if (cContactParams.get_use_variable_thickness())
		hC = cMesh.getTriangleThickness()(castingTri);
	else
		hC = cContactParams.get_thickness();

	if (cContactParams.get_use_variable_elastic_modulus())
		EC = cMesh.getTriangleElasticModulus()(castingTri);
	else
		EC = cContactParams.get_elastic_modulus();

	if (cContactParams.get_use_variable_poissons_ratio())
		vC = cMesh.getTrianglePoissonsRatio()(castingTri);
	else
		vC = cContactParams.get_poissons_ratio();

	//Compute Pressure
	//----------------
	if (get_use_lumped_contact_model()) {
		double E = (ET + EC) / 2;
		double v = (vT + vC) / 2;
		double h = (hT + hC);

		double K = (1 - v)*E / ((1 + v)*(1 - 2 * v));


		if (get_elastic_foundation_formulation() == "linear") {
			return K*depth/h;
		}

		if (get_elastic_foundation_formulation() == "nonlinear") {
			return -K*log(1 - depth / h);
		}
	}

	//linear solution
	double kT = ((1 - vT)*ET) / ((1 + vT)*(1 - 2 * vT)*hT);
	double kC = ((1 - vC)*EC) / ((1 + vC)*(1 - 2 * vC)*hC);

	double linearPressure = (kT*kC) / (kT + kC)*depth;


	//linear
    if (get_elastic_foundation_formulation() == "linear") {
		return linearPressure;
    }

    //nonlinear
    else if (get_elastic_foundation_formulation() == "nonlinear") {

		nonlinearContactParams cp;

		cp.dc = depth;
		cp.h1 = hC;
		cp.h2 = hT;
		cp.k1 = (1 - vC)*EC / ((1 + vC)*(1 - 2 * vC));
		cp.k2 = (1 - vT)*ET / ((1 + vT)*(1 - 2 * vT));

		int nEqn = 1;
		int nVar = 1;
		double x[1], fvec[1];

		//solution params
		double ftol = 1e-4, xtol = 1e-4, gtol = 0.0;
		int maxfev = 500; //max iterations
		double epsfcn = 0.0;
		double diag[1];
		int mode = 1; //variables scaled internally
		double step_factor = 100;
		int nprint = 0;
		int info;
		int num_func_calls;
		double fjac[1];
		int ldfjac = 1;
		int ipvt[1];
		double qtf[1];
		double wa1[1], wa2[1], wa3[1], wa4[1];

		//initial guess
		x[0] = linearPressure;

		//Solve nonlinear equation
		lmdif_C(calcNonlinearPressureResid, nEqn, nVar, x, fvec,
			ftol, xtol, gtol, maxfev, epsfcn, diag, mode, step_factor,
			nprint, &info, &num_func_calls, fjac, ldfjac, ipvt, qtf,
			wa1, wa2, wa3, wa4, (void*)&cp);

		double nonlinearPressure = x[0];

		return nonlinearPressure;
    }

	else {
		OPENSIM_THROW(Exception,"Property: 'elastic_foundation_formulation' is not valid")
	}
}

/**
* A utility function used by computeTriPressure for the function lmdif_C, which
* is used to solve the nonlinear equation for pressure.
*
* h1(1-exp(-P1/k1))+h2(1-exp(P1/k2))-dc = 0;
*
* @param nEqn The number of Equations (1)
* @param nVars The number of variables (1)
* @param q Array of values of the degrees of freedom
* @param resid Array of residuals to be calculated
* @param flag2 A status flag
* @param ptr Pointer to data structure containing values for h1, h2, k1, k2, dc
*/
void WISCO_ElasticFoundationForce::calcNonlinearPressureResid(
	int nEqn, int nVar, double x[], double fvec[], int *flag2, void *ptr)
{
	nonlinearContactParams * cp = (nonlinearContactParams*)ptr;

	double h1 = cp->h1;
	double h2 = cp->h2;
	double k1 = cp->k1;
	double k2 = cp->k2;
	double dc = cp->dc;

	double P = x[0];

	fvec[0] = h1*(1 - exp(-P / k1)) + h2*(1 - exp(-P / k2)) - dc;

}

Vec3 WISCO_ElasticFoundationForce::computeForceVector(double pressure, double area, Vec3 normal) const
{
    Vec3 force;
    force(0) = normal(0) * pressure * area;
    force(1) = normal(1) * pressure * area;
    force(2) = normal(2) * pressure * area;
    return force;
}

Vec3 WISCO_ElasticFoundationForce::computeMomentVector(double pressure, double area, Vec3 normal, Vec3 center) const
{
    Vec3 moment;
    moment(0) = normal(0) * pressure * area * center(0);
    moment(1) = normal(1) * pressure * area * center(1);
    moment(2) = normal(2) * pressure * area * center(2);
    return moment;
}

Transform WISCO_ElasticFoundationForce::getTransformCastingToTargetMesh(const State& state) const
{

	const PhysicalFrame& frameT = (!getModelingOption(state, "flip_meshes")) ?
		getConnectee<WISCO_ContactMesh>("target_mesh").get_mesh_frame() :
		getConnectee<WISCO_ContactMesh>("casting_mesh").get_mesh_frame();

	const PhysicalFrame& frameC = (!getModelingOption(state, "flip_meshes")) ?
		getConnectee<WISCO_ContactMesh>("casting_mesh").get_mesh_frame() :
		getConnectee<WISCO_ContactMesh>("target_mesh").get_mesh_frame();


	return frameC.findTransformBetween(state,frameT);

}

Vector_<Vec3> WISCO_ElasticFoundationForce::getMeshVerticesInFrame(const State& state,
	const std::string mesh_name, const std::string frame_name) const
{
	getModel().realizePosition(state);

	Vector_<Vec3> vertices = getConnectee<WISCO_ContactMesh>(mesh_name).getVertexLocationsInGround(state);

	Transform bodyInGround = getModel().getComponent<Frame>(frame_name).getTransformInGround(state);

	Vector_<Vec3> verticesInFrame(vertices.size());

	for (int i = 0; i < vertices.nrow(); ++i) {

		verticesInFrame(i) =  bodyInGround.shiftBaseStationToFrame(vertices(i));

	}

	return verticesInFrame;

}

void WISCO_ElasticFoundationForce::computeVertexValues(const SimTK::State& state,
	const WISCO_ContactMesh& mesh, const SimTK::Vector& tri_pressure,
	const SimTK::Vector& tri_proximity, const bool flipMeshes) const
{
	//Compute Vertex Data
	const PolygonalMesh& meshC = mesh.getPolygonalMesh();

	Vector vertex_pressure = interpolateVertexFromFaceValues(tri_pressure,meshC);
	Vector vertex_proximity = interpolateVertexFromFaceValues(tri_proximity,meshC);

	if (!flipMeshes) {
		setCacheVariableValue<Vector>
			(state, "casting_mesh.vertex.pressure",vertex_pressure);
		setCacheVariableValue<Vector>
			(state, "casting_mesh.vertex.proximity", vertex_proximity);
	}
	else {
		setCacheVariableValue<Vector>
			(state, "target_mesh.vertex.pressure", vertex_pressure);
		setCacheVariableValue<Vector>
			(state, "target_mesh.vertex.proximity", vertex_proximity);
	}
}

/*
interpolateVertexFromFaceValues

Solve a linear system of equations Ax=b in the least squares sense to determine
the values of mesh vertice data from mesh face data. To reduce the number of
equations that must be solved, only the active (non-zero) faces and their
corresponding vertices are considered.


A = matrix of face-vertice connectivity [nActiveTri nActiveFace]
b = vector of face values
x = vector of vertex values
*/

Vector WISCO_ElasticFoundationForce::interpolateVertexFromFaceValues(
	Vector face_values, PolygonalMesh mesh) const
{
	int nFace = mesh.getNumFaces();
	int nVer = mesh.getNumVertices();

	Vector activeFaceInd;
	activeFaceInd.resize(nFace);
	int ind = 0;

	Vector activeVer;
	Vector activeFace;
	activeVer.resize(nVer);
	activeVer.setToZero();
	activeFace.resize(nFace);
	activeFace.setToZero();

	//Find active triangles and corresponding nodes

	for (int k = 0; k < nFace; ++k) {
		if (face_values(k) != 0) {
			activeFace(k) = 1;
			activeFaceInd(ind) = k;
			ind++;

			for (int j = 0; j < mesh.getNumVerticesForFace(k); j++) {
				int iFV = mesh.getFaceVertex(k, j);
				activeVer(iFV) = 1;
			}
		}
	}

	int nActiveFace = activeFace.sum();
	int nActiveVer = activeVer.sum();
	Vector activeVerInd(nVer);
	Vector verNumActive(nVer);
	Vector b(nActiveFace);
	Matrix A(nActiveFace, nActiveVer);
	Vector x(nActiveVer);
	b.setToZero();
	A.setToZero();

	//Only solve if there are active tris
	Vector vertex_values(nVer);
	vertex_values.setToZero();

	if (nActiveFace != 0) {

		//Renumber the active nodes (to keep track in A matrix)
		ind = 0;
		activeVerInd = 0;

		for (int p = 0; p < nVer; ++p) {
			if (activeVer(p) == 1) {
				activeVerInd(ind) = p;
				verNumActive(p) = ind;
				ind++;
			}
		}


		for (int j = 0; j < nActiveFace; ++j) {

			//Assign active face values to right hand side (b)
			int face_ind = activeFaceInd(j);
			b(j) = face_values(face_ind);

			//Build triangle-vertices connectivity matrix (A)
			for (int k = 0; k < mesh.getNumVerticesForFace(face_ind); k++) {
				int iFV = mesh.getFaceVertex(face_ind, k);
				A(j, verNumActive(iFV)) = 1.0 / mesh.getNumVerticesForFace(face_ind);
			}

		}
		//Solve least squares for node values
		auto LSmatrix = FactorQTZ(A);
		LSmatrix.solve(b, x);

		//Ensure all interpolated values are greater than zero
		for (int j = 0; j < nActiveVer; ++j) {
			if (x(j) < 0.0) {
				x(j) = 0.0;
			}
		}

		//Assign active node values to node matrix
		for (int j = 0; j < nActiveVer; ++j) {
			vertex_values(activeVerInd(j)) = x(j);
		}
	}
	return vertex_values;
}

void WISCO_ElasticFoundationForce::computeContactStats(
	const SimTK::State& state, const WISCO_ContactMesh& mesh, const SimTK::Vector& tri_pressure,
	const SimTK::Vector& tri_proximity, int nActiveTri, bool flipMeshes) const
{
	//Get mesh info
	Vector tri_area = mesh.getTriangleAreas();

	Vector_<Vec3> tri_normal = mesh.getTriangleNormals();
	int nTri = mesh.getNumFaces();

	Vector_<Vec3> tri_center = mesh.getTriangleCenters();
	Vector tri_cenX(nTri);
	Vector tri_cenY(nTri);
	Vector tri_cenZ(nTri);

	for (int i = 0; i < nTri; ++i) {
		tri_cenX(i) = tri_center(i)(0);
		tri_cenY(i) = tri_center(i)(1);
		tri_cenZ(i) = tri_center(i)(2);
	}

	//Mean Pressure
	double mean_pressure = tri_pressure.sum() / nActiveTri;
	double mean_proximity = tri_proximity.sum() / nActiveTri;

	//Max Pressure
	double max_pressure = tri_pressure.normInf();
	double max_proximity = tri_proximity.normInf();

	//Contact Area
	double contact_area = 0.0;

	for (int i = 0; i < nTri; i++) {
		if (tri_pressure(i) > 0.0) {
			contact_area += 1000000 * tri_area(i); //convert to mm^2
		}
	}

	//Center of Pressure
	Vector Num = tri_pressure.elementwiseMultiply(tri_area);
	Vector Den = tri_pressure.elementwiseMultiply(tri_area);
	double DenVal = Den.sum();

	Vector xNum = Num.elementwiseMultiply(tri_cenX);
	double xNumVal = xNum.sum() * 1000;
	double COPx = xNumVal / DenVal;

	Vector yNum = Num.elementwiseMultiply(tri_cenY);
	double yNumVal = yNum.sum() * 1000;
	double COPy = yNumVal / DenVal;

	Vector zNum = Num.elementwiseMultiply(tri_cenZ);
	double zNumVal = zNum.sum() * 1000;
	double COPz = zNumVal / DenVal;

	Vec3 COP(COPx, COPy, COPz);

	//Contact Force
	Vec3 contact_force(0.0);

	for (int i = 0; i < nTri; ++i) {
		contact_force += computeForceVector(tri_pressure(i), tri_area(i), -tri_normal(i));
	}

	//Save contact stats as cache variable
	if (flipMeshes) {
		setCacheVariableValue(state, "target_mesh.total.mean_pressure", mean_pressure);		
		setCacheVariableValue(state, "target_mesh.total.mean_proximity", mean_proximity);
		setCacheVariableValue(state, "target_mesh.total.max_pressure", max_pressure);
		setCacheVariableValue(state, "target_mesh.total.max_proximity", max_proximity);
		setCacheVariableValue(state, "target_mesh.total.contact_area", contact_area);
		setCacheVariableValue(state, "target_mesh.total.cop", COP);
		setCacheVariableValue(state, "target_mesh.total.contact_force", contact_force);


	}
	else {
		setCacheVariableValue(state, "casting_mesh.total.mean_pressure", mean_pressure);
		setCacheVariableValue(state, "casting_mesh.total.mean_proximity", mean_proximity);
		setCacheVariableValue(state, "casting_mesh.total.max_pressure", max_pressure);
		setCacheVariableValue(state, "casting_mesh.total.max_proximity", max_proximity);
		setCacheVariableValue(state, "casting_mesh.total.contact_area", contact_area);
		setCacheVariableValue(state, "casting_mesh.total.cop", COP);
		setCacheVariableValue(state, "casting_mesh.total.contact_force", contact_force);
	}
	//Compute Medial-Lateral Stats
	//----------------------------
	if (getModelingOption(state, "contact_stats_medial_lateral")) {

		Vector med_ind = mesh.getMedialTriangleIndices();
		Vector lat_ind = mesh.getLateralTriangleIndices();

		int nMedTri = mesh.getNumMedialTriangles();
		int nLatTri = mesh.getNumLateralTriangles();

		Vector med_pressure(nMedTri);
		Vector med_proximity(nMedTri);

		Vector lat_pressure(nLatTri);
		Vector lat_proximity(nLatTri);

		Vector med_area(nMedTri);
		Vector lat_area(nLatTri);

		Vector_<Vec3> med_normal(nMedTri);
		Vector_<Vec3> lat_normal(nLatTri);

		Vector med_tri_cenX(nMedTri);
		Vector med_tri_cenY(nMedTri);
		Vector med_tri_cenZ(nMedTri);

		Vector lat_tri_cenX(nLatTri);
		Vector lat_tri_cenY(nLatTri);
		Vector lat_tri_cenZ(nLatTri);

		int nMed = 0, nLat = 0;
		int nMedActive = 0, nLatActive = 0;


		for (int i = 0; i < nTri; ++i) {
			if (i == med_ind(nMed)) {
				med_pressure(nMed) = tri_pressure(i);
				med_proximity(nMed) = tri_proximity(i);
				med_area(nMed) = tri_area(i);
				med_normal(nMed) = tri_normal(i);
				med_tri_cenX(nMed) = tri_cenX(i);
				med_tri_cenY(nMed) = tri_cenY(i);
				med_tri_cenZ(nMed) = tri_cenZ(i);
				nMed++;

				if (tri_pressure(i) > 0.0) nMedActive++;

			}
			if (i == lat_ind(nLat)) {
				lat_pressure(nLat) = tri_pressure(i);
				lat_proximity(nLat) = tri_proximity(i);
				lat_area(nLat) = tri_area(i);
				lat_normal(nLat) = tri_normal(i);
				lat_tri_cenX(nLat) = tri_cenX(i);
				lat_tri_cenY(nLat) = tri_cenY(i);
				lat_tri_cenZ(nLat) = tri_cenZ(i);
				nLat++;

				if (tri_pressure(i) > 0.0) nLatActive++;
			}
		}

		//mean
		double med_mean_pressure = med_pressure.sum() / nMedActive;
		double med_mean_proximity = med_proximity.sum() / nMedActive;

		double lat_mean_pressure = lat_pressure.sum() / nLatActive;
		double lat_mean_proximity = lat_proximity.sum() / nLatActive;

		//max
		double med_max_pressure = med_pressure.normInf();
		double med_max_proximity = med_proximity.normInf();

		double lat_max_pressure = lat_pressure.normInf();
		double lat_max_proximity = lat_proximity.normInf();

		//area
		double med_contact_area = 0.0;

		for (int i = 0; i < nMed; i++) {
			if (med_pressure(i) > 0.0) {
				med_contact_area += 1000000.0 * med_area(i); //convert to mm^2
			}
		}

		double lat_contact_area = 0.0;

		for (int i = 0; i < nLat; i++) {
			if (lat_pressure(i) > 0.0) {
				lat_contact_area += 1000000 * lat_area(i); //convert to mm^2
			}
		}

		//COP
		Vector med_Num = med_pressure.elementwiseMultiply(med_area);
		Vector med_Den = med_pressure.elementwiseMultiply(med_area);
		double med_DenVal = med_Den.sum();

		double med_COPx, med_COPy, med_COPz;
		if (med_DenVal == 0) {
			med_COPx = med_COPy = med_COPz = -1;
		}
		else {
			Vector med_xNum = med_Num.elementwiseMultiply(med_tri_cenX);
			double med_xNumVal = med_xNum.sum() * 1000;
			med_COPx = med_xNumVal / med_DenVal;

			Vector med_yNum = med_Num.elementwiseMultiply(med_tri_cenY);
			double med_yNumVal = med_yNum.sum() * 1000;
			med_COPy = med_yNumVal / med_DenVal;

			Vector med_zNum = med_Num.elementwiseMultiply(med_tri_cenZ);
			double med_zNumVal = med_zNum.sum() * 1000;
			med_COPz = med_zNumVal / med_DenVal;
		}
		Vec3 med_COP(med_COPx, med_COPy, med_COPz);

		Vector lat_Num = lat_pressure.elementwiseMultiply(lat_area);
		Vector lat_Den = lat_pressure.elementwiseMultiply(lat_area);
		double lat_DenVal = lat_Den.sum();
		
		double lat_COPx, lat_COPy, lat_COPz;
		if (lat_DenVal == 0) {
			lat_COPx = lat_COPy = lat_COPz = -1;
		}
		else {
			Vector lat_xNum = lat_Num.elementwiseMultiply(lat_tri_cenX);
			double lat_xNumVal = lat_xNum.sum() * 1000;
			lat_COPx = lat_xNumVal / lat_DenVal;

			Vector lat_yNum = lat_Num.elementwiseMultiply(lat_tri_cenY);
			double lat_yNumVal = lat_yNum.sum() * 1000;
			lat_COPy = lat_yNumVal / lat_DenVal;

			Vector lat_zNum = lat_Num.elementwiseMultiply(lat_tri_cenZ);
			double lat_zNumVal = lat_zNum.sum() * 1000;
			lat_COPz = lat_zNumVal / lat_DenVal;

		}
		Vec3 lat_COP(lat_COPx, lat_COPy, lat_COPz);

		//contact force
		Vec3 med_contact_force(0.0);

		for (int i = 0; i < nMed; ++i) {
			med_contact_force += computeForceVector(med_pressure(i), med_area(i), -med_normal(i));
		}

		Vec3 lat_contact_force(0.0);

		for (int i = 0; i < nLat; ++i) {
			lat_contact_force += computeForceVector(lat_pressure(i), lat_area(i), -lat_normal(i));
		}

		//Save to cache variables
		if (flipMeshes) {
			setCacheVariableValue(state, "target_mesh.medial.mean_pressure", med_mean_pressure);
			setCacheVariableValue(state, "target_mesh.medial.mean_proximity", med_mean_proximity);
			setCacheVariableValue(state, "target_mesh.medial.max_pressure", med_max_pressure);
			setCacheVariableValue(state, "target_mesh.medial.max_proximity", med_max_proximity);
			setCacheVariableValue(state, "target_mesh.medial.contact_area", med_contact_area);
			setCacheVariableValue(state, "target_mesh.medial.cop", med_COP);
			setCacheVariableValue(state, "target_mesh.medial.contact_force", med_contact_force);

			setCacheVariableValue(state, "target_mesh.lateral.mean_pressure", lat_mean_pressure);
			setCacheVariableValue(state, "target_mesh.lateral.mean_proximity", lat_mean_proximity);
			setCacheVariableValue(state, "target_mesh.lateral.max_pressure", lat_max_pressure);
			setCacheVariableValue(state, "target_mesh.lateral.max_proximity", lat_max_proximity);
			setCacheVariableValue(state, "target_mesh.lateral.contact_area", lat_contact_area);
			setCacheVariableValue(state, "target_mesh.lateral.cop", lat_COP);
			setCacheVariableValue(state, "target_mesh.lateral.contact_force", lat_contact_force);

		}
		else {
			setCacheVariableValue(state, "casting_mesh.medial.mean_pressure", med_mean_pressure);
			setCacheVariableValue(state, "casting_mesh.medial.mean_proximity", med_mean_proximity);
			setCacheVariableValue(state, "casting_mesh.medial.max_pressure", med_max_pressure);
			setCacheVariableValue(state, "casting_mesh.medial.max_proximity", med_max_proximity);
			setCacheVariableValue(state, "casting_mesh.medial.contact_area", med_contact_area);
			setCacheVariableValue(state, "casting_mesh.medial.cop", med_COP);
			setCacheVariableValue(state, "casting_mesh.medial.contact_force", med_contact_force);

			setCacheVariableValue(state, "casting_mesh.lateral.mean_pressure", lat_mean_pressure);
			setCacheVariableValue(state, "casting_mesh.lateral.mean_proximity", lat_mean_proximity);
			setCacheVariableValue(state, "casting_mesh.lateral.max_pressure", lat_max_pressure);
			setCacheVariableValue(state, "casting_mesh.lateral.max_proximity", lat_max_proximity);
			setCacheVariableValue(state, "casting_mesh.lateral.contact_area", lat_contact_area);
			setCacheVariableValue(state, "casting_mesh.lateral.cop", lat_COP);
			setCacheVariableValue(state, "casting_mesh.lateral.contact_force", lat_contact_force);
		}
	}
}

OpenSim::Array<std::string> WISCO_ElasticFoundationForce::getRecordLabels() const {
	OpenSim::Array<std::string> labels("");

	labels.append(getName()+".force_x");
	labels.append(getName() + ".force_y");
	labels.append(getName() + ".force_z");
	labels.append(getName()+".torque_x");
	labels.append(getName() + ".torque_y");
	labels.append(getName() + ".torque_z");
/*
	std::vector<std::string> double_names, vec3_names;
	double_names.push_back("mean_pressure");
	double_names.push_back("max_pressure");
	double_names.push_back("mean_proximity");
	double_names.push_back("max_proximity");
	double_names.push_back("contact_area");
	vec3_names.push_back("cop");
	vec3_names.push_back("contact_force");

	for (std::string name : double_names) {
		labels.append(getName() + ".casting_mesh_total" + "_" + name);
		labels.append(getName() + ".casting_mesh_medial" + "_" + name);
		labels.append(getName() + ".casting_mesh_lateral" + "_" + name);
	}

	for (std::string name : vec3_names) {
		labels.append(getName() + ".casting_mesh_total" + "_" + name + "_x");
		labels.append(getName() + ".casting_mesh_total" + "_" + name + "_y");
		labels.append(getName() + ".casting_mesh_total" + "_" + name + "_z");

		labels.append(getName() + ".casting_mesh_medial" + "_" + name + "_x");
		labels.append(getName() + ".casting_mesh_medial" + "_" + name + "_y");
		labels.append(getName() + ".casting_mesh_medial" + "_" + name + "_z");

		labels.append(getName() + ".casting_mesh_lateral" + "_" + name + "_x");
		labels.append(getName() + ".casting_mesh_lateral" + "_" + name + "_y");
		labels.append(getName() + ".casting_mesh_lateral" + "_" + name + "_z");
	}
	*/
	return labels;
}


OpenSim::Array<double> WISCO_ElasticFoundationForce::getRecordValues(const SimTK::State& s) const {

	getModel().realizeDynamics(s);

	OpenSim::Array<double> values(1);

	Vec3 force = getCacheVariableValue<Vec3>(s, "force");
	Vec3 torque = getCacheVariableValue<Vec3>(s, "torque");
	values.append(force(0));
	values.append(force(1));
	values.append(force(2));
	values.append(torque(0));
	values.append(torque(1));
	values.append(torque(2));

	/*
	std::vector<std::string> double_names, vec3_names;
	double_names.push_back("mean_pressure");
	double_names.push_back("max_pressure");
	double_names.push_back("mean_proximity");
	double_names.push_back("max_proximity");
	double_names.push_back("contact_area");
	vec3_names.push_back("cop");
	vec3_names.push_back("contact_force");

	std::string delim{ "_" };

	for (std::string name : double_names) {
		values.append(getCacheVariableValue<double>(s, "casting_mesh_total" + delim + name));
		values.append(getCacheVariableValue<double>(s, "casting_mesh_medial" + delim + name));
		values.append(getCacheVariableValue<double>(s, "casting_mesh_lateral" + delim + name));
	}

	for (std::string name : vec3_names) {
		SimTK::Vec3 tot_val = getCacheVariableValue<SimTK::Vec3>(s, "casting_mesh_total" + delim + name);
		for (int i = 0; i < 3; ++i) {values.append(tot_val(i)); }

		SimTK::Vec3 med_val = getCacheVariableValue<SimTK::Vec3>(s, "casting_mesh_medial" + delim + name);
		for (int i = 0; i < 3; ++i) { values.append(med_val(i)); }

		SimTK::Vec3 lat_val = getCacheVariableValue<SimTK::Vec3>(s, "casting_mesh_lateral" + delim + name);
		for (int i = 0; i < 3; ++i) { values.append(lat_val(i)); }

	}
	*/
	return values;
}

//==============================================================================
//               WISCO_ELASTIC FOUNDATION FORCE :: CONTACT PARAMETERS
//==============================================================================

// Default constructor.
WISCO_ElasticFoundationForce::ContactParameters::ContactParameters()
{
    constructProperties();
}

// Constructor specifying material properties.
WISCO_ElasticFoundationForce::ContactParameters::ContactParameters
(double youngs_modulus, double poissons_ratio, double thickness)
{
    constructProperties();
    set_elastic_modulus(youngs_modulus);
    set_poissons_ratio(poissons_ratio);
    set_thickness(thickness);
}

void WISCO_ElasticFoundationForce::ContactParameters::constructProperties()
{
    constructProperty_elastic_modulus(0.0);
    constructProperty_poissons_ratio(0.0);
    constructProperty_thickness(0.0);
	constructProperty_use_variable_thickness(false);
	constructProperty_use_variable_elastic_modulus(false);
	constructProperty_use_variable_poissons_ratio(false);
}

