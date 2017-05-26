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
#include "OpenSim/OpenSim.h"
//==============================================================================
// USING
//==============================================================================
using namespace OpenSim;
using SimTK::Vec3;

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
	WISCO_ContactMesh& mesh1,
    WISCO_ElasticFoundationForce::ContactParameters mesh1_params,
    WISCO_ContactMesh& mesh2,
    WISCO_ElasticFoundationForce::ContactParameters mesh2_params,
    int verbose)
{
	setNull();
	constructProperties();

    set_mesh1_contact_params(mesh1_params);
    set_mesh2_contact_params(mesh2_params);
	set_verbose(verbose);
    updSocket<WISCO_ContactMesh>("mesh1").connect(mesh1);
    updSocket<WISCO_ContactMesh>("mesh2").connect(mesh2);
}


void WISCO_ElasticFoundationForce::setNull()
{
    setAuthors("Colin Smith");

}

void WISCO_ElasticFoundationForce::constructProperties()
{   
    constructProperty_mesh1_contact_params(WISCO_ElasticFoundationForce::ContactParameters());
    constructProperty_mesh2_contact_params(WISCO_ElasticFoundationForce::ContactParameters());

	constructProperty_verbose();

}

void WISCO_ElasticFoundationForce::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    
    auto mesh1_params = get_mesh1_contact_params();
    auto mesh2_params = get_mesh2_contact_params();
    
    int mesh1_nTri = getSocket<WISCO_ContactMesh>("mesh1").getConnectee().getNumFaces();
    int mesh2_nTri = getSocket<WISCO_ContactMesh>("mesh2").getConnectee().getNumFaces();
    
    SimTK::Vector mesh1_def_vec(mesh1_nTri);
    SimTK::Vector mesh2_def_vec(mesh2_nTri);
    mesh1_def_vec = 0;
    mesh2_def_vec = 0;

    addCacheVariable<SimTK::Vector>("mesh2_target_tri", mesh2_def_vec, SimTK::Stage::LowestRuntime);
    
    addCacheVariable<SimTK::Vector>("mesh1_tri_pressure", mesh1_def_vec, SimTK::Stage::Dynamics);
    addCacheVariable<SimTK::Vector>("mesh2_tri_pressure", mesh2_def_vec, SimTK::Stage::Dynamics);

    addCacheVariable<SimTK::Vector>("mesh1_tri_proximity", mesh1_def_vec, SimTK::Stage::Dynamics);
    addCacheVariable<SimTK::Vector>("mesh2_tri_proximity", mesh2_def_vec, SimTK::Stage::Dynamics);

    addCacheVariable<int>("mesh1_n_active_tri", 0, SimTK::Stage::Dynamics);
    addCacheVariable<int>("mesh2_n_active_tri", 0, SimTK::Stage::Dynamics);


    //CollisionModel3D* def_coldet_model;

    //addCacheVariable<CollisionModel3D*>("m1_coldet_model", def_coldet_model, SimTK::Stage::LowestRuntime);
}

void WISCO_ElasticFoundationForce::extendInitStateFromProperties(SimTK::State & 	state)	const

{
    Super::extendInitStateFromProperties(state);
    
    int mesh2_nTri = getSocket<WISCO_ContactMesh>("mesh2").getConnectee().getNumFaces();
    SimTK::Vector default_vec(mesh2_nTri);
    default_vec = -1;

    setCacheVariableValue<SimTK::Vector>(state, "mesh2_target_tri", default_vec);

    //CollisionModel3D* m1 = getConnector<WISCO_ContactMesh>("mesh1").getConnectee().updColdetModel();
    //setCacheVariableValue<CollisionModel3D*>(state,"m1_coldet_model",)
}

void WISCO_ElasticFoundationForce::extendRealizeReport(const SimTK::State & state)	const
{
	if (get_verbose() > 0) {
		std::cout << std::endl;
		std::cout << "Time: " << state.getTime() << std::endl;
		std::cout << getName() << " Mesh 2 Active Tris: ";
		std::cout << getCacheVariableValue<int>(state, "mesh2_n_active_tri");
		std::cout << "/" << getConnectee<WISCO_ContactMesh>("mesh2").getNumFaces() << std::endl;
		std::cout << std::endl;
	}
}

//-----------------------------------------------------------------------------
// FORCE METHODS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void WISCO_ElasticFoundationForce::computeForce(const SimTK::State& state,
	SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
	SimTK::Vector& generalizedForces) const
{
	double distance;
	distance = collision_detection(state, bodyForces);

}

double WISCO_ElasticFoundationForce::collision_detection(const SimTK::State& state, SimTK::Vector_<SimTK::SpatialVec>& bodyForces) const
{
	// Get Mesh Properties
    double min_proximity = get_mesh1_contact_params().get_min_proximity();
	double max_proximity = get_mesh1_contact_params().get_max_proximity();

    const WISCO_ContactMesh& mm1 = getConnectee<WISCO_ContactMesh>("mesh1");
    const WISCO_ContactMesh& mm2 = getConnectee<WISCO_ContactMesh>("mesh2");
	
    SimTK::Vector_<Vec3> tri_cen2 = mm2.getTriangleCenters();

    SimTK::Vector tri_area1 = mm1.getTriangleAreas();
    SimTK::Vector tri_area2 = mm2.getTriangleAreas();

    // Get Mesh Transforms 
    SimTK::Transform GtoM1 = mm1.getTransformGroundToMesh(state);
    SimTK::Transform GtoM2 = mm2.getTransformGroundToMesh(state);
	SimTK::Transform M2toM1 = getTransformMesh2ToMesh1(state);
	
    //Reset depth arrays
    SimTK::Vector mesh1_tri_pressure = updCacheVariableValue<SimTK::Vector>(state, "mesh1_tri_pressure");
    SimTK::Vector mesh2_tri_pressure = updCacheVariableValue<SimTK::Vector>(state, "mesh2_tri_pressure");
    mesh1_tri_pressure = 0;
    mesh2_tri_pressure = 0;

    SimTK::Vector mesh1_tri_proximity = updCacheVariableValue<SimTK::Vector>(state, "mesh1_tri_proximity");
    SimTK::Vector mesh2_tri_proximity = updCacheVariableValue<SimTK::Vector>(state, "mesh2_tri_proximity");
    mesh1_tri_proximity = 0;
    mesh2_tri_proximity = 0;

    //Transfrom mesh centers and normals to ground reference frame for ray casting

	SimTK::Matrix_<Vec3> mesh1_ver_loc_ground = mm1.getVertexLocationsInGround(state);

	SimTK::Vector_<Vec3> tri_cen1_ground = mm1.getTriangleCentersInGround(state);
	SimTK::Vector_<Vec3> tri_cen2_ground = mm2.getTriangleCentersInGround(state);

	SimTK::Vector_<Vec3> tri_nor1_ground = mm1.getTriangleNormalsInGround(state);
	SimTK::Vector_<Vec3> tri_nor2_ground = mm2.getTriangleNormalsInGround(state);

	//Collision Detection
	//===================
    
	//Get the collision object from mesh
    CollisionModel3D* m1 = getConnectee<WISCO_ContactMesh>("mesh1").updCacheVariableValue<CollisionModel3D*>(state,"coldet_model");

	auto coldet_transform = ~GtoM1.toMat44();
    
	float m1_transform[16];
	int count = 0;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			m1_transform[count] =coldet_transform(i, j);
			count++;
		}
	}
	m1->setTransform(m1_transform);
	
	//Check for triangles in contact
	//==============================
    int mesh1_nActiveTri = 0;
    int mesh2_nActiveTri = 0;
    float origin[3], vector[3];

    auto mesh2_target_tri = getCacheVariableValue<SimTK::Vector>(state, "mesh2_target_tri");
    Vec3 force(0);
    Vec3 moment(0);
	
	//Keep track of triangle collision type for debugging
    int same = 0;
    int diff = 0;
	int neighbor = 0;

	//Loop through all triangles in mesh2
    for (int i = 0; i < mm2.getNumFaces(); ++i) {
        bool contact_detected = false;
		double depth = 0.0;
		Vec3 contact_pnt_ground(0);

        Vec3 center = tri_cen2_ground(i);
        Vec3 normal = tri_nor2_ground(i);
        
        Vec3 center_in_m1 = M2toM1.shiftBaseStationToFrame(center);

        //If triangle was in contact in previous timestep, recheck same contact triangle and neighbors
        if (mesh2_target_tri(i) >= 0) {
			
            //Check same triangle as previous time step
            Vec3 vertex0 = mesh1_ver_loc_ground(mesh2_target_tri(i),0);
            Vec3 vertex1 = mesh1_ver_loc_ground(mesh2_target_tri(i), 1);
            Vec3 vertex2 = mesh1_ver_loc_ground(mesh2_target_tri(i), 2);
			
            if (rayIntersectTriTest(center, -normal, vertex0, vertex1, vertex2, contact_pnt_ground, depth))
            {
                if (verifyTriContact(tri_nor1_ground(mesh2_target_tri(i)), tri_nor2_ground(i), depth)) {
                    mesh2_tri_proximity(i) = depth;
                    mesh2_tri_pressure(i) = computeTriPressure(depth);
                    force += computeForceVector(mesh2_tri_pressure(i), tri_area2(i), -normal);
                    moment += computeMomentVector(mesh2_tri_pressure(i), tri_area2(i), -normal, tri_cen2(i));
                    mesh2_nActiveTri++;
                    same++;
                    continue;
                }
            }
            
            //Check neighboring triangles
            int nNeighborTri;
            SimTK::Vector neighborTri = mm1.getNeighborTris(mesh2_target_tri(i), nNeighborTri);

	        for (int j = 0; j < nNeighborTri; ++j) {

                vertex0 = mesh1_ver_loc_ground(neighborTri(j),0);
                vertex1 = mesh1_ver_loc_ground(neighborTri(j), 1);
                vertex2 = mesh1_ver_loc_ground(neighborTri(j), 2);

                if (rayIntersectTriTest(center, -normal, vertex0, vertex1, vertex2, contact_pnt_ground, depth)) {
                    if (verifyTriContact(tri_nor1_ground(neighborTri(j)), tri_nor2_ground(i), depth)) {
						mesh2_tri_proximity(i) = depth;
						mesh2_tri_pressure(i) = computeTriPressure(depth);
						force += computeForceVector(mesh2_tri_pressure(i), tri_area2(i), -normal);
						moment += computeMomentVector(mesh2_tri_pressure(i), tri_area2(i), -normal, tri_cen2(i));
						mesh2_nActiveTri++;

                        mesh2_target_tri(i) = neighborTri(j);
                        contact_detected = true;
						neighbor++;
                        break; 
                    }
                }
            }
            if (contact_detected)
                continue;
                
        }
        
        //Go through the OBB hierarchy
        //----------------------------
        
        for (int j = 0; j < 3; ++j) {
            origin[j] = tri_cen2_ground(i)(j);
            vector[j] = -tri_nor2_ground(i)(j);
        }

        if (m1->rayCollision(origin, vector, true, min_proximity, max_proximity)) {
		    //Colliding Triangles
            int tri1, tri2;
            m1->getCollidingTriangles(tri1, tri2);

			// Compute Proximity
			float cnt_point[3];
			m1->getCollisionPoint(cnt_point, false);

			for (int j = 0; j < 3; ++j) {
				contact_pnt_ground(j) = cnt_point[j];
			}

			depth = computeTriProximity(contact_pnt_ground, tri_cen2_ground(i));

			//Verify Collision
			if (verifyTriContact(tri_nor1_ground(tri1), tri_nor2_ground(i),depth))
			{
				mesh2_target_tri(i) = tri1;
				mesh2_tri_proximity(i) = depth;
				mesh2_tri_pressure(i) = computeTriPressure(mesh2_tri_proximity(i));
				force += computeForceVector(mesh2_tri_pressure(i), tri_area2(i), -normal);
				moment += computeMomentVector(mesh2_tri_pressure(i), tri_area2(i), -normal, tri_cen2(i));
				mesh2_nActiveTri++;
				diff++;
				continue;
			}
        }

        //Else - triangle is not in contact
        mesh2_target_tri[i] = -1;
        
    }

    //Apply Resultant Force and Moment
    const PhysicalFrame& frame1 = getConnectee<WISCO_ContactMesh>("mesh1").getConnectee<PhysicalFrame>("frame");
    const PhysicalFrame& frame2 = getConnectee<WISCO_ContactMesh>("mesh2").getConnectee<PhysicalFrame>("frame");

    applyForceToPoint(state, frame2, Vec3(0), force, bodyForces);
    applyTorque(state, frame1, moment, bodyForces);
    
    Vec3 point_in_m1 = M2toM1.shiftBaseStationToFrame(Vec3(0));
    applyTorque(state, frame2, -moment, bodyForces);
    applyForceToPoint(state, frame1, point_in_m1, -force, bodyForces);

    //Store Contact Info
    setCacheVariableValue(state, "mesh1_tri_pressure", mesh1_tri_pressure);
    setCacheVariableValue(state, "mesh2_tri_pressure", mesh2_tri_pressure);
    setCacheVariableValue(state, "mesh1_tri_proximity", mesh1_tri_proximity);
    setCacheVariableValue(state, "mesh2_tri_proximity", mesh2_tri_proximity);
	setCacheVariableValue<SimTK::Vector>(state, "mesh2_target_tri", mesh2_target_tri);
    setCacheVariableValue<int>(state, "mesh1_n_active_tri",mesh1_nActiveTri);
    setCacheVariableValue<int>(state, "mesh2_n_active_tri", mesh2_nActiveTri);
    getConnectee<WISCO_ContactMesh>("mesh1").markCacheVariableValid(state, "coldet_model");
    
	//Debugging Report to console
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

	return 0;
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
    double a, f, u, v, t;

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

bool WISCO_ElasticFoundationForce::verifyTriContact(SimTK::Vec3 tri_nor1_ground, SimTK::Vec3 tri_nor2_ground, double depth) const
{
	//Check proximity in range
	double min_proximity = get_mesh1_contact_params().get_min_proximity();
	double max_proximity = get_mesh1_contact_params().get_max_proximity();

	if ((depth < min_proximity) || (depth > max_proximity))
		return false;

	// Check to make sure backside of mesh2 isn't causing contact
	double dir_vec = SimTK::dot(-tri_nor2_ground, tri_nor1_ground.normalize());

	if (dir_vec > 0)
		return true;
	else
		return false;
}

double WISCO_ElasticFoundationForce::computeTriProximity(SimTK::Vec3 cnt_pnt_ground, SimTK::Vec3 tri_cen_ground) const
{

	//Find Depth
		SimTK::Vec3 cen_cnt_point;
		cen_cnt_point = cnt_pnt_ground - tri_cen_ground;

		double depth = sqrt(pow(cen_cnt_point(0), 2) + pow(cen_cnt_point(1), 2) + pow(cen_cnt_point(2), 2));

    return depth;
}

double WISCO_ElasticFoundationForce::computeTriPressure(double depth) const
{
    double pressure;

    //Get Contact Parameters
    auto thickness = get_mesh1_contact_params().get_thickness();
    auto E = get_mesh1_contact_params().get_youngs_modulus();
    auto v = get_mesh1_contact_params().get_poissons_ratio();
    
    //linear
    if (true) {
        pressure = ((1 - v)*E) / ((1 + v)*(1 - 2 * v)) * depth / thickness;
    }

    //nonlinear
    if (false) {
        pressure = ((1 - v)*E) / ((1 + v)*(1 - 2 * v)) * log(1 - depth / thickness);
    }

    return pressure;
}

/** 
Normal in ground reference frame, in direction force is applied (opposite triangle normal)
Center in body reference frame

*/
void WISCO_ElasticFoundationForce::applyTriPressureForce(const SimTK::State &state, SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
    double pressure, double area, SimTK::Vec3 center, SimTK::Vec3 normal, SimTK::Vec3 center_in_m1) const
{
    // Apply Force to Frame 2
    Vec3 force;
    force(0) = normal(0) * pressure * area;
    force(1) = normal(1) * pressure * area;
    force(2) = normal(2) * pressure * area;

    const PhysicalFrame& frame1 = getConnectee<WISCO_ContactMesh>("mesh1").getConnectee<PhysicalFrame>("frame");
    const PhysicalFrame& frame2 = getConnectee<WISCO_ContactMesh>("mesh2").getConnectee<PhysicalFrame>("frame");

    applyForceToPoint(state, frame2, center, force, bodyForces);

    // Apply Equal and Opposite Force to Frame 1
    applyForceToPoint(state, frame1, center_in_m1, -force, bodyForces);


}

SimTK::Vec3 WISCO_ElasticFoundationForce::computeForceVector(double pressure, double area, SimTK::Vec3 normal) const 
{
    Vec3 force;
    force(0) = normal(0) * pressure * area;
    force(1) = normal(1) * pressure * area;
    force(2) = normal(2) * pressure * area;
    return force;
}

SimTK::Vec3 WISCO_ElasticFoundationForce::computeMomentVector(double pressure, double area, SimTK::Vec3 normal, SimTK::Vec3 center) const
{
    Vec3 moment;
    moment(0) = normal(0) * pressure * area * center(0);
    moment(1) = normal(1) * pressure * area * center(1);
    moment(2) = normal(2) * pressure * area * center(2);
    return moment;
}

SimTK::Vector WISCO_ElasticFoundationForce::getTransformVectorGroundToMesh1(const SimTK::State& state) const
{
    const PhysicalFrame& frame = getConnectee<WISCO_ContactMesh>("mesh1").getConnectee<PhysicalFrame>("frame");
    SimTK::Mat44 T_matrix = frame.getTransformInGround(state).toMat44();
    SimTK::Vector T_vector(16);

    int c = 0;
    for (int i = 0; i < 4; ++i){ 
        for (int j = 0; j < 4; ++j){
            T_vector(c) = T_matrix(i, j);
            c++;
        }
    }
    
    return T_vector;
}

SimTK::Vector WISCO_ElasticFoundationForce::getTransformVectorGroundToMesh2(const SimTK::State& state) const
{
    const PhysicalFrame& frame = getConnectee<WISCO_ContactMesh>("mesh2").getConnectee<PhysicalFrame>("frame");
    SimTK::Mat44 T_matrix = frame.getTransformInGround(state).toMat44();
    SimTK::Vector T_vector(16);

    int c = 0;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            T_vector(c) = T_matrix(i, j);
            c++;
        }
    }

    return T_vector;
}

SimTK::Transform WISCO_ElasticFoundationForce::getTransformGroundToMesh1(const SimTK::State& state) const
{
    const PhysicalFrame& frame = getConnectee<WISCO_ContactMesh>("mesh1").getConnectee<PhysicalFrame>("frame");
    return frame.getTransformInGround(state);
}

SimTK::Transform WISCO_ElasticFoundationForce::getTransformGroundToMesh2(const SimTK::State& state) const
{
    const PhysicalFrame& frame = getConnectee<WISCO_ContactMesh>("mesh2").getConnectee<PhysicalFrame>("frame");
    return frame.getTransformInGround(state);
}

SimTK::Transform WISCO_ElasticFoundationForce::getTransformMesh2ToMesh1(const SimTK::State& state) const
{
    const WISCO_ContactMesh& mesh1 = getConnectee<WISCO_ContactMesh>("mesh1");
	const WISCO_ContactMesh& mesh2 = getConnectee<WISCO_ContactMesh>("mesh2");
	
    const PhysicalFrame& frame1 = mesh1.getConnectee<PhysicalFrame>("frame");
    const PhysicalFrame& frame2 = mesh2.getConnectee<PhysicalFrame>("frame");

	return frame2.findTransformBetween(state,frame1);

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
    set_youngs_modulus(youngs_modulus);
    set_poissons_ratio(poissons_ratio);
    set_thickness(thickness);
}

WISCO_ElasticFoundationForce::ContactParameters::ContactParameters
(double youngs_modulus, double poissons_ratio, double thickness,
    double min_proximity, double max_proximity): 
    ContactParameters(youngs_modulus, poissons_ratio,thickness)
{
    
    set_min_proximity(min_proximity);
    set_max_proximity(max_proximity);
    
}

void WISCO_ElasticFoundationForce::ContactParameters::constructProperties()
{
    constructProperty_youngs_modulus(0.0);
    constructProperty_poissons_ratio(0.0);
    constructProperty_thickness(0.0);
    constructProperty_min_proximity(0.0);
    constructProperty_max_proximity(0.2);
    constructProperty_min_thickness(0.0);
    constructProperty_max_thickness(0.2);
}

