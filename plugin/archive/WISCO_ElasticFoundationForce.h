#ifndef OPENSIM_WISCO_ELASTIC_FOUNDATION_FORCE_H_
#define OPENSIM_WISCO_ELASTIC_FOUNDATION_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  UWElasticFoundationForce.h               *
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
// INCLUDE
#include "OpenSim/Simulation/Model/Force.h"
#include "coldet.h"
#include "WISCO_ContactMesh.h"
#include "osimPluginDLL.h"

namespace OpenSim {

    /**
     * An Elastic Foundation Force
     *
     * @author Colin Smith
     */
    //class WISCO_ElasticFoundationForce : public Force {
    class OSIMPLUGIN_API WISCO_ElasticFoundationForce : public Force {
        OpenSim_DECLARE_CONCRETE_OBJECT(WISCO_ElasticFoundationForce, Force)

        public:
            class ContactParameters;
            //==============================================================================
            // PROPERTIES
            //==============================================================================
            OpenSim_DECLARE_PROPERTY(mesh1_contact_params, WISCO_ElasticFoundationForce::ContactParameters, "Mesh1 Material Properties")
            OpenSim_DECLARE_PROPERTY(mesh2_contact_params, WISCO_ElasticFoundationForce::ContactParameters, "Mesh2 Material Properties")
			OpenSim_DECLARE_OPTIONAL_PROPERTY(verbose,int,"Level of reporting for debugging purposes (0-silent, 1-simple info)")

            //==============================================================================
            // Connectors
            //==============================================================================
            OpenSim_DECLARE_SOCKET(mesh1, WISCO_ContactMesh, "Contact Mesh1")
			OpenSim_DECLARE_SOCKET(mesh2, WISCO_ContactMesh, "Contact Mesh1")

			//==============================================================================
            // OUTPUTS
            //==============================================================================
            OpenSim_DECLARE_OUTPUT(mesh1_tri_pressure, SimTK::Vector, getMesh1TriPressure, SimTK::Stage::Dynamics)
            OpenSim_DECLARE_OUTPUT(mesh2_tri_pressure, SimTK::Vector, getMesh2TriPressure, SimTK::Stage::Dynamics)
            OpenSim_DECLARE_OUTPUT(mesh1_tri_proximity, SimTK::Vector, getMesh1TriProximity, SimTK::Stage::Dynamics)
            OpenSim_DECLARE_OUTPUT(mesh2_tri_proximity, SimTK::Vector, getMesh2TriProximity, SimTK::Stage::Dynamics)
            OpenSim_DECLARE_OUTPUT(mesh1_n_active_tri, int, getMesh1ActiveTri, SimTK::Stage::Dynamics)
            OpenSim_DECLARE_OUTPUT(mesh2_n_active_tri, int, getMesh2ActiveTri, SimTK::Stage::Dynamics)
            OpenSim_DECLARE_OUTPUT(ground_to_mesh1, SimTK::Vector, getTransformVectorGroundToMesh1, SimTK::Stage::Position)
            OpenSim_DECLARE_OUTPUT(ground_to_mesh2, SimTK::Vector, getTransformVectorGroundToMesh2, SimTK::Stage::Position)
            //OpenSim_DECLARE_OUTPUT(ground_to_mesh1, SimTK::Transform, getTransformGroundToMesh1, SimTK::Stage::Position)
            //OpenSim_DECLARE_OUTPUT(ground_to_mesh2, SimTK::Transform, getTransformGroundToMesh2, SimTK::Stage::Position)

            //==============================================================================
            // PUBLIC METHODS
            //==============================================================================
            /**
             * Default Construct of an WISCO_ElasticFoundationForce.
             */
            WISCO_ElasticFoundationForce();

            /**
             * Convience Constructor of WISCO_ElasticFoundationForce
             */
            WISCO_ElasticFoundationForce(
                WISCO_ContactMesh& mesh1,
                WISCO_ElasticFoundationForce::ContactParameters mesh1_params,
                WISCO_ContactMesh& mesh2,
                WISCO_ElasticFoundationForce::ContactParameters mesh2_params,
				int verbose = 0);

            SimTK::Vector getMesh1TriPressure(const SimTK::State& state) const
            {
                return updCacheVariableValue<SimTK::Vector>(state, "mesh1_tri_pressure");
            }

            SimTK::Vector getMesh2TriPressure(const SimTK::State& state) const
            {
                return updCacheVariableValue<SimTK::Vector>(state, "mesh2_tri_pressure");
            }

            SimTK::Vector getMesh1TriProximity(const SimTK::State& state) const
            {
                return updCacheVariableValue<SimTK::Vector>(state, "mesh1_tri_proximity");
            }

            SimTK::Vector getMesh2TriProximity(const SimTK::State& state) const
            {
                return updCacheVariableValue<SimTK::Vector>(state, "mesh2_tri_proximity");
            }

            int getMesh1ActiveTri(const SimTK::State& state) const
            {
                return updCacheVariableValue<int>(state, "mesh1_n_active_tri");
            }

            int getMesh2ActiveTri(const SimTK::State& state) const
            {
                return updCacheVariableValue<int>(state, "mesh2_n_active_tri");
            }


        protected:

            /**  ModelComponent interface */
			void extendAddToSystem(SimTK::MultibodySystem& system) const override;
            void extendInitStateFromProperties(SimTK::State & 	state)	const override;
            void extendRealizeReport(const SimTK::State & state)	const override;
            /**
             * Compute the force.
             */
            void computeForce(const SimTK::State& state,
                SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                SimTK::Vector& generalizedForces) const override;

        private:
            double computeTriProximity(SimTK::Vec3 cnt_pnt_ground, SimTK::Vec3 tri_cen_ground) const;
            double computeTriPressure(double depth) const;
			bool WISCO_ElasticFoundationForce::verifyTriContact(SimTK::Vec3 tri_nor1_ground, SimTK::Vec3 tri_nor2_ground, double depth) const;

            void applyTriPressureForce(const SimTK::State &state, SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
				double pressure, double area, SimTK::Vec3 center, SimTK::Vec3 normal, SimTK::Vec3 center_in_m1) const;
            
            bool rayIntersectTriTest(SimTK::Vec3 p, SimTK::Vec3 d,
                SimTK::Vec3 v0, SimTK::Vec3 v1, SimTK::Vec3 v2, SimTK::Vec3& intersection_pt, double& distance) const;

            double collision_detection(const SimTK::State& state, SimTK::Vector_<SimTK::SpatialVec>& bodyForces) const;

            SimTK::Vector getTransformVectorGroundToMesh1(const SimTK::State& state) const;
            SimTK::Vector getTransformVectorGroundToMesh2(const SimTK::State& state) const;
            SimTK::Transform getTransformGroundToMesh1(const SimTK::State& state) const;
            SimTK::Transform getTransformGroundToMesh2(const SimTK::State& state) const;
            SimTK::Transform getTransformMesh2ToMesh1(const SimTK::State& state) const;

            SimTK::Vec3 WISCO_ElasticFoundationForce::computeForceVector(double pressure, double area, SimTK::Vec3 normal) const;
            
            SimTK::Vec3 WISCO_ElasticFoundationForce::computeMomentVector(double pressure, double area, SimTK::Vec3 normal, SimTK::Vec3 center) const;

            void setNull();
            void constructProperties();


    //==============================================================================
    };  // END of class WISCO_ElasticFoundationForce
    //==============================================================================


    //==============================================================================
    //              ELASTIC FOUNDATION FORCE :: CONTACT PARAMETERS
    //==============================================================================
	//class WISCO_ElasticFoundationForce::ContactParameters : public Object {
    class OSIMPLUGIN_API WISCO_ElasticFoundationForce::ContactParameters : public Object {
        OpenSim_DECLARE_CONCRETE_OBJECT(WISCO_ElasticFoundationForce::ContactParameters, Object)
        public:
        //==============================================================================
        // PROPERTIES
        //==============================================================================
            OpenSim_DECLARE_PROPERTY(youngs_modulus, double, "Youngs Modulus")
            OpenSim_DECLARE_PROPERTY(poissons_ratio, double, "Poissons Ratio")
            OpenSim_DECLARE_PROPERTY(thickness, double, "Thickness of elastic layer")
            OpenSim_DECLARE_PROPERTY(min_proximity, double, "Minimum overlap depth between contacting meshes")
            OpenSim_DECLARE_PROPERTY(max_proximity, double, "Maximum overlap depth between contacting meshes")
            OpenSim_DECLARE_PROPERTY(min_thickness, double, "Minimum thickness for variable cartilage thickness")
            OpenSim_DECLARE_PROPERTY(max_thickness, double, "Maximum thickness for variable cartilage thickness")

            //==============================================================================
            // PUBLIC METHODS
            //==============================================================================
            ContactParameters();
            ContactParameters(double youngs_modulus, double poissons_ratio, double thickness);
            ContactParameters(double youngs_modulus, double poissons_ratio, double thickness,
                double min_proximity, double max_proximity);

     private:
        void constructProperties();
    };


} // end of namespace OpenSim

#endif // OPENSIM_UW_ELASTIC_FOUNDATION_FORCE_H_
