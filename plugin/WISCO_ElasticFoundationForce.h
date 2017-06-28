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
//#include "coldet.h"
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
		OpenSim_DECLARE_PROPERTY(min_proximity, double, "Minimum overlap depth between contacting meshes")
		OpenSim_DECLARE_PROPERTY(max_proximity, double, "Maximum overlap depth between contacting meshes")
		OpenSim_DECLARE_PROPERTY(elastic_foundation_formulation, std::string,
			"Formulation for depth-pressure relationship: 'linear' or 'nonlinear'")
		OpenSim_DECLARE_PROPERTY(use_smart_backside_contact, bool,
			"Use algorithm to ignore backside contact on highly curved meshes.")
		OpenSim_DECLARE_PROPERTY(use_lumped_contact_model, bool, 
			"Average the ContactParams for both meshes and use Bei & Fregly 2003 lumped parameter EF model")
		OpenSim_DECLARE_OPTIONAL_PROPERTY(verbose, int, "Level of reporting for debugging purposes (0-silent, 1-simple, 2-detailed)")
		OpenSim_DECLARE_PROPERTY(target_mesh_contact_params, WISCO_ElasticFoundationForce::ContactParameters, "target_mesh Material Properties")
		OpenSim_DECLARE_PROPERTY(casting_mesh_contact_params, WISCO_ElasticFoundationForce::ContactParameters, "casting_mesh Material Properties")


		//==============================================================================
		// Connectors
		//==============================================================================
		OpenSim_DECLARE_SOCKET(target_mesh, WISCO_ContactMesh, "Target mesh for collision detection.")
		OpenSim_DECLARE_SOCKET(casting_mesh, WISCO_ContactMesh, "Ray casting mesh for collision detection.")

		//==============================================================================
		// OUTPUTS
		//==============================================================================
		OpenSim_DECLARE_OUTPUT(target_mesh_n_active_tri, int, getTargetMeshActiveTri, SimTK::Stage::Dynamics)
		OpenSim_DECLARE_OUTPUT(casting_mesh_n_active_tri, int, getCastingMeshActiveTri, SimTK::Stage::Dynamics)
		
		OpenSim_DECLARE_LIST_OUTPUT(casting_tri_data, SimTK::Vector, getTriData, SimTK::Stage::Dynamics)
		OpenSim_DECLARE_LIST_OUTPUT(target_tri_data, SimTK::Vector, getTriData, SimTK::Stage::Dynamics)

		OpenSim_DECLARE_LIST_OUTPUT(casting_vertex_data, SimTK::Vector, getVertexData, SimTK::Stage::Dynamics)
		OpenSim_DECLARE_LIST_OUTPUT(target_vertex_data, SimTK::Vector, getVertexData, SimTK::Stage::Dynamics)

		OpenSim_DECLARE_LIST_OUTPUT(casting_contact_stats_total, double, getContactStatsTotal, SimTK::Stage::Dynamics)
		OpenSim_DECLARE_LIST_OUTPUT(target_contact_stats_total, double, getContactStatsTotal, SimTK::Stage::Dynamics)

		OpenSim_DECLARE_LIST_OUTPUT(casting_contact_stats_total_vec3, SimTK::Vec3, getContactStatsTotalVec3, SimTK::Stage::Dynamics)
		OpenSim_DECLARE_LIST_OUTPUT(target_contact_stats_total_vec3, SimTK::Vec3, getContactStatsTotalVec3, SimTK::Stage::Dynamics)

		OpenSim_DECLARE_LIST_OUTPUT(casting_contact_stats_medial_lateral, double, getContactStatsMedialLateral,SimTK::Stage::Dynamics)
		OpenSim_DECLARE_LIST_OUTPUT(target_contact_stats_medial_lateral, double, getContactStatsMedialLateral, SimTK::Stage::Dynamics)

		OpenSim_DECLARE_LIST_OUTPUT(casting_contact_stats_medial_lateral_vec3, SimTK::Vec3, getContactStatsMedialLateralVec3, SimTK::Stage::Dynamics)
		OpenSim_DECLARE_LIST_OUTPUT(target_contact_stats_medial_lateral_vec3, SimTK::Vec3, getContactStatsMedialLateralVec3, SimTK::Stage::Dynamics)

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
			WISCO_ContactMesh& target_mesh,
			WISCO_ElasticFoundationForce::ContactParameters target_mesh_params,
			WISCO_ContactMesh& casting_mesh,
			WISCO_ElasticFoundationForce::ContactParameters casting_mesh_params,
			int verbose = 0);


	public:
		
		/*-----------------------------------------------------------------
		Output Methods
		-----------------------------------------------------------------*/
		//Mesh Data
		//----
		SimTK::Vector getTriData(const SimTK::State& state,
			const std::string& channel) const {
				return getCacheVariableValue<SimTK::Vector>(state, channel);
		}

		SimTK::Vector getVertexData(const SimTK::State& state,
			const std::string& channel) const {
			return getCacheVariableValue<SimTK::Vector>(state, channel);
		}

		int getTargetMeshActiveTri(const SimTK::State& state) const {
			return getCacheVariableValue<int>(state, "target_mesh_n_active_tri");
		}

		int getCastingMeshActiveTri(const SimTK::State& state) const {
			return getCacheVariableValue<int>(state, "casting_mesh_n_active_tri");
		}

		//Contact Stats
		//-------------
		double getContactStatsTotal(const SimTK::State& state,
			const std::string& channel) const {

			for (std::string stat : _stat_names) {

				if (channel == "casting_mesh.total." + stat) {
					return getCacheVariableValue<double>(state, "casting_mesh.total." + stat);
				}
				else if (channel == "target_mesh.total." + stat) {
					return getCacheVariableValue<double>(state, "target_mesh.total." + stat);
				}
			}
		}

		SimTK::Vec3 getContactStatsTotalVec3(const SimTK::State& state,
			const std::string& channel) const {

			for (std::string stat_vec3 : _stat_names_vec3) {

				if (channel == "casting_mesh.total." + stat_vec3) {
					return getCacheVariableValue<SimTK::Vec3>(state, "casting_mesh.total." + stat_vec3);
				}
				else if (channel == "target_mesh.total." + stat_vec3) {
					return getCacheVariableValue<SimTK::Vec3>(state, "target_mesh.total." + stat_vec3);
				}
			}
		}

		double getContactStatsMedialLateral(const SimTK::State& state,
			const std::string& channel) const {

			for (std::string stat : _stat_names) {

				if (channel == "casting_mesh.medial." + stat) {
					return getCacheVariableValue<double>(state, "casting_mesh.medial." + stat);
				}
				else if (channel == "target_mesh.medial." + stat) {
					return getCacheVariableValue<double>(state, "target_mesh.medial." + stat);
				}
				else if (channel == "casting_mesh.lateral." + stat) {
					return getCacheVariableValue<double>(state, "casting_mesh.lateral." + stat);
				}
				else if (channel == "target_mesh.lateral." + stat) {
					return getCacheVariableValue<double>(state, "target_mesh.lateral." + stat);
				}
			}
		}

		SimTK::Vec3 getContactStatsMedialLateralVec3(const SimTK::State& state,
			const std::string& channel) const {

			for (std::string stat_vec3 : _stat_names_vec3) {

				if (channel == "casting_mesh.medial." + stat_vec3) {
					return getCacheVariableValue<SimTK::Vec3>(state, "casting_mesh.medial." + stat_vec3);
				}
				else if (channel == "target_mesh.medial." + stat_vec3) {
					return getCacheVariableValue<SimTK::Vec3>(state, "target_mesh.medial." + stat_vec3);
				}
				else if (channel == "casting_mesh.lateral." + stat_vec3) {
					return getCacheVariableValue<SimTK::Vec3>(state, "casting_mesh.lateral." + stat_vec3);
				}
				else if (channel == "target_mesh.lateral." + stat_vec3) {
					return getCacheVariableValue<SimTK::Vec3>(state, "target_mesh.lateral." + stat_vec3);					
				}
			}
			OPENSIM_THROW(Exception, "Output channel " + channel + " was not found.");
		}

			

		OpenSim::Array<double> getRecordValues(const SimTK::State& s) const;
		OpenSim::Array<std::string> getRecordLabels() const;


		SimTK::Vector_<SimTK::Vec3> getMeshVerticesInFrame(const SimTK::State& state,
			const std::string mesh_name, std::string frame_name) const;

		std::vector<std::string> getContactStatNames() {
			return _stat_names;
		}

		std::vector<std::string> getContactStatNamesVec3() {
			return _stat_names_vec3;
		}

		std::vector<std::string> getMeshDataNames() {
			return _mesh_data_names;
		}

	protected:

		void extendFinalizeFromProperties() override;
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
		double computeTriProximity(SimTK::Vec3 cnt_pnt_ground,
			SimTK::Vec3 tri_cen_ground) const;

		double computeTriPressure(int castingTri, int targetTri,
			double depth, bool flipMeshes) const;

		static void calcNonlinearPressureResid(
			int nEqn, int nVar, double q[], double resid[],
			int *flag2, void *ptr);

		bool verifyTriContact(
			SimTK::Vec3 tri_nor1_ground, SimTK::Vec3 tri_nor2_ground,
			double depth) const;

		bool rayIntersectTriTest(SimTK::Vec3 p, SimTK::Vec3 d,
			SimTK::Vec3 v0, SimTK::Vec3 v1, SimTK::Vec3 v2,
			SimTK::Vec3& intersection_pt, double& distance) const;

		void meshCollision(const SimTK::State& state,
			SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
			bool applyContactForces, bool flipMeshes) const;

		SimTK::Transform getTransformCastingToTargetMesh(
			const SimTK::State& state) const;

		SimTK::Vec3 computeForceVector(double pressure, double area, SimTK::Vec3 normal) const;

		SimTK::Vec3 computeMomentVector(double pressure, double area, SimTK::Vec3 normal, SimTK::Vec3 center) const;

		void computeVertexValues(const SimTK::State& state,
			const WISCO_ContactMesh& mesh, const SimTK::Vector& tri_pressure,
			const SimTK::Vector& tri_proximity, const bool flipMeshes) const;

		void computeContactStats(const SimTK::State& state, const WISCO_ContactMesh& mesh, const SimTK::Vector& tri_pressure,
			const SimTK::Vector& tri_proximity, int nActiveTri, bool flipMeshes) const;

		SimTK::Vector interpolateVertexFromFaceValues(
			SimTK::Vector face_values, SimTK::PolygonalMesh mesh) const;
		void setNull();
		void constructProperties();

		struct nonlinearContactParams {
			double h1, h2, k1, k2, dc;
		};

		std::vector<std::string> _region_names;
		std::vector<std::string> _stat_names;
		std::vector<std::string> _stat_names_vec3;
		std::vector<std::string> _mesh_data_names;


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
			OpenSim_DECLARE_PROPERTY(use_variable_thickness, bool,
				"Flag to use variable thickness."
				"Note: mesh_back_file must defined in WISCO_ContactMesh")
			OpenSim_DECLARE_PROPERTY(use_variable_elastic_modulus, bool,
				"Flag to use variable youngs modulus."
				"Note: material_properties_file must defined in WISCO_ContactMesh")
			OpenSim_DECLARE_PROPERTY(use_variable_poissons_ratio, bool,
				"Flag to use variable poissons ratio."
				"Note: material_properties_file must defined in WISCO_ContactMesh")
			OpenSim_DECLARE_PROPERTY(elastic_modulus, double, "Elastic Modulus")
			OpenSim_DECLARE_PROPERTY(poissons_ratio, double, "Poissons Ratio")
			OpenSim_DECLARE_PROPERTY(thickness, double,
					"Uniform thickness of elastic layer")
            //==============================================================================
            // PUBLIC METHODS
            //==============================================================================
            ContactParameters();
            ContactParameters(double youngs_modulus, double poissons_ratio, double thickness);


     private:
        void constructProperties();
    };


} // end of namespace OpenSim

#endif // OPENSIM_UW_ELASTIC_FOUNDATION_FORCE_H_
