#ifndef OPENSIM_WISCO_CONTACT_MESH_H_
#define OPENSIM_WISCO_CONTACT_MESH_H_ 

// INCLUDE
#include "OpenSim/Common/Object.h"
#include "OpenSim/Simulation/SimbodyEngine/Body.h"
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "SimTKsimbody.h"
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include "osimPluginDLL.h"
#include "coldet.h"

namespace OpenSim {

	class ScaleSet;

	/** This class represents the physical shape of an object for use in contact
	* modeling.  It is an abstract class, with subclasses for particular geometric
	* representations. The geometry is attached to a PhysicalFrame, which is
	* specified using a Connector named "frame".
	*
	* @author Colin Smith
	*/
	class OSIMPLUGIN_API WISCO_ContactMesh : public ModelComponent {
    //class WISCO_ContactMesh : public ModelComponent {
    
		OpenSim_DECLARE_CONCRETE_OBJECT(WISCO_ContactMesh, ModelComponent)

	public:

		//=============================================================================
		// PROPERTIES
		//=============================================================================
		OpenSim_DECLARE_PROPERTY(file_name, std::string,
			"Path to mesh geometry file (supports .obj, .stl, .vtp). ");
		OpenSim_DECLARE_PROPERTY(mesh_frame, PhysicalOffsetFrame,
			"Local mesh frame to locate mesh in parent body.");
		OpenSim_DECLARE_PROPERTY(display_preference, int,
			"0:Hide 1:Wire 3:Flat 4:Shaded");
		OpenSim_DECLARE_PROPERTY(medial_lateral_axis, std::string,
			"Axis (mesh frame) that defines M-L. Options: 'x','y','z'. ")
		OpenSim_DECLARE_OPTIONAL_PROPERTY(mesh_back_file, std::string,
			"Path to backside (bone) mesh geometry file (supports .obj, .stl, .vtp). ");
		OpenSim_DECLARE_OPTIONAL_PROPERTY(material_properties_file, std::string,
			"Path to backside (bone) mesh geometry file (supports .obj, .stl, .vtp). ");
		OpenSim_DECLARE_OPTIONAL_PROPERTY(min_thickness, double,
			"Minimum thickness of cartilage [m] for variable cartilage thickness");
		OpenSim_DECLARE_OPTIONAL_PROPERTY(max_thickness, double,
			"Minimum thickness of cartilage [m] for variable cartilage thickness");
		OpenSim_DECLARE_LIST_PROPERTY_SIZE(color, double, 3,
			"Display Color to apply to the contact geometry.");
		
		OpenSim_DECLARE_SOCKET(parent_frame, PhysicalFrame, 
			"The frame to which this geometry is attached.");
		
		//=============================================================================
		// METHODS
		//=============================================================================
	public:
		// CONSTRUCTION
		/** Construct an empty ContactGeometry. */
		WISCO_ContactMesh();

		/** This constructor connects this ContactGeometry to the provided `frame`,
		* and uses the default location and orientation (both `Vec3(0)`).
		*
		* @param frame        the PhysicalFrame this geometry is attached to;
		*/
		WISCO_ContactMesh(const std::string& name, const std::string& mesh_file, const PhysicalFrame& frame);

		WISCO_ContactMesh(const std::string& name, const std::string& mesh_file, const PhysicalFrame& frame, 
			const SimTK::Vec3& location, const SimTK::Vec3& orientation);

		WISCO_ContactMesh(const std::string& name, const std::string& mesh_file, const PhysicalFrame& frame, 
			const SimTK::Vec3& location, const SimTK::Vec3& orientation, 
			const std::string& mesh_back_file, double  min_thickness, double max_thickness);



		const int getDisplayPreference();
		void setDisplayPreference(const int dispPref);

		SimTK::Transform getTransformGroundToMesh(const SimTK::State& state) const;
		
		const SimTK::PolygonalMesh& getPolygonalMesh() const 
		{
			return mesh;
		}

		int getNumFaces() const
		{
			return mesh.getNumFaces();
		}

		int getNumVertices() const
		{
			return mesh.getNumVertices();
		}

		const SimTK::Vector_<SimTK::Vec3>& getTriangleCenters() const;
		const SimTK::Vector_<SimTK::Vec3>& getTriangleNormals() const;
		SimTK::Vector_<SimTK::Vec3> getTriangleCentersInBody() const;

		SimTK::Vector_<SimTK::Vec3> getTriangleCentersInGround(const SimTK::State& state) const;
		SimTK::Vector_<SimTK::Vec3> getTriangleNormalsInGround(const SimTK::State& state) const;
		
		const SimTK::Vector& getTriangleAreas() const;

		const SimTK::Vector_<SimTK::Vec3>& getVertexLocations() const;
		const SimTK::Vector_<SimTK::Vec3> getVertexLocationsInGround(const SimTK::State& state) const;
		const SimTK::Vector_<SimTK::Vec3> getVertexLocationsInFrame(
			const SimTK::State& state, const std::string& frame_name) const;

		const SimTK::Matrix_<SimTK::Vec3>& WISCO_ContactMesh::getFaceVertexLocations() const;
		SimTK::Matrix_<SimTK::Vec3> WISCO_ContactMesh::getFaceVertexLocationsInGround(const SimTK::State& state) const;

		SimTK::Vector WISCO_ContactMesh::getNeighborTris(int tri, int& nNeighborTri) const;

		const SimTK::Vector& getMedialTriangleIndices() const {
			return med_tri_ind;
		}

		const SimTK::Vector& getLateralTriangleIndices() const {
			return lat_tri_ind;
		}

		int getNumMedialTriangles() const {
			return _n_medial_tri;
		}

		int getNumLateralTriangles() const {
			return _n_lateral_tri;
		}

		const SimTK::Vector& getTriangleThickness() const {
			return tri_thickness;
		}

		const SimTK::Vector& getTriangleElasticModulus() const {
			return tri_elastic_modulus;
		}

		const SimTK::Vector& getTrianglePoissonsRatio() const {
			return tri_poissons_ratio;
		}

		virtual void scale(const ScaleSet& aScaleSet);

	protected:

	private:
		// INITIALIZATION
		void setNull();
		void constructProperties();
		void extendFinalizeFromProperties() override;
		//void extendSetPropertiesFromState(const SimTK::State& state) override;
        void extendAddToSystem(SimTK::MultibodySystem &system) const override;
        void extendInitStateFromProperties(SimTK::State &state) const override;
		void extendRealizeReport(const SimTK::State &state) const override;
		void initializeMesh();
		std::string findMeshFile(const std::string& file);
		CollisionModel3D* initColdetModel(const SimTK::PolygonalMesh& cnt_mesh) const;
		void computeVariableCartilageThickness();
		
		// Member Variables
		SimTK::PolygonalMesh mesh;
		SimTK::PolygonalMesh mesh_back;
		SimTK::Vector_<SimTK::Vec3> tri_center;
		SimTK::Vector_<SimTK::Vec3> tri_normal;
		SimTK::Vector tri_area;
		SimTK::Vector med_tri_ind;
		SimTK::Vector lat_tri_ind;
		int _n_medial_tri;
		int _n_lateral_tri;
		SimTK::Matrix tri_neighbors;
		SimTK::Vector nTriNeighbors;
		SimTK::Vector_<SimTK::Vec3> vertex_locations;
		SimTK::Matrix_<SimTK::Vec3> face_vertex_locations;
        SimTK::CacheEntryIndex m_coldet_model_index;
		SimTK::Vector tri_thickness;
		SimTK::Vector tri_elastic_modulus;
		SimTK::Vector tri_poissons_ratio;
		bool mesh_is_cached;

		//=============================================================================
	};  // END of class ContactGeometry
		//=============================================================================
		//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_WISCO_CONTACT_MESH_H_ 
