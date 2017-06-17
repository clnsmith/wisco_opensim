#include <OpenSim/OpenSim.h>
#include "WISCO_ElasticFoundationForce.h"
#include "WISCO_VTKFileAdapter.h"
#include "WISCO_Ligament.h"
#include <vector>
#include "buildFBKnee.h"


//using namespace SimTK;
using namespace OpenSim;
using SimTK::Vec3;

int main()
{
    try {

        // Set Input Files 
		static const std::string femur_cart_file{ "./inputs/geometry/ACLC01-R-femur-cart.stl" };
        static const std::string tibia_cart_file{ "./inputs/geometry/ACLC01-R-tibia-cart.stl" };
		static const std::string patella_cart_file{ "./inputs/geometry/ACLC01-R-patella-cart_shift.stl" };
		
		static const std::string model_file{ "./inputs/JKRLfbkid_ligs.osim" };
		static const std::string out_file{"./fbknee.osim"};


		//Load WISCO Plugin
		static const std::string plugin_file{ "../../../install/plugin/WISCO_Plugin" };
		LoadOpenSimLibrary(plugin_file, true);

		//Load Model 
		Model model = Model(model_file);
		model.initSystem();

		// Add Contact Meshes
		Body& femur = model.updComponent<Body>("femur_distal_r");
		Body& tibia = model.updComponent<Body>("tibia_proximal_r");
		Body& patella = model.updComponent<Body>("patella_r");

		WISCO_ContactMesh* femurMesh = new WISCO_ContactMesh("femurMesh", femur_cart_file, femur, Vec3(0), Vec3(0));
		WISCO_ContactMesh* tibiaMesh = new WISCO_ContactMesh("tibiaMesh", tibia_cart_file, tibia, Vec3(0), Vec3(0));
		WISCO_ContactMesh* patellaMesh = new WISCO_ContactMesh("patellaMesh", patella_cart_file, patella, Vec3(0), Vec3(0));

		model.addComponent(femurMesh);
		model.addComponent(tibiaMesh);
		model.addComponent(patellaMesh);

		// Add WISCO_ElasticFoundationForce
		double E = 5000000.0;
		double v = 0.45;
		double h = 0.003;

        WISCO_ElasticFoundationForce::ContactParameters* femurContactParams = 
			new WISCO_ElasticFoundationForce::ContactParameters(E, v, h);
		
		WISCO_ElasticFoundationForce::ContactParameters* tibiaContactParams = 
			new WISCO_ElasticFoundationForce::ContactParameters(E, v, h);
		
		WISCO_ElasticFoundationForce::ContactParameters* patellaContactParams =
			new WISCO_ElasticFoundationForce::ContactParameters(E, v, h);


		WISCO_ElasticFoundationForce* TFcontact = 
			new WISCO_ElasticFoundationForce(*femurMesh,*femurContactParams,*tibiaMesh, *tibiaContactParams,2);
		TFcontact->setName("TF_contact");
		TFcontact->set_min_proximity(0.0);
		TFcontact->set_max_proximity(0.01);
		TFcontact->set_elastic_foundation_formulation("linear");
		
		WISCO_ElasticFoundationForce* PFcontact =
			new WISCO_ElasticFoundationForce(*femurMesh, *femurContactParams, *patellaMesh, *patellaContactParams, 2);
		PFcontact->setName("PF_contact");
		PFcontact->set_min_proximity(0.0);
		PFcontact->set_max_proximity(0.01);
		PFcontact->set_elastic_foundation_formulation("linear");

		model.addForce(TFcontact);
		model.addForce(PFcontact);

		//Add busing force
		BushingForce* TF_bushing = new BushingForce("TF_bushing", "femur_distal_r","tibia_proximal_r");
		TF_bushing->set_rotational_damping(Vec3(2.0,2.0,2.0));
		TF_bushing->set_translational_damping(Vec3(10.0, 10.0, 10.0));

		BushingForce* PF_bushing = new BushingForce("PF_bushing", "femur_distal_r", "patella_r");
		PF_bushing->set_rotational_damping(Vec3(2.0, 2.0, 2.0));
		PF_bushing->set_translational_damping(Vec3(10.0, 10.0, 10.0));

		model.addForce(TF_bushing);
		model.addForce(PF_bushing);

		//Fix ligament parameters
		for (WISCO_Ligament& ligament : model.updComponentList<WISCO_Ligament>()) {
			ligament.set_transition_strain(0.06);
			ligament.set_defining_slack_length_property("reference_strain");
		}

		// Configure the model.
	    SimTK::State& state = model.initSystem();

	
		//Write Model
        model.print(out_file);
		// **********  END CODE  **********
	}
	catch (OpenSim::Exception ex)
	{
		std::cout << ex.getMessage() << std::endl;
		std::cin.get();
		return 1;
	}
	catch (SimTK::Exception::Base ex)
	{
		std::cout << ex.getMessage() << std::endl;
		std::cin.get();
		return 1;
	}
	catch (std::exception ex)
	{
		std::cout << ex.what() << std::endl;
		std::cin.get();
		return 1;
	}
	catch (...)
	{
		std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
		std::cin.get();
		return 1;
	}
	std::cout << "OpenSim example completed successfully" << std::endl;
	std::cout << "Press return to continue" << std::endl;
	std::cin.get();
	return 0;
}

