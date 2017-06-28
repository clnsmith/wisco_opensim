#include <OpenSim/OpenSim.h>
#include "WISCO_ElasticFoundationForce.h"
#include "WISCO_ContactMesh.h"
#include "WISCO_ContactAnalysis.h"

//using namespace SimTK;
using namespace OpenSim;
using SimTK::Vec3;

int main()
{
	try {
		//Load WISCO_Plugin
		static const std::string plugin_file{ "../../../install/plugin/WISCO_Plugin" };
		LoadOpenSimLibrary(plugin_file, true);

		if (true) {
			//Build Model
			Model model;
			model.setName("knee_contact");
			//model.setGravity(SimTK::Vec3(0, 0, -9.80665));
			model.setUseVisualizer(true);
			/*
			static const std::string femur_cart_file{ "../shared/fbknee/geometry/ACLC01-R-femur-cart.stl" };
			static const std::string tibia_cart_file{ "../shared/fbknee/geometry/ACLC01-R-tibia-cart.stl" };
			static const std::string patella_cart_file{ "../shared/fbknee/geometry/ACLC01-R-patella-cart_shift.stl" };
			*/
			static const std::string femur_cart_file{ "C:/github/wisco_opensim/source/tests/shared/fbknee/Geometry/ACLC01-R-femur-cart-coarse.stl" };
			static const std::string tibia_cart_file{ "C:/github/wisco_opensim/source/tests/shared/fbknee/Geometry/ACLC01-R-tibia-cart-coarse.stl" };
			static const std::string patella_cart_file{ "C:/github/wisco_opensim/source/tests/shared/fbknee/Geometry/ACLC01-R-patella-cart_shift-coarse.stl" };

			Body* femur = new Body("femur_distal_r", 1.0, Vec3(0), SimTK::Inertia(0.01));
			Body* tibia = new Body("tibia_proximal_r", 1.0, Vec3(0), SimTK::Inertia(0.01));
			Body* patella = new Body("patella_r", 1.0, Vec3(0), SimTK::Inertia(0.01));

			model.addComponent(femur);
			model.addComponent(tibia);
			model.addComponent(patella);

			WISCO_ContactMesh* femurMesh = new WISCO_ContactMesh("femurMesh", femur_cart_file, *femur, Vec3(0), Vec3(0));
			WISCO_ContactMesh* tibiaMesh = new WISCO_ContactMesh("tibiaMesh", tibia_cart_file, *tibia, Vec3(0), Vec3(0));
			WISCO_ContactMesh* patellaMesh = new WISCO_ContactMesh("patellaMesh", patella_cart_file, *patella, Vec3(0), Vec3(0));

			model.addComponent(femurMesh);
			model.addComponent(tibiaMesh);
			model.addComponent(patellaMesh);

			// Add WISCO_ElasticFoundationForce
			double E = 5000000.0;
			double v = 0.45;
			double h = 0.01;

			WISCO_ElasticFoundationForce::ContactParameters* femurContactParams =
				new WISCO_ElasticFoundationForce::ContactParameters(E, v, h);

			WISCO_ElasticFoundationForce::ContactParameters* tibiaContactParams =
				new WISCO_ElasticFoundationForce::ContactParameters(E, v, h);

			WISCO_ElasticFoundationForce::ContactParameters* patellaContactParams =
				new WISCO_ElasticFoundationForce::ContactParameters(E, v, h);

			WISCO_ElasticFoundationForce* TFcontact =
				new WISCO_ElasticFoundationForce(*femurMesh, *femurContactParams, *tibiaMesh, *tibiaContactParams, 2);
			TFcontact->setName("TF_contact");
			TFcontact->set_min_proximity(0.0);
			TFcontact->set_max_proximity(0.02);
			TFcontact->set_elastic_foundation_formulation("linear");

			WISCO_ElasticFoundationForce* PFcontact =
				new WISCO_ElasticFoundationForce(*femurMesh, *femurContactParams, *patellaMesh, *patellaContactParams, 2);
			PFcontact->setName("PF_contact");
			PFcontact->set_min_proximity(0.0);
			PFcontact->set_max_proximity(0.02);
			PFcontact->set_elastic_foundation_formulation("linear");

			model.addForce(TFcontact);
			model.addForce(PFcontact);

			//Add Prescribed Joint
			
			/*SpatialTransform spat_trans = SpatialTransform();
			spat_trans[0].setCoordinateNames(OpenSim::Array<std::string>("trans1", 1, 1));
			spat_trans[0].setFunction(LinearFunction());
			spat_trans[1].setCoordinateNames(OpenSim::Array<std::string>("trans2", 1, 1));
			spat_trans[1].setFunction(LinearFunction());
			spat_trans[2].setCoordinateNames(OpenSim::Array<std::string>("trans3", 1, 1));
			spat_trans[2].setFunction(LinearFunction());
			spat_trans[3].setCoordinateNames(OpenSim::Array<std::string>("rot1", 1, 1));
			spat_trans[3].setFunction(LinearFunction());
			spat_trans[4].setCoordinateNames(OpenSim::Array<std::string>("rot2", 1, 1));
			spat_trans[4].setFunction(LinearFunction());
			spat_trans[5].setCoordinateNames(OpenSim::Array<std::string>("rot3", 1, 1));
			spat_trans[5].setFunction(LinearFunction());

			CustomJoint* tf_joint = new CustomJoint("tf_joint", *femur, *tibia, spat_trans);
			CustomJoint* pf_joint = new CustomJoint("pf_joint", *femur, *patella, spat_trans);*/			
			
			FreeJoint* tf_joint = new FreeJoint("tf_joint", *femur, Vec3(0), Vec3(0), *tibia, Vec3(0), Vec3(0));
			FreeJoint* pf_joint = new FreeJoint("pf_joint", *femur, Vec3(0), Vec3(0), *patella, Vec3(0), Vec3(0));			

			WeldJoint* ground_knee = new WeldJoint("ground_to_knee", model.getGround(), *femur);
			model.addComponent(ground_knee);
			
			LinearFunction tf_func = LinearFunction(0.02, 0.0);
			LinearFunction pf_func = LinearFunction(-0.02, 0.053);

			tf_joint->upd_coordinates(4).setDefaultIsPrescribed(true);
			tf_joint->upd_coordinates(4).setPrescribedFunction(tf_func);

			tf_joint->upd_coordinates(0).setDefaultLocked(true);
			tf_joint->upd_coordinates(1).setDefaultLocked(true);
			tf_joint->upd_coordinates(2).setDefaultLocked(true);
			tf_joint->upd_coordinates(3).setDefaultLocked(true);
			tf_joint->upd_coordinates(5).setDefaultLocked(true);

			pf_joint->upd_coordinates(3).setDefaultIsPrescribed(true);
			pf_joint->upd_coordinates(3).setPrescribedFunction(pf_func);

			pf_joint->upd_coordinates(0).setDefaultLocked(true);
			pf_joint->upd_coordinates(1).setDefaultLocked(true);
			pf_joint->upd_coordinates(2).setDefaultLocked(true);
			pf_joint->upd_coordinates(4).setDefaultLocked(true);
			pf_joint->upd_coordinates(4).setDefaultValue(0.005);
			pf_joint->upd_coordinates(5).setDefaultLocked(true);
			pf_joint->upd_coordinates(5).setDefaultValue(0.004);

			model.addJoint(tf_joint);
			model.addJoint(pf_joint);

			model.print("./results/knee_contact.osim");

			SimTK::State& state = model.initSystem();

			WISCO_ElasticFoundationForce& tf = model.updComponent<WISCO_ElasticFoundationForce>("TF_contact");

			//Simulate
			state = model.initSystem();

			//model.realizeReport(state);
			
			model.updMatterSubsystem().setShowDefaultGeometry(false);
			SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
			viz.setBackgroundColor(SimTK::White);

			SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
			integrator.setMaximumStepSize(1.0e-2);
			Manager manager(model, integrator);
			manager.setInitialTime(0); manager.setFinalTime(0.05);
			manager.integrate(state);

			static const std::string out_mot_file{ "./results/kneeContact.mot" };
			manager.getStateStorage().print(out_mot_file);
		}
		
		
		
	
		//Perform ContactAnalysis
		static const std::string settings_file{ "./inputs/ContactAnalysis_settings.xml" };

		//Run Analysis
		AnalyzeTool analyzeTool = AnalyzeTool(settings_file);
		analyzeTool.run();

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

