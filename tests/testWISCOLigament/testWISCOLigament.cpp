#include <OpenSim/OpenSim.h>
#include "testWISCOLigament.h"
#include "WISCO_Ligament.h"


//using namespace SimTK;
using namespace OpenSim;
using SimTK::Vec3;

int main()
{
    try {
		
		// Timer
        std::clock_t start;
        double duration;
		start = std::clock();

		//Load WISCO_Plugin
		static const std::string plugin_file{ "../../../install/plugin/WISCO_Plugin" };
		LoadOpenSimLibrary(plugin_file,true);

		//Reference Values 
		double lin_stiff = 100;
		double ref_strain = 0.0;
		double trans_strain = 0.06;
		double damp_coeff = 0.0;

		//Create Model
		Model model;
		model.setUseVisualizer(true);
		Ground& ground = model.updGround();

		Body* block = new Body("block", 0.1, Vec3(0.0), SimTK::Inertia(0.001));
		Mesh* mesh = new Mesh("./inputs/geometry/block.stl");
		block->attachGeometry(mesh);
		model.addComponent(block);

		SliderJoint* gb_joint = new SliderJoint("gb_joint", ground, *block);
		double def_val = 0.05;
		gb_joint->upd_coordinates(0).setDefaultValue(def_val);
		model.addComponent(gb_joint);
		
		WISCO_Ligament* ligament = new WISCO_Ligament(ground,
			SimTK::Vec3(0),*block,SimTK::Vec3(0),lin_stiff,ref_strain);
		ligament->setName("lig");
		ligament->set_transition_strain(trans_strain);
		ligament->set_normalized_damping_coefficient(damp_coeff);
		ligament->set_appliesForce(true);
		model.addComponent(ligament);
		
		SimTK::State& state = model.initSystem();

		// Add display geometry.
		model.updMatterSubsystem().setShowDefaultGeometry(false);
		SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
		viz.setBackgroundColor(SimTK::White);

		//=====================================================================
		//Test for ligament spring and damping forces at multiple strains/rates
		//=====================================================================
		std::cout << "===================" << std::endl;
		std::cout << "WISCO_Ligament test" << std::endl;
		std::cout << "===================\n\n" << std::endl;
		std::cout << "Spring & Damping Force Test" << std::endl;
		std::cout << "---------------------------" << std::endl;
		std::cout << "[IDEAL   TEST]" << std::endl;
		std::cout << "------   -----" << std::endl;

		//Pose Model in Default state
		
		WISCO_Ligament& lig = model.updComponent<WISCO_Ligament>("lig");
		lig.equilibriateSlackLengthProperties(state);
		model.realizeReport(state);
		double t_ref_force = lig.getTension(state);
		double ref_force = lig.get_reference_force();
		std::cout << "reference_force: " << ref_force << " "<< t_ref_force << std::endl;

		//Stretch the Ligament
		SliderJoint& jnt = model.updComponent<SliderJoint>("gb_joint");
		Coordinate& coord = jnt.updCoordinate(SliderJoint::Coord::TranslationX);
		
		// 0.03
		double strain = 0.03;
		coord.setValue(state, def_val*(strain+1.0));
		model.realizeReport(state);
		double test_force = lig.getTension(state);
		double ideal_force = 0.5*lin_stiff / trans_strain*pow(strain, 2);
		std::cout << "force: " << ideal_force << " " << test_force << std::endl;

		double strain_rate = 0.1;
		coord.setSpeedValue(state, def_val*strain_rate);
		model.realizeReport(state);
		double t_speed_force = lig.getTension(state);
		double speed_force = 0.5*lin_stiff / trans_strain*pow(strain, 2) + lin_stiff*damp_coeff*strain_rate;
		double t_damp_force = t_speed_force - test_force;
		double damp_force =  speed_force - ideal_force;
		std::cout << "damping force: " << damp_force << " " << t_damp_force << std::endl;
		std::cout << "total force: " << speed_force << " " << t_speed_force << std::endl;


		// 0.06
		strain = 0.06;
		coord.setValue(state, def_val*(strain + 1.0));
		model.realizeReport(state);
		test_force = lig.getTension(state);
		ideal_force = 0.5*lin_stiff / trans_strain*pow(strain, 2);
		std::cout << "force: " << ideal_force << " " << test_force << std::endl;

		// 0.09
		strain = 0.09;
		coord.setValue(state, def_val*(strain + 1.0));
		model.realizeReport(state);
		test_force = lig.getTension(state);
		ideal_force = lin_stiff* (strain-trans_strain/2);
		std::cout << "force: " << ideal_force << " " << test_force << std::endl;

		

		
		// Report Timer Results
		//duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
		//std::cout << "printf: " << duration << '\n';
		
		//=====================================================================
		// Simulate Hanging Block
		//=====================================================================
		model.set_gravity(SimTK::Vec3(-1.00, 0,0));
		state = model.initSystem();

		//Set initial conditions
		coord.setValue(state, def_val);
		coord.setSpeedValue(state, 0.0);
		
		
		model.updComponent<SliderJoint>("gb_joint").upd_coordinates(0).setLocked(state, false);
		
		model.realizeReport(state);

		double pot_nrg1 = model.calcPotentialEnergy(state);
		double kin_nrg1 = model.calcKineticEnergy(state);
		double tot_nrg1 = pot_nrg1 + kin_nrg1;

		SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
		integrator.setMaximumStepSize(1.0e-2);
		integrator.setAccuracy(0.00000001);
		Manager manager(model, integrator);
		manager.setInitialTime(0); manager.setFinalTime(1.0);
		manager.integrate(state);

		model.realizeReport(state);

		double pot_nrg2 = model.calcPotentialEnergy(state);
		double kin_nrg2 = model.calcKineticEnergy(state);
		double tot_nrg2 = pot_nrg2 + kin_nrg2;

		std::cout << "\nSpring & Damping Force Test" << std::endl;
		std::cout << "---------------------------" << std::endl;
		std::cout << "[PE] [KE] [Total]" << std::endl;
		std::cout << "Start Energy: " << pot_nrg1 << " " << kin_nrg1 << " " << tot_nrg1 << std::endl;
		std::cout << "End Energy: " << pot_nrg2 << " " << kin_nrg2 << " " << tot_nrg2 << std::endl;

		//Write Outputs
		//-------------
		//static const std::string out_mot_file{ "./results/ligamentTest.mot" };
		//manager.getStateStorage().print(out_mot_file);
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

