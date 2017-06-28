#include <OpenSim/OpenSim.h>
#include <vector>
//#include "testContactBallFloor.h"
#include "WISCO_VTKFileAdapter.h"

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

		//Load Model
		static const std::string model_file{ "../inputs/fbknee/fbknee_knee_only.osim" };
		Model model(model_file);

		//=====================================================================
		// Configure the model
		//=====================================================================
		model.setUseVisualizer(true);

		Joint& knee = model.updJointSet().get("knee_r");

		//Prescribe knee flexion
		/*double times[7] = { 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0 };
		double flex_ang[7] = { 0, 0, 0, SimTK::Pi/4, SimTK::Pi/2,SimTK::Pi/4, 0 };

		SimmSpline flexion_func = SimmSpline(7, times, flex_ang, "flexion_func");

		knee.upd_coordinates(0).setDefaultIsPrescribed(true);
		knee.upd_coordinates(0).setPrescribedFunction(flexion_func);
		*/

		knee.upd_coordinates(0).setDefaultLocked(true);

		SimTK::State& state = model.initSystem();

		// Add display geometry.
		model.updMatterSubsystem().setShowDefaultGeometry(false);

		SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
		viz.setBackgroundColor(SimTK::White);

		//=====================================================================
		// Simulate
		//=====================================================================

		SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
		//integrator.setMaximumStepSize(1.0e-2);
		integrator.setAccuracy(0.1);

		/*
		SimTK::CPodesIntegrator integrator(model.getSystem(), SimTK::CPodes::BDF, SimTK::CPodes::Newton);
		integrator.setAccuracy(0.01);
		*/
		Manager manager(model, integrator);
		state.setTime(0);
		manager.integrate(state, 0.5);

		// Report Timer Results
		duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
		std::cout << "printf: " << duration << '\n';

		//=====================================================================
		//Write Outputs
		//=====================================================================

		//Motion file
		static const std::string out_mot_file{ "./fbknee_passive_flex.mot" };
		manager.getStateStorage().print(out_mot_file);



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

