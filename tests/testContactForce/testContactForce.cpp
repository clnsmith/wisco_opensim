#include <OpenSim/OpenSim.h>
#include <WISCO_ElasticFoundationForce.h>

//using namespace SimTK;
using namespace OpenSim;
using SimTK::Vec3;

int main()
{
    try {
		//Load WISCO_Plugin
		static const std::string plugin_file{ "../../../install/plugin/WISCO_Plugin" };
		LoadOpenSimLibrary(plugin_file, true);

		//Load Model
		static const std::string model_file{ "../shared/ball_and_floor/ball_and_floor.osim" };
		Model model(model_file);

		model.setUseVisualizer(true);
		model.set_gravity(Vec3(0));

		Joint& ground_to_ball = model.updJointSet().get("ground_to_ball");

		//Set initial height
		ground_to_ball.upd_coordinates(5).setDefaultValue(0.05);

		//Lock everything but vertical translation
		ground_to_ball.upd_coordinates(0).setDefaultLocked(true);
		ground_to_ball.upd_coordinates(1).setDefaultLocked(true);
		ground_to_ball.upd_coordinates(2).setDefaultLocked(true);
		ground_to_ball.upd_coordinates(3).setDefaultLocked(true);
		ground_to_ball.upd_coordinates(4).setDefaultLocked(true);
		ground_to_ball.upd_coordinates(5).setDefaultLocked(false);
		//Prescribe a vertical force 
		model.initSystem();
		Body ball = model.getComponent<Body>("ball");

		PrescribedForce* ext_force = new PrescribedForce("prescribed_force", ball);
		
		Constant Fx_func = Constant(0.0);
		Constant Fy_func = Constant(0.0);
		Constant Fz_func = Constant(-1.0);

		ext_force->setForceFunctions(&Fx_func, &Fy_func, &Fz_func);
		model.addComponent(ext_force);

		//FreeJoint ground_to_ball = model.getComponent<FreeJoint>("ground_to_ball");

		TableReporterVector* tab_rep = new TableReporterVector();
		tab_rep->setName("reporter");
		tab_rep->set_report_time_interval(0);
		tab_rep->addToReport(model.getComponent<WISCO_ElasticFoundationForce>("ball_contact").getOutput("casting_mesh_tri_pressure"));
		model.addComponent(tab_rep);

		// Configure the model.
	    SimTK::State& state = model.initSystem();
		model.realizeReport(state);
		// Add display geometry.

		model.updMatterSubsystem().setShowDefaultGeometry(false);

		SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
		viz.setBackgroundColor(SimTK::White);

		// Simulate.

		SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
		integrator.setMaximumStepSize(1.0e-2);
		Manager manager(model, integrator);
		manager.initialize(state);
		manager.integrate(1.0);


		//Write Output Files
		model.print("./results/ball_and_floor.osim");
		manager.getStateStorage().print("./results/contactForce.mot");
		
	 		
		//Perform Analysis
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

