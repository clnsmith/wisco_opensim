#include <OpenSim/OpenSim.h>
#include <vector>
#include "testContactBallFloor.h"
#include "WISCO_VTKFileAdapter.h"

//using namespace SimTK;
using namespace OpenSim;
using SimTK::Vec3;

int main()
{
    try {
        //Ball Radius = 0.05
		
		// Timer
        std::clock_t start;
        double duration;
		start = std::clock();

		//Load WISCO_Plugin
		static const std::string plugin_file{ "../../../install/plugin/WISCO_Plugin" };
		LoadOpenSimLibrary(plugin_file,true);

		//Load Model
		static const std::string model_file{ "../shared/ball_and_floor_Custom.osim" };
		Model model(model_file,true);
		
		//=====================================================================
		// Configure the model
		//=====================================================================
		model.setUseVisualizer(true);

		Joint& ground_to_ball = model.updJointSet().get("ground_to_ball");
			
		//Set initial height
		ground_to_ball.upd_coordinates(5).setDefaultValue(0.05);

		//Lock everything but vertical translation
		ground_to_ball.upd_coordinates(0).setDefaultLocked(true);
		ground_to_ball.upd_coordinates(1).setDefaultLocked(true);
		ground_to_ball.upd_coordinates(2).setDefaultLocked(true);
		ground_to_ball.upd_coordinates(3).setDefaultLocked(true);
		ground_to_ball.upd_coordinates(4).setDefaultLocked(true);

		//Prescribe vertical translation
		LinearFunction height_function = LinearFunction(-0.05, 0.05);

		ground_to_ball.upd_coordinates(5).setDefaultIsPrescribed(true);
		ground_to_ball.upd_coordinates(5).setPrescribedFunction(height_function);
		
		//Add Reporters
		double REPORT_INTERVAL = 0;

		Force& contactForce = model.updForceSet().get("ball_contact");
		
		TableReporterVector* ball_prs_reporter = new TableReporterVector();
		ball_prs_reporter->setName("ball_prs_reporter");
		ball_prs_reporter->set_report_time_interval(REPORT_INTERVAL);
		ball_prs_reporter->addToReport(contactForce.getOutput("casting_mesh_tri_pressure"));
		model.addComponent(ball_prs_reporter);

		TableReporterVector* ball_prx_reporter = new TableReporterVector();
		ball_prx_reporter->setName("ball_prx_reporter");
		ball_prx_reporter->set_report_time_interval(REPORT_INTERVAL);
		ball_prx_reporter->addToReport(contactForce.getOutput("target_mesh_tri_proximity"));
		model.addComponent(ball_prx_reporter);

		SimTK::State& state = model.initSystem();

		// Add display geometry.
		model.updMatterSubsystem().setShowDefaultGeometry(false);

		SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
		viz.setBackgroundColor(SimTK::White);

		//=====================================================================
		// Simulate
		//=====================================================================
		
		SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
		integrator.setMaximumStepSize(1.0e-2);
		Manager manager(model, integrator);
		manager.setInitialTime(0); manager.setFinalTime(1.0);
		manager.integrate(state);
		
		// Report Timer Results
		duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
		std::cout << "printf: " << duration << '\n';
		
		//=====================================================================       
		//Write Outputs
		//=====================================================================

		//Motion file
		static const std::string out_mot_file{ "./ContactBallFloor.mot" };
		manager.getStateStorage().print(out_mot_file);
		

		//Contact files
		SimTK::Matrix matty = ball_prs_reporter->getTable().getMatrix().getAsMatrix();
		std::cout << matty.ncol() << " " << matty.nrow() << std::endl;

		std::vector<SimTK::Matrix> OutputMatrix;
		OutputMatrix.push_back(ball_prs_reporter->getTable().getMatrix().getAsMatrix());
		OutputMatrix.push_back(ball_prx_reporter->getTable().getMatrix().getAsMatrix());

		std::vector<std::string> OutputNames;
		OutputNames.push_back("pressure");
		OutputNames.push_back("proximity");

		SimTK::PolygonalMesh ball_mesh = SimTK::PolygonalMesh();
		static const std::string ball_file{ "../shared/geometry/ball.stl" };
		ball_mesh.loadFile(ball_file);
		/*
		WISCO_VTKFileAdapter* ball_prs_vtk = new WISCO_VTKFileAdapter();
		ball_prs_vtk->write("ball_contact", "./contact_results/ball_contact/", 
			ball_mesh, OutputMatrix, OutputNames);*/
		
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

