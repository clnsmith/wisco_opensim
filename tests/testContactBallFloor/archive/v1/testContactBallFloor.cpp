#include <OpenSim/OpenSim.h>
#include "WISCO_ElasticFoundationForce.h"
#include "WISCO_VTKFileAdapter.h"
#include <vector>
//#include "WISCO_PostViewDataFileAdapter.h"
#include "testContactBallFloor.h"
//#include "WISCO_Ligament.h"
//#include "WISCO_KeywordFileAdapter.h"
//#include "WISCO_PostViewKineFileAdapter.h"

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

        // Set Variables
        static const std::string ball_file{ "./geometry/ball.stl" };
        static const std::string floor_file{ "./geometry/floor_coarse.stl" };

		//Build Model 
		Model model;
		model.setName("Simple_Knee");
        model.setGravity(SimTK::Vec3(0, 0, -9.80665));
		model.setUseVisualizer(true);

		Ground& ground = model.updGround();
		ground.attachGeometry(new Mesh(floor_file));

		Body* ball = new OpenSim::Body("femur", 0.2, Vec3(0), SimTK::Inertia(0.01));
		ball->attachGeometry(new Mesh(ball_file));
		model.addBody(ball);
		
		

		FreeJoint* ground_to_ball = new FreeJoint("ground_to_ball", ground, Vec3(0), Vec3(0), *ball, Vec3(0), Vec3(0));

		// Set the angle and position ranges for the coordinate set
		double angleRange[2] = { -SimTK::Pi / 2, SimTK::Pi / 2 };
		double positionRange[2] = { -1, 1 };
		ground_to_ball->upd_coordinates(0).setRange(angleRange);
		ground_to_ball->upd_coordinates(1).setRange(angleRange);
		ground_to_ball->upd_coordinates(2).setRange(angleRange);
		ground_to_ball->upd_coordinates(3).setRange(positionRange);
		ground_to_ball->upd_coordinates(4).setRange(positionRange);
		ground_to_ball->upd_coordinates(5).setRange(positionRange);

		//Set Initial Position
		ground_to_ball->upd_coordinates(0).setDefaultValue(0);
		ground_to_ball->upd_coordinates(1).setDefaultValue(0);
		ground_to_ball->upd_coordinates(2).setDefaultValue(0);
		ground_to_ball->upd_coordinates(3).setDefaultValue(0);
		ground_to_ball->upd_coordinates(4).setDefaultValue(0);
		ground_to_ball->upd_coordinates(5).setDefaultValue(0.05);

		//Lock everything but vertical translation
		ground_to_ball->upd_coordinates(0).setDefaultLocked(true);
		ground_to_ball->upd_coordinates(1).setDefaultLocked(true);
		ground_to_ball->upd_coordinates(2).setDefaultLocked(true);
		ground_to_ball->upd_coordinates(3).setDefaultLocked(true);
		ground_to_ball->upd_coordinates(4).setDefaultLocked(true);

		//Prescribe vertical translation
		LinearFunction height_function = LinearFunction(-0.05, 0.05);

		ground_to_ball->upd_coordinates(5).setDefaultIsPrescribed(true);
		ground_to_ball->upd_coordinates(5).setPrescribedFunction(height_function);

		model.addJoint(ground_to_ball);
		
		
		// USE CUSTOM FORCE
		WISCO_ContactMesh* floorMesh = new WISCO_ContactMesh(floor_file, Vec3(0), Vec3(0), ground, "floorMesh");
		WISCO_ContactMesh* ballMesh = new WISCO_ContactMesh(ball_file, Vec3(0), Vec3(0), *ball, "ballMesh");
        
		model.addComponent(floorMesh);
		model.addComponent(ballMesh);

        WISCO_ElasticFoundationForce::ContactParameters* floorContactParams = new WISCO_ElasticFoundationForce::ContactParameters(30000.0, 0.45, 0.01,0.0,0.1);
		WISCO_ElasticFoundationForce::ContactParameters* ballContactParams = new WISCO_ElasticFoundationForce::ContactParameters(30000.0, 0.45, 0.01, 0.0, 0.1);
		
		WISCO_ElasticFoundationForce* contactForce = new WISCO_ElasticFoundationForce(*floorMesh,*floorContactParams,*ballMesh, *ballContactParams,2);
		contactForce->setName("ball_contact");

		model.addForce(contactForce);

		//Add Reporters
		auto ball_prs_reporter = new TableReporterVector();
		ball_prs_reporter->setName("ball_prs_reporter");
		ball_prs_reporter->set_report_time_interval(0.01);
		ball_prs_reporter->addToReport(contactForce->getOutput("mesh2_tri_pressure"));
		model.addComponent(ball_prs_reporter);

		auto ball_prx_reporter = new TableReporterVector();
		ball_prx_reporter->setName("ball_prx_reporter");
		ball_prx_reporter->set_report_time_interval(0.01);
		ball_prx_reporter->addToReport(contactForce->getOutput("mesh2_tri_proximity"));
		model.addComponent(ball_prx_reporter);

		// Configure the model.
	    SimTK::State& state = model.initSystem();

		// Add display geometry.

		model.updMatterSubsystem().setShowDefaultGeometry(false);

		SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
		viz.setBackgroundColor(SimTK::White);

		// Simulate.

		SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
		integrator.setMaximumStepSize(1.0e-2);
		Manager manager(model, integrator);
		manager.setInitialTime(0); manager.setFinalTime(1.0);
		manager.integrate(state);

	    // Add display geometry.
		model.updMatterSubsystem().setShowDefaultGeometry(false);
		
	 		
		//simulate(model, state, false);

		//Timer
		duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;

		std::cout << "printf: " << duration << '\n';

		//Write Output Files
		
		std::vector<SimTK::Matrix> OutputMatrix;
		OutputMatrix.push_back(ball_prs_reporter->getTable().getMatrix().getAsMatrix());
		OutputMatrix.push_back(ball_prx_reporter->getTable().getMatrix().getAsMatrix());

		std::vector<std::string> OutputNames;
		OutputNames.push_back("pressure");
		OutputNames.push_back("proximity");

		SimTK::PolygonalMesh ball_mesh = SimTK::PolygonalMesh();
		ball_mesh.loadFile(ball_file);

		WISCO_VTKFileAdapter* ball_prs_vtk = new WISCO_VTKFileAdapter();
		//ball_prs_vtk->write("ball_pressure", "C:/github/wisco_opensim/tests/testContactBallFloor/results/ball_pressure/", ball_mesh, ball_table);
		ball_prs_vtk->write("ball_contact", "C:/github/wisco_opensim/tests/testContactBallFloor/results/ball_contact/", 
			ball_mesh, OutputMatrix, OutputNames);
        //model.print(out_file);
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

