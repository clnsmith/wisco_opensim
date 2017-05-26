#include <OpenSim/OpenSim.h>
#include "WISCO_ElasticFoundationForce.h"
#include "WISCO_VTKFileAdapter.h"
#include <vector>
#include "mainJointStiffness.h"


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
        static const std::string floor_file{ "./geometry/floor.stl" };
		static const std::string out_file{"ball_and_floor.osim"};
		//Build Model 
		Model model;
		model.setName("ball_and_floor");
        model.setGravity(SimTK::Vec3(0, 0, -9.80665));
		model.setUseVisualizer(true);

		Ground& ground = model.updGround();
		ground.attachGeometry(new Mesh(floor_file));

		Body* ball = new OpenSim::Body("ball", 1.0, Vec3(0), SimTK::Inertia(0.01));
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
		ground_to_ball->upd_coordinates(5).setDefaultValue(0);

		model.addJoint(ground_to_ball);
		// USE CUSTOM FORCE
		WISCO_ContactMesh* floorMesh = new WISCO_ContactMesh("floorMesh", floor_file, ground, Vec3(0), Vec3(0));
		WISCO_ContactMesh* ballMesh = new WISCO_ContactMesh("ballMesh", ball_file, *ball, Vec3(0), Vec3(0));

		model.addComponent(floorMesh);
		model.addComponent(ballMesh);

        WISCO_ElasticFoundationForce::ContactParameters* floorContactParams = new WISCO_ElasticFoundationForce::ContactParameters(30000.0, 0.45, 0.01);
		WISCO_ElasticFoundationForce::ContactParameters* ballContactParams = new WISCO_ElasticFoundationForce::ContactParameters(30000.0, 0.45, 0.01);
		
		WISCO_ElasticFoundationForce* contactForce = new WISCO_ElasticFoundationForce(*floorMesh,*floorContactParams,*ballMesh, *ballContactParams,2);
		contactForce->setName("ball_contact");

		model.addForce(contactForce);

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

