#include <OpenSim/OpenSim.h>
#include <vector>
#include "WISCO_ElasticFoundationForce.h"
#include "WISCO_Ligament.h"
#include "WISCO_IdealMuscle.h"
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
		//static const std::string plugin_file{ "../../../install/plugin/WISCO_Plugin" };
		static const std::string plugin_file{ "C:/github/wisco_opensim/install/plugin/WISCO_Plugin" };

		LoadOpenSimLibrary(plugin_file, true);

		//Load Model
		static const std::string model_file{ "./inputs/fbknee.osim" };
		static const std::string settings_file{ "./inputs/ContactAnalysis_settings.xml" };

		Model model(model_file);

		//=====================================================================
		// Configure the model
		//=====================================================================
		model.setUseVisualizer(true);

		SimTK::State& state = model.initSystem();

		for (Coordinate& coord : model.updComponentList<Coordinate>()) {
			coord.set_locked(true);
		}

		Joint& knee = model.updJointSet().get("knee_r");
		Joint& pf = model.updJointSet().get("pf_r");

		//Prescribe knee flexion
		std::string pres_knee_mot_file{ "C:/github/wisco_opensim/source/examples/exampleFBKneePassiveFlexion/inputs/kfec_halfhz_0050.mot" };
		STOFileAdapter* pres_mot = new STOFileAdapter();
		TimeSeriesTable pres_table = pres_mot->read(pres_knee_mot_file);

		std::vector<double> time = pres_table.getIndependentColumn();
		int flex_col_i = pres_table.getColumnIndex("knee_flex_r");
		const SimTK::Matrix& knee_flex_view = pres_table.getMatrix().getAsMatrix();

		std::vector<double> knee_flex(pres_table.getNumRows());

		for (int i = 0; i < pres_table.getNumRows(); ++i) {
			knee_flex[i] = knee_flex_view(i, flex_col_i)*SimTK::Pi / 180;
		}

		SimmSpline knee_flex_func = SimmSpline(time.size(), &time[0], &knee_flex[0], "prescribed_knee_flex");


		knee.upd_coordinates(0).set_prescribed(true);
		knee.upd_coordinates(0).set_prescribed_function(knee_flex_func);

		for (int i = 0; i < 6; ++i) {
			knee.upd_coordinates(i).set_locked(false);
			knee.upd_coordinates(i).set_clamped(false);
			pf.upd_coordinates(i).set_locked(false);
			pf.upd_coordinates(i).set_clamped(false);

		}


		/*
		CustomJoint& pelvis_jnt = model.updComponent<CustomJoint>("gnd_pelvis");
		pelvis_jnt.upd_coordinates(0).set_default_value( 90 * SimTK::Pi / 180);

		CustomJoint& hip = model.updComponent<CustomJoint>("hip_r");
		hip.upd_coordinates(0).set_default_value(20 * SimTK::Pi / 180);

		CustomJoint& ankle = model.updComponent<CustomJoint>("ankle_r");
		ankle.upd_coordinates(0).set_default_value(-30 * SimTK::Pi / 180);
		*/
		model.set_gravity(Vec3(0, 0, 0));


		for (WrapObject& wrap : model.updComponentList<WrapObject>()) {
			wrap.upd_Appearance().set_visible(false);

		}

		//Add 2% muscle activation
		PrescribedController* msl_control = new PrescribedController();
		msl_control->setActuators(model.updActuators());

		for (WISCO_IdealMuscle& msl : model.updComponentList<WISCO_IdealMuscle>()) {
			msl_control->prescribeControlForActuator(msl.getName(), new Constant(0.02));
		}

		model.addComponent(msl_control);

		for (WISCO_Ligament& lig : model.updComponentList<WISCO_Ligament>()){
			lig.set_normalized_damping_coefficient(0.003);
		}
			/*ConsoleReporter* con = new ConsoleReporter();
			con->setName("itb_report");
			con->addToReport(model.getComponent<WISCO_Ligament>("ITB_r").getOutput("dynamic_quantities").getChannel("force_total"));
			con->set_report_time_interval(0.001);
			model.addComponent(con);*/

			state = model.initSystem();
			model.equilibrateMuscles(state);

			// Add display geometry.
			model.updMatterSubsystem().setShowDefaultGeometry(false);

			SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
			viz.setBackgroundColor(SimTK::White);
			viz.setShowSimTime(true);

			//=====================================================================
			// Simulate
			//=====================================================================
			if (true) {
				double initialTime = 0.0;
				double finalTime = 5.0;


				SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
				integrator.setAccuracy(0.00001);

				//SimTK::CPodesIntegrator integrator(model.getSystem(), SimTK::CPodes::BDF, SimTK::CPodes::Newton);
				//integrator.setAccuracy(0.00001);

				Manager manager(model, integrator);
				manager.setInitialTime(initialTime); manager.setFinalTime(finalTime);

				manager.integrate(state);
				manager.getStateStorage().resampleLinear(0.01);


				// Report Timer Results
				duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
				std::cout << "printf: " << duration << '\n';


				//=====================================================================
				//Write Outputs
				//=====================================================================
				model.print("./results/fbknee.osim");

				//Motion file
				static const std::string out_mot_file{ "./results/fbknee_passive_flex.mot" };
				manager.getStateStorage().print(out_mot_file);

			}

		//Perform ContactAnalysis
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

