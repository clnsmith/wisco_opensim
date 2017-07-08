#include <OpenSim/OpenSim.h>
#include <vector>
#include "WISCO_ElasticFoundationForce.h"
#include "WISCO_Ligament.h"
#include "WISCO_IdealMuscle.h"
#include "WISCO_HelperFunctions.h"

using namespace OpenSim;
using SimTK::Vec3;

int main(int argc, const char * argv[])
{
	try {
		// Timer
		std::clock_t start;
		double duration;
		start = std::clock();

		// Read Command Line Arguments
		const std::string plugin_file = argv[1];
		const std::string model_file = argv[2];
		const std::string prescribed_mot_file = argv[3];
		const std::string out_mot_file = argv[4];
		const std::string settings_file = argv[5];

		//Load WISCO_Plugin
		//LoadOpenSimLibrary(plugin_file, true);
		LoadOpenSimLibrary("../../install/plugin/WISCO_plugin", true);

		//Load Model
		Model model(model_file);

		//Load Prescribed .mot file
		STOFileAdapter* pres_mot = new STOFileAdapter();
		TimeSeriesTable pres_table = pres_mot->read(prescribed_mot_file);
		std::vector<std::string> pres_labels = pres_table.getColumnLabels();
		std::vector<double> pres_time = pres_table.getIndependentColumn();
			//=====================================================================
			// Configure the model
			//=====================================================================
			//model.setUseVisualizer(false);

			SimTK::State& state = model.initSystem();

			for (Coordinate& coord : model.updComponentList<Coordinate>()) {
				coord.set_locked(true);
			}

			Joint& knee = model.updJointSet().get("knee_r");
			Joint& pf = model.updJointSet().get("pf_r");

			//Prescribe knee flexion



			int flex_col_i = pres_table.getColumnIndex("knee_flex_r");
			const SimTK::Matrix& knee_flex_view = pres_table.getMatrix().getAsMatrix();

			std::vector<double> knee_flex(pres_table.getNumRows());

			for (int i = 0; i < pres_table.getNumRows(); ++i) {
				knee_flex[i] = knee_flex_view(i,flex_col_i)*SimTK::Pi/180;
			}

			SimmSpline knee_flex_func = SimmSpline(pres_time.size(), &pres_time[0], &knee_flex[0], "prescribed_knee_flex");


			knee.upd_coordinates(0).set_prescribed(true);
			knee.upd_coordinates(0).set_prescribed_function(knee_flex_func);

			for (int i = 0; i < 6; ++i) {
				knee.upd_coordinates(i).set_locked(false);
				knee.upd_coordinates(i).set_clamped(false);
				pf.upd_coordinates(i).set_locked(false);
				pf.upd_coordinates(i).set_clamped(false);
			}

			model.set_gravity(Vec3(0, 0, 0));


			for (WrapObject& wrap : model.updComponentList<WrapObject>()) {
				wrap.upd_Appearance().set_visible(false);

			}

			//Add 2% muscle activation

			PrescribedController* msl_control = new PrescribedController();
			msl_control->setActuators(model.updActuators());

			for (WISCO_IdealMuscle& msl : model.updComponentList<WISCO_IdealMuscle>()) {
				int pres_ind;
				if (contains_string(pres_labels, msl.getName(),pres_ind)) {
					const SimTK::Matrix& pres_table_view = pres_table.getMatrix().getAsMatrix();

					std::vector<double> Fval(pres_time.size());
					for (int i = 0; i < pres_table.getNumRows(); ++i) {
						Fval[i] = pres_table_view(i, pres_ind);
					}

					GCVSpline* control_func = new GCVSpline(5, Fval.size(), &pres_time[0], &Fval[0]);
					msl_control->prescribeControlForActuator(msl.getName(), control_func);
				}
				else {
					msl_control->prescribeControlForActuator(msl.getName(), new Constant(0.5));
				}
			}

			model.addComponent(msl_control);

			state = model.initSystem();
			model.equilibrateMuscles(state);

			// Add display geometry.
			model.updMatterSubsystem().setShowDefaultGeometry(false);

			//SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
			//viz.setBackgroundColor(SimTK::White);
			//viz.setShowSimTime(true);

			//=====================================================================
			// Simulate
			//=====================================================================
			if (false) {
				double initialTime = pres_time[0];
				int size = pres_time.size();
				double finalTime = pres_time.back();


				SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
				integrator.setAccuracy(0.00001);

				//SimTK::CPodesIntegrator integrator(model.getSystem(), SimTK::CPodes::BDF, SimTK::CPodes::Newton);
				//integrator.setAccuracy(0.000001);

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
	//std::cout << "Press return to continue" << std::endl;
	//std::cin.get();
	return 0;
}

