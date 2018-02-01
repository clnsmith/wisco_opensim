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
		static const std::string plugin_file{ "../../../install/plugin/WISCO_Plugin" };
		LoadOpenSimLibrary(plugin_file, true);

		//Load Simple Knee Model
		static const std::string model_file{ "./inputs/fbknee.osim" };
		static const std::string settings_file{ "./inputs/ContactAnalysis_settings.xml" };
		
		Model model(model_file);

		//Load Motion Data
		std::string mot_file{ "C:/github/wisco_opensim/source/examples/exampleFBKneePassiveFlexion/inputs/kfec_halfhz_0050.mot" };

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
				knee_flex[i] = knee_flex_view(i,flex_col_i)*SimTK::Pi/180;
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

			model.set_gravity(Vec3(0, 0, 0));


			for (WrapObject& wrap : model.updComponentList<WrapObject>()) {
				wrap.upd_Appearance().set_visible(false);

			}

			state = model.initSystem();


			// Add display geometry.
			model.updMatterSubsystem().setShowDefaultGeometry(false);

			SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
			viz.setBackgroundColor(SimTK::White);
			viz.setShowSimTime(true);			
		
			//=====================================================================
			// Simulate
			//=====================================================================
			if (true) {
				SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
				//integrator.setMaximumStepSize(1.0e-2);
				integrator.setAccuracy(0.0001);

				//SimTK::CPodesIntegrator integrator(model.getSystem(), SimTK::CPodes::BDF, SimTK::CPodes::Newton);
				//integrator.setAccuracy(0.000001);
				//integrator.setMinimumStepSize(0.000001);

				Manager manager(model, integrator);
				manager.initialize(state);
				manager.integrate(5.0);

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

