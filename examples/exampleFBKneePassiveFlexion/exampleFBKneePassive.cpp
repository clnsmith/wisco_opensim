#include <OpenSim/OpenSim.h>
#include <vector>
#include "WISCO_ElasticFoundationForce.h"
#include "WISCO_Ligament.h"
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

		//Load Model
		static const std::string model_file{ "./inputs/fbknee.osim" };
		static const std::string settings_file{ "./inputs/ContactAnalysis_settings.xml" };
		//static const std::string model_file{ "./inputs/fbknee.osim" };
		//static const std::string model_file{ "C:/github/wisco_opensim/source/tests/shared/fbknee/fbknee_simple.osim" };
		
		if (true) {
			Model model(model_file);

			//=====================================================================
			// Configure the model
			//=====================================================================
			model.setUseVisualizer(true);

			SimTK::State& state = model.initSystem();

			ForceReporter* force_reporter = new ForceReporter();
			model.addAnalysis(force_reporter);
			
			MuscleAnalysis* msl_rep = new MuscleAnalysis();
			msl_rep->setStepInterval(20);
			model.addAnalysis(msl_rep);
			
			//model.equilibrateMuscles(state);
			for (Coordinate& coord : model.updComponentList<Coordinate>()) {
				coord.set_locked(true);
			}

			for (Millard2012EquilibriumMuscle& msl : model.updComponentList<Millard2012EquilibriumMuscle>()) {
				msl.set_appliesForce(true);
				msl.set_ignore_activation_dynamics(true);
				msl.set_ignore_tendon_compliance(true);
				msl.set_max_isometric_force(100);
				//msl.upd_FiberForceLengthCurve().set_stiffness_at_low_force(1.0);
				//msl.upd_FiberForceLengthCurve().set_stiffness_at_one_norm_force(2.0);
				//msl.upd_FiberForceLengthCurve().set_curviness(0.3);
				//msl.set_max_isometric_force(50);
				//msl.set_minimum_activation(0.5);
			}

			Joint& knee = model.updJointSet().get("knee_r");
			Joint& pf = model.updJointSet().get("pf_r");

			//Prescribe knee flexion
			double times[7] = { 0.0, 0.5, 1.0,2.0,3.0,4.0,5.0};
			double flex_ang[7] = { 0, 0, 0, SimTK::Pi / 4, SimTK::Pi / 2, SimTK::Pi / 4, 0 };



			SimmSpline flexion_func = SimmSpline(7, times, flex_ang, "flexion_func");
			

			knee.upd_coordinates(0).set_prescribed(true);
			knee.upd_coordinates(0).set_prescribed_function(flexion_func);

			knee.upd_coordinates(0).set_locked(false);


			knee.upd_coordinates(1).set_locked(false);
			knee.upd_coordinates(2).set_locked(false);

			knee.upd_coordinates(3).set_locked(false);
			knee.upd_coordinates(4).set_locked(false);
			knee.upd_coordinates(5).set_locked(false);
			//knee.upd_coordinates(4).setDefaultValue(-0.001);

			pf.upd_coordinates(0).set_locked(false);
			pf.upd_coordinates(1).set_locked(false);
			pf.upd_coordinates(2).set_locked(false);

			pf.upd_coordinates(3).set_locked(false);
			pf.upd_coordinates(4).set_locked(false);
			pf.upd_coordinates(5).set_locked(false);
			/*
			CustomJoint& pelvis_jnt = model.updComponent<CustomJoint>("gnd_pelvis");
			pelvis_jnt.upd_coordinates(0).set_default_value( SimTK::Pi / 2);
			CustomJoint& hip = model.updComponent<CustomJoint>("hip_r");
			hip.upd_coordinates(0).set_default_value(SimTK::Pi / 6);
			*/
			int i = 0;
			for (WISCO_Ligament& lig : model.updComponentList<WISCO_Ligament>()) {
				lig.set_normalized_damping_coefficient(0.003);
				lig.set_appliesForce(true);
				++i;

			}

			for (WISCO_ElasticFoundationForce& force : model.updComponentList<WISCO_ElasticFoundationForce>()) {
				//force.set_appliesForce(false);
				//force.setModelingOption(state, "contact_stats", 1);
				//force.setModelingOption(state, "contact_stats_medial_lateral", 1);
				force.upd_casting_mesh_contact_params().set_elastic_modulus(5000000);
				force.upd_target_mesh_contact_params().set_elastic_modulus(5000000);
				std::vector<std::string> type;
				type.push_back("total");
				type.push_back("medial");
				type.push_back("lateral");

				std::vector<std::string> param;
				param.push_back("mean_pressure");
				param.push_back("max_pressure");
				param.push_back("mean_proximity");
				param.push_back("max_proximity");
				param.push_back("contact_area");

				std::vector<std::string> paramVec3;
				paramVec3.push_back("cop");
				paramVec3.push_back("contact_force");

				TableReporter* cntRep = new TableReporter();
				cntRep->setName(force.getName() + "_Reporter");
				for (std::string t : type) {
					for (std::string p : param) {
						cntRep->addToReport(force.getOutput("casting_mesh_" + t + "_" + p));
					}
				}
				cntRep->set_report_time_interval(0.001);
				//model.addComponent(cntRep);

				TableReporterVec3* cntRepVec3 = new TableReporterVec3();
				cntRepVec3->setName(force.getName() + "_ReporterVec3");
				for (std::string t : type) {
					for (std::string p : paramVec3) {
						cntRepVec3->addToReport(force.getOutput("casting_mesh_" + t + "_" + p));
					}
				}
				cntRepVec3->set_report_time_interval(0.001);
				//model.addComponent(cntRepVec3);
			}



			for (BushingForce& bush : model.updComponentList<BushingForce>()) {
				bush.set_rotational_damping(Vec3(1.0));
				bush.set_translational_damping(Vec3(10.0));
				bush.set_rotational_stiffness(Vec3(0.0));
				bush.set_translational_stiffness(Vec3(0.0));
				bush.set_appliesForce(true);
			}
			model.set_gravity(Vec3(0, 0, 0));


			for (WrapObject& wrap : model.updComponentList<WrapObject>()) {
				wrap.upd_Appearance().set_visible(false);

			}

			state = model.initSystem();
			model.equilibrateMuscles(state);

			for (WISCO_ElasticFoundationForce& force : model.updComponentList<WISCO_ElasticFoundationForce>()) {
				//force.set_appliesForce(false);
				//force.setModelingOption(state, "contact_stats", 1);
				//force.setModelingOption(state, "contact_stats_medial_lateral", 1);
				//force.set_elastic_foundation_formulation("linear");
				force.set_use_smart_backside_contact(false);
			}

			// Add display geometry.
			model.updMatterSubsystem().setShowDefaultGeometry(false);

			SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
			viz.setBackgroundColor(SimTK::White);
			viz.setMode(SimTK::Visualizer::Mode::Sampling);
			//viz.setDesiredBufferLengthInSec(0.01);
			viz.setDesiredFrameRate(300);
			//viz.setDesiredBufferLengthInSec(0.1);
			viz.setShowSimTime(true);
			//=====================================================================
			// Simulate
			//=====================================================================
			if (true) {
				//SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
				//integrator.setMaximumStepSize(1.0e-2);
				//integrator.setAccuracy(0.1);
				//integrator.setMinimumStepSize(0.0001);
				//integrator.setFixedStepSize(0.01);

				//SimTK::SemiExplicitEulerIntegrator integrator(model.getSystem(), 0.000001);

				/*
				SimTK::ExplicitEulerIntegrator integrator(model.getSystem());
				integrator.setAccuracy(0.1);
				*/
				SimTK::CPodesIntegrator integrator(model.getSystem(), SimTK::CPodes::BDF, SimTK::CPodes::Newton);
				integrator.setAccuracy(0.00001);
				//integrator.setMinimumStepSize(0.00001);


				/*
				SimTK::RungeKutta2Integrator integrator(model.getSystem());
				integrator.setAccuracy(0.1);
				integrator.setMinimumStepSize(0.00001);
				*/

				Manager manager(model, integrator);
				manager.setInitialTime(0); manager.setFinalTime(5.0);
				manager.integrate(state);

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

				force_reporter->printResults("./results/");
				msl_rep->printResults("./results/");
				for (TableReporter report : model.getComponentList<TableReporter>()) {
					TimeSeriesTable table = report.getTable();
					STOFileAdapter sto = STOFileAdapter();
					sto.write(table, "./results/" + report.getName() + ".sto");
				}

				for (TableReporterVec3 report : model.getComponentList<TableReporterVec3>()) {
					TimeSeriesTable_<SimTK::Vec3> table = report.getTable();
					STOFileAdapterVec3 sto = STOFileAdapterVec3();
					sto.write(table, "./results/" + report.getName() + ".sto");
				}
			}
		}
		//Perform ContactAnalysis
		

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

