#include <OpenSim/OpenSim.h>
#include <vector>
#include "wipCOMAKMain.h"
#include "WISCO_ContactAnalysis.h"
#include "COMAKParameters.h"
#include "WISCO_HelperFunctions.h"
#include "WISCO_Settings.h"

//using namespace SimTK;
using namespace OpenSim;
using SimTK::Vec3;

int main(){
    try {
		
		
		//Load WISCO_Plugin
		static const std::string plugin_file{ "../../../install/plugin/WISCO_Plugin" };
		LoadOpenSimLibrary(plugin_file, true);

		//Initialize Parameters Class
		COMAKParameters comakParams;
		
		comakParams.set_perform_scale(true);
		comakParams.set_scale_settings_file(WISCO_HOME + "source/sandbox/wipCOMAKMain/inputs/scale_settings.xml");
		comakParams.set_generic_model_file("generic_model.osim");
		comakParams.set_model_file("scaled_model.osim");
		
		comakParams.set_perform_inverse_kinematics(true);
		comakParams.set_ik_settings_file("ik_settings_file.xml");
		comakParams.set_trc_kinematics_file("overground_17.trc");
		comakParams.set_ik_motion_file("overground_17_ik.mot");
		
		comakParams.set_results_motion_file("overground_17_comak.mot");
		comakParams.set_verbose(1);
		comakParams.set_start_time(0.0);
		comakParams.set_end_time(0.0);
		comakParams.set_time_step(0.01);

		comakParams.set_equilibriate_secondary_coordinates_at_start(true);
		comakParams.set_settle_threshold(0.00001);
		comakParams.set_settle_time_step(0.01);
		comakParams.set_settle_tolerance(0.00001);
		
		//Set COMAK Parameters
		
		Array<std::string> prescribed_coord;
		prescribed_coord.append("pelvis_tx");
		prescribed_coord.append("pelvis_ty");
		prescribed_coord.append("pelvis_tz");
		prescribed_coord.append("pelvis_tilt");
		prescribed_coord.append("pelvis_list");
		prescribed_coord.append("pelvis_rot");
		comakParams.set_prescribed_coordinates(prescribed_coord);

		Array<std::string> primary_coord;
		primary_coord.append("hip_flex_r");
		primary_coord.append("hip_add_r");
		primary_coord.append("hip_rot_r");
		primary_coord.append("knee_flex_r");
		primary_coord.append("ankle_flex_r");
		comakParams.set_primary_coordinates(primary_coord);

		Array<std::string> secondary_coord;
		secondary_coord.append("knee_add_r");
		secondary_coord.append("knee_rot_r");
		secondary_coord.append("knee_tx_r");
		secondary_coord.append("knee_ty_r");
		secondary_coord.append("knee_tz_r");
		secondary_coord.append("pf_flex_r");
		secondary_coord.append("pf_rot_r");
		secondary_coord.append("pf_tilt_r");
		secondary_coord.append("pf_tx_r");
		secondary_coord.append("pf_ty_r");
		secondary_coord.append("pf_tz_r");
		comakParams.set_secondary_coordinates(secondary_coord);
		
		//Initialize Model
		//----------------
		static const std::string model_file = WISCO_HOME + "source/models/fbknee/lenhart/fbknee.osim";
		Model model(model_file);
		SimTK::State state = model.initSystem();

		//Load Motion File
		/*static const std::string ik_motion_file = WISCO_HOME + "source/sandbox/wipCOMAKMain/inputs/overground_17_invkin_opensim.mot";
		StatesTrajectory ik_states;
		Storage sto = Storage(ik_motion_file);
		*/

		Array<std::string> rotation_coords;
		rotation_coords.append("pelvis_tilt");
		rotation_coords.append("pelvis_list");
		rotation_coords.append("pelvis_rot");
		rotation_coords.append("hip_flex_r");
		rotation_coords.append("hip_flex_r");
		rotation_coords.append("hip_flex_r");
		rotation_coords.append("knee_flex_r");
		rotation_coords.append("knee_add_r");
		rotation_coords.append("knee_rot_r");
		rotation_coords.append("pf_flex_r");
		rotation_coords.append("pf_rot_r");
		rotation_coords.append("pf_tilt_r");
		rotation_coords.append("ankle_flex_r");
		/*
		Array<std::string> col_labels = sto.getColumnLabels();

		for (int i = 0; i < col_labels.size(); ++i) {
			if (rotation_coords.findIndex(col_labels.get(i)) != -1) {
				sto.multiplyColumn(i, SimTK::Pi / 180);
			}
		}
		sto.setInDegrees(false);
		ik_states.createFromStatesStorage(model, sto);
		*/
		//ik_states.createFromStatesStorage(model, ik_motion_file);

		/*
		STOFileAdapter sto;
		TimeSeriesTable ik_data = sto.read(ik_motion_file);
		std::vector<double> time = ik_data.getIndependentColumn();
		*/

		//Verify COMAK Coordinates in Model, Input IK file etc
		verifyCoordinateParameters(model, comakParams);

		//Scale Model
		if (comakParams.get_perform_scale()) {
			performScaleModel(comakParams);
		}
		//Inverse Kinematics 

		//Set States to first timestep
		/*SimTK::State first_state = ik_states.get(0);
		state.setQ(first_state.getQ());
		state.setU(first_state.getU());

		//Passive Settle to Find Initial State
		if (comakParams.get_equilibriate_secondary_coordinates_at_start()) {
			ComponentList<Coordinate> coord_list = model.updComponentList<Coordinate>();
			for (Coordinate& coord : coord_list) {
				if (primary_coord.findIndex(coord.getName()) != -1) {
					coord.setLocked(state, true);
				}
			}
		}*/

		//Loop Over Each Time Step
		/*for (int i = 0; i < time.size(); ++i) {
			std::cout << "Frame: " << i+1 << "/" << time.size() << std::endl;
			std::cout << "Time: " << time[i] << std::endl;
			std::cout << "=====================================================\n" << std::endl;
			
			

		}*/
		

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

void performScaleModel(COMAKParameters& comakParams) {
	ScaleTool scale_tool = ScaleTool(comakParams.get_scale_settings_file());
	
	//Property<GenericModelMaker> genModelMaker = dynamic_cast<Property<GenericModelMaker>>(scale_tool.updPropertyByName("GenericModelMaker"));
	//genModelMaker.setModelFileName(comakParams.get_generic_model_file());
	scale_tool.run();
}

void performSecondaryConstraintSimulation() {

}

void performInverseKinematics() {

}

void verifyCoordinateParameters(Model& model, COMAKParameters& comakParams) {
	ComponentList<Coordinate>& coord_list = model.updComponentList<Coordinate>();

	for (Coordinate& coord : coord_list) {

		bool isPrescribed = false;
		if (comakParams.getProperty_prescribed_coordinates().findIndex(coord.getName()) > -1)
			isPrescribed = true;
		bool isPrimary = (comakParams.getProperty_primary_coordinates().findIndex(coord.getName()) > -1);
		bool isSecondary = (comakParams.getProperty_secondary_coordinates().findIndex(coord.getName()) > -1);

		if (!isPrescribed && !isPrimary && !isSecondary) {
			comakParams.append_prescribed_coordinates(coord.getName());

			std::cout << "WARNING: Coordinate (" << coord.getName() <<
				") was not listed in COMAK params file. Assumed PRESCRIBED."
				<< std::endl;
		}

		int counted = 0;
		if (isPrescribed)
			counted++;
		if (isPrimary)
			counted++;
		if (isSecondary)
			counted++;

		if (counted > 1) {
			OPENSIM_THROW(Exception, "Coordinate: " + coord.getName() + " was listed as multiple COMAK coordinate types (Prescribed, Primary, Secondary) in parameters file.")
		}
	}
}