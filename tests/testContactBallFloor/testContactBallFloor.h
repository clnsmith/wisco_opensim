/* -------------------------------------------------------------------------- *
 *                          OpenSim:  PostViewDataFileAdapter.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#ifndef OPENSIM_TEST_CONTACT_BALL_FLOOR_H_
#define OPENSIM_TEST_CONTACT_BALL_FLOOR_H_


#include <OpenSim\OpenSim.h>

namespace OpenSim {

inline void simulate(Model& model, SimTK::State& state, bool saveStatesFile){
/*
		SimTK::State initState = state;
		SimTK::Visualizer::InputSilo* silo;

		// Configure the visualizer.
		if (model.getUseVisualizer()) {
			SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
			// We use the input silo to get key presses. OpenSim::ModelVisualizer
			// adds two InputListeners; the second is InputSilo.
			silo = dynamic_cast<SimTK::Visualizer::InputSilo*>(&viz.updInputListener(1));

            //Draw Line
            auto line = new SimTK::DecorativeLine(SimTK::Vec3(0),SimTK::Vec3(1.0));
            line->setLineThickness(5.0);
            
            //auto brick = new SimTK::DecorativeBrick();

            auto trans = new SimTK::Transform(SimTK::Vec3(0));
            auto ind = model.getBodySet()[0].getMobilizedBodyIndex();
            viz.addDecoration(ind, *trans, *line);
            
			SimTK::DecorativeText help("Press any key to start a new simulation; "
				"ESC to quit.");
			help.setIsScreenText(true);
			viz.addDecoration(SimTK::MobilizedBodyIndex(0), SimTK::Vec3(0), help);

			viz.setBackgroundType(viz.GroundAndSky).setShowSimTime(true);
			viz.drawFrameNow(state);
			std::cout << "A visualizer window has opened." << std::endl;

 
		}

		// Simulate until the user presses ESC (or enters 'q' if visualization has
		// been disabled).
		while (true) {
			if (model.getUseVisualizer()) {
				// Get a key press.
				silo->clear(); // Ignore any previous key presses.
				unsigned key, modifiers;
				silo->waitForKeyHit(key, modifiers);
				if (key == SimTK::Visualizer::InputListener::KeyEsc) { break; }
			}
			else {
				std::cout << "Press <Enter> to begin simulating, or 'q' followed "
					<< "by <Enter> to quit . . . " << std::endl;
				if (std::cin.get() == 'q') { break; }
			}

			// Set up manager and simulate.
			state = initState;
			SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
			integrator.setMaximumStepSize(1.0e-2);

			Manager manager(model, integrator);
			manager.setInitialTime(0.); manager.setFinalTime(0.5);
			manager.integrate(state);

			// Save the states to a storage file (if requested).
			if (saveStatesFile) {
				manager.getStateStorage().print("hopperStates.sto");
			}
		}*/
	}

}
#endif // OPENSIM_TEST_CONTACT_BALL_FLOOR_H_
