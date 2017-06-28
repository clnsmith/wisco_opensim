/* -------------------------------------------------------------------------- *
 *                   OpenSim:  WISCO_IdealMuscleReporter.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Chris Dembia                                                    *
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

#include "WISCO_IdealMuscleReporter.h"
#include "WISCO_IdealMuscle.h"
#include <OpenSim\Simulation\Model\Model.h>

using namespace OpenSim;



void WISCO_IdealMuscleReporter::extendConnectToModel(Model& model) {
	Super::extendConnectToModel(model);
	ComponentList<WISCO_IdealMuscle> msl_list = model.updComponentList<WISCO_IdealMuscle>();

	for (WISCO_IdealMuscle& msl : msl_list) {
		_table_rep.addToReport(msl.getOutput("dynamic_quantities").getChannel("activation"), msl.getName() + ".activation");
		_table_rep.addToReport(msl.getOutput("dynamic_quantities").getChannel("force"), msl.getName() + ".force");
	}

	_table_rep.set_report_time_interval(0);
	model.addComponent(&_table_rep);
}
const TableReporter& WISCO_IdealMuscleReporter::getTableReporter() const {
    return _table_rep;
}




