/* -------------------------------------------------------------------------- *
 *                   OpenSim:  WISCO_LigamentReporter.cpp                   *
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

#include "WISCO_LigamentReporter.h"
#include "WISCO_Ligament.h"
using namespace OpenSim;


void WISCO_LigamentReporter::clear() {
	_table_rep.clearTable();
}

void WISCO_LigamentReporter::extendConnectToModel(Model& model) {
	Super::extendConnectToModel(model);
	ComponentList<WISCO_Ligament> lig_list = model.updComponentList<WISCO_Ligament>();

	for (WISCO_Ligament& lig : lig_list) {
		_table_rep.addToReport(lig.getOutput("dynamic_quantities").getChannel("force_spring"), lig.getName() + ".force_spring");
		_table_rep.addToReport(lig.getOutput("dynamic_quantities").getChannel("force_damping"), lig.getName() + ".force_damping");
		_table_rep.addToReport(lig.getOutput("dynamic_quantities").getChannel("force_total"), lig.getName() + ".force_total");
		_table_rep.addToReport(lig.getOutput("dynamic_quantities").getChannel("length"), lig.getName() + ".length");
		_table_rep.addToReport(lig.getOutput("dynamic_quantities").getChannel("lengthening_speed"), lig.getName() + ".lengthening_speed");
		_table_rep.addToReport(lig.getOutput("dynamic_quantities").getChannel("strain"), lig.getName() + ".strain");
		_table_rep.addToReport(lig.getOutput("dynamic_quantities").getChannel("strain_rate"), lig.getName() + ".strain_rate");
	}

	_table_rep.set_report_time_interval(0);
	model.addComponent(&_table_rep);
}
const TableReporter& WISCO_LigamentReporter::getTableReporter() const {
    return _table_rep;
}




