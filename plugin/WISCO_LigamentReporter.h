#ifndef OPENSIM_WISCO_LIGAMENT_REPORTER_H_
#define OPENSIM_WISCO_LIGAMENT_REPORTER_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  WISCO_LigamentReporter.h                   *
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


#include <OpenSim/Common/Reporter.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include "osimPluginDLL.h"
#include "WISCO_Ligament.h"
namespace OpenSim {

/** Stores the states during a simulation in a StatesTrajectory.
 *
 * This class was introduced in v4.0 and is intended to replace the
 * StatesReporter analysis.
 *
 * @ingroup reporters
 */
class OSIMPLUGIN_API WISCO_LigamentReporter : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(WISCO_LigamentReporter, ModelComponent);

public:
    /** Access the accumulated states. */
    const TableReporter& getTableReporter() const; 
    /** Clear the accumulated states. */ 
    void clear();
	
	double getReportTimeInterval() {
		return _table_rep.get_report_time_interval();
	}

	void setReportTimeInterval(double time_interval) {
		_table_rep.set_report_time_interval(time_interval);
	}
protected:
	void extendConnectToModel(Model& model) override;

private:
    TableReporter _table_rep;
};

} // namespace

#endif // OPENSIM_WISCO_LIGAMENT_REPORTER_H_
