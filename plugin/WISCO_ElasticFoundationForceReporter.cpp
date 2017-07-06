/* -------------------------------------------------------------------------- *
 *                   OpenSim:  WISCO_ElasticFoundationForceReporter.cpp                   *
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

#include <OpenSim/Simulation/Model/Model.h>
#include "WISCO_ElasticFoundationForceReporter.h"
#include "WISCO_ElasticFoundationForce.h"

using namespace OpenSim;

void WISCO_ElasticFoundationForceReporter::extendConnectToModel(Model& model) {
	Super::extendConnectToModel(model);
	
	ComponentList<WISCO_ElasticFoundationForce> frc_list = model.updComponentList<WISCO_ElasticFoundationForce>();

	for (WISCO_ElasticFoundationForce& frc : frc_list) {
		std::string frc_name = frc.getName();
		std::string Cmesh_name = frc.getConnectee<WISCO_ContactMesh>("casting_mesh").getName();
		std::string Tmesh_name = frc.getConnectee<WISCO_ContactMesh>("target_mesh").getName();

		std::vector<std::string> mesh_data_names = frc.getMeshDataNames();

		//Face and Vertex
		for (std::string mesh_data : mesh_data_names) {
			TableReporterVector* casting_rep = new TableReporterVector();
			adoptSubcomponent(casting_rep);
			setNextSubcomponentInSystem(*casting_rep);
			casting_rep->setName(frc_name + "." + Cmesh_name + ".tri." + mesh_data);
			casting_rep->addToReport(frc.getOutput("casting_tri_data").getChannel("casting_mesh.tri." + mesh_data));
			casting_rep->set_report_time_interval(0);

			TableReporterVector* target_rep = new TableReporterVector();
			adoptSubcomponent(target_rep);
			setNextSubcomponentInSystem(*target_rep);
			target_rep->setName(frc_name + "." + Tmesh_name + ".tri." + mesh_data);
			target_rep->addToReport(frc.getOutput("target_tri_data").getChannel("target_mesh.tri." + mesh_data));
			target_rep->set_report_time_interval(0);

		}

		for (std::string mesh_data : mesh_data_names) {
			TableReporterVector* casting_rep = new TableReporterVector();
			adoptSubcomponent(casting_rep);
			setNextSubcomponentInSystem(*casting_rep);
			casting_rep->setName(frc_name + "." + Cmesh_name + ".vertex." + mesh_data);
			casting_rep->addToReport(frc.getOutput("casting_vertex_data").getChannel("casting_mesh.vertex." + mesh_data));
			casting_rep->set_report_time_interval(0);

			TableReporterVector* target_rep = new TableReporterVector();
			adoptSubcomponent(target_rep);
			setNextSubcomponentInSystem(*target_rep);
			target_rep->setName(frc_name + "." + Tmesh_name + ".vertex." + mesh_data);
			target_rep->addToReport(frc.getOutput("target_vertex_data").getChannel("target_mesh.vertex." + mesh_data));
			target_rep->set_report_time_interval(0);
		}

		//Contact Stats
		std::vector<std::string> stat_names = frc.getContactStatNames();
		std::vector<std::string> stat_names_vec3 = frc.getContactStatNamesVec3();

		TableReporter* Crep = new TableReporter();
		adoptSubcomponent(Crep);
		setNextSubcomponentInSystem(*Crep);
		Crep->setName(frc_name + "." + Cmesh_name);
		Crep->set_report_time_interval(0);
		Crep->addToReport(frc.getOutput("casting_contact_stats_total"));
		Crep->addToReport(frc.getOutput("casting_contact_stats_medial_lateral"));

		TableReporter* Trep = new TableReporter();
		adoptSubcomponent(Trep);
		setNextSubcomponentInSystem(*Trep);
		Trep->setName(frc_name + "." + Tmesh_name);
		Trep->set_report_time_interval(0);
		Trep->addToReport(frc.getOutput("target_contact_stats_total"));
		Trep->addToReport(frc.getOutput("target_contact_stats_medial_lateral"));

		TableReporterVec3* Cvec3_rep = new TableReporterVec3();
		adoptSubcomponent(Cvec3_rep);
		setNextSubcomponentInSystem(*Cvec3_rep);
		Cvec3_rep->setName(frc_name + "." + Cmesh_name + ".vec3");
		Cvec3_rep->set_report_time_interval(0);		
		Cvec3_rep->addToReport(frc.getOutput("casting_contact_stats_total_vec3"));
		Cvec3_rep->addToReport(frc.getOutput("casting_contact_stats_medial_lateral_vec3"));

		TableReporterVec3* Tvec3_rep = new TableReporterVec3();
		adoptSubcomponent(Tvec3_rep);
		setNextSubcomponentInSystem(*Tvec3_rep);
		Tvec3_rep->setName(frc_name + "." + Tmesh_name + ".vec3");
		Tvec3_rep->set_report_time_interval(0);
		Tvec3_rep->addToReport(frc.getOutput("target_contact_stats_total_vec3"));
		Tvec3_rep->addToReport(frc.getOutput("target_contact_stats_medial_lateral_vec3"));
	}

		//_table_rep_int.addToReport(frc.getOutput("casting_mesh_n_active_tri"), frc_name + "." + Cmesh_name + ".n_active_tri");
		//_table_rep_int.addToReport(frc.getOutput("target_mesh_n_active_tri"), frc_name + "." + Tmesh_name + ".n_active_tri");

}

std::vector<std::string> WISCO_ElasticFoundationForceReporter::getReporterNames() {
	ComponentList<const TableReporter> reportList = getComponentList<TableReporter>();
	std::vector<std::string> rep_names;
	for (TableReporter rep_vector : reportList) {
		rep_names.push_back(rep_vector.getName());
	}
	return rep_names;
}

std::vector<std::string> WISCO_ElasticFoundationForceReporter::getReporterVec3Names() {
	ComponentList<const TableReporterVec3> reportList = getComponentList<TableReporterVec3>();
	std::vector<std::string> rep_names;
	for (TableReporterVec3 rep_vector : reportList) {
		rep_names.push_back(rep_vector.getName());
	}
	return rep_names;
}

std::vector<std::string> WISCO_ElasticFoundationForceReporter::getReporterVectorNames() {
	ComponentList<const TableReporterVector> reportList = getComponentList<TableReporterVector>();
	std::vector<std::string> rep_names;
	for (TableReporterVector rep_vector : reportList) {
		rep_names.push_back(rep_vector.getName());
	}
	return rep_names;
}



