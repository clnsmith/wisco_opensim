#ifndef _OPENSIM_COMAK_Parameters_H_
#define _OPENSIM_COMAK_Parameters_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  COMAKParameters                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

#include <OpenSim/Common/Object.h>

namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 * A class to store and generate the settings parameters for COMAK.
 *
 * @author Colin Smith
 * @version 1.0
 */
class COMAKParameters: public Object {
	OpenSim_DECLARE_CONCRETE_OBJECT(COMAKParameters, Object)

public:
	OpenSim_DECLARE_PROPERTY(ik_motion_file, std::string, "Location of the inverse kinematics input .mot file.")
	OpenSim_DECLARE_PROPERTY(results_motion_file, std::string, "Location to write output results .mot file.")
	OpenSim_DECLARE_PROPERTY(model_file, std::string, "Location of model to use in COMAK simulation")

	OpenSim_DECLARE_PROPERTY(perform_scale, bool, "Scale Generic Model")
	OpenSim_DECLARE_OPTIONAL_PROPERTY(scale_settings_file,std::string,"Path to scale settings .xml file.")
	OpenSim_DECLARE_OPTIONAL_PROPERTY(generic_model_file,std::string, "Path to generic .osim file")
	
	
	OpenSim_DECLARE_PROPERTY(perform_inverse_kinematics,bool,"Perform Inverse Kinematics")
	OpenSim_DECLARE_OPTIONAL_PROPERTY(trc_kinematics_file, std::string, "Path to .trc file with marker kinematics")
	OpenSim_DECLARE_OPTIONAL_PROPERTY(ik_settings_file,std::string,"Path to Inverse Kinematics settings .xml file.")
	
	OpenSim_DECLARE_PROPERTY(verbose,int,"Level of debug information reported (0: low, 1: medium, 2: high)")
	OpenSim_DECLARE_PROPERTY(start_time, double, "First time step of COMAK simulation.")
	OpenSim_DECLARE_PROPERTY(end_time, double, "Last time step of COMAK simulation.")
	OpenSim_DECLARE_PROPERTY(time_step,double,"Time increment between steps in COMAK simulation.")

	OpenSim_DECLARE_LIST_PROPERTY(prescribed_coordinates,std::string,"List the Prescribed Coordinates in the model.")
	OpenSim_DECLARE_LIST_PROPERTY(primary_coordinates,std::string,"List the Primary Coordinates in the model.")
	OpenSim_DECLARE_LIST_PROPERTY(secondary_coordinates, std::string, "List the Secondary Coordinates in the model.")
	
	OpenSim_DECLARE_PROPERTY(equilibriate_secondary_coordinates_at_start, bool, "Perform a forward simulation to settle secondary coordinates into equilbrium at initial time step of COMAK.")
	OpenSim_DECLARE_PROPERTY(settle_time_step,double,"Integrator timestep for initializing forward simulation.")
	OpenSim_DECLARE_PROPERTY(settle_tolerance, double, "Integrator tolerance for initializing forward simulation.")
	OpenSim_DECLARE_PROPERTY(settle_threshold, double, "Maximum change in secondary coordinates between timesteps that defines equilibrium.")
	
	//=============================================================================
// METHODS
//=============================================================================    
  
    /**
    * Default constructor.
    */
    COMAKParameters();
    
    /**
    * Construct from file, and an optional GuiModel
    *
    * The object is constructed from the root element of the XML document.
    * The type of object is the tag name of the XML root element.
    *
    * @param aFileName File name of the document.
    * @param aUpdateFromXMLNode
    */
    COMAKParameters(const std::string &aFileName, bool aUpdateFromXMLNode = true);
    
    /**
    * Copy constructor.
    *
    * Copy constructors for all SimulationTools only copy the non-XML variable
    * members of the object; that is, the object's DOMnode and XMLDocument
    * are not copied but set to NULL.  This is because the object and all its 
    * derived classes need to establish the correct connection to the XML 
    * document nodes. Thus the object needs to reconstruct based on the XML 
    * document, not the values of the object's member variables.
    *
    * There are three proper ways to generate an XML document for a AbstractTool:
    *
    * 1) Construction based on XML file (@see AbstractTool(const char *aFileName)).
    * In this case, the XML document is created by parsing the XML file.
    *
    * 2) Construction by AbstractTool(const XMLDocument *aDocument).
    * This constructor explicitly requests construction based on an
    * XML document.  In this way the proper connection between an object's node
    * and the corresponding node within the XML document is established.
    * This constructor is a copy constructor of sorts because all essential
    * AbstractTool member variables should be held within the XML document.
    * The advantage of this style of construction is that nodes
    * within the XML document, such as comments that may not have any
    * associated AbstractTool member variable, are preserved.
    *
    * 3) A call to generateXMLDocument().
    * This method generates an XML document for the AbstractTool from scratch.
    * Only the essential document nodes are created (that is, nodes that
    * correspond directly to member variables.).
    *
    * @param aObject Object to be copied.
    * @see AbstractTool(const XMLDocument *aDocument)
    * @see AbstractTool(const char *aFileName)
    */
    COMAKParameters(const COMAKParameters &aObject);

private:
	void setNull();
	void constructProperties();

    //--------------------------------------------------------------------------
    // Members
    //--------------------------------------------------------------------------
public:

    

//=============================================================================
};  // END of class COMAKParameters

}; //namespace
//=============================================================================
//=============================================================================

#endif // _OPENSIM_COMAK_PARAMETERS_H_


