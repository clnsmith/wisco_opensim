/* -------------------------------------------------------------------------- *
 *                         OpenSim:  AbstractTool.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/XMLDocument.h>
#include "COMAKParameters.h"

using namespace OpenSim;
using namespace SimTK;



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================

//_____________________________________________________________________________
/**
 * Default constructor.
 */
COMAKParameters::COMAKParameters()
{
    setNull();
}

/**
 * Construct from file, and an optional GuiModel
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
COMAKParameters::COMAKParameters(const std::string &aFileName, bool aUpdateFromXMLNode):
    Object(aFileName, false)   
{

    setNull();
    if(aUpdateFromXMLNode) updateFromXMLDocument();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all SimulationTools only copy the non-XML variable
 * members of the object; that is, the object's DOMnode and XMLDocument
 * are not copied but set to NULL.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
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
 * @param aTool Object to be copied.
 * @see AbstractTool(const XMLDocument *aDocument)
 * @see AbstractTool(const char *aFileName)
 * @see generateXMLDocument()
 */
COMAKParameters::COMAKParameters(const COMAKParameters &aTool):
    Object(aTool)
{
    setNull();
    *this = aTool;
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void COMAKParameters::setNull()
{
	constructProperties();
	   
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void COMAKParameters::constructProperties()
{
	
	constructProperty_perform_scale(false);
	constructProperty_scale_settings_file("");
	constructProperty_generic_model_file("");

	constructProperty_perform_inverse_kinematics(false);
	constructProperty_ik_settings_file("");
	constructProperty_trc_kinematics_file("");
	constructProperty_ik_motion_file("");

	constructProperty_results_motion_file("");
	constructProperty_model_file("");

	constructProperty_verbose(0);
	constructProperty_start_time(-1);
	constructProperty_end_time(-1);
	constructProperty_time_step(0.01);

	constructProperty_prescribed_coordinates();
	constructProperty_primary_coordinates();
	constructProperty_secondary_coordinates();

	constructProperty_equilibriate_secondary_coordinates_at_start(true);
	constructProperty_settle_time_step(0.00001);
	constructProperty_settle_threshold(0.00001);
	constructProperty_settle_tolerance(0.00001);

}





