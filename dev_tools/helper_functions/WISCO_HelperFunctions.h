#ifndef _WISCO_HELPER_FUNCTIONS_h_
#define _WISCO_HELPER_FUNCTIONS_h_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Kinematics.h                           *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/Analysis.h>

//=============================================================================
//=============================================================================
//namespace OpenSim {

#ifdef _WIN32
#  ifdef WISCO_API_EXPORTS
#    define WISCO_API __declspec(dllexport)
#  else
#    define WISCO_API __declspec(dllimport)
#  endif
#else
#  define WISCO_API
#endif

//=============================================================================
//STRING TOOLS
//=============================================================================
	
/** Split string at delimiter
*/
std::vector<std::string> split_string(std::string s, std::string delimiter);

WISCO_API bool contains_string(std::vector<std::string> s_vector, std::string s);
WISCO_API bool contains_string(std::vector<std::string> s_vector, std::string s, int& index);

//}; //namespace
#endif // #ifndef __WISCO_HELPER_FUNCTIONS_h__