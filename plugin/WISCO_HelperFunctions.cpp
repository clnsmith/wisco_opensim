/* -------------------------------------------------------------------------- *
 *                          OpenSim:  WISCO_ContactAnalysis.cpp               *
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
#include "WISCO_HelperFunctions.h"
#include <algorithm>
#include <vector>

using namespace OpenSim;

//=============================================================================
// String Tools
//=============================================================================
std::vector<std::string> split_string(std::string s, std::string delimiter)
{
	std::vector<std::string> split_s;

	size_t pos = 0;

	while ((pos = s.find(delimiter)) != std::string::npos) {

		split_s.push_back(s.substr(0, pos));

		s.erase(0, pos + delimiter.length());
	}
	split_s.push_back(s.substr(0, pos));

	return split_s;
}


bool contains_string(std::vector<std::string> s_vector, std::string s)
{
	int index;
	return contains_string(s_vector, s, index);
}

bool contains_string(std::vector<std::string> s_vector, std::string s, int& index)
{
	bool found;

	auto it = std::find(s_vector.begin(), s_vector.end(), s);

	if (it == s_vector.end()) {
		found = false;
		index = -1;
	}
	else {
		found = true;
		index = std::distance(s_vector.begin(), it);
	}
	return found;
}
