#ifndef _OPENSIM_WISCO_LIGAMENT_PLUGIN_H_
#define _OPENSIM_WISCO_LIGAMENT_PLUGIN_H_
/* ---------------------------------------------------------------------------- *
 *                             UWLigament.h                                     *
 * ---------------------------------------------------------------------------- *
 * This ligament model was based on the ligament model found in 			    *
 * Blankevoort L and Huiskes R (1991). Ligament-Bone Interaction in a           *
 * Three-Dimensional Model of the Knee. Journal of Biomechanical                *
 * Engineering. 113:263-269.                                                    *
 * 																				*
 * Important variables in the model: 											*
 * linear_stiffness: Stiffness representing the slope of the linear portion of  *
 * 				     the force-strain curve. 								    *
 * 																				*
 * ligament_transition_strain: Strain at which the force-strain relationship of	*
 * 							   the ligament transitions from quadratic to 		*
 *							   linear. Typically defined to be 0.06 in the 		*
 *						 	   literature. 										*
 *																				*
 * reference_strain: Strain in the ligament when the joint is in the reference  *
 * 				     position. The reference position is full extension for the *
 * 				     knee. 														*
 * 																				*
 * reference_length: Length of the ligament when the joint is in the reference 	*
 * 				     position. 													*
 * 																				*
 * normalized_damping_coefficient: Coefficient for normalized damping of the 	*
 * 								 ligament. Be aware, this is not the same as a 	*
 * 								 standard damping coefficient.  				*
 * 								 See UWLigament::computeForce in UWLigament.cpp *
 * 								 for the definition of the damping force. 		*
 *  																			*
 *                                                                              *
 * Author(s): Michael Vignos, Colin Smith, Rachel Lenhart, Darryl Thelen        *
 *                                                                              *
 *
 *																			    *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may      *
 * not use this file except in compliance with the License. You may obtain a    *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.           *
 *                                                                              *
 * Unless required by applicable law or agreed to in writing, software          *
 * distributed under the License is distributed on an "AS IS" BASIS,            *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.     *
 * See the License for the specific language governing permissions and          *
 * limitations under the License.                                               *
 *                                                                              *
 * ---------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
// Headers define the various property types that OpenSim objects can read
#include <string>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/Model/GeometryPath.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/Model.h>
//#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include "osimPluginDLL.h"

//=============================================================================
//=============================================================================
/*
 * A class template for writing a custom Force plugin.
 * Applies a body drag force to a given OpenSim Body at
 * it's center of mass location.
 *
 */
namespace OpenSim {


//=============================================================================
//=============================================================================
/**
 * A class implementing a ligament. The path of the ligament is
 * stored in a GeometryPath object.
 */
class OSIMPLUGIN_API WISCO_Ligament : public Force  {
//class WISCO_Ligament : public Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(WISCO_Ligament, Force)
public:
//=============================================================================
// PROPERTIES
//=============================================================================

	OpenSim_DECLARE_UNNAMED_PROPERTY(GeometryPath,
		"The set of points defining the path of the ligament");
	OpenSim_DECLARE_PROPERTY(linear_stiffness, double,
		"Slope of the linear portion of the force-strain curve of ligament");
	OpenSim_DECLARE_PROPERTY(transition_strain, double,
		"Strain at which ligament force-strain curve transitions from"
		"quadratic to linear. Commonly 0.06 in literature.");
	OpenSim_DECLARE_PROPERTY(normalized_damping_coefficient, double,
		"Coefficient for normalized damping of ligament");
	OpenSim_DECLARE_PROPERTY(defining_slack_length_property, std::string,
		"Options: 'slack_length','reference_strain','reference_force' property to compute slack length."
		"NOTE: other properties will be ignored.");
	OpenSim_DECLARE_OPTIONAL_PROPERTY(reference_strain, double,
		"Strain of ligament when model is posed a default coordinates.");
	OpenSim_DECLARE_OPTIONAL_PROPERTY(reference_force, double,
		"Force in ligament when model is in reference pose.")
	OpenSim_DECLARE_OPTIONAL_PROPERTY(slack_length, double,
		"Length at which ligament begins developing tension");
//=============================================================================
// OUTPUTS
//=============================================================================

	OpenSim_DECLARE_LIST_OUTPUT(dynamic_quantities, double, getDynamicQuantities, SimTK::Stage::Dynamics);



//=============================================================================
// METHODS
//=============================================================================
public:
	// Default Constructor
	WISCO_Ligament();
	WISCO_Ligament(PhysicalFrame& frame1, SimTK::Vec3 point1,
		PhysicalFrame& frame2, SimTK::Vec3 point2);
	WISCO_Ligament(PhysicalFrame& frame1, SimTK::Vec3 point1,
		PhysicalFrame& frame2, SimTK::Vec3 point2,
		double linear_stiffness, double reference_strain);

	//-------------------------------------------------------------------------
	//Outputs
	//-------------------------------------------------------------------------
	double getDynamicQuantities(const SimTK::State& state,
		const std::string& channel) const {
		if (channel == "force_spring") {
			return getCacheVariableValue<double>(state, "force_spring");
		}
		else if (channel == "force_damping") {
			return getCacheVariableValue<double>(state, "force_damping");
		}
		else if (channel == "force_total") {
			return getCacheVariableValue<double>(state, "force_total");
		}
		else if (channel == "length") {
			return getCacheVariableValue<double>(state, "length");
		}
		else if (channel == "lengthening_speed") {
			return getCacheVariableValue<double>(state, "lengthening_speed");
		}
		else if (channel == "strain") {
			return getCacheVariableValue<double>(state, "strain");
		}
		else if (channel == "strain_rate") {
			return getCacheVariableValue<double>(state, "strain_rate");
		}
	}

	//--------------------------------------------------------------------------
	// GET
	//--------------------------------------------------------------------------
	// Properties
	virtual bool hasGeometryPath() const { return true;};
	virtual double getLength(const SimTK::State& s) const;
	virtual double getLengtheningSpeed(const SimTK::State& s) const;


	// computed variables
	double getTension(const SimTK::State& s) const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double computeMomentArm(const SimTK::State& s, Coordinate& aCoord) const;
	virtual void computeForce(const SimTK::State& s,
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
							  SimTK::Vector& generalizedForces) const;
	double computePotentialEnergy(const SimTK::State& state) const override;
	//--------------------------------------------------------------------------
	// SCALE
	//--------------------------------------------------------------------------
	virtual void preScale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual void scale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual void postScale(const SimTK::State& s, const ScaleSet& aScaleSet);

	//-----------------------------------------------------------------------------
	// REPORTING
	//-----------------------------------------------------------------------------
	virtual OpenSim::Array<std::string> getRecordLabels() const ;
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const ;

	void equilibriateSlackLengthProperties(const SimTK::State& state);

	double computeReferenceLength(SimTK::State state) const;
	double computeSlackLength(const SimTK::State& state, SimTK::String property_name, double property_value) const;
	double computeReferenceStrain(const SimTK::State& state, SimTK::String property_name, double property_value) const;
	double computeReferenceForce(const SimTK::State& state, SimTK::String property_name, double property_value) const;
	void printPropertiesToConsole();

protected:
	/** Override this method if you would like to calculate a color for use
    when the Ligament's path is displayed in the visualizer. You do not have
    to invoke the base class ("Super") method, just replace it completely. This
    method will be invoked during realizeDynamics() so the supplied a state has
    already been realized through Stage::Velocity and you can access time,
    position, and velocity dependent quantities. You must not attempt to
    realize the passed-in \a state any further since we are already in the
    middle of realizing here. Return SimTK::Vec3(SimTK::NaN) if you want to
    leave the color unchanged (that's what the base class implementation does).

    @param[in] state
        A SimTK::State already realized through Stage::Velocity. Do not
        attempt to realize it any further.
    @returns
        The desired color for the path as an RGB vector with each
        component ranging from 0 to 1, or NaN to indicate that the color
        should not be changed. **/
    virtual SimTK::Vec3 computePathColor(const SimTK::State& state) const;

	void extendFinalizeFromProperties() override;
	void extendAddToSystem(SimTK::MultibodySystem& system) const override;
	void extendRealizeDynamics(const SimTK::State& state) const override;
    void extendInitStateFromProperties(SimTK::State &state) const override;
	void extendSetPropertiesFromState(const SimTK::State & 	state) override;

private:
	void setNull();
	void constructProperties();

//=============================================================================
};	// END of class UWLigament
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // _OPENSIM_WISCO_LIGAMENT_PLUGIN_H_


