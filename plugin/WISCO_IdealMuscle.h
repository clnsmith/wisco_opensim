#ifndef OPENSIM_WISCO_IDEAL_MUSCLE_H_
#define OPENSIM_WISCO_IDEAL_MUSCLE_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  WISCO_IdealMuscle.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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


// INCLUDE
//#include "osimActuatorsDLL.h"
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Common/Function.h>
#include "osimPluginDLL.h"

namespace OpenSim {

//==============================================================================
//                          RIGID TENDON MUSCLE
//==============================================================================
/**
 * A class implementing a WISCO_IdealMuscle actuator with no states.
 * The path information for a WISCO_IdealMuscle is contained
 * in the base class, and the force-generating behavior should is defined in
 * this class. The force (muscle tension) assumes rigid tendon so that 
 * fiber-length and velocity are kinematics dependent and the force-length
 * force-velocity relationships are evaluated directly.
 * The control of this model is its activation. Force production is instantaneous  
 * with no excitation-to-activation dynamics and excitation=activation.
 *
 * @author Ajay Seth
 */
class OSIMPLUGIN_API WISCO_IdealMuscle : public Muscle {
OpenSim_DECLARE_CONCRETE_OBJECT(WISCO_IdealMuscle, Muscle);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
	OpenSim_DECLARE_PROPERTY(ignore_pennation,bool,
		"Flag to ignore pennation angle when computing muscle force")
	OpenSim_DECLARE_PROPERTY(applies_passive_force, bool,
		"Flag to turn on passive (parallel elastic) muscle force");
	OpenSim_DECLARE_PROPERTY(use_force_length_velocity_properties, bool,
		"Flag to turn on force length properties");
	OpenSim_DECLARE_PROPERTY(active_force_length_curve, Function,
        "Function representing active force-length behavior of muscle fibers");
    OpenSim_DECLARE_PROPERTY(passive_force_length_curve, Function,
        "Function representing passive force-length behavior of muscle fibers");
    OpenSim_DECLARE_PROPERTY(force_velocity_curve, Function,
        "Function representing force-velocity behavior of muscle fibers");

//=============================================================================
// Outputs
//=============================================================================
	OpenSim_DECLARE_LIST_OUTPUT(dynamic_quantities,double,getDynamicQuantities,SimTK::Stage::Dynamics)

	double getDynamicQuantities(const SimTK::State& state,
			const std::string& channel) const {

		if (channel == "activation") {
			return getControl(state);			
		}
		if(channel == "force") {
			return getActuation(state);
		}
		else {
			OPENSIM_THROW(Exception,"WISCO_IdealMuscle does not have a channel: " + channel)
		}
	}

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    WISCO_IdealMuscle();
	WISCO_IdealMuscle(const std::string& aName,
		double aMaxIsometricForce);
	WISCO_IdealMuscle(const std::string&    name,
                      double                maxIsometricForce,
                      double                optimalFiberLength,
                      double                tendonSlackLength,
                      double                pennationAngle);

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    /** activation level for this muscle */
    void setActivation(SimTK::State& s, double activation) const override {setExcitation(s, activation); }

protected:

    /** calculate muscle's length related values such fiber and tendon lengths,
        normalized lengths, pennation angle, etc... */
    void calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli) const override;

    /** calculate muscle's velocity related values such fiber and tendon velocities,
        normalized velocities, pennation angular velocity, etc... */
    void  calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi) const override;

    /** calculate muscle's active and passive force-length, force-velocity, 
        tendon force, relationships and their related values */
    void  calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi) const override;

    /** calculate muscle's fiber and tendon potential energy */
    void calcMusclePotentialEnergyInfo(const SimTK::State& s,
        MusclePotentialEnergyInfo& mpei) const override;

    /** compute initial fiber length (velocity) such that muscle fiber and tendon are 
        in static equilibrium and update the state */
    void computeInitialFiberEquilibrium(SimTK::State& s) const override {}

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
    double computeActuation( const SimTK::State& s ) const override;
    double computeIsometricForce(SimTK::State& s, double activation) const;
    
	void extendFinalizeFromProperties() override;

private:
    void setNull();
    void constructProperties();

protected:

//==============================================================================
};  // END of class WISCO_IdealMuscle
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_WISCO_IDEAL_MUSCLE_H_


