/* ---------------------------------------------------------------------------- *
 *                             WISCO_Ligament.cpp                                   *
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
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Simulation/Model/GeometryPath.h>
#include <OpenSim/Simulation/Model/PointForceDirection.h>
#include <OpenSim/Simulation/Model/Ligament.h>
#include <OpenSim/OpenSim.h>
#include "WISCO_Ligament.h"




//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;
using SimTK::Vec3;

static const Vec3 DefaultLigamentColor(.9,.9,.9); // mostly white 


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor
 */
WISCO_Ligament::WISCO_Ligament() : Force()
{
    constructProperties();
	setNull();
}

//_____________________________________________________________________________



//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of UWLigament to their null values.
 */
void WISCO_Ligament::setNull()
{
	setAuthors("Colin Smith, Michael Vignos, Rachel Lenhart, Darryl Thelen");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void WISCO_Ligament::constructProperties()
{
	constructProperty_GeometryPath(GeometryPath());
	constructProperty_linear_stiffness(0.0);
	constructProperty_ligament_transition_strain(0.0);
	constructProperty_normalized_damping_coefficient(0.0);
	constructProperty_defining_length_parameter("reference_strain");
	constructProperty_reference_force(0.0);
	constructProperty_reference_strain(0.0);
	constructProperty_slack_length(0.0);
}

void WISCO_Ligament::extendFinalizeFromProperties()
{
	Super::extendFinalizeFromProperties();

	GeometryPath& path = upd_GeometryPath();
	path.setDefaultColor(DefaultLigamentColor);
}

void WISCO_Ligament::extendRealizeDynamics(const SimTK::State& state) const {
	Super::extendRealizeDynamics(state); 

	if (appliesForce(state)) {
		const SimTK::Vec3 color = computePathColor(state);
		if (!color.isNaN())
			get_GeometryPath().setColor(state, color);
	}
}




void WISCO_Ligament::extendAddToSystem(SimTK::MultibodySystem& system) const
{
	Super::extendAddToSystem(system);
	
	addCacheVariable<double>("tension", 0.0, SimTK::Stage::Velocity);
	addCacheVariable<double>("strain", 0.0, SimTK::Stage::Velocity);

    //Slack Length
    //This is computed after the model is initialized and posed according to 
    //the default coordinates, then unalted for the rest of the simulation
    addCacheVariable<double>("slack_length", 0.0, SimTK::Stage::LowestRuntime); 
}

void WISCO_Ligament::extendInitStateFromProperties(SimTK::State &state) const
{
    
    Super::extendInitStateFromProperties(state);
    
	if (get_defining_length_parameter() == "reference_strain") {
		//Ensure that this is the default state
		auto coord_set = getModel().getCoordinateSet();
		auto q_vals = state.getQ();

		for (int i = 0; i < coord_set.getSize(); ++i) {
			if (q_vals[i] != coord_set[i].get_default_value()) {
				std::string msg = "Exception: Coordinate: " + coord_set[i].getName()
					+ " was not set to default value when ligament slack length computed.";
			}

		}

		getModel().realizePosition(state);

		//Compute Slack Length
		auto referenceLength = get_GeometryPath().getLength(state);

		auto slackLength = referenceLength / (1.0 + get_reference_strain());
		setCacheVariableValue<double>(state, "slack_length", slackLength);
	}
	else if (get_defining_length_parameter() == "reference_force") {
		//Ensure that this is the default state
		auto coord_set = getModel().getCoordinateSet();
		auto q_vals = state.getQ();

		for (int i = 0; i < coord_set.getSize(); ++i) {
			if (q_vals[i] != coord_set[i].get_default_value()) {
				std::string msg = "Exception: Coordinate: " + coord_set[i].getName()
					+ " was not set to default value when ligament slack length computed.";
				OPENSIM_THROW(Exception, msg);
			}

		}

		getModel().realizePosition(state);

		//Check reference force is positive
		double ref_force = get_reference_force();
		double lin_stiff = get_linear_stiffness();
		double trans_strain = get_ligament_transition_strain();
		
		double trans_force = trans_strain*lin_stiff;
		double ref_strain;
		
		//Compute Reference Strain
		if (ref_force < 0.0) {
			OPENSIM_THROW(Exception, "WISCO_ElasticLigament::ERROR Reference Force was negative.");
		}
		else if (ref_force > trans_force) {
			ref_strain = ref_force / lin_stiff + trans_strain / 2;
		}
		else {
			ref_strain = sqrt(2 * trans_strain*ref_force / lin_stiff);
		}

		//Compute Slack Length
		double ref_length = get_GeometryPath().getLength(state);
		double slack_length = ref_length / (1.0 + ref_strain);

		setCacheVariableValue<double>(state, "slack_length", slack_length);
	}
	else if(get_defining_length_parameter() == "slack_length") {
		setCacheVariableValue<double>(state, "slack_length", get_slack_length());
	}
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the length of the ligament. This is a convenience function that passes
 * the request on to the ligament path.
 *
 * @return Current length of the ligament path.
 */
double WISCO_Ligament::getLength(const SimTK::State& s) const
{
	return get_GeometryPath().getLength(s);
}

//-----------------------------------------------------------------------------
// LENGTHENGING SPEED
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the lengthening speed of the ligament.
 *
 * @return Current lengthening speed of the ligament path.
 */
double WISCO_Ligament::getLengtheningSpeed(const SimTK::State& s) const
{
	return get_GeometryPath().getLengtheningSpeed(s);
}



//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform computations that need to happen before the ligament is scaled.
 * For this object, that entails calculating and storing the
 * length in the current body position.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void WISCO_Ligament::preScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	upd_GeometryPath().preScale(s, aScaleSet);
}

//_____________________________________________________________________________
/**
 * Scale the ligament.
 *
 * @param aScaleSet XYZ scale factors for the bodies
 * @return Whether or not the ligament was scaled successfully
 */
void WISCO_Ligament::scale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	upd_GeometryPath().scale(s, aScaleSet);
}

//_____________________________________________________________________________
/**
 * Perform computations that need to happen after the ligament is scaled.
 * For this object, that entails comparing the length before and after scaling,
 * and scaling the resting length a proportional amount.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void WISCO_Ligament::postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	GeometryPath& path = upd_GeometryPath();

	path.postScale(s, aScaleSet);

	if (path.getPreScaleLength(s) > 0.0)
	{
		double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);

		// Scale resting length by the same amount as the change in
		// total ligament length (in the current body position).
		//restingLength *= scaleFactor;

		path.setPreScaleLength(s, 0.0);
	}
}

const double& WISCO_Ligament::getTension(const SimTK::State& s) const
{
	return getCacheVariableValue<double>(s, "tension"); 
}

//=============================================================================
// COMPUTATION
//=============================================================================
/**
 * Compute the moment-arm of this muscle about a coordinate.
 */
double WISCO_Ligament::computeMomentArm(const SimTK::State& s, Coordinate& aCoord) const
{
	return get_GeometryPath().computeMomentArm(s, aCoord);
}

void WISCO_Ligament::computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const
{
	const GeometryPath& path = get_GeometryPath();
	const double& referenceStrain = get_reference_strain();
	const double& linearStiffness = get_linear_stiffness();
	const double& ligamentTransitionStrain = get_ligament_transition_strain();
	const double& normalizedDampingCoefficient = get_normalized_damping_coefficient();

    double slackLength = getCacheVariableValue<double>(s, "slack_length");

	

	if (path.getLength(s) <= slackLength){
		setCacheVariableValue<double>(s, "tension", 0.0);
		return;
	}

	// Compute length
	double length = path.getLength(s);

	// Compute strain
	double strain = (length*(1 + referenceStrain) - slackLength*(1+referenceStrain))/(slackLength*(1+referenceStrain));

	// Compute strain rate
	double strainRate = path.getLengtheningSpeed(s)/slackLength;
	
	// evaluate force
    double forceSpring = 0.0;

	if (strain >= (ligamentTransitionStrain))
		forceSpring = linearStiffness*(strain - (ligamentTransitionStrain/2));
	else if ((strain > 0) && (strain < (ligamentTransitionStrain)))
		forceSpring = 0.25*linearStiffness*strain*strain/(ligamentTransitionStrain/2);
	else
		forceSpring =0.0;

	// evaluate damping force
    double forceDamping = 0.0;
	forceDamping = linearStiffness*normalizedDampingCoefficient*strainRate;

	// total force
	double forceTotal = forceSpring + forceDamping;

	// make sure the ligament is only acting in tension
	if (forceTotal < 0.0)
		forceTotal = 0.0;

	OpenSim::Array<PointForceDirection*> PFDs;
	path.getPointForceDirections(s, &PFDs);

	for (int i=0; i < PFDs.getSize(); i++) {
		applyForceToPoint(s, PFDs[i]->frame(), PFDs[i]->point(), 
                          forceTotal*PFDs[i]->direction(), bodyForces);
	}
	for(int i=0; i < PFDs.getSize(); i++)
		delete PFDs[i];
}


//------------------------------------------------------------------------------
//                          COMPUTE PATH COLOR
//------------------------------------------------------------------------------
// This is the Ligament base class implementation for choosing the path
// color. Derived classes can override this with something meaningful.
SimTK::Vec3 WISCO_Ligament::computePathColor(const SimTK::State& state) const {
	return SimTK::Vec3(SimTK::NaN);
}

//-----------------------------------------------------------------------------
	// Reporting
	//-----------------------------------------------------------------------------
	/** 
	 * Methods to query a Force for the value actually applied during simulation
	 * The names of the quantities (column labels) is returned by this first function
	 * getRecordLabels()
	 */
	OpenSim::Array<std::string> WISCO_Ligament::getRecordLabels() const {
		OpenSim::Array<std::string> labels("");
		labels.append(getName()+".force_spring");
		labels.append(getName()+".force_damping");
		labels.append(getName()+".force_total");
		labels.append(getName()+".length");
		labels.append(getName()+".lengthening_speed");
		return labels;
	}
	
	/**
	 * Given SimTK::State object extract all the values necessary to report forces, application location
	 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
	 */
	OpenSim::Array<double> WISCO_Ligament::getRecordValues(const SimTK::State& s) const {
		OpenSim::Array<double> values(1);

		const GeometryPath& path = get_GeometryPath();
		const double& referenceStrain = get_reference_strain();
		const double& linearStiffness = get_linear_stiffness();
		const double& ligamentTransitionStrain = get_ligament_transition_strain();
		const double& normalizedDampingCoefficient = get_normalized_damping_coefficient();

		auto slackLength = getCacheVariableValue<double>(s, "slack_length");
		

		// Compute strain
		double length = path.getLength(s);
        double strain = (length*(1 + referenceStrain) - slackLength*(1 + referenceStrain)) / (slackLength*(1 + referenceStrain));

		// Compute strain rate
		double strainRate = path.getLengtheningSpeed(s)/slackLength;
	
		// Compute spring force
		//----------------------
        double forceSpring = 0.0;
        if (strain >= (ligamentTransitionStrain))
			forceSpring = linearStiffness*(strain - (ligamentTransitionStrain/2));
		else if ((strain > 0) && (strain < (ligamentTransitionStrain)))
			forceSpring = 0.25*linearStiffness*strain*strain/(ligamentTransitionStrain/2);
		else
			forceSpring =0.0;

		// Compute damping force
		//----------------------
        double forceDamping = 0.0;
		
		if (strain > 0.0) {
			if (strainRate <= 0.0)
				forceDamping = linearStiffness*normalizedDampingCoefficient*strainRate;
			else
				forceDamping = -linearStiffness*normalizedDampingCoefficient*strainRate;
		}
		else {
			forceDamping = 0;
		}

		//Use smooth step to transition damping on when strain becomes positive
		StepFunction step = StepFunction(0, 0.01, 0, 1);
		SimTK::Vector strainVec(1);

		strainVec(0) = strain;

		double dampingScale = step.calcValue(strainVec);

		forceDamping = forceDamping*dampingScale;

		// total force
		double forceTotal = forceSpring + forceDamping;

		// make sure ligament is only acting in tension
		if (forceTotal < 0.0)
			forceTotal = 0.0;

		// Report values
		values.append(forceSpring);
		values.append(forceDamping);
		values.append(forceTotal);
		values.append(length);
		values.append(path.getLengtheningSpeed(s));
		return values;
	}
