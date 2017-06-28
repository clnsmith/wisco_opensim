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
WISCO_Ligament::WISCO_Ligament(PhysicalFrame& frame1, Vec3 point1,
	PhysicalFrame& frame2, Vec3 point2) : WISCO_Ligament()
{
	upd_GeometryPath().appendNewPathPoint("p1", frame1, point1);
	upd_GeometryPath().appendNewPathPoint("p2", frame2, point2);
}

WISCO_Ligament::WISCO_Ligament(PhysicalFrame& frame1, Vec3 point1,
	PhysicalFrame& frame2, Vec3 point2,
	double linear_stiffness, double reference_strain) :
	WISCO_Ligament(frame1, point1, frame2, point2)
{
	set_linear_stiffness(linear_stiffness);
	set_reference_strain(reference_strain);
	set_defining_slack_length_property("reference_strain");
	set_transition_strain(0.06);
	set_normalized_damping_coefficient(0.003);
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
	constructProperty_transition_strain(0.0);
	constructProperty_normalized_damping_coefficient(0.0);
	constructProperty_defining_slack_length_property("reference_strain");
	constructProperty_reference_force(0.0);
	constructProperty_reference_strain(0.0);
	constructProperty_slack_length(0.0);
}

void WISCO_Ligament::extendFinalizeFromProperties()
{
	Super::extendFinalizeFromProperties();

	GeometryPath& path = upd_GeometryPath();
	path.setDefaultColor(DefaultLigamentColor);

	//Add channels to output list
	auto& dyn_quan = updOutput("dynamic_quantities");
	dyn_quan.addChannel("force_spring");
	dyn_quan.addChannel("force_damping");
	dyn_quan.addChannel("force_total");
	dyn_quan.addChannel("length");
	dyn_quan.addChannel("lengthening_speed");
	dyn_quan.addChannel("strain");
	dyn_quan.addChannel("strain_rate");


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


    //Slack Length
    //This is computed after the model is initialized and posed according to
    //the default coordinates, then unalted for the rest of the simulation
    addCacheVariable<double>("slack_length", 0.0, SimTK::Stage::LowestRuntime);
	addCacheVariable<double>("reference_length", 0.0, SimTK::Stage::LowestRuntime);

	addCacheVariable<double>("force_spring",0.0, SimTK::Stage::Dynamics);
	addCacheVariable<double>("force_damping", 0.0, SimTK::Stage::Dynamics);
	addCacheVariable<double>("force_total", 0.0, SimTK::Stage::Dynamics);
	addCacheVariable<double>("length", 0.0, SimTK::Stage::Dynamics);
	addCacheVariable<double>("lengthening_speed", 0.0, SimTK::Stage::Dynamics);
	addCacheVariable<double>("strain", 0.0, SimTK::Stage::Dynamics);
	addCacheVariable<double>("strain_rate", 0.0, SimTK::Stage::Dynamics);
}

void WISCO_Ligament::extendInitStateFromProperties(SimTK::State &state) const
{

    Super::extendInitStateFromProperties(state);
	double ref_length = computeReferenceLength(state);
	setCacheVariableValue<double>(state, "reference_length", ref_length);

	double slack_length;
	if (get_defining_slack_length_property() == "reference_strain") {
		slack_length = computeSlackLength(state,"reference_strain",get_reference_strain());
	}
	else if (get_defining_slack_length_property() == "reference_force") {
		slack_length = computeSlackLength(state, "reference_force",get_reference_force());
	}
	else if (get_defining_slack_length_property() == "slack_length") {
		slack_length = get_slack_length();
	}

	setCacheVariableValue<double>(state, "slack_length", slack_length);
}

void WISCO_Ligament::extendSetPropertiesFromState(const SimTK::State& state) {
	Super::extendSetPropertiesFromState(state);
	/*
	double ref_length = computeReferenceLength(state);
	setCacheVariableValue<double>(state, "reference_length", ref_length);

	double slack_length;
	if (get_defining_slack_length_property() == "reference_strain") {
		double slack_length = computeSlackLength(state, "reference_strain", get_reference_strain());
	}
	else if (get_defining_slack_length_property() == "reference_force") {
		slack_length = computeSlackLength(state, "reference_force", get_reference_force());
	}
	else if (get_defining_slack_length_property() == "slack_length") {
		slack_length = get_slack_length();
	}

	setCacheVariableValue<double>(state, "slack_length", slack_length);

	equilibriateSlackLengthProperties(state);*/
}

void WISCO_Ligament::equilibriateSlackLengthProperties(const SimTK::State& state) {
	double& slack_length = upd_slack_length();
	double& ref_strain = upd_reference_strain();
	double& ref_force = upd_reference_force();

	double linear_stiff = get_linear_stiffness();

	std::string defining_prop = SimTK::String::toLower(upd_defining_slack_length_property());

	if (defining_prop == "reference_strain") {
		slack_length = computeSlackLength(state, "reference_strain", ref_strain);
		ref_force = computeReferenceForce(state, "reference_strain", ref_strain);
	}

	else if (defining_prop == "reference_force") {
		slack_length = computeSlackLength(state, "reference_force",ref_force);
		ref_strain = computeReferenceStrain(state, "reference_force", ref_force);
	}

	else if (defining_prop == "slack_length") {
		ref_strain = computeReferenceStrain(state, "slack_length", slack_length);
		ref_force = computeReferenceForce(state, "slack_length", slack_length);
	}

	double ref_length = computeReferenceLength(state);
	setCacheVariableValue<double>(state, "slack_length", slack_length);
	setCacheVariableValue<double>(state, "reference_length", ref_length);


}

double WISCO_Ligament::computeReferenceLength(SimTK::State state) const {

	//Check if it has already been computed
	if (isCacheVariableValid(state, "reference_length")) {
		return getCacheVariableValue<double>(state, "reference_length");
	}

	SimTK::Stage stage = state.getSystemStage();

	if (stage > SimTK::Stage::Instance) {
		//Pose model to reference (default) coordinates
		for (Coordinate coord : getModel().getComponentList<Coordinate>()) {
			coord.setValue(state, coord.getDefaultValue());
		}
	}
	/*CoordinateSet coordset = getModel().getCoordinateSet();
	for (int i = 0; i < coordset.getSize();++i) {
		Coordinate coord = coordset.get(i);
		//coord.setValue(state, coord.get_default_value());
		coord.getValue(state);
	}*/
	getModel().realizePosition(state);

	return get_GeometryPath().getLength(state);
}

double WISCO_Ligament::computeSlackLength(const SimTK::State& state, SimTK::String property_name, double property_value) const {
	double slack_length;
	double ref_length = computeReferenceLength(state);

	if (property_name.toLower() == "reference_strain") {
		slack_length = ref_length / (1.0 + property_value);
	}
	else if (property_name.toLower() == "reference_force") {
		double ref_strain = computeReferenceStrain(state, "reference_force", property_value);
		slack_length = computeSlackLength(state, "reference_strain", ref_strain);
	}
	return slack_length;
}

double WISCO_Ligament::computeReferenceStrain(const SimTK::State& state, SimTK::String property_name, double property_value) const {
	double ref_strain;
	double lin_stiff = get_linear_stiffness();
	double trans_strain = get_transition_strain();
	double ref_length = computeReferenceLength(state);

	if (property_name.toLower() == "reference_force") {
		double trans_force = trans_strain*lin_stiff;

		if (property_value < 0.0) {
			OPENSIM_THROW(Exception, "ERROR: Input Reference Force was negative.");
		}
		else if (property_value > trans_force) {
			ref_strain = property_value / lin_stiff + trans_strain / 2;
		}
		else {
			ref_strain = sqrt(2 * trans_strain* property_value / lin_stiff);
		}
	}
	else if (property_name.toLower() == "slack_length") {
		double ref_strain = ref_length / property_value - 1;
	}

}

double WISCO_Ligament::computeReferenceForce(const SimTK::State& state, SimTK::String property_name, double property_value) const {
	double trans_strain = get_transition_strain();
	double lin_stiff = get_linear_stiffness();
	double ref_length = computeReferenceLength(state);
	double ref_force;

	if (property_name.toLower() == "reference_strain") {
		if (property_value < 0) {
			ref_force = 0;
		}
		else if (property_value > trans_strain) {
			ref_force = (property_value - trans_strain / 2)*lin_stiff;
		}
		else {
			ref_force = pow(property_value, 2)*lin_stiff / (2 * trans_strain);
		}
	}
	else if (property_name.toLower() == "slack_length") {
		double ref_strain = ref_length / property_value - 1;
		ref_force = computeReferenceForce(state, "reference_strain", ref_strain);
	}

	return ref_force;
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

double WISCO_Ligament::getTension(const SimTK::State& s) const
{
	if (get_appliesForce()) {
		return getCacheVariableValue<double>(s, "force_total");
	}
	else
		return 0.0;
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
	const double& ligamentTransitionStrain = get_transition_strain();
	const double& normalizedDampingCoefficient = get_normalized_damping_coefficient();
	std::string name = getName();
    double slackLength = getCacheVariableValue<double>(s, "slack_length");

	double length = path.getLength(s);
	double lengthening_speed = path.getLengtheningSpeed(s);

	if (length <= slackLength){
		setCacheVariableValue<double>(s,"force_spring", 0.0);
		setCacheVariableValue<double>(s,"force_damping", 0.0);
		setCacheVariableValue<double>(s,"force_total", 0.0);
		setCacheVariableValue<double>(s,"length", length);
		setCacheVariableValue<double>(s,"lengthening_speed", lengthening_speed);
		setCacheVariableValue<double>(s, "strain", 0.0);
		setCacheVariableValue<double>(s,"strain_rate", 0.0);
		return;
	}

	double strain = (length - slackLength)/slackLength;
	double strain_rate = lengthening_speed /slackLength;

	// evaluate force
    double force_spring = 0.0;

	if (strain >= (ligamentTransitionStrain))
		force_spring = linearStiffness*(strain - (ligamentTransitionStrain/2));
	else if ((strain > 0) && (strain < (ligamentTransitionStrain)))
		force_spring = 0.25*linearStiffness*strain*strain/(ligamentTransitionStrain/2);
	else
		force_spring =0.0;

	// Calculate Damping Force
	double force_damping = 0.0;
	if (strain > 0) {
		force_damping = linearStiffness*normalizedDampingCoefficient*strain_rate;
	}
	else {
		force_damping = 0.0;
	}

	//Phase-out damping as strain goes to zero with smooth-step function
	SimTK::Function::Step step(0, 1, 0, 0.01);
	SimTK::Vector in_vec(1);
	in_vec = strain;
	force_damping = force_damping*step.calcValue(in_vec);

	// total force
	double force_total = force_spring + force_damping;

	// make sure the ligament is only acting in tension
	if (force_total < 0.0)
		force_total = 0.0;

	setCacheVariableValue<double>(s,"force_spring", force_spring);
	setCacheVariableValue<double>(s,"force_damping", force_damping);
	setCacheVariableValue<double>(s,"force_total", force_total);
	setCacheVariableValue<double>(s,"length", length);
	setCacheVariableValue<double>(s,"lengthening_speed", lengthening_speed);
	setCacheVariableValue<double>(s,"strain", strain);
	setCacheVariableValue<double>(s,"strain_rate", strain_rate);


	OpenSim::Array<PointForceDirection*> PFDs;
	path.getPointForceDirections(s, &PFDs);

	for (int i=0; i < PFDs.getSize(); i++) {
		applyForceToPoint(s, PFDs[i]->frame(), PFDs[i]->point(),
                          force_total*PFDs[i]->direction(), bodyForces);
	}
	for(int i=0; i < PFDs.getSize(); i++)
		delete PFDs[i];
}

double WISCO_Ligament::computePotentialEnergy(const SimTK::State& state) const {
	double strain = getCacheVariableValue<double>(state, "strain");
	double lin_stiff = get_linear_stiffness();
	double trans_strain = get_transition_strain();
	double slack_len = get_slack_length();

	if (strain < trans_strain) {
		return 1 / 6 * lin_stiff / trans_strain*pow(strain, 3);
	}
	else {
		return 1 / 6 * lin_stiff / trans_strain*pow(trans_strain, 3)+1/2*lin_stiff*strain*(strain-trans_strain);
	}

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
		labels.append(getName() + ".strain");
		labels.append(getName() + ".strain_rate");
		return labels;
	}

	/**
	 * Given SimTK::State object extract all the values necessary to report forces, application location
	 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
	 */
	OpenSim::Array<double> WISCO_Ligament::getRecordValues(const SimTK::State& s) const {
		OpenSim::Array<double> values(1);

		// Report values
		values.append(getCacheVariableValue<double>(s, "force_spring"));
		values.append(getCacheVariableValue<double>(s, "force_damping"));
		values.append(getCacheVariableValue<double>(s, "force_total"));
		values.append(getCacheVariableValue<double>(s, "length"));
		values.append(getCacheVariableValue<double>(s, "lengthening_speed"));
		values.append(getCacheVariableValue<double>(s, "strain"));
		values.append(getCacheVariableValue<double>(s, "strain_rate"));
		return values;
	}

	void WISCO_Ligament::printPropertiesToConsole() {
		std::string def_prop = get_defining_slack_length_property();
		double lin_stiff = get_linear_stiffness();
		double ref_strain = get_reference_strain();
		double ref_force = get_reference_force();
		double slack_len = get_slack_length();
		double damp_c = get_normalized_damping_coefficient();
		double trans_strain = get_transition_strain();

		std::cout << "WISCO_Ligament: " << getName() << std::endl;
		std::cout << "==============================" << std::endl;
		std::cout << "Linear Stiffness: " << lin_stiff << std::endl;
		std::cout << "Defining Slack Length Property:" << def_prop << std::endl;
		std::cout << "Reference Strain: " << ref_strain << std::endl;
		std::cout << "Reference Force: " << ref_force << std::endl;
		std::cout << "Slack Length: " << slack_len << std::endl;
		std::cout << "Transition Strain: " << trans_strain << std::endl;
		std::cout << "Normalized Damping Coeff: " << damp_c<< std::endl;
		std::cout << std::endl;


	}
