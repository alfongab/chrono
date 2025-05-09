// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Electric actuator model based on the paper:
//   Rahikainen, J., Gonzalez, F., Naya, M.A., Sopanen, J. and Mikkola, A.,
//   "On the cosimulation of multibody systems and hydraulic dynamics."
//   Multibody System Dynamics, 50(2), pp.143-167, 2020.
//
// =============================================================================

#ifndef CH_ELECTRIC_ACTUATOR_H
#define CH_ELECTRIC_ACTUATOR_H

#include <array>

#include "chrono/physics/ChExternalDynamicsODE.h"
#include "chrono/physics/ChBody.h"
#include "chrono/functions/ChFunction.h"

namespace chrono {

/// Base class for a electric actuator.
class ChApi ChElectricActuator : public ChExternalDynamicsODE {
  public:
    ChElectricActuator();
    ~ChElectricActuator() {}

    void SetActuatorInitialLength(double len);
    void SetInitialLoad(double initial_load);
    double GetInput(double t) const;

    /// Initialize this electric actuator by connecting it between the two specified bodies.
    void Initialize(std::shared_ptr<ChBody> body1,  ///< first connected body
                    std::shared_ptr<ChBody> body2,  ///< second connected body
                    bool local,                     ///< true if locations given in body local frames
                    ChVector3d loc1,                ///< location of connection point on body 1
                    ChVector3d loc2                 ///< location of connection point on body 2
    );

    /// Get the endpoint location on 1st body (expressed in absolute coordinate system).
    /// Returns a zero location if the actuator is not attached to bodies.
    ChVector3d GetPoint1Abs() const { return m_aloc1; }

    /// Get the endpoint location on 2nd body (expressed in body coordinate system).
    /// Returns a zero location if the actuator is not attached to bodies.
    ChVector3d GetPoint2Abs() const { return m_aloc2; }

    /// Set the current actuator length and rate of change.
    /// Can be used in a co-simulation interface.
    void SetActuatorLength(double len, double vel);

    /// Get the current actuator force.
    /// Can be used in a co-simulation interface.
    double GetActuatorForce();

  private:

    /// Declare the EOM of this physics item as stiff or non-stiff.
    virtual bool IsStiff() const override { return true; }

    /// Update the physics item at current state.
    virtual void Update(double time, bool update_assets) override;

    /// Load generalized forces.
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;

    bool is_attached;            ///< true if actuator attached to bodies
    ChBody* m_body1;             ///< first conected body
    ChBody* m_body2;             ///< second connected body
    ChVector3d m_loc1;           ///< point on body 1 (local frame)
    ChVector3d m_loc2;           ///< point on body 2 (local frame)
    ChVector3d m_aloc1;          ///< point on body 1 (global frame)
    ChVector3d m_aloc2;          ///< point on body 2 (global frame)
    ChVectorDynamic<> m_Qforce;  ///< generalized forcing terms

    std::shared_ptr<ChFunction> ref_fun;  ///< actuation function

    double s_0;  ///< initial actuator length [m]
    double s;    ///< current actuator length [m]
    double sd;   ///< current actuator speed [m/s]

    bool calculate_consistent_IC;  ///< solve initialization nonlinear system
    double F0;                     ///< estimated initial load
};

}  // end namespace chrono

#endif
