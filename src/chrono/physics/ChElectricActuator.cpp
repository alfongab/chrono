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

#include "chrono/utils/ChUtils.h"
#include "chrono/physics/ChElectricActuator.h"

namespace chrono {

ChElectricActuator::ChElectricActuator() : is_attached(false), calculate_consistent_IC(false) {}

void ChElectricActuator::SetActuatorInitialLength(double len) {
    s_0 = len;
    s = len;
    sd = 0;
}

void ChElectricActuator::SetActuatorLength(double len, double vel) {
    // Do nothing if not attached to bodies
    if (is_attached)
        return;

    s = len;
    sd = vel;
}

void ChElectricActuator::SetInitialLoad(double initial_load) {
    calculate_consistent_IC = true;
    F0 = initial_load;
}

void ChElectricActuator::Initialize(std::shared_ptr<ChBody> body1,  // first connected body
                                         std::shared_ptr<ChBody> body2,  // second connected body
                                         bool local,                     // true if locations given in body local frames
                                         ChVector3d loc1,                // location of connection point on body 1
                                         ChVector3d loc2                 // location of connection point on body 2
) {
    is_attached = true;

    // Call base initialization (external dynamics)
    ChExternalDynamicsODE::Initialize();


    // Cache connected bodies and body local connection points
    m_body1 = body1.get();
    m_body2 = body2.get();

    if (local) {
        m_loc1 = loc1;
        m_loc2 = loc2;
        m_aloc1 = body1->TransformPointLocalToParent(loc1);
        m_aloc2 = body2->TransformPointLocalToParent(loc2);
    } else {
        m_loc1 = body1->TransformPointParentToLocal(loc1);
        m_loc2 = body2->TransformPointParentToLocal(loc2);
        m_aloc1 = loc1;
        m_aloc2 = loc2;
    }

    s_0 = (m_aloc1 - m_aloc2).Length();

    // Resize temporary vector of generalized body forces
    m_Qforce.resize(12);
}

double ChElectricActuator::GetActuatorForce() {
    //// TODO
    return 0;
}

double ChElectricActuator::GetInput(double t) const {
    return ChClamp(ref_fun->GetVal(t), -1.0, +1.0);
}

void ChElectricActuator::Update(double time, bool update_assets) {
    // Update the external dynamics
    ChExternalDynamicsODE::Update(time, update_assets);

    // If the actuator is attached to bodies, update its length and rate from the body states
    // and calculate the generated force to the two bodies.
    if (is_attached) {
        m_aloc1 = m_body1->TransformPointLocalToParent(m_loc1);
        m_aloc2 = m_body2->TransformPointLocalToParent(m_loc2);

        auto avel1 = m_body1->PointSpeedLocalToParent(m_loc1);
        auto avel2 = m_body2->PointSpeedLocalToParent(m_loc2);

        ChVector3d dir = (m_aloc1 - m_aloc2).GetNormalized();

        s = (m_aloc1 - m_aloc2).Length();
        sd = Vdot(dir, avel1 - avel2);

        ////std::cout << "time = " << time << "    s=" << s << "   sd=" << sd << std::endl;

        // Actuator force
        auto f = GetActuatorForce();
        ChVector3d force = f * dir;

        // Force and moment acting on body 1
        auto atorque1 = Vcross(m_aloc1 - m_body1->GetPos(), force);          // applied torque (absolute frame)
        auto ltorque1 = m_body1->TransformDirectionParentToLocal(atorque1);  // applied torque (local frame)
        m_Qforce.segment(0, 3) = force.eigen();
        m_Qforce.segment(3, 3) = ltorque1.eigen();

        // Force and moment acting on body 2
        auto atorque2 = Vcross(m_aloc2 - m_body2->GetPos(), -force);         // applied torque (absolute frame)
        auto ltorque2 = m_body2->TransformDirectionParentToLocal(atorque2);  // applied torque (local frame)
        m_Qforce.segment(6, 3) = -force.eigen();
        m_Qforce.segment(9, 3) = ltorque2.eigen();
    }
}

void ChElectricActuator::IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) {
    if (!IsActive())
        return;

    // Load external dynamics
    ChExternalDynamicsODE::IntLoadResidual_F(off, R, c);

    if (is_attached) {
        // Add forces to connected bodies (calculated in Update)
        if (m_body1->Variables().IsActive()) {
            R.segment(m_body1->Variables().GetOffset() + 0, 3) += c * m_Qforce.segment(0, 3);
            R.segment(m_body1->Variables().GetOffset() + 3, 3) += c * m_Qforce.segment(3, 3);
        }
        if (m_body2->Variables().IsActive()) {
            R.segment(m_body2->Variables().GetOffset() + 0, 3) += c * m_Qforce.segment(6, 3);
            R.segment(m_body2->Variables().GetOffset() + 3, 3) += c * m_Qforce.segment(9, 3);
        }
    }
}

}  // end namespace chrono
