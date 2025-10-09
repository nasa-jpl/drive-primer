#ifndef PERSEVERENCE_GOTO_CONTROLLER_H
#define PERSEVERENCE_GOTO_CONTROLLER_H

#include "chrono_parsers/ChParserURDF.h"
#include "perseverance_controller.h"
#include <cmath>

using namespace chrono::parsers;
using namespace chrono;

class PerseveranceStraightDriveController : public PerseveranceController {

private:
    bool m_is_drive_started = false;
    ChFrame<> m_pose;
    double m_terminate_x = 0.5;

public:

    void Advance(ChFrame<> pose, double dt) override {
        if(!m_is_drive_started) {
            locomotion.SetDriveVelocity(0.04);
            locomotion.SetDriveStraight();
            m_is_drive_started = true;
        }
        m_pose = pose;
    }

    bool IsComplete() {
        if(m_is_drive_started) {
            if(m_pose.GetPos().x() > m_terminate_x) {
                return true;
            }
        }
        return false;
    }
};

#endif