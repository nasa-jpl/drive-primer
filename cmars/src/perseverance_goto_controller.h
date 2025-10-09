#ifndef PERSEVERENCE_GOTO_CONTROLLER_H
#define PERSEVERENCE_GOTO_CONTROLLER_H

#include "chrono_parsers/ChParserURDF.h"
#include "perseverance_controller.h"
#include <cmath>

using namespace chrono::parsers;
using namespace chrono;

/*
    Naive FSM, doesn't do transitions at all, better implementations are out there...
*/

class PerseveranceGotoController : public PerseveranceController {

private:
    enum Mode {
        START,
        TURN_IN_PLACE,
        STRAIGHT_DRIVE,
        WAYPOINT_PRELUDE,
        ARC_UNTIL_PRELUDE,
        ARC_UNTIL
    };

    int waypoint_idx = 0;
    Mode mode = Mode::START;

    
    ChVector3d m_start_pose;

    bool m_terminate = false;
    bool m_started = false;

    double m_bearing = 0.0;
    double m_distance_to_go = 100.0;
    double m_clock = 0.0;

    double m_turn_wait_duration = 1.0;
    double m_turn_wait_clock = 0.0;
    bool m_turn_started = false;
    double m_rover_yaw_init = 0.0;
    double m_turn_odometer = 0.0;
    
public:

    struct Waypoint {
        ChVector3d pos;
        double straight_distance;
        double distance;
        double waydisc_radius;
    };
    std::vector<Waypoint> m_waypoints;

    void SetWaypoints(std::vector<Waypoint> waypoints) {
        m_waypoints = waypoints;
    }

    double wrap(double ang) {
        return fmod(ang + 3.14,2.0*3.14)-3.14;
    }

    double unwrap(double ang) {
        return fmod(ang + (2*3.14), 2*3.14);
    }

    void Advance(ChFrame<> pose, double dt) override {

        double rover_yaw = unwrap(pose.GetRot().GetCardanAnglesXYZ().z());        
        
        if(mode == Mode::START) {
            std::cout << "Going to " << m_waypoints.size() << " waypoint(s)" << std::endl;;
            mode = Mode::WAYPOINT_PRELUDE;
            std::cout << "Initial Yaw: " << rover_yaw << std::endl;
        }
        if(mode == Mode::WAYPOINT_PRELUDE) {
            m_distance_to_go = m_waypoints[waypoint_idx].straight_distance;

            if(fabs((m_waypoints[waypoint_idx].pos - pose.GetPos()).Length()) < 0.5) {
                std::cout << "Incrementing waypoint counter" << std::endl;
                waypoint_idx++;
                m_distance_to_go = m_waypoints[waypoint_idx].distance;
            } else if (m_started) {
                std::cout << "Not close enough to waypoint, terminating " << waypoint_idx << std::endl;
                m_terminate = true;
            }

            double bearing_circ = atan2(m_waypoints[waypoint_idx].pos.y() - pose.GetPos().y(),m_waypoints[waypoint_idx].pos.x() - pose.GetPos().x());
            m_bearing = unwrap(bearing_circ);
            
            std::cout << "Preparing to go to id = " << waypoint_idx << std::endl;
            std::cout << "Bearing... " << m_bearing << std::endl;
            std::cout << "DTG..." << m_distance_to_go << std::endl;
            
            mode = Mode::TURN_IN_PLACE;
            m_turn_started = false;
            m_turn_wait_clock = 0.0;
            m_started = true;
        }

        if(mode == Mode::TURN_IN_PLACE) {
            if(!m_turn_started) {
                std::cout << "Preparing to turn" << std::endl;
                locomotion.PrepareTurnInPlace();
                m_rover_yaw_init = rover_yaw;
                m_turn_started = true;
            }

            double target = unwrap(m_bearing - rover_yaw);

            if(fabs(target) < 1e-3) {
                std::cout << "Turn complete!" << std::endl;
                locomotion.HaltTurnInPlace();
                mode = Mode::STRAIGHT_DRIVE;
                m_start_pose = pose.GetPos();
            }

            if(m_turn_started && m_turn_wait_clock < m_turn_wait_duration) {
                m_turn_wait_clock += dt;
                if(m_turn_wait_clock >= m_turn_wait_duration) {
                    int dir = 1;
                    if(target > 3.14) {
                        dir = -1;
                    }
                    locomotion.TurnInPlace(dir);
                }
            }

            if(m_turn_started && m_turn_wait_clock >= m_turn_wait_duration && fmod(m_clock, 10.0) < 0.01) {
                std::cout << "Heading Diff " << rover_yaw  << " Target " << target << std::endl; 
            }
        }
        if(mode == Mode::STRAIGHT_DRIVE) {
            double odometer = m_distance_to_go - (m_start_pose - pose.GetPos()).Length();
            if(odometer <= 0) {
                mode = Mode::ARC_UNTIL_PRELUDE;
            } else {
                locomotion.SetDriveVelocity(0.04);
                locomotion.SetDriveStraight();
            }
            if(fmod(m_clock, 10.0) < 0.01) {
                std::cout << "Progress: " << odometer << " Real " << (m_waypoints[waypoint_idx].pos - pose.GetPos()).Length() << std::endl;
            }
        }
        if(mode == Mode::ARC_UNTIL_PRELUDE){
            double R = 0;
            locomotion.SetArcUntilTurn(pose.GetPos(),m_waypoints[waypoint_idx].pos,1.0,1.75,R);
            locomotion.SetDriveVelocity(0.04);
            mode = Mode::ARC_UNTIL;
            m_turn_odometer = 1.0;
        }
        if(mode == Mode::ARC_UNTIL) {
            m_turn_odometer -= 0.04*dt;
            if(m_turn_odometer <= 0.0) {
                mode = Mode::WAYPOINT_PRELUDE;
            }
        }

    }

    bool IsComplete() {

        return waypoint_idx > m_waypoints.size() || m_terminate;
    }
};

#endif