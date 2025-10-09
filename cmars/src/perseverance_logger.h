#ifndef PERSEVERENCE_LOGGER_H
#define PERSEVERENCE_LOGGER_H

#include "chrono_parsers/ChParserURDF.h"
#include <iostream>
#include <fstream>
#include <stdexcept>

using namespace chrono::parsers;

using namespace chrono;

class PerseveranceLogger {
private:
    double m_logging_rate = 0; // Logging rate [s]
    double m_clock = 0;
    std::shared_ptr<ChBody> m_chassis;

    ChParserURDF* m_parser;

    std::string m_filename;
    std::ofstream log_file;

    double m_slow_slip = 0.0;

public:

    void SetClock(double clock) {
        m_clock = clock;
    }
    
    void Initialize(std::shared_ptr<ChBody> chassis, ChParserURDF* parser, std::string filename, double logging_rate = 0.2) {
        m_chassis = chassis;
        m_parser = parser;
        m_logging_rate = logging_rate;
        m_filename = filename;
        log_file.open(m_filename, std::ios::in | std::ios::out | std::ios::trunc);
        if(!log_file.is_open()) {
            throw std::runtime_error(("[Logger] Error opening file at " + m_filename));
        }
        log_file << "m_clock,x,y,z,q_x,q_y,q_z,q_w,rf_s,rr_s,fl_s,rl_s,lr_d,rr_d,fl_d,fr_d,rm_d,lm_d,lb_rot,rb_rot,ld_rot,rd_rot,slip,slow_slip,wrf_x,wrf_y,wrf_z,wrc_x,wrc_y,wrc_z,wrb_x,wrb_y,wrb_z,wlf_x,wlf_y,wlf_z,wlc_x,wlc_y,wlc_z,wlb_x,wlb_y,wlb_z" << std::endl;
    }

    void Advance(double slow_slip, double dt) {
        if(fabs(fmod(m_clock, m_logging_rate)) < dt) {

            double RF_STEER = m_parser->GetChMotor("RF_STEER")->GetMotorFunction()->GetVal(0);
            double RR_STEER = m_parser->GetChMotor("RR_STEER")->GetMotorFunction()->GetVal(0);
            double FL_STEER = m_parser->GetChMotor("LF_STEER")->GetMotorFunction()->GetVal(0);
            double RL_STEER = m_parser->GetChMotor("LR_STEER")->GetMotorFunction()->GetVal(0);

            double RF_DRIVE = m_parser->GetChMotor("RF_DRIVE")->GetMotorFunction()->GetVal(0);
            double RR_DRIVE = m_parser->GetChMotor("RR_DRIVE")->GetMotorFunction()->GetVal(0);
            double FL_DRIVE = m_parser->GetChMotor("LF_DRIVE")->GetMotorFunction()->GetVal(0);
            double LR_DRIVE = m_parser->GetChMotor("LR_DRIVE")->GetMotorFunction()->GetVal(0);
            double RM_DRIVE = m_parser->GetChMotor("RM_DRIVE")->GetMotorFunction()->GetVal(0);
            double LM_DRIVE = m_parser->GetChMotor("LM_DRIVE")->GetMotorFunction()->GetVal(0);
           
            auto left_bogie =   std::dynamic_pointer_cast<ChLinkLockRevolute>(m_parser->GetChLink("LEFT_BOGIE"));
            auto right_bogie =  std::dynamic_pointer_cast<ChLinkLockRevolute>(m_parser->GetChLink("RIGHT_BOGIE")); 
            auto right_diff =   std::dynamic_pointer_cast<ChLinkLockRevolute>(m_parser->GetChLink("RIGHT_DIFFERENTIAL")); 
            auto left_diff =    std::dynamic_pointer_cast<ChLinkLockRevolute>(m_parser->GetChLink("LEFT_DIFFERENTIAL"));

            // auto left_front_force = m_parser->GetChBody("RF_DRIVE")->
            auto WRF = m_parser->GetChBody("Body_WheelRightFront")->GetAccumulatedForce(0);
            auto WRC = m_parser->GetChBody("Body_WheelRightMiddle")->GetAccumulatedForce(0);
            auto WRB = m_parser->GetChBody("Body_WheelRightRear")->GetAccumulatedForce(0);
            auto WLF = m_parser->GetChBody("Body_WheelLeftFront")->GetAccumulatedForce(0);
            auto WLC = m_parser->GetChBody("Body_WheelLeftMiddle")->GetAccumulatedForce(0);
            auto WLB = m_parser->GetChBody("Body_WheelLeftRear")->GetAccumulatedForce(0);


            double lb_rot = left_bogie->GetRelAngle();
            double rb_rot = right_bogie->GetRelAngle();
            double rd_rot = right_diff->GetRelAngle();
            double ld_rot = left_diff->GetRelAngle();

            double slip = std::fmax(std::fmin(1 - (m_chassis->GetPosDt().Length()/0.042),1.0),-1.0);
            double slow_slip_t = slow_slip;
            if(fabs(slow_slip - m_slow_slip) < 1e-4) {
                slow_slip_t = std::nan("");
            }

            ChQuaterniond quat_ned = m_chassis->GetRot();

            m_slow_slip = slow_slip;

            log_file << std::fixed << m_clock << ", "; 
            log_file << m_chassis->GetFrameRefToAbs().GetPos().x() << ", "; 
            log_file << m_chassis->GetFrameRefToAbs().GetPos().y() << ", "; 
            log_file << m_chassis->GetFrameRefToAbs().GetPos().z() << ", "; 
            log_file << quat_ned.e1() << ", "; 
            log_file << quat_ned.e2() << ", "; 
            log_file << quat_ned.e3() << ", ";     
            log_file << quat_ned.e0() << ", ";
            log_file << -RF_STEER << ", ";
            log_file << -RR_STEER << ", ";
            log_file << -FL_STEER << ", ";
            log_file << -RL_STEER << ", ";
            log_file << -RF_DRIVE << ", ";
            log_file << -RR_DRIVE << ", ";
            log_file << -FL_DRIVE << ", ";
            log_file << -LR_DRIVE << ", ";
            log_file << -RM_DRIVE << ", ";
            log_file << -LM_DRIVE << ", ";
            log_file << -lb_rot << ", ";
            log_file << -rb_rot << ", ";
            log_file << -ld_rot << ", ";
            log_file << -rd_rot << ", ";
            log_file << slip << ", ";
            log_file << slow_slip_t << ", ";
            log_file << WRF.x() << ", ";
            log_file << WRF.y() << ", ";
            log_file << WRF.z() << ", ";
            log_file << WRC.x() << ", ";
            log_file << WRC.y() << ", ";
            log_file << WRC.z() << ", ";
            log_file << WRB.x() << ", ";
            log_file << WRB.y() << ", ";
            log_file << WRB.z() << ", ";
            log_file << WLF.x() << ", ";
            log_file << WLF.y() << ", ";
            log_file << WLF.z() << ", ";
            log_file << WLC.x() << ", ";
            log_file << WLC.y() << ", ";
            log_file << WLC.z() << ", ";
            log_file << WLB.x() << ", ";
            log_file << WLB.y() << ", ";
            log_file << WLB.z() << "";
            log_file << std::endl;
        }
        m_clock += dt;
    }
};

#endif