#ifndef PERSEVERENCE_SLIP_H
#define PERSEVERENCE_SLIP_H

#include "chrono_parsers/ChParserURDF.h"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <queue>

using namespace chrono::parsers;

using namespace chrono;

class PerseveranceSlip {
private:
    double m_clock = 0;
    std::shared_ptr<ChBody> m_chassis;
    std::shared_ptr<ChBody> m_lm_drive;
    std::shared_ptr<ChBody> m_rm_drive;

    std::string openloop;
    std::queue<std::pair<double,int>> evr_window_queue;
    ChParserURDF* parser;

    std::vector<std::pair<double,double>> slip;

    std::vector<double> joint_states {
        0,0,0,0,0,0,0,0,0,0,0
    };

    double W = 2.0;
    double L = 2.7;
    double r = 0.26;

    double s_i = 0.0;

    ChVector3d m_pos;
    double m_yaw;
    double m_slip_t;

public:

    void SetClock(double clock) {
        m_clock = clock;
    }
    
    void Initialize(ChParserURDF* parser) {
        this->parser = parser;
        this->m_chassis = parser->GetRootChBody();
        // std::ifstream inputFile(trajectory_csv);
        // if (!inputFile.is_open()) {
        //     throw std::runtime_error("Error opening input CSV");
        // }
        // bool initialized = false;
        // for (std::string line; std::getline(inputFile, line);) {
        //     if(!initialized) {
        //         initialized = true; // Skip first line
        //         continue;
        //     }

        //     std::istringstream ss(std::move(line));
        //     std::vector<std::string> row;

        //     int i = 0;
        //     std::vector<double> tokens;

        //     for(std::string token; std::getline(ss,token,',');) {
        //         tokens.push_back(std::stod(token));
        //         i++;
        //     }
            
        //     double sclk = tokens[0];
        //     // int drive_state = tokens[19];
        //     int drive_state = 1;
            
        //     evr_window_queue.push({sclk,drive_state});
        // }

        m_pos = m_chassis->GetPos();
        m_yaw = m_chassis->GetRot().GetCardanAnglesZYX().x(); 

        m_lm_drive = parser->GetChBody("Body_WheelLeftMiddle");
        m_rm_drive = parser->GetChBody("Body_WheelRightMiddle");
        // InitializeFront();
    }


    // void InitializeFront() {
        // while(evr_window_queue.front().first < m_clock) {
            // evr_window_queue.pop();
        // }
    // }

    std::vector<double> GetJointStates(ChParserURDF* parser) {
        std::vector<std::string> joints {
            "LF_STEER","LR_STEER","RF_STEER","RR_STEER",
            "LF_DRIVE","LM_DRIVE","LR_DRIVE",
            "RF_DRIVE","RM_DRIVE","RR_DRIVE"
        };

        int i = 0;
        for(auto& j : joints) {
            joint_states[i++] = parser->GetChMotor(j)->GetMotorFunction()->GetVal(0);
        }

        return joint_states;
    }

    void Advance(double dt) {
        m_clock += dt;

        /* Progress by dead reckoning */
        auto v = GetJointStates(parser);
        double s_pred = ComputeArcLengthNoSlip(v,s_i,dt);

        // std::cout << "s_pred " << s_pred << " s_i" << s_i <<std::endl;

        // Trigger VO update
        if(s_pred > 0.15) {
            double yaw = m_chassis->GetRot().GetCardanAnglesZYX().x();
            ChVector3d pos = m_chassis->GetPos();
            double s_vo = ComputeArcLengthVO(v,m_pos,pos,m_yaw,yaw);
            double slip_t = 1 - (s_vo/s_pred);
            slip.push_back({m_clock,slip_t});
            m_yaw = yaw;
            m_pos = pos;
            s_i = 0.0; // Reset integral term
            // std::cout << "s_pred " << s_pred << " s_vo " << s_vo <<std::endl;
            // std::cout << "SLIP: " << slip_t << std::endl;
            m_slip_t = slip_t;
        }

        // std::cout << s_i << " // " << s_pred << std::endl;
    }

    /*
     * Compute distance traveled so far from "wheel odometry"
     * Integrates on $s_i$, beware this stateful variable
    */
    double ComputeArcLengthNoSlip(std::vector<double> v, double& s_i, double dt) {
        double front = v[0];
        double rear = v[2];
        double kappa = (tan(front) - tan(rear))*(1/L);

        if(fabs(kappa) < 1e-6) {
            s_i += -r*m_lm_drive->GetAngVelLocal().y()*dt;
            return s_i;
        } 


        double R_c = 1/fabs(kappa); // Radius from chassis to ICR
        double r_i = sqrt(pow(R_c - (0.5*W),2) + pow(-0.5*L,2));
        double ds_i = -r*m_lm_drive->GetAngVelLocal().y();


        // std::cout << "Ang Vel.." << ds_i << std::endl;

        s_i += ds_i*dt; // Update integral term
        
        double phi = s_i/r_i;
        double s_pred = R_c*phi;
        return s_pred;
    }

    /*
     * Compute ground truth arc length, emulates "VO updates"
     * Stateless
    */
    double ComputeArcLengthVO(std::vector<double> v, ChVector3d last, ChVector3d now, double yaw_last, double yaw_now) {
        double dx = now.x() - last.x();
        double dy = now.y() - last.y();
        double dist = sqrt(pow(dx,2) + pow(dy,2));
        // double dpsi = atan2()
        double dpsi = fmod((yaw_last - yaw_now + 3.14), 2*3.14) - 3.14;

        if (fabs(dpsi) > 1e-3) {
            double R = dist * (1/(2*sin(dpsi*0.5)));
            return R*dpsi;
        } else { 
            return dist;
        }

    }

    double GetLastSlip() {
        return m_slip_t;
    }
};

#endif