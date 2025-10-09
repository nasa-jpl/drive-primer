#ifndef PERSEVERENCE_OPENLOOP_CONTROLLER_H
#define PERSEVERENCE_OPENLOOP_CONTROLLER_H

#include "chrono_parsers/ChParserURDF.h"
#include "perseverance_controller.h"
#include <cmath>
#include <queue>

using namespace chrono::parsers;
using namespace chrono;

class PerseveranceOpenLoopController : public PerseveranceController {

public:


    std::queue<PerseveranceLocomotion::Command> m_command_stack;

    double m_clock = 0.0;
    bool m_pos_init = false;

    void SetClock(double clock) {
        m_clock = clock;
    }

    void Initialize(double t_init, ChParserURDF* parser, std::string csv) {
        SetClock(t_init);
        
        std::ifstream inputFile(csv);
        if (!inputFile.is_open()) {
            throw std::runtime_error("Error opening input CSV");
        }
        bool initialized = false;
        for (std::string line; std::getline(inputFile, line);) {
            if(!initialized) {
                initialized = true; // Skip first line
                continue;
            }

            std::istringstream ss(std::move(line));
            std::vector<std::string> row;

            int i = 0;
            std::vector<double> tokens;

            for(std::string token; std::getline(ss,token,',');) {
                tokens.push_back(std::stod(token));
                i++;
            }
            
            //,SCLK,ROVER_X [METERS],ROVER_Y [METERS],ROVER_Z [METERS],QUAT_X,QUAT_Y,QUAT_Z,QUAT_C,
            // LF_ANG_VEL,LM_ANG_VEL,LR_ANG_VEL,RF_ANG_VEL,RR_ANG_VEL,RM_ANG_VEL,
            // LF_STEER [RADIANS],LR_STEER [RADIANS],RF_STEER [RADIANS],RR_STEER [RADIANS]
            PerseveranceLocomotion::Command command {
                tokens[0],tokens[9],tokens[10],tokens[11],tokens[12],tokens[13],tokens[14],
                tokens[15],tokens[16],tokens[17],tokens[18]
            };
            m_command_stack.push(command);

        }

        PerseveranceController::Initialize(parser);
        InitializeFront();
        std::cout << "Successfully initialized " << m_command_stack.size() << " commands." << std::endl;

    }

    void InitializeFront() {
        while(m_command_stack.front().SCLK < m_clock) {
            m_command_stack.pop();
        }
        locomotion.SetJointStates(m_clock, m_command_stack.front());
    }

    void Advance(ChFrame<> pose, double dt) override {
        m_clock += dt;
        if(m_command_stack.front().SCLK < m_clock) {
            locomotion.SetLastCommand(m_command_stack.front());
            m_command_stack.pop();
        }
        locomotion.SetJointStates(m_clock, m_command_stack.front());
    }
    
    bool IsComplete() {
        return false;
    }
};

#endif