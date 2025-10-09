#ifndef PERSEVERENCE_LOCOMOTION_H
#define PERSEVERENCE_LOCOMOTION_H

#include "chrono_parsers/ChParserURDF.h"

using namespace chrono::parsers;

using namespace chrono;

class PerseveranceLocomotion {

private:
    ChParserURDF* m_parser;

    std::shared_ptr<ChFunctionConst> m_lf_drive_func;   // [rad]
    std::shared_ptr<ChFunctionConst> m_lm_drive_func;   // [rad]
    std::shared_ptr<ChFunctionConst> m_lr_drive_func;   // [rad]
    std::shared_ptr<ChFunctionConst> m_rf_drive_func;   // [rad]
    std::shared_ptr<ChFunctionConst> m_rm_drive_func;   // [rad]
    std::shared_ptr<ChFunctionConst> m_rr_drive_func;   // [rad]
    
    std::shared_ptr<ChFunctionConst> m_lf_steer_func;  //  [radians] or [radians/s]
    std::shared_ptr<ChFunctionConst> m_rf_steer_func;  // [radians] or [radians/s]
    std::shared_ptr<ChFunctionConst> m_rr_steer_func;  //  [radians] or [radians/s]
    std::shared_ptr<ChFunctionConst> m_lr_steer_func;  // [radians] or [radian/s]

public:    

    struct Command
    {
        double SCLK;
        double LF_ANG_VEL;
        double LM_ANG_VEL;
        double LR_ANG_VEL;
        double RF_ANG_VEL;
        double RM_ANG_VEL;
        double RR_ANG_VEL;
        double LF_STEER;
        double LR_STEER;
        double RF_STEER;
        double RR_STEER;
    };

    Command last_command;
    bool m_lc_init = false;

    void Initialize(ChParserURDF* parser) {
        m_parser = parser;

        m_lf_drive_func = std::make_shared<ChFunctionConst>(0);  
        m_lm_drive_func = std::make_shared<ChFunctionConst>(0);  
        m_lr_drive_func = std::make_shared<ChFunctionConst>(0);  
        m_rf_drive_func = std::make_shared<ChFunctionConst>(0);  
        m_rm_drive_func = std::make_shared<ChFunctionConst>(0);  
        m_rr_drive_func = std::make_shared<ChFunctionConst>(0);  

        m_rf_steer_func = std::make_shared<ChFunctionConst>(0);
        m_lf_steer_func = std::make_shared<ChFunctionConst>(0);
        m_rr_steer_func = std::make_shared<ChFunctionConst>(0);
        m_lr_steer_func = std::make_shared<ChFunctionConst>(0);

        m_parser->SetMotorFunction("RF_DRIVE", m_rf_drive_func);
        m_parser->SetMotorFunction("RM_DRIVE", m_rm_drive_func);
        m_parser->SetMotorFunction("RR_DRIVE", m_rr_drive_func);

        m_parser->SetMotorFunction("LF_DRIVE", m_lf_drive_func);
        m_parser->SetMotorFunction("LM_DRIVE", m_lm_drive_func);
        m_parser->SetMotorFunction("LR_DRIVE", m_lr_drive_func);
        
        m_parser->SetMotorFunction("RF_STEER", m_rf_steer_func);
        m_parser->SetMotorFunction("LF_STEER", m_lf_steer_func);

        m_parser->SetMotorFunction("RR_STEER", m_rr_steer_func);
        m_parser->SetMotorFunction("LR_STEER", m_lr_steer_func);

        

    }

    void SetDriveVelocity(double lin_vel) {

        double ang_vel = -lin_vel/0.263; // rad * ds
        m_lf_drive_func->SetConstant(ang_vel);
        m_lm_drive_func->SetConstant(ang_vel);
        m_lr_drive_func->SetConstant(ang_vel);
        
        m_rf_drive_func->SetConstant(ang_vel);
        m_rm_drive_func->SetConstant(ang_vel);
        m_rr_drive_func->SetConstant(ang_vel);
    
    }


    void PrepareTurnInPlace() {
        m_lf_steer_func->SetConstant(-0.785);
        m_rr_steer_func->SetConstant(-0.785);
        m_rf_steer_func->SetConstant(0.785);
        m_lr_steer_func->SetConstant(0.785);
    }

    
    // dir = 1 => Turn right
    // dir = -1 => Turn left
    void TurnInPlace(int dir) {
        double ang_vel_wheels = 0.11*dir;
        m_rf_drive_func->SetConstant(ang_vel_wheels);
        m_rm_drive_func->SetConstant(ang_vel_wheels);
        m_rr_drive_func->SetConstant(ang_vel_wheels);
        
        m_lf_drive_func->SetConstant(-ang_vel_wheels);
        m_lm_drive_func->SetConstant(-ang_vel_wheels);
        m_lr_drive_func->SetConstant(-ang_vel_wheels);
    }

    void HaltTurnInPlace() {
        m_rf_drive_func->SetConstant(0);
        m_rm_drive_func->SetConstant(0);
        m_rr_drive_func->SetConstant(0);

        m_lf_drive_func->SetConstant(0);
        m_lm_drive_func->SetConstant(0);
        m_lr_drive_func->SetConstant(0);
    }

    void SetDriveStraight() {
        m_lr_steer_func->SetConstant(0.0);
        m_lf_steer_func->SetConstant(0.0);
        m_rf_steer_func->SetConstant(0.0);
        m_rr_steer_func->SetConstant(0.0);
    }

    void SetArcUntilTurn(ChVector3d position, ChVector3d goal, double distance, double waydisc_radius, double& R) {
        ChVector3d diff = goal - position;
        double theta_delta = 2*atan2(diff.y(), diff.x() + waydisc_radius);
        R = distance / theta_delta;
        double L = 2.2; // [m]
        double T = 1.3; // [m]

        int dir = -1;

        double front_inner_angle = atan((0.5*L) / (R-(T*0.5)));
        double front_outer_angle = atan((0.5*L) / (R+(T*0.5)));
        double rear_inner_angle = atan(-(0.5*L) / (R+(T*0.5)));
        double rear_outer_angle = atan(-(0.5*L) / (R-(T*0.5)));
        
        if(dir == 1) {
            m_rr_steer_func->SetConstant(rear_outer_angle);
            m_lr_steer_func->SetConstant(rear_inner_angle);
            m_rf_steer_func->SetConstant(front_outer_angle);
            m_lf_steer_func->SetConstant(front_inner_angle);

        } else {
            m_rr_steer_func->SetConstant(rear_inner_angle);
            m_lr_steer_func->SetConstant(rear_outer_angle);
            m_rf_steer_func->SetConstant(front_inner_angle);
            m_lf_steer_func->SetConstant(front_outer_angle);
        }

        std::cout << "R " << R << " " << theta_delta << std::endl;
        std::cout << "Joint Angles " << rear_outer_angle << " " << rear_inner_angle << " " << front_inner_angle << " " << front_outer_angle << std::endl;
    }

    void SetJointStates(double clock, Command command) {
       
        SetJointCommand(clock, command.SCLK, last_command.SCLK, m_lf_drive_func,command.LF_ANG_VEL, last_command.LF_ANG_VEL);
        SetJointCommand(clock, command.SCLK, last_command.SCLK, m_lm_drive_func,command.LM_ANG_VEL, last_command.LM_ANG_VEL);
        SetJointCommand(clock, command.SCLK, last_command.SCLK, m_lr_drive_func,command.LR_ANG_VEL, last_command.LR_ANG_VEL);
        
        SetJointCommand(clock, command.SCLK, last_command.SCLK, m_rf_drive_func,command.RF_ANG_VEL, last_command.RF_ANG_VEL);
        SetJointCommand(clock, command.SCLK, last_command.SCLK, m_rm_drive_func,command.RM_ANG_VEL, last_command.RM_ANG_VEL);
        SetJointCommand(clock, command.SCLK, last_command.SCLK, m_rr_drive_func,command.RR_ANG_VEL, last_command.RR_ANG_VEL);
        
        SetJointCommand(clock, command.SCLK, last_command.SCLK, m_lf_steer_func, command.LF_STEER, last_command.LF_STEER);
        SetJointCommand(clock, command.SCLK, last_command.SCLK, m_rf_steer_func, command.RF_STEER, last_command.RF_STEER);
        SetJointCommand(clock, command.SCLK, last_command.SCLK, m_lr_steer_func, command.LR_STEER, last_command.LR_STEER);
        SetJointCommand(clock, command.SCLK, last_command.SCLK, m_rr_steer_func, command.RR_STEER, last_command.RR_STEER);


    }

    void SetLastCommand(Command c) {
        m_lc_init = true;
        last_command = c;
    }

    void SetJointCommand(double clock_now, double command_sclk, double last_clock, std::shared_ptr<ChFunctionConst> function, double commanded_value, double last_commanded_value) {

        // Lin interp locally
        if(m_lc_init && clock_now < command_sclk) { //&& command_sclk - clock_now < 5e-1) {
           double dcmd = (commanded_value - last_commanded_value)/(command_sclk - last_clock);
           commanded_value += dcmd * (clock_now - last_clock);
        }
        

        function->SetConstant(commanded_value);
    }
};

#endif