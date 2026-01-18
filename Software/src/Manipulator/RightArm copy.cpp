#include "RightArm.h"
#include <iostream>

using namespace Eigen;
using std::cout;
using std::endl;

RightArm *RightArm::inst_ = nullptr;
RightArm *RightArm::GetRightArm()
{
	if (inst_ == nullptr)
		inst_ = new RightArm();
	return inst_;
}
static const Eigen::Matrix<double,7,4> right_arm_DH_table = (Eigen::Matrix<double,7,4>() <<
    0,     0,      0,    233,
    0, -M_PI/2,    0,      0,
    0,     0, 265.7,      0,
    0, -M_PI/2,   30,      0,
    0,  M_PI/2,    0,    264,
    0, -M_PI/2,    0,      0,
    0,     0,      0,   242.27).finished(); //236

RightArm::RightArm()
       : ArmInfo(right_arm_DH_table,  {6, 7, 8, 9, 10, 11, 31}, {"Pro200", "Pro200", "Pro100", "Pro100", "Pro20+", "Pro20+", "RH"}),

        gripper_offset_(100.0),
        object_offset_(100.0),
        z_offset_(50.0),
        right_world_pos_(Vector3d::Zero()),
        right_orientation_deg_(Vector3d::Zero()),
        vel_gain_linear_(0.15), // 1.2
        vel_gain_angular_(0.01), // 0.125
        ki_gain_(0.1),
        angular_threshold_(1 * M_PI / 180.0), // 1 * M_PI/ 180.0f
        linear_threshold_(1.0), // 1 mm
        stop_counter_(0),
        last_position_(Vector3d::Zero())
{
    // socket.connect(addr);
    is_out_of_limit_ = false;
    is_working_ = false;


    setWorldToBaseTransform(M_PI/2, 0.0);
    // gripper_ = std::make_shared<GripperController>(MotorUnion::tmp_portHandler, 31, 4000000);
    // openGripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_TIME_));
    updateRobotPose();
    last_position_ = getCurrentPosition();

    std::cout << "Class constructed: Right Arm" << std::endl;
    std::cout << "================================================================================" << std::endl;
}

/* IK helper functions 

*/


Quaterniond RightArm::quatConjugate(const Quaterniond& q) {
    return Quaterniond(q.w(), -q.x(), -q.y(), -q.z());
}

Quaterniond RightArm::quatMul(const Quaterniond& a, const Quaterniond& b) {
    return a * b;
}

// 簡單來說：給定目標角度，計算各軸速度
bool RightArm::stepMoveJ(const VectorXd& target_joints, double max_speed, double accel) {
    VectorXd current_q = getMotorAngle();
    VectorXd diff = target_joints - current_q;
    double max_diff = diff.cwiseAbs().maxCoeff();

    if (max_diff < 0.02) { // 誤差小於 0.02 rad 視為到達
        setJointVelocity(VectorXd::Zero(6));
        return true;
    }
    // 同步速度規劃
    VectorXd cmd_vel = (diff / max_diff) * max_speed * accel;
    setJointVelocity(cmd_vel);
    return false;
}

bool RightArm::stepIK(double ox_deg, double oy_deg, double oz_deg,
                   double px, double py, double pz,  double acceleration_factor)
{
    Vector3d pos(px,py,pz);
   
    setTargetOrientation(ox_deg,oy_deg,oz_deg);
    std::cout<< "Target Orientation (deg): "<<ox_deg<<","<< oy_deg<< ","<< oz_deg<< std::endl;
    // std::cout<< "Target Orientation (rad): "<< target_orientation_.transpose() << std::endl;
    setTargetPosition(pos[0],pos[1],pos[2]);
    last_position_ = getTargetPosition();
    updateRobotPose();
    // std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_TIME_));
    Vector3d ori_rad(ox_deg*M_PI/180.0, oy_deg*M_PI/180.0, oz_deg*M_PI/180.0);
    std::cout << "Target Position (base frame): " << getTargetPosition().transpose() << std::endl; 
    std::cout << "Current Position (base frame): " << getCurrentPosition().transpose() << std::endl;
    std::cout << "Target Orientation (rad): " << ori_rad.transpose() << std::endl;
    std::cout << "Current Orientation (rad): " << current_orientation_.transpose() << std::endl; 
    // Compute quaternions
    Eigen::Quaterniond q_target = Eigen::AngleAxisd(ori_rad[0], Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(ori_rad[1], Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(ori_rad[2], Eigen::Vector3d::UnitZ());
    q_target.normalize();
    // std::cout << "q_target: " << q_target.coeffs().transpose() << std::endl;
    Eigen::Quaterniond q_current = Eigen::AngleAxisd(current_orientation_[0], Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(current_orientation_[1], Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(current_orientation_[2], Eigen::Vector3d::UnitZ());
    q_current.normalize();

    // std::cout << "q_current: " << q_current.coeffs().transpose() << std::endl;

    double dot = q_current.dot(q_target);
    if (dot < 0.0) {
        q_target.coeffs() *= -1;
    }
    Eigen::Quaterniond q_err = q_target * q_current.conjugate();
    q_err.normalize();  // 確保正規化
    // 直接使用 AngleAxis 更穩定
    Eigen::AngleAxisd angle_axis(q_err);
    double angle = angle_axis.angle();
    Vector3d axis = angle_axis.axis();
    // std::cout << "Axis: " << axis.transpose() << std::endl;
    // std::cout << "Angle (rad): " << angle << std::endl;
    
    Vector3d error_ang_vec = axis * angle;
    double angular_error_norm = error_ang_vec.norm();

    Vector3d error_pos = getTargetPosition() - getCurrentPosition();
    double linear_error_norm = error_pos.norm();
    // 收斂條件也只檢查位置
    if(linear_error_norm < linear_threshold_ && angular_error_norm < angular_threshold_) {
        setErrorIntegral(VectorXd::Zero(6));
        return true;
    }
     // === 測試：完全關閉姿態控制 ===
    std::cout << "acceleration_factor: "<<acceleration_factor<<std::endl; // acceleration_factor = 0.3
    


    // VectorXd w = VectorXd::Zero(3);  // 強制角速度為 0
    // VectorXd w = vel_gain_angular_ * error_ang_vec  ;
    // VectorXd v = vel_gain_linear_ * error_pos  ;
    VectorXd w = vel_gain_angular_ * error_ang_vec  * acceleration_factor ;
    VectorXd v = vel_gain_linear_ * error_pos   * acceleration_factor ;
    // test1
    std::cout << "w: " << w.transpose() << std::endl;
    std::cout << "v: " << v.transpose() << std::endl;

    std::cout << "Angular Error (rad): " << error_ang_vec.transpose() << " | Norm: " << angular_error_norm << std::endl;
    std::cout << "Linear Error (mm): " << error_pos.transpose() << " | Norm: " << linear_error_norm << std::endl;

    
    // if(linear_error_norm > 150) v *= 1.5;
    
    if(linear_error_norm < 50) {
        v *= 0.1; //0.7
        std::cout << "Slowing down for precision (linear error < 50mm)" << std::endl;
    }

    if (v == Vector3d::Zero() ) {
        w *= 1.2;
    }


    Eigen::Matrix<double,6,1> xdot_;
    xdot_.head<3>() = w;
    xdot_.tail<3>() = v;
    
    // std::cout << "Velocity Command (rad/s & mm/s): " << xdot_.transpose() << std::endl;


    setJointVelocity(xdot_);
    checkMotorLimit();

    // Deadzone
    for(int i=0;i<6;++i)
        if(std::abs(getJointVelocity()[i])< 1e-6) //1e-5
            setJointIdxVelocity(0.0,i);

    // Stall detection
    Vector3d curr = getCurrentPosition();
    if((last_position_-curr).norm() < 0.01) {
        if(++stop_counter_ > 50) {
            std::cout << "Motion stalled, resetting acceleration factor" << std::endl;
            stop_counter_ = 0;
            return true;
        }
    } else stop_counter_ = 0;

    if (getJointVelocity()[0] == 0.0 && getJointVelocity()[1] == 0.0 && getJointVelocity()[2] == 0.0 &&
        getJointVelocity()[3] == 0.0 && getJointVelocity()[4] == 0.0 && getJointVelocity()[5] == 0.0) {
        std::cout << "All joint velocities are zero, possible singularity or limit reached." << std::endl;
        return true;
    }

    
    std::cout<< "Final getJointVelocity(): "<< getJointVelocity().transpose()<<std::endl;
    return false;
}

bool RightArm::step_move(double ox_deg, double oy_deg, double oz_deg, double px, double py, double pz, double acceleration_factor) 
{
    updateRobotPose();
    Finished = stepIK(ox_deg, oy_deg, oz_deg, px,  py, pz,  acceleration_factor);
    // Finished = stepIK(ox_deg, oy_deg, oz_deg, px,  py, pz, acceleration_counter, acceleration_factor, prev_qdot);
    is_working_ = true;
    std::cout << "Joint Velocity (rad/s): " << getJointVelocity().transpose() << std::endl;
    if (Finished) {
        Stop_all_motors();
        std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_TIME_));
        return true;
    }else{
        SetArmVelocity(getIdxJointVelocity(0)*pro_radpersec2scale_,
                       getIdxJointVelocity(1)*pro_radpersec2scale_,
                        getIdxJointVelocity(2)*pro_radpersec2scale_,
                        getIdxJointVelocity(3)*pro_radpersec2scale_,
                        getIdxJointVelocity(4)*pro_radpersec2scale_,
                        getIdxJointVelocity(5)*pro_radpersec2scale_);

        
        // std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_TIME_));
        // std::cout << "Set velocity command to motors." << std::endl;
        // Stop_all_motors();
        // std::this_thread::sleep_for(std::chrono::milliseconds(70));
        // int a = 0;
        // std::cin >> a;


        //=============================================
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // std::cout << "Set velocity command to motors." << std::endl;
        // Stop_all_motors();
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // int a = 0;
        // std::cin >> a;
        
        
    }
    
   
    return false;
}

void RightArm::trajectory_planning(double ox, double oy, double oz, double px, double py, double pz)
{
    
    auto start_time = std::chrono::steady_clock::now();
    const double timeout = 1000.0; // seconds
    double acceleration_factor = 0.005;

    int iteration = 0;
    Eigen::Matrix<double,6,1> prev_qdot = Eigen::Matrix<double,6,1>::Zero();


    // if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < timeout) {
    while  (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < timeout) {
    // while (true) {
        checkConnected();
        iteration += 1;
        if (acceleration_factor < 1.0 && iteration % 100 == 0)
            acceleration_factor = std::min(1.0d, acceleration_factor + 0.001d);
        if (step_move(ox, oy, oz, px, py, pz,  acceleration_factor)) {
        // if (step_move(ox, oy, oz, px, py, pz, acceleration_counter, acceleration_factor, prev_qdot)) {
            Eigen::Vector3d target_position(px, py, pz);
            double error = (target_position - getCurrentPosition()).norm();
            std::cout << "\t[INFO] Move is over." << std::endl;
            std::cout << "Target reached! distance error: " << error << " mm (iter: " << iteration << ")" << std::endl;
            std::cout << "acceleration_factor final: "<<acceleration_factor<<std::endl;
            updateRobotPose();
            Stop_all_motors();
            is_working_ = false;
            break;
            
        }
        
        
    }
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() > timeout) {
        
        std::cout << "Timeout: Failed to reach the target within the time limit." << std::endl;
        updateRobotPose();
    }
        
    is_working_ = false;
}

void RightArm::Trajectory_Planning(double ox_deg, double oy_deg, double oz_deg, double X, double Y, double Z, std::string pick_mode)
{
    if(is_working_) std::cout << "\t[INFO] Arm is moving. Wait for the arm to stop!" << std::endl;

    else {
        
        Vector3d POS(X,Y,Z);
        Vector3d ori_deg(ox_deg, oy_deg, oz_deg);
        std::cout << "\t[INFO] Start trajectory planning to (" << X << ", " << Y << ", " << Z << ")"
                  << " with orientation (" << ori_deg[0] << ", " << ori_deg[1] << ", " << ori_deg[2] << ")"
                  << " in " << pick_mode << " mode." << std::endl;
        // std::this_thread::sleep_for(std::chrono::seconds(500));
        auto workspace_result = checkPositionWorkspace(POS, ori_deg, pick_mode, "left");
        POS       = workspace_result.first;                // Eigen::Vector3d
        ori_deg   = workspace_result.second.first;         // Eigen::Vector3d
        pick_mode = workspace_result.second.second;        // std::string
        
        Vector4d pos = transformToBasePosition({POS[0], POS[1], POS[2]});
        
        trajectory_planning(ori_deg[0], ori_deg[1], ori_deg[2], pos[0], pos[1], pos[2]);
    }
    
}
void RightArm::mode_controll(double X, double Y, double Z, std::string pick_mode, double angle)
{
    if (pick_mode == "down" || pick_mode == "side" || pick_mode == "forward" || pick_mode == "reversal" || pick_mode == "side_forward") {
        std::cout << "\t[INFO] Selected pick mode: " << pick_mode << std::endl;
    } else {
        std::cout << "\t[WARNING] Unknown pick mode: " << pick_mode << ". Defaulting to 'side' mode." << std::endl;
        pick_mode = "side";
    }
    auto p = getPickTypePose("right", pick_mode, angle);
    Vector3d ori_deg = p.first;
    pick_mode = p.second;

    if(is_working_) std::cout << "\t[INFO] Arm is moving. Wait for the arm to stop!" << std::endl;
    else {
        Vector3d POS(X,Y,Z);
        std::cout << "\t[INFO] Start trajectory planning to (" << X << ", " << Y << ", " << Z << ")"
                  << " with orientation (" << ori_deg[0] << ", " << ori_deg[1] << ", " << ori_deg[2] << ")"
                  << " in " << pick_mode << " mode." << std::endl;
        // std::this_thread::sleep_for(std::chrono::seconds(500));
        auto workspace_result = checkPositionWorkspace(POS, ori_deg, pick_mode, "right");
        POS       = workspace_result.first;                // Eigen::Vector3d
        ori_deg   = workspace_result.second.first;         // Eigen::Vector3d
        pick_mode = workspace_result.second.second;        // std::string
        Vector4d pos = transformToBasePosition({POS[0], POS[1], POS[2]});
        std::cout << "Transformed base position: " << pos.transpose() << std::endl;
        std::cout << "Using orientation (deg): " << ori_deg.transpose() << " in " << pick_mode << " mode." << std::endl;
        trajectory_planning(ori_deg[0], ori_deg[1], ori_deg[2], pos[0], pos[1], pos[2]);
    }
    
}



void RightArm::setInitialPose() {
    updateRobotPose();
    double init_x = 420;
    double init_y = -120;
    double init_z = -130;
    // double init_ox = left_orientation_deg_[0];
    // double init_oy = left_orientation_deg_[1];
    // double init_oz = left_orientation_deg_[2];
    double init_ox = 0.0;
    double init_oy = 180.0-35.0; // 往外偏 35 度
    double init_oz = 0.0;
    Trajectory_Planning(init_ox, init_oy, init_oz, init_x, init_y, -200, "side");
    Trajectory_Planning(init_ox, init_oy, init_oz, init_x, init_y, init_z, "side");
    openGripper();
}

void RightArm::setpositionPose(double x, double y, double z) {
    updateRobotPose();
    double init_ox = right_orientation_deg_[0];
    double init_oy = right_orientation_deg_[1];
    double init_oz = right_orientation_deg_[2];
    // double init_ox = -130;
    // double init_oy = 0;
    // double init_oz = 0;
    Trajectory_Planning(init_ox, init_oy, init_oz, x, y, z, "side");
}
void RightArm::updateRobotPose() {
    update();
    
    setMotorAngle(1, motor_angles_[1]+6.48*M_PI/180.0); // 6.48 degree offset
    setMotorAngle(2, motor_angles_[2]-6.48*M_PI/180.0); // 6.48 degree offset

    if (motor_angles_[1] < -M_PI) {
        std::cout<<motor_angles_[1]<<std::endl;
        std::cout << "Motor 1 angle is less than PI" << std::endl;
        setMotorAngle(1, motor_angles_[1] + 2 * M_PI);
    }
    if (motor_angles_[2] > M_PI) {
        std::cout << "Motor 2 angle is greater than PI" << std::endl;
        setMotorAngle(2, motor_angles_[2] + 2 * M_PI);
    }
    setDeltaAngle(motor_angles_);
    forwardKinematics(); 
    jacobian();
    Eigen::Vector4d pos4  = transformToWorldPosition(getCurrentPosition());
    right_world_pos_ = pos4.head<3>();
    right_orientation_deg_ = getCurrentOrientationInDegree();
    getArmInformation();
    
}
void RightArm::getArmInformation() const {
    cout<<"==================================="<<endl;
    cout<<"Current right base Position = "<<getCurrentPosition().transpose()<<endl;
    cout<<"Current right world Position = "<<right_world_pos_.transpose()<<endl;
    cout<<"Current right Orientation(deg) = "<<right_orientation_deg_.transpose()<<endl;
    cout<<"Current right Joint Angles(deg) = "<<(getMotorAngle()*180.0/M_PI).transpose()<<endl;
    cout<<"==================================="<<endl;
}







/* Implementation that can be used whenever you need a transform matrix.

 * @param const float &theta - yeah, it's theta in DH table
 * @param const float &alpha - alpha in DH table
 * @param const float &a - a in DH table
 * @param const float &d - d in DH table
 * 
 * @retval Eigen::Matrix<float, 4, 4> transform_matrix - full transform matrix from i-1 to i
 */




bool RightArm::GetWorkingState() { return is_working_; }



int RightArm::Sign(float x) { return (x >= 0 ? 1 : -1); }

