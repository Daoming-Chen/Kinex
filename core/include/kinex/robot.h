#pragma once

#include "kinex/export.h"
#include "kinex/robot_model.h"
#include "kinex/kinematics.h"
#include "kinex/inverse_kinematics.h"
#include <string>
#include <memory>
#include <optional>
#include <unordered_map>

namespace kinex {

/**
 * @brief Unified Robot interface for kinematics and dynamics
 * 
 * This class provides a high-level interface for robot operations,
 * encapsulating the structural model (RobotModel) and various solvers
 * (ForwardKinematics, JacobianCalculator, IKSolver).
 * 
 * Usage:
 * @code
 * auto robot = Robot::fromURDF("robot.urdf", "tool0");
 * auto pose = robot.forwardKinematics(q);
 * auto [sol, status] = robot.inverseKinematics(target_pose, q_init);
 * @endcode
 */
class KINEX_API Robot {
public:
    // Static factories
    static Robot fromURDF(const std::string& filepath, 
                         const std::string& end_link, 
                         const std::string& base_link = "");
                         
    static Robot fromURDFString(const std::string& urdf_string, 
                               const std::string& end_link, 
                               const std::string& base_link = "");

    // Constructors
    Robot(std::shared_ptr<const RobotModel> model,
          const std::string& end_link,
          const std::string& base_link = "");
          
    // Copy operations (deep copy via clone recommended)
    Robot(const Robot& other) = delete;
    Robot& operator=(const Robot& other) = delete;
    
    // Move operations
    Robot(Robot&&) = default;
    Robot& operator=(Robot&&) = default;
    
    /**
     * @brief Create a deep copy of the robot (thread-safe independent instance)
     * 
     * Copies the RobotModel and configuration, but NOT the solver states.
     * Useful for creating thread-local instances for multi-threaded applications.
     */
    Robot clone() const;

    // Forward Kinematics
    Transform forwardKinematics(const Eigen::VectorXd& q, const std::string& link = "") const;
    Transform computePose(const Eigen::VectorXd& q, const std::string& link = "") const; // Alias
    
    /**
     * @brief Compute transforms for all links in the robot (visual/collision update)
     */
    std::unordered_map<std::string, Transform> computeAllLinkTransforms(const Eigen::VectorXd& q) const;

    // Inverse Kinematics
    std::pair<Eigen::VectorXd, SolverStatus> inverseKinematics(
        const Transform& target, 
        const Eigen::VectorXd& q_init, 
        const std::string& link = "");
        
    std::pair<Eigen::VectorXd, SolverStatus> solveIK(
        const Transform& target, 
        const Eigen::VectorXd& q_init, 
        const std::string& link = ""); // Alias

    // Jacobian
    Eigen::MatrixXd computeJacobian(
        const Eigen::VectorXd& q, 
        const std::string& link = "", 
        JacobianType type = JacobianType::Analytic) const;
        
    double getManipulability(const Eigen::VectorXd& q, const std::string& link = "") const;
    bool isSingular(const Eigen::VectorXd& q, double threshold = 1e-6, const std::string& link = "") const;
    double getConditionNumber(const Eigen::VectorXd& q, const std::string& link = "") const;

    // Configuration
    void setIKTolerance(double tol);
    void setPositionOnlyIK(bool enable);
    void setOrientationOnlyIK(bool enable);
    void setSolverConfig(const SolverConfig& config);
    SolverConfig getSolverConfig() const;

    // Accessors
    std::shared_ptr<const RobotModel> getRobotModel() const { return model_; }
    const std::string& getName() const { return model_->getName(); }
    const std::string& getEndLink() const { return end_link_; }
    const std::string& getBaseLink() const { return base_link_; }
    
    /**
     * @brief Get number of Degrees of Freedom (actuated joints)
     */
    size_t getDOF() const;

private:
    std::shared_ptr<const RobotModel> model_;
    std::string end_link_;
    std::string base_link_;
    
    // Solver configuration (persisted across lazy instantiations)
    SolverConfig ik_config_;
    bool ik_position_only_ = false;
    bool ik_orientation_only_ = false;

    // Lazy-initialized solvers
    mutable std::unique_ptr<ForwardKinematics> fk_;
    mutable std::unordered_map<std::string, std::unique_ptr<ForwardKinematics>> fk_cache_;
    
    mutable std::unique_ptr<JacobianCalculator> jacobian_;
    mutable std::unordered_map<std::string, std::unique_ptr<JacobianCalculator>> jacobian_cache_;
    
    mutable std::unique_ptr<SQPIKSolver> ik_solver_;
    mutable std::unordered_map<std::string, std::unique_ptr<SQPIKSolver>> ik_cache_;

    // Helper methods
    const std::string& resolveLink(const std::string& link) const;
    ForwardKinematics& ensureFK(const std::string& link) const;
    JacobianCalculator& ensureJacobian(const std::string& link) const;
    SQPIKSolver& ensureIKSolver(const std::string& link);
};

} // namespace kinex
