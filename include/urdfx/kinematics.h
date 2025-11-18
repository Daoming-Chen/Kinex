#pragma once

#include "robot_model.h"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <string>
#include <optional>

namespace urdfx {

/**
 * @brief Represents a kinematic chain from base to end-effector
 * 
 * A kinematic chain is an ordered sequence of joints and their associated
 * transformations from a base link to an end-effector link. This class
 * pre-computes static transformations for efficiency.
 */
class KinematicChain {
public:
    /**
     * @brief Construct a kinematic chain
     * @param robot The robot model
     * @param end_link Name of the end-effector link
     * @param base_link Name of the base link (defaults to robot root)
     */
    KinematicChain(
        std::shared_ptr<const Robot> robot,
        const std::string& end_link,
        const std::string& base_link = "");
    
    /**
     * @brief Get the number of actuated joints in the chain
     */
    size_t getNumJoints() const { return joints_.size(); }
    
    /**
     * @brief Get the ordered list of joints in the chain
     */
    const std::vector<std::shared_ptr<Joint>>& getJoints() const { return joints_; }
    
    /**
     * @brief Get the ordered list of link names in the chain
     */
    const std::vector<std::string>& getLinkNames() const { return link_names_; }
    
    /**
     * @brief Get the name of the end-effector link
     */
    const std::string& getEndLink() const { return end_link_; }
    
    /**
     * @brief Get the name of the base link
     */
    const std::string& getBaseLink() const { return base_link_; }
    
    /**
     * @brief Get pre-computed static transforms for each joint
     * These are the fixed transformations from parent to child link frames
     */
    const std::vector<Transform>& getStaticTransforms() const { return static_transforms_; }

private:
    std::shared_ptr<const Robot> robot_;
    std::string base_link_;
    std::string end_link_;
    
    // Ordered list of actuated joints from base to end
    std::vector<std::shared_ptr<Joint>> joints_;
    
    // Ordered list of link names from base to end
    std::vector<std::string> link_names_;
    
    // Pre-computed static transformations for each joint
    std::vector<Transform> static_transforms_;
    
    /**
     * @brief Build the kinematic chain by traversing from end to base
     */
    void buildChain();
    
    /**
     * @brief Find path from end_link to base_link
     * @return Ordered list of joints from base to end
     */
    std::vector<std::shared_ptr<Joint>> findPath(
        const std::string& from_link,
        const std::string& to_link);
};

/**
 * @brief Computes forward kinematics for a robot
 * 
 * Given joint angles, computes the pose of the end-effector or any
 * intermediate link in the kinematic chain.
 */
class ForwardKinematics {
public:
    /**
     * @brief Construct a forward kinematics solver
     * @param robot The robot model
     * @param end_link Name of the end-effector link
     * @param base_link Name of the base link (defaults to robot root)
     */
    ForwardKinematics(
        std::shared_ptr<const Robot> robot,
        const std::string& end_link,
        const std::string& base_link = "");
    
    /**
     * @brief Compute forward kinematics for given joint angles
     * @param joint_angles Vector of joint angles (size must match number of joints)
     * @param check_bounds If true, verify joint angles are within limits (default: false)
     * @return Transform from base to end-effector
     * @throws std::invalid_argument if joint_angles size is incorrect
     * @throws std::runtime_error if bounds check fails and check_bounds is true
     */
    Transform compute(
        const Eigen::VectorXd& joint_angles,
        bool check_bounds = false) const;
    
    /**
     * @brief Compute forward kinematics to an intermediate link
     * @param joint_angles Vector of joint angles
     * @param target_link Name of the target link
     * @param check_bounds If true, verify joint angles are within limits
     * @return Transform from base to target link
     * @throws std::invalid_argument if target_link is not in the chain
     */
    Transform computeToLink(
        const Eigen::VectorXd& joint_angles,
        const std::string& target_link,
        bool check_bounds = false) const;
    
    /**
     * @brief Get the kinematic chain
     */
    const KinematicChain& getChain() const { return chain_; }
    
    /**
     * @brief Get the number of joints in the chain
     */
    size_t getNumJoints() const { return chain_.getNumJoints(); }

private:
    std::shared_ptr<const Robot> robot_;
    KinematicChain chain_;
    
    /**
     * @brief Check if joint angles are within limits
     * @throws std::runtime_error if any joint is out of bounds
     */
    void checkJointLimits(const Eigen::VectorXd& joint_angles) const;
    
    /**
     * @brief Compute FK up to a specific joint index
     * @param joint_angles Vector of joint angles
     * @param end_joint_idx Index of the last joint to include (inclusive)
     * @return Transform from base to the link after end_joint_idx
     */
    Transform computeToJointIndex(
        const Eigen::VectorXd& joint_angles,
        size_t end_joint_idx) const;
};

} // namespace urdfx
