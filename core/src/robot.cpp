#include "kinex/robot.h"
#include "kinex/urdf_parser.h"
#include <stdexcept>

namespace kinex {

Robot Robot::fromURDF(const std::string &filepath, const std::string &end_link,
                      const std::string &base_link) {
  URDFParser parser;
  auto model = parser.parseFile(filepath);
  return Robot(model, end_link, base_link);
}

Robot Robot::fromURDFString(const std::string &urdf_string,
                            const std::string &end_link,
                            const std::string &base_link) {
  URDFParser parser;
  auto model = parser.parseString(urdf_string);
  return Robot(model, end_link, base_link);
}

Robot::Robot(std::shared_ptr<const RobotModel> model,
             const std::string &end_link, const std::string &base_link)
    : model_(std::move(model)), end_link_(end_link), base_link_(base_link) {
  if (!model_) {
    throw std::invalid_argument("RobotModel cannot be null");
  }

  // Use model root if base_link is empty
  if (base_link_.empty()) {
    base_link_ = model_->getRootLink();
  }

  // Validate links exist
  if (!model_->getLink(base_link_)) {
    throw std::invalid_argument("Base link '" + base_link_ +
                                "' not found in model");
  }
  if (!model_->getLink(end_link_)) {
    throw std::invalid_argument("End link '" + end_link_ +
                                "' not found in model");
  }
}

Robot Robot::clone() const {
  // Deep copy the model (structure) but not the solvers
  auto cloned_model = std::make_shared<RobotModel>(*model_);

  Robot cloned(cloned_model, end_link_, base_link_);

  // Copy configuration
  cloned.setSolverConfig(ik_config_);
  cloned.setGlobalSolverConfig(global_ik_config_);
  cloned.setPositionOnlyIK(ik_position_only_);
  cloned.setOrientationOnlyIK(ik_orientation_only_);

  return cloned;
}

Transform Robot::forwardKinematics(const Eigen::VectorXd &q,
                                   const std::string &link) const {
  const std::string &target_link = resolveLink(link);
  return ensureFK(target_link).compute(q);
}

Transform Robot::computePose(const Eigen::VectorXd &q,
                             const std::string &link) const {
  return forwardKinematics(q, link);
}

std::unordered_map<std::string, Transform>
Robot::computeAllLinkTransforms(const Eigen::VectorXd &q) const {
  // Use the FK solver associated with the end-effector, but it doesn't matter
  // which one we use for full tree traversal as long as it has the same base.
  // However, ensureFK creates a chain. computeAllLinkTransforms in FK class
  // traverses the whole tree starting from base, so we need an FK solver
  // initialized with the base. The end link doesn't restrict the full tree
  // computation in the current implementation of ForwardKinematics (it computes
  // for the whole model).
  return ensureFK(end_link_).computeAllLinkTransforms(q);
}

std::pair<Eigen::VectorXd, SolverStatus>
Robot::inverseKinematics(const Transform &target, const Eigen::VectorXd &q_init,
                         const std::string &link) {

  const std::string &target_link = resolveLink(link);
  Eigen::VectorXd solution;
  auto status = ensureIKSolver(target_link).solve(target, q_init, solution);
  return {solution, status};
}

std::pair<Eigen::VectorXd, SolverStatus>
Robot::solveIK(const Transform &target, const Eigen::VectorXd &q_init,
               const std::string &link) {
  return inverseKinematics(target, q_init, link);
}

std::pair<Eigen::VectorXd, SolverStatus>
Robot::solveRobustIK(const Transform &target, const Eigen::VectorXd &q_init,
                     const std::string &link) {
  const std::string &target_link = resolveLink(link);
  
  // Configure for robust mode (single solution)
  GlobalSolverConfig config = global_ik_config_;
  config.return_all_solutions = false;
  config.sqp_config = ik_config_; // Sync SQP config
  
  auto &solver = ensureGlobalIKSolver(target_link);
  solver.setConfig(config);
  
  GlobalSolverResult result = solver.solve(target, q_init);
  
  if (result.success && !result.solutions.empty()) {
    return {result.solutions[0], result.best_status};
  }
  
  // If failed, return the best attempt or just failure
  if (!result.statuses.empty()) {
      return {result.solutions.empty() ? q_init : result.solutions[0], result.best_status};
  }
  
  SolverStatus fail_status;
  fail_status.converged = false;
  return {q_init, fail_status};
}

GlobalSolverResult
Robot::solveGlobalIK(const Transform &target, const Eigen::VectorXd &q_init,
                     const std::string &link) {
  const std::string &target_link = resolveLink(link);
  
  // Configure for global mode (all solutions)
  GlobalSolverConfig config = global_ik_config_;
  config.return_all_solutions = true;
  config.sqp_config = ik_config_; // Sync SQP config
  
  auto &solver = ensureGlobalIKSolver(target_link);
  solver.setConfig(config);
  
  return solver.solve(target, q_init);
}

Eigen::MatrixXd Robot::computeJacobian(const Eigen::VectorXd &q,
                                       const std::string &link,
                                       JacobianType type) const {

  const std::string &target_link = resolveLink(link);
  return ensureJacobian(target_link).compute(q, type);
}

double Robot::getManipulability(const Eigen::VectorXd &q,
                                const std::string &link) const {
  const std::string &target_link = resolveLink(link);
  return ensureJacobian(target_link).getManipulability(q);
}

bool Robot::isSingular(const Eigen::VectorXd &q, double threshold,
                       const std::string &link) const {
  const std::string &target_link = resolveLink(link);
  return ensureJacobian(target_link).isSingular(q, threshold);
}

double Robot::getConditionNumber(const Eigen::VectorXd &q,
                                 const std::string &link) const {
  const std::string &target_link = resolveLink(link);
  return ensureJacobian(target_link).getConditionNumber(q);
}

void Robot::setIKTolerance(double tol) {
  ik_config_.tolerance = tol;
  // Update existing solvers
  if (ik_solver_)
    ik_solver_->setSolverConfig(ik_config_);
  for (auto &[link, solver] : ik_cache_) {
    solver->setSolverConfig(ik_config_);
  }
}

void Robot::setPositionOnlyIK(bool enable) {
  ik_position_only_ = enable;
  if (ik_solver_)
    ik_solver_->setPositionOnly(enable);
  for (auto &[link, solver] : ik_cache_) {
    solver->setPositionOnly(enable);
  }
}

void Robot::setOrientationOnlyIK(bool enable) {
  ik_orientation_only_ = enable;
  if (ik_solver_)
    ik_solver_->setOrientationOnly(enable);
  for (auto &[link, solver] : ik_cache_) {
    solver->setOrientationOnly(enable);
  }
}

void Robot::setSolverConfig(const SolverConfig &config) {
  ik_config_ = config;
  if (ik_solver_)
    ik_solver_->setSolverConfig(config);
  for (auto &[link, solver] : ik_cache_) {
    solver->setSolverConfig(config);
  }
}

SolverConfig Robot::getSolverConfig() const { return ik_config_; }

void Robot::setGlobalSolverConfig(const GlobalSolverConfig &config) {
  global_ik_config_ = config;
  // Update existing solvers
  if (global_ik_solver_)
    global_ik_solver_->setConfig(config);
  for (auto &[link, solver] : global_ik_cache_) {
    solver->setConfig(config);
  }
}

GlobalSolverConfig Robot::getGlobalSolverConfig() const {
  return global_ik_config_;
}

size_t Robot::getDOF() const { return model_->getActuatedJoints().size(); }

const std::string &Robot::resolveLink(const std::string &link) const {
  if (link.empty()) {
    return end_link_;
  }
  return link;
}

ForwardKinematics &Robot::ensureFK(const std::string &link) const {
  if (link == end_link_) {
    if (!fk_) {
      fk_ = std::make_unique<ForwardKinematics>(model_, end_link_, base_link_);
    }
    return *fk_;
  }

  auto it = fk_cache_.find(link);
  if (it == fk_cache_.end()) {
    auto fk = std::make_unique<ForwardKinematics>(model_, link, base_link_);
    it = fk_cache_.emplace(link, std::move(fk)).first;
  }
  return *it->second;
}

JacobianCalculator &Robot::ensureJacobian(const std::string &link) const {
  if (link == end_link_) {
    if (!jacobian_) {
      jacobian_ =
          std::make_unique<JacobianCalculator>(model_, end_link_, base_link_);
    }
    return *jacobian_;
  }

  auto it = jacobian_cache_.find(link);
  if (it == jacobian_cache_.end()) {
    auto jac = std::make_unique<JacobianCalculator>(model_, link, base_link_);
    it = jacobian_cache_.emplace(link, std::move(jac)).first;
  }
  return *it->second;
}

SQPIKSolver &Robot::ensureIKSolver(const std::string &link) {
  if (link == end_link_) {
    if (!ik_solver_) {
      ik_solver_ = std::make_unique<SQPIKSolver>(model_, end_link_, base_link_);
      ik_solver_->setSolverConfig(ik_config_);
      ik_solver_->setPositionOnly(ik_position_only_);
      ik_solver_->setOrientationOnly(ik_orientation_only_);
    }
    return *ik_solver_;
  }

  auto it = ik_cache_.find(link);
  if (it == ik_cache_.end()) {
    auto ik = std::make_unique<SQPIKSolver>(model_, link, base_link_);
    ik->setSolverConfig(ik_config_);
    ik->setPositionOnly(ik_position_only_);
    ik->setOrientationOnly(ik_orientation_only_);
    it = ik_cache_.emplace(link, std::move(ik)).first;
  }
  return *it->second;
}

GlobalIKSolver &Robot::ensureGlobalIKSolver(const std::string &link) {
  if (link == end_link_) {
    if (!global_ik_solver_) {
      global_ik_solver_ =
          std::make_unique<GlobalIKSolver>(model_, end_link_, base_link_);
      global_ik_solver_->setConfig(global_ik_config_);
    }
    return *global_ik_solver_;
  }

  auto it = global_ik_cache_.find(link);
  if (it == global_ik_cache_.end()) {
    auto solver = std::make_unique<GlobalIKSolver>(model_, link, base_link_);
    solver->setConfig(global_ik_config_);
    it = global_ik_cache_.emplace(link, std::move(solver)).first;
  }
  return *it->second;
}

} // namespace kinex