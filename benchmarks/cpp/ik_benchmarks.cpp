#include <benchmark/benchmark.h>

#include "urdfx/inverse_kinematics.h"
#include "urdfx/kinematics.h"
#include "urdfx/logging.h"
#include "urdfx/urdf_parser.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <random>
#include <stdexcept>
#include <vector>

using namespace urdfx;

namespace {

std::filesystem::path resolveUrdfPath() {
#ifdef URDFX_UR5_URDF_PATH
    std::filesystem::path configured{URDFX_UR5_URDF_PATH};
    if (std::filesystem::exists(configured)) {
        return configured;
    }
#endif
    auto fallback = std::filesystem::path(__FILE__).parent_path().parent_path() / "tests" / "ur5_urdf" / "ur5e.urdf";
    if (!std::filesystem::exists(fallback)) {
        throw std::runtime_error("Unable to locate UR5e URDF for benchmarks");
    }
    return fallback;
}

std::string selectEndLink(const std::shared_ptr<Robot>& robot) {
    static const std::vector<std::string> preferred = {
        "tool0",
        "ee_link",
        "wrist_3_link"
    };
    for (const auto& name : preferred) {
        if (robot->getLink(name)) {
            return name;
        }
    }
    for (const auto& link : robot->getLinks()) {
        if (link->getName() != robot->getRootLink()) {
            return link->getName();
        }
    }
    return robot->getRootLink();
}

struct PoseSample {
    Transform pose;
    Eigen::VectorXd nominal;
};

class IKBenchmarkFixture {
public:
    IKBenchmarkFixture() {
        // Keep benchmark output clean.
        setLogLevel(spdlog::level::err);

        URDFParser parser;
        auto urdf_path = resolveUrdfPath();
        robot_ = parser.parseFile(urdf_path.string());
        if (!robot_) {
            throw std::runtime_error("Failed to parse URDF at " + urdf_path.string());
        }

        base_link_ = robot_->getRootLink();
        end_link_ = selectEndLink(robot_);

        ForwardKinematics fk(robot_, end_link_, base_link_);
        dof_ = fk.getNumJoints();
        zero_config_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dof_));

        computeJointLimits(fk);
        preparePrimaryTargets(fk);
        prepareTrajectoryTargets(fk);

        base_config_ = SolverConfig{};
        base_config_.tolerance = 5e-4;
        base_config_.max_iterations = 64;
        base_config_.enable_warm_start = true;
        base_config_.regularization = 1e-5;
    }

    std::unique_ptr<SQPIKSolver> makeSolver(bool warm_start_enabled) const {
        auto solver = std::make_unique<SQPIKSolver>(robot_, end_link_, base_link_);
        SolverConfig cfg = base_config_;
        cfg.enable_warm_start = warm_start_enabled;
        solver->setSolverConfig(cfg);
        return solver;
    }

    size_t dof() const { return dof_; }
    const Eigen::VectorXd& zero() const { return zero_config_; }
    const std::vector<PoseSample>& targets() const { return primary_targets_; }
    const std::vector<Transform>& trajectoryTargets() const { return trajectory_poses_; }

private:
    void computeJointLimits(const ForwardKinematics& fk) {
        lower_limits_.resize(static_cast<Eigen::Index>(dof_));
        upper_limits_.resize(static_cast<Eigen::Index>(dof_));
        const auto& joints = fk.getChain().getJoints();
        for (size_t i = 0; i < dof_; ++i) {
            const auto& joint = joints[i];
            if (auto limits = joint->getLimits()) {
                lower_limits_[static_cast<Eigen::Index>(i)] = limits->lower;
                upper_limits_[static_cast<Eigen::Index>(i)] = limits->upper;
            } else {
                constexpr double kPi = std::numbers::pi_v<double>;
                lower_limits_[static_cast<Eigen::Index>(i)] = -kPi;
                upper_limits_[static_cast<Eigen::Index>(i)] = kPi;
            }
        }
    }

    Eigen::VectorXd clamp(const Eigen::VectorXd& q) const {
        return q.cwiseMax(lower_limits_).cwiseMin(upper_limits_);
    }

    void preparePrimaryTargets(const ForwardKinematics& fk) {
        // Sample 1000 random joint configurations within joint limits
        constexpr size_t num_samples = 1000;
        constexpr unsigned int seed = 42;
        std::mt19937 rng(seed);
        
        // Create uniform distribution for each joint
        std::vector<std::uniform_real_distribution<double>> joint_dists;
        joint_dists.reserve(dof_);
        for (size_t i = 0; i < dof_; ++i) {
            joint_dists.emplace_back(
                lower_limits_[static_cast<Eigen::Index>(i)],
                upper_limits_[static_cast<Eigen::Index>(i)]
            );
        }
        
        // Sample random joint configurations and compute FK to get target poses
        primary_targets_.reserve(num_samples);
        for (size_t sample = 0; sample < num_samples; ++sample) {
            Eigen::VectorXd q = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dof_));
            for (size_t joint = 0; joint < dof_; ++joint) {
                q[static_cast<Eigen::Index>(joint)] = joint_dists[joint](rng);
            }
            primary_targets_.push_back({fk.compute(q), q});
        }
    }

    void prepareTrajectoryTargets(const ForwardKinematics& fk) {
        // Sample random joint configurations for trajectory test
        constexpr size_t num_trajectory_samples = 100;
        constexpr unsigned int seed = 43;  // Different seed from primary targets
        std::mt19937 rng(seed);
        
        // Create uniform distribution for each joint
        std::vector<std::uniform_real_distribution<double>> joint_dists;
        joint_dists.reserve(dof_);
        for (size_t i = 0; i < dof_; ++i) {
            joint_dists.emplace_back(
                lower_limits_[static_cast<Eigen::Index>(i)],
                upper_limits_[static_cast<Eigen::Index>(i)]
            );
        }
        
        // Sample random joint configurations and compute FK to get trajectory poses
        trajectory_poses_.reserve(num_trajectory_samples);
        for (size_t sample = 0; sample < num_trajectory_samples; ++sample) {
            Eigen::VectorXd q = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(dof_));
            for (size_t joint = 0; joint < dof_; ++joint) {
                q[static_cast<Eigen::Index>(joint)] = joint_dists[joint](rng);
            }
            trajectory_poses_.push_back(fk.compute(q));
        }
    }

    std::shared_ptr<Robot> robot_;
    std::string base_link_;
    std::string end_link_;
    size_t dof_ = 0;
    Eigen::VectorXd lower_limits_;
    Eigen::VectorXd upper_limits_;
    Eigen::VectorXd zero_config_;
    SolverConfig base_config_;
    std::vector<PoseSample> primary_targets_;
    std::vector<Transform> trajectory_poses_;
};

const IKBenchmarkFixture& fixture() {
    static IKBenchmarkFixture instance;
    return instance;
}

} // namespace

static void BM_IK_ColdStart(benchmark::State& state) {
    const auto& fx = fixture();
    auto solver = fx.makeSolver(false);
    auto fk = std::make_shared<ForwardKinematics>(solver->getRobot(), solver->getEndLink(), solver->getBaseLink());
    Eigen::VectorXd result(static_cast<Eigen::Index>(fx.dof()));
    size_t solves = 0;
    size_t successes = 0;
    size_t total_iterations = 0;
    double total_position_error = 0.0;
    double total_rotation_error = 0.0;

    for (auto _ : state) {
        for (const auto& sample : fx.targets()) {
            const auto status = solver->solve(sample.pose, fx.zero(), result);
            total_iterations += status.iterations;
            successes += status.converged ? 1 : 0;
            
            if (status.converged) {
                Transform achieved = fk->compute(result);
                Eigen::Vector3d pos_error = achieved.translation() - sample.pose.translation();
                total_position_error += pos_error.norm();
                
                Eigen::Quaterniond q_target(sample.pose.rotation());
                Eigen::Quaterniond q_achieved(achieved.rotation());
                double rotation_error = q_target.angularDistance(q_achieved);
                total_rotation_error += rotation_error;
            }
            
            ++solves;
            benchmark::DoNotOptimize(result.data());
        }
    }

    if (solves == 0) {
        state.SkipWithError("No IK target poses configured");
        return;
    }

    state.counters["iterations_per_solve"] = static_cast<double>(total_iterations) / static_cast<double>(solves);
    state.counters["success_rate"] = (static_cast<double>(successes) / static_cast<double>(solves)) * 100.0;
    state.counters["avg_position_error_mm"] = (total_position_error / static_cast<double>(successes)) * 1000.0;
    state.counters["avg_rotation_error_deg"] = (total_rotation_error / static_cast<double>(successes)) * 180.0 / std::numbers::pi;
}

BENCHMARK(BM_IK_ColdStart)->Unit(benchmark::kMicrosecond);

static void BM_IK_WarmStart(benchmark::State& state) {
    const auto& fx = fixture();
    auto solver = fx.makeSolver(true);
    auto fk = std::make_shared<ForwardKinematics>(solver->getRobot(), solver->getEndLink(), solver->getBaseLink());
    Eigen::VectorXd result(static_cast<Eigen::Index>(fx.dof()));

    // Prime warm start outside measured region.
    solver->solve(fx.targets().front().pose, fx.zero(), result);

    size_t solves = 0;
    size_t successes = 0;
    size_t total_iterations = 0;
    double total_position_error = 0.0;
    double total_rotation_error = 0.0;

    for (auto _ : state) {
        for (const auto& sample : fx.targets()) {
            const auto status = solver->solve(sample.pose, Eigen::VectorXd(), result);
            total_iterations += status.iterations;
            successes += status.converged ? 1 : 0;
            
            if (status.converged) {
                Transform achieved = fk->compute(result);
                Eigen::Vector3d pos_error = achieved.translation() - sample.pose.translation();
                total_position_error += pos_error.norm();
                
                Eigen::Quaterniond q_target(sample.pose.rotation());
                Eigen::Quaterniond q_achieved(achieved.rotation());
                double rotation_error = q_target.angularDistance(q_achieved);
                total_rotation_error += rotation_error;
            }
            
            ++solves;
            benchmark::DoNotOptimize(result.data());
        }
    }

    if (solves == 0) {
        state.SkipWithError("No IK target poses configured");
        return;
    }

    state.counters["iterations_per_solve"] = static_cast<double>(total_iterations) / static_cast<double>(solves);
    state.counters["success_rate"] = (static_cast<double>(successes) / static_cast<double>(solves)) * 100.0;
    state.counters["avg_position_error_mm"] = (total_position_error / static_cast<double>(successes)) * 1000.0;
    state.counters["avg_rotation_error_deg"] = (total_rotation_error / static_cast<double>(successes)) * 180.0 / std::numbers::pi;
}

BENCHMARK(BM_IK_WarmStart)->Unit(benchmark::kMicrosecond);

static void BM_IK_Trajectory(benchmark::State& state) {
    const auto& fx = fixture();
    auto solver = fx.makeSolver(true);
    auto fk = std::make_shared<ForwardKinematics>(solver->getRobot(), solver->getEndLink(), solver->getBaseLink());
    Eigen::VectorXd result(static_cast<Eigen::Index>(fx.dof()));

    if (fx.trajectoryTargets().empty()) {
        state.SkipWithError("Trajectory dataset not generated");
        return;
    }

    size_t solves = 0;
    size_t successes = 0;
    size_t total_iterations = 0;
    double total_position_error = 0.0;
    double total_rotation_error = 0.0;

    for (auto _ : state) {
        for (const auto& pose : fx.trajectoryTargets()) {
            const auto status = solver->solve(pose, Eigen::VectorXd(), result);
            total_iterations += status.iterations;
            successes += status.converged ? 1 : 0;
            
            if (status.converged) {
                Transform achieved = fk->compute(result);
                Eigen::Vector3d pos_error = achieved.translation() - pose.translation();
                total_position_error += pos_error.norm();
                
                Eigen::Quaterniond q_target(pose.rotation());
                Eigen::Quaterniond q_achieved(achieved.rotation());
                double rotation_error = q_target.angularDistance(q_achieved);
                total_rotation_error += rotation_error;
            }
            
            ++solves;
            benchmark::DoNotOptimize(result.data());
        }
    }

    state.counters["iterations_per_solve"] = static_cast<double>(total_iterations) / static_cast<double>(solves);
    state.counters["success_rate"] = (static_cast<double>(successes) / static_cast<double>(solves)) * 100.0;
    state.counters["avg_position_error_mm"] = (total_position_error / static_cast<double>(successes)) * 1000.0;
    state.counters["avg_rotation_error_deg"] = (total_rotation_error / static_cast<double>(successes)) * 180.0 / std::numbers::pi;
}

BENCHMARK(BM_IK_Trajectory)->Unit(benchmark::kMicrosecond);
