#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/unordered_map.h>

#include "kinex/robot_model.h"
#include "kinex/robot.h"
#include "kinex/kinematics.h"
#include "kinex/urdf_parser.h"
#include "kinex/inverse_kinematics.h"

namespace nb = nanobind;
using namespace kinex;

struct IKResult {
    SolverStatus status;
    Eigen::VectorXd solution;
};

#ifndef KINEX_VERSION
#define KINEX_VERSION "0.0.0-dev"
#endif

NB_MODULE(_kinex, m) {
    m.doc() = "Python bindings for kinex robotics library";
    m.attr("__version__") = KINEX_VERSION;

    // 2.4 Enums
    nb::enum_<JointType>(m, "JointType")
        .value("Fixed", JointType::Fixed)
        .value("Revolute", JointType::Revolute)
        .value("Continuous", JointType::Continuous)
        .value("Prismatic", JointType::Prismatic)
        .value("Floating", JointType::Floating)
        .value("Planar", JointType::Planar)
        .export_values();

    nb::enum_<GeometryType>(m, "GeometryType")
        .value("Box", GeometryType::Box)
        .value("Cylinder", GeometryType::Cylinder)
        .value("Sphere", GeometryType::Sphere)
        .value("Mesh", GeometryType::Mesh)
        .export_values();

    nb::enum_<JacobianType>(m, "JacobianType")
        .value("Analytic", JacobianType::Analytic)
        .value("Geometric", JacobianType::Geometric)
        .export_values();

    // 2.3 Transform
    nb::class_<Transform>(m, "Transform")
        .def(nb::init<>())
        .def(nb::init<const Eigen::Isometry3d&>())
        .def_static("from_position_quaternion", 
            [](const Eigen::Vector3d& position, const Eigen::Vector4d& quaternion_xyzw) {
                // Convert [x, y, z, w] numpy array to Eigen::Quaterniond
                Eigen::Quaterniond quat(quaternion_xyzw[3], quaternion_xyzw[0], 
                                       quaternion_xyzw[1], quaternion_xyzw[2]);
                return Transform::fromPositionQuaternion(position, quat);
            },
            nb::arg("position"), nb::arg("quaternion"),
            "Create transform from position and quaternion (x,y,z,w)")
        .def_static("from_position_rpy", &Transform::fromPositionRPY,
            nb::arg("position"), nb::arg("rpy"),
            "Create transform from position and RPY angles")
        .def("inverse", &Transform::inverse, "Get inverse transform")
        .def("translation", &Transform::translation, "Get translation vector")
        .def("rotation", &Transform::rotation, "Get rotation matrix")
        .def("as_matrix", &Transform::asMatrix, "Get 4x4 transformation matrix")
        .def("as_position_quaternion", [](const Transform& t) {
            auto [pos, quat] = t.asPositionQuaternion();
            Eigen::Vector4d quat_xyzw;
            quat_xyzw << quat.x(), quat.y(), quat.z(), quat.w();
            return std::make_pair(pos, quat_xyzw);
        }, "Get position and orientation as quaternion (x,y,z,w)")
        .def("as_position_rpy", &Transform::asPositionRPY,
             "Get position and orientation as RPY angles")
        .def("__mul__", [](const Transform& a, const Transform& b) { return a * b; })
        .def("__repr__", [](const Transform& t) {
            auto [pos, rpy] = t.asPositionRPY();
            return "Transform(pos=[" + std::to_string(pos.x()) + ", " + 
                   std::to_string(pos.y()) + ", " + std::to_string(pos.z()) + 
                   "], rpy=[" + std::to_string(rpy.x()) + ", " + 
                   std::to_string(rpy.y()) + ", " + std::to_string(rpy.z()) + "])";
        });

    // 2.5 Data Structures
    nb::class_<JointLimits>(m, "JointLimits")
        .def(nb::init<>())
        .def_rw("lower", &JointLimits::lower)
        .def_rw("upper", &JointLimits::upper)
        .def_rw("effort", &JointLimits::effort)
        .def_rw("velocity", &JointLimits::velocity);

    nb::class_<JointDynamics>(m, "JointDynamics")
        .def(nb::init<>())
        .def_rw("damping", &JointDynamics::damping)
        .def_rw("friction", &JointDynamics::friction);

    nb::class_<Geometry>(m, "Geometry")
        .def(nb::init<>())
        .def_rw("type", &Geometry::type)
        .def_rw("box_size", &Geometry::box_size)
        .def_rw("cylinder_radius", &Geometry::cylinder_radius)
        .def_rw("cylinder_length", &Geometry::cylinder_length)
        .def_rw("sphere_radius", &Geometry::sphere_radius)
        .def_rw("mesh_filename", &Geometry::mesh_filename)
        .def_rw("mesh_scale", &Geometry::mesh_scale);

    nb::class_<Visual>(m, "Visual")
        .def(nb::init<>())
        .def_rw("name", &Visual::name)
        .def_rw("origin", &Visual::origin)
        .def_rw("geometry", &Visual::geometry)
        .def_rw("color", &Visual::color)
        .def_rw("material_name", &Visual::material_name);

    nb::class_<Collision>(m, "Collision")
        .def(nb::init<>())
        .def_rw("name", &Collision::name)
        .def_rw("origin", &Collision::origin)
        .def_rw("geometry", &Collision::geometry);

    nb::class_<Inertial>(m, "Inertial")
        .def(nb::init<>())
        .def_rw("origin", &Inertial::origin)
        .def_rw("mass", &Inertial::mass)
        .def_rw("inertia", &Inertial::inertia);

    // 3.1 Link
    nb::class_<Link>(m, "Link")
        .def(nb::init<const std::string&>())
        .def("get_name", &Link::getName)
        .def("get_inertial", &Link::getInertial)
        .def("set_inertial", &Link::setInertial)
        .def("get_visuals", &Link::getVisuals)
        .def("add_visual", &Link::addVisual)
        .def("get_collisions", &Link::getCollisions)
        .def("add_collision", &Link::addCollision);

    // 3.2 Joint
    nb::class_<Joint>(m, "Joint")
        .def(nb::init<const std::string&, JointType>())
        .def("get_name", &Joint::getName)
        .def("get_type", &Joint::getType)
        .def("get_parent_link", &Joint::getParentLink)
        .def("set_parent_link", &Joint::setParentLink)
        .def("get_child_link", &Joint::getChildLink)
        .def("set_child_link", &Joint::setChildLink)
        .def("get_origin", &Joint::getOrigin)
        .def("set_origin", &Joint::setOrigin)
        .def("get_axis", &Joint::getAxis)
        .def("set_axis", &Joint::setAxis)
        .def("get_limits", &Joint::getLimits)
        .def("set_limits", &Joint::setLimits)
        .def("get_dynamics", &Joint::getDynamics)
        .def("set_dynamics", &Joint::setDynamics)
        .def("is_actuated", &Joint::isActuated)
        .def("get_transform", &Joint::getTransform, "Get transformation for given joint value");

    // 3.3 RobotModel (formerly Robot)
    nb::class_<RobotModel>(m, "RobotModel")
        .def(nb::init<const std::string&>())
        .def("get_name", &RobotModel::getName)
        .def("get_root_link", &RobotModel::getRootLink)
        .def("set_root_link", &RobotModel::setRootLink)
        .def("add_link", &RobotModel::addLink)
        .def("get_link", &RobotModel::getLink)
        .def("get_links", &RobotModel::getLinks)
        .def("add_joint", &RobotModel::addJoint)
        .def("get_joint", &RobotModel::getJoint)
        .def("get_joints", &RobotModel::getJoints)
        .def("get_actuated_joints", &RobotModel::getActuatedJoints)
        .def("get_child_joints", &RobotModel::getChildJoints)
        .def("get_parent_joint", &RobotModel::getParentJoint)
        .def("validate", &RobotModel::validate)
        .def_prop_ro("dof", [](const RobotModel& r) { return r.getActuatedJoints().size(); })
        .def("get_joint_names", [](const RobotModel& r) {
            std::vector<std::string> names;
            for (const auto& j : r.getActuatedJoints()) {
                names.push_back(j->getName());
            }
            return names;
        })
        .def("get_joint_limits", [](const RobotModel& r) {
            auto joints = r.getActuatedJoints();
            Eigen::MatrixXd limits(joints.size(), 2);
            for (size_t i = 0; i < joints.size(); ++i) {
                if (auto l = joints[i]->getLimits()) {
                    limits(i, 0) = l->lower;
                    limits(i, 1) = l->upper;
                } else {
                    limits(i, 0) = -std::numeric_limits<double>::infinity();
                    limits(i, 1) = std::numeric_limits<double>::infinity();
                }
            }
            return limits;
        })
        .def_static("from_urdf_file", [](const std::string& path) {
             URDFParser parser;
             return parser.parseFile(path);
        }, "Parse robot model from URDF file")
        .def_static("from_urdf_string", [](const std::string& xml) {
             URDFParser parser;
             return parser.parseString(xml);
        }, "Parse robot model from URDF string");

    // 3.4 URDFParser
    nb::class_<URDFParser>(m, "URDFParser")
        .def(nb::init<>())
        .def("parse_file", &URDFParser::parseFile)
        .def("parse_string", &URDFParser::parseString)
        .def_prop_rw("base_directory", &URDFParser::getBaseDirectory, &URDFParser::setBaseDirectory);

    // 4. Kinematics
    nb::class_<KinematicChain>(m, "KinematicChain")
        .def(nb::init<std::shared_ptr<const RobotModel>, const std::string&, const std::string&>(),
             nb::arg("robot"), nb::arg("end_link"), nb::arg("base_link") = "")
        .def("get_num_joints", &KinematicChain::getNumJoints)
        .def("get_joints", &KinematicChain::getJoints)
        .def("get_link_names", &KinematicChain::getLinkNames)
        .def("get_end_link", &KinematicChain::getEndLink)
        .def("get_base_link", &KinematicChain::getBaseLink)
        .def("get_static_transforms", &KinematicChain::getStaticTransforms)
        .def("get_all_joints", &KinematicChain::getAllJoints);

    nb::class_<ForwardKinematics>(m, "ForwardKinematics")
        .def(nb::init<std::shared_ptr<const RobotModel>, const std::string&, const std::string&>(),
             nb::arg("robot"), nb::arg("end_link"), nb::arg("base_link") = "")
        .def("compute", &ForwardKinematics::compute,
             nb::arg("joint_angles"), nb::arg("check_bounds") = false)
        .def("compute_to_link", &ForwardKinematics::computeToLink,
             nb::arg("joint_angles"), nb::arg("target_link"), nb::arg("check_bounds") = false)
        .def("compute_all_link_transforms", &ForwardKinematics::computeAllLinkTransforms,
             nb::arg("joint_angles"), nb::arg("check_bounds") = false)
        .def("get_chain", &ForwardKinematics::getChain)
        .def("get_num_joints", &ForwardKinematics::getNumJoints)
        .def_prop_ro("num_joints", &ForwardKinematics::getNumJoints);

    nb::class_<JacobianCalculator>(m, "JacobianCalculator")
        .def(nb::init<std::shared_ptr<const RobotModel>, const std::string&, const std::string&>(),
             nb::arg("robot"), nb::arg("end_link"), nb::arg("base_link") = "")
        .def("compute", &JacobianCalculator::compute,
             nb::arg("joint_angles"), nb::arg("type") = JacobianType::Analytic, 
             nb::arg("target_link") = "")
        .def("compute_jacobian_derivative", &JacobianCalculator::computeJacobianDerivative,
             nb::arg("joint_angles"), nb::arg("joint_velocities"),
             nb::arg("type") = JacobianType::Analytic, 
             nb::arg("target_link") = "")
        .def("is_singular", &JacobianCalculator::isSingular,
             nb::arg("joint_angles"), nb::arg("threshold") = 1e-6,
             nb::arg("type") = JacobianType::Analytic, 
             nb::arg("target_link") = "")
        .def("get_manipulability", &JacobianCalculator::getManipulability,
             nb::arg("joint_angles"), nb::arg("type") = JacobianType::Analytic, 
             nb::arg("target_link") = "")
        .def("get_condition_number", &JacobianCalculator::getConditionNumber,
             nb::arg("joint_angles"), nb::arg("type") = JacobianType::Analytic, 
             nb::arg("target_link") = "")
        .def_static("convert_jacobian", &JacobianCalculator::convertJacobian,
             nb::arg("jacobian"), nb::arg("pose"), nb::arg("from_type"), nb::arg("to_type"));

    // 5. Inverse Kinematics
    nb::class_<SolverConfig>(m, "SolverConfig")
        .def(nb::init<>())
        .def_rw("max_iterations", &SolverConfig::max_iterations)
        .def_rw("tolerance", &SolverConfig::tolerance)
        .def_rw("regularization", &SolverConfig::regularization)
        .def_rw("max_step_size", &SolverConfig::max_step_size)
        .def_rw("max_line_search_steps", &SolverConfig::max_line_search_steps)
        .def_rw("line_search_shrink", &SolverConfig::line_search_shrink)
        .def_rw("line_search_min_alpha", &SolverConfig::line_search_min_alpha)
        .def_rw("line_search_improvement", &SolverConfig::line_search_improvement)
        .def_rw("position_weight", &SolverConfig::position_weight)
        .def_rw("orientation_weight", &SolverConfig::orientation_weight)
        .def_rw("position_anchor_weight", &SolverConfig::position_anchor_weight)
        .def_rw("orientation_anchor_weight", &SolverConfig::orientation_anchor_weight)
        .def_rw("joint_limit_margin", &SolverConfig::joint_limit_margin)
        .def_rw("unbounded_joint_limit", &SolverConfig::unbounded_joint_limit)
        .def_rw("enable_warm_start", &SolverConfig::enable_warm_start);

    nb::class_<SolverStatus>(m, "SolverStatus")
        .def(nb::init<>())
        .def_rw("converged", &SolverStatus::converged)
        .def_rw("iterations", &SolverStatus::iterations)
        .def_rw("final_error_norm", &SolverStatus::final_error_norm)
        .def_rw("final_step_norm", &SolverStatus::final_step_norm)
        .def_rw("qp_status", &SolverStatus::qp_status)
        .def_rw("message", &SolverStatus::message)
        .def_rw("error_history", &SolverStatus::error_history)
        .def("__repr__", [](const SolverStatus& s) {
             return "SolverStatus(converged=" + std::string(s.converged ? "True" : "False") + 
                    ", iterations=" + std::to_string(s.iterations) + 
                    ", error=" + std::to_string(s.final_error_norm) + ")";
        });

    nb::class_<IKResult>(m, "IKResult")
        .def_rw("status", &IKResult::status)
        .def_rw("solution", &IKResult::solution)
        .def("__repr__", [](const IKResult& r) {
            return "IKResult(status=" + std::string(r.status.converged ? "Converged" : "Failed") + ")";
        });

    nb::class_<IKSolver>(m, "IKSolver")
        .def("set_solver_config", &IKSolver::setSolverConfig)
        .def("get_solver_config", &IKSolver::getSolverConfig)
        .def("set_position_only", &IKSolver::setPositionOnly)
        .def("set_orientation_only", &IKSolver::setOrientationOnly)
        .def("set_warm_start", &IKSolver::setWarmStart)
        .def_prop_rw("tolerance", 
            [](const IKSolver& s) { return s.getSolverConfig().tolerance; },
            [](IKSolver& s, double t) { 
                auto c = s.getSolverConfig(); 
                c.tolerance = t; 
                s.setSolverConfig(c); 
            })
        .def_prop_rw("max_iterations", 
            [](const IKSolver& s) { return s.getSolverConfig().max_iterations; },
            [](IKSolver& s, size_t i) { 
                auto c = s.getSolverConfig(); 
                c.max_iterations = i; 
                s.setSolverConfig(c); 
            });

    nb::class_<SQPIKSolver, IKSolver>(m, "SQPIKSolver")
        .def(nb::init<std::shared_ptr<const RobotModel>, const std::string&, const std::string&>(),
             nb::arg("robot"), nb::arg("end_link"), nb::arg("base_link") = "")
        .def("solve", [](SQPIKSolver& self, const Transform& target_pose, const Eigen::VectorXd& initial_guess) {
             Eigen::VectorXd solution;
             SolverStatus status = self.solve(target_pose, initial_guess, solution);
             return IKResult{status, solution};
        }, nb::arg("target_pose"), nb::arg("initial_guess"));

    // 6. New Unified Robot API
    nb::class_<Robot>(m, "Robot")
        .def_static("from_urdf", &Robot::fromURDF, 
            nb::arg("filepath"), nb::arg("end_link"), nb::arg("base_link") = "",
            nb::call_guard<nb::gil_scoped_release>())
        .def_static("from_urdf_string", &Robot::fromURDFString,
            nb::arg("urdf_string"), nb::arg("end_link"), nb::arg("base_link") = "",
            nb::call_guard<nb::gil_scoped_release>())
        .def("clone", &Robot::clone)
        .def("forward_kinematics", &Robot::forwardKinematics,
            nb::arg("q"), nb::arg("link") = "")
        .def("compute_pose", &Robot::computePose,
            nb::arg("q"), nb::arg("link") = "")
        .def("inverse_kinematics", [](Robot& self, const Transform& target, const Eigen::VectorXd& q_init, const std::string& link) {
             auto [solution, status] = self.inverseKinematics(target, q_init, link);
             return std::make_pair(solution, status);
        }, nb::arg("target"), nb::arg("q_init"), nb::arg("link") = "",
           nb::call_guard<nb::gil_scoped_release>())
        .def("solve_ik", [](Robot& self, const Transform& target, const Eigen::VectorXd& q_init, const std::string& link) {
             auto [solution, status] = self.solveIK(target, q_init, link);
             return std::make_pair(solution, status);
        }, nb::arg("target"), nb::arg("q_init"), nb::arg("link") = "",
           nb::call_guard<nb::gil_scoped_release>())
        .def("compute_jacobian", &Robot::computeJacobian,
            nb::arg("q"), nb::arg("link") = "", nb::arg("type") = JacobianType::Analytic)
        .def("get_manipulability", &Robot::getManipulability,
            nb::arg("q"), nb::arg("link") = "")
        .def("is_singular", &Robot::isSingular,
            nb::arg("q"), nb::arg("threshold") = 1e-6, nb::arg("link") = "")
        .def("get_condition_number", &Robot::getConditionNumber,
            nb::arg("q"), nb::arg("link") = "")
        .def("set_ik_tolerance", &Robot::setIKTolerance, nb::arg("tol"))
        .def("set_position_only_ik", &Robot::setPositionOnlyIK, nb::arg("enable"))
        .def("set_orientation_only_ik", &Robot::setOrientationOnlyIK, nb::arg("enable"))
        .def("set_solver_config", &Robot::setSolverConfig, nb::arg("config"))
        .def("get_solver_config", &Robot::getSolverConfig)
        .def_prop_ro("name", &Robot::getName)
        .def_prop_ro("end_link", &Robot::getEndLink)
        .def_prop_ro("base_link", &Robot::getBaseLink)
        .def_prop_ro("dof", &Robot::getDOF)
        .def_prop_ro("model", &Robot::getRobotModel);
}