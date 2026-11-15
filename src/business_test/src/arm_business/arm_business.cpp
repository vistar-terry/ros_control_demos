/**
 * @file arm_business.cpp
 * @author 万俟淋曦 
 * @brief arm_business.h 中业务主类的实现：参数、ROS 通信、随机点生成与轨迹下发全流程。
 * @date 2026-11-15 11:08:25
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#include "business_test/arm_business/arm_business.h"

#include <algorithm>

namespace arm_business
{
namespace
{

/** 下发一条单点轨迹并等待控制器执行完成 */
bool executeGoal(TrajectoryClient &client,
                 const control_msgs::FollowJointTrajectoryGoal &goal,
                 double timeout)
{
    client.sendGoal(goal);

    const ros::WallTime start = ros::WallTime::now();
    ros::WallRate poll(50.0);
    while (ros::ok() && !client.getState().isDone())
    {
        if ((ros::WallTime::now() - start).toSec() > timeout)
        {
            ROS_ERROR("Trajectory execution timed out after %.1f s; goal cancelled", timeout);
            client.cancelGoal();
            return false;
        }
        poll.sleep();
    }
    if (!ros::ok())
        return false;

    const actionlib::SimpleClientGoalState state = client.getState();
    control_msgs::FollowJointTrajectoryResultConstPtr result = client.getResult();
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        // ABORT 时把控制器给出的 error_code / error_string 一并打出：
        // 可区分是关节超限位 (PATH_TOLERANCE_VIOLATED)、跟踪超差还是目标容差失败。
        if (result)
            ROS_ERROR("Trajectory goal did not succeed, state: %s, error_code: %d, error_string: %s",
                      state.toString().c_str(), result->error_code, result->error_string.c_str());
        else
            ROS_ERROR("Trajectory goal did not succeed, state: %s (no result)", state.toString().c_str());
        return false;
    }

    if (result && result->error_code != control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
    {
        ROS_ERROR("Controller returned error code %d: %s", result->error_code, result->error_string.c_str());
        return false;
    }
    return true;
}

} // namespace

// ===========================================================================
// ArmBusiness
// ===========================================================================

ArmBusiness::ArmBusiness(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), dof_(0)
{
    action_name_ = "/arm_trajectory_controller/follow_joint_trajectory";
    joint_states_topic_ = "/joint_states";
    description_param_ = "/robot_description";
    marker_topic_ = "/arm_business/workspace_markers";
    tip_link_ = "tool0";
    cube_side_ = 0.0;            // 0 = 由可达活动范围包围盒自动推导；>0 强制成立方体
    bbox_margin_ = 0.0;          // 包围盒外扩余量 (比例)
    max_reach_ = 1.75;           // 肩关节起算的名义可达半径 (m)
    reach_ratio_ = 1.0;          // 随机点半径上限 (× max_reach)，会自动下调
    min_z_ = 0.20;               // 随机点最低高度 (m)
    link_radius_ = 0.075;        // 连杆近似半径 (m)，用于把可达球外扩
    ground_clearance_ = 0.10;    // 整条臂(连杆轴线)离地最小高度 (m)，避免触碰地面
    traverse_time_ = 3.0;        // 相邻目标点运动时间 (s)，越大越慢越平滑
    hold_time_ = 0.3;            // 到达目标点后的停留时间 (s)
    ik_epsilon_ = 1e-4;
    ik_damping_ = 0.03;
    loops_ = 0;                  // <= 0 表示无限循环
    ik_max_iterations_ = 200;
    orientation_mode_ = 1;       // 0 = 保持启动姿态不变; 1 = 末端 Z 轴与可达球半径平行（正向指向球外）
    tool_roll_ = 0.0;            // 径向模式下绕末端 Z 轴（径向轴）的滚转角 (rad)
    random_order_ = true;        // 是否每轮随机打乱 8 个目标点的遍历顺序

    pnh_.param("action_name", action_name_, action_name_);
    pnh_.param("joint_states_topic", joint_states_topic_, joint_states_topic_);
    pnh_.param("robot_description_param", description_param_, description_param_);
    pnh_.param("marker_topic", marker_topic_, marker_topic_);
    pnh_.param("tip_link", tip_link_, tip_link_);
    pnh_.param("cube_side", cube_side_, cube_side_);
    pnh_.param("bbox_margin", bbox_margin_, bbox_margin_);
    pnh_.param("max_reach", max_reach_, max_reach_);
    pnh_.param("reach_ratio", reach_ratio_, reach_ratio_);
    pnh_.param("min_z", min_z_, min_z_);
    pnh_.param("link_radius", link_radius_, link_radius_);
    pnh_.param("ground_clearance", ground_clearance_, ground_clearance_);
    pnh_.param("traverse_time", traverse_time_, traverse_time_);
    pnh_.param("hold_time", hold_time_, hold_time_);
    pnh_.param("ik_epsilon", ik_epsilon_, ik_epsilon_);
    pnh_.param("ik_damping", ik_damping_, ik_damping_);
    pnh_.param("loops", loops_, loops_);
    pnh_.param("ik_max_iterations", ik_max_iterations_, ik_max_iterations_);
    pnh_.param("orientation_mode", orientation_mode_, orientation_mode_);
    pnh_.param("tool_roll", tool_roll_, tool_roll_);
    pnh_.param("random_order", random_order_, random_order_);
}

BoundingBox ArmBusiness::computeWorkspaceBox(const double shoulder[3], double reach,
                                             double margin, double floor_z) const
{
    BoundingBox b;
    const double half_xy = reach * (1.0 + margin);
    b.lo[0] = shoulder[0] - half_xy;
    b.hi[0] = shoulder[0] + half_xy;
    b.lo[1] = shoulder[1] - half_xy;
    b.hi[1] = shoulder[1] + half_xy;
    b.lo[2] = std::max(floor_z, shoulder[2] - reach);
    b.hi[2] = shoulder[2] + reach * (1.0 + margin);
    return b;
}

visualization_msgs::MarkerArray ArmBusiness::makeWorkspaceMarkers(
    const std::vector<Transform> &targets, const std::string &frame_id) const
{
    visualization_msgs::MarkerArray array;
    const ros::Time stamp = ros::Time::now();

    visualization_msgs::Marker clear;
    clear.header.frame_id = frame_id;
    clear.header.stamp = stamp;
    clear.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(clear);

    visualization_msgs::Marker dots;
    dots.header = clear.header;
    dots.ns = "traversal_vertices";
    dots.id = 2;
    dots.type = visualization_msgs::Marker::SPHERE_LIST;
    dots.action = visualization_msgs::Marker::ADD;
    dots.pose.orientation.w = 1.0;
    dots.scale.x = 0.06;
    dots.scale.y = 0.06;
    dots.scale.z = 0.06;
    dots.color.r = 1.0f;
    dots.color.g = 0.85f;
    dots.color.b = 0.2f;
    dots.color.a = 0.95f;
    for (size_t i = 0; i < targets.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = targets[i].p[0];
        p.y = targets[i].p[1];
        p.z = targets[i].p[2];
        dots.points.push_back(p);
    }
    array.markers.push_back(dots);

    return array;
}

/** 构造"末端 Z 轴平行于可达球半径方向 (p - shoulder)"的期望旋转矩阵 R */
void ArmBusiness::radialToolRotation(const double p[3], const double shoulder[3],
                                     const double startup_R[3][3], double roll, double R[3][3]) const
{
    double z[3] = {p[0] - shoulder[0], p[1] - shoulder[1], p[2] - shoulder[2]};
    const double n = std::sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]);
    if (n < 1e-9)
    {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                R[i][j] = (i == j) ? 1.0 : 0.0;
        return;
    }
    for (int j = 0; j < 3; ++j)
        z[j] /= n;

    // 参考 X：启动姿态末端 X 轴投影到垂直于 z 的平面
    double x[3] = {startup_R[0][0], startup_R[1][0], startup_R[2][0]};
    double dot = x[0] * z[0] + x[1] * z[1] + x[2] * z[2];
    for (int j = 0; j < 3; ++j)
        x[j] -= dot * z[j];
    double xn = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    if (xn < 1e-6)
    {
        x[0] = 1.0; x[1] = 0.0; x[2] = 0.0;
        dot = x[0] * z[0] + x[1] * z[1] + x[2] * z[2];
        for (int j = 0; j < 3; ++j)
            x[j] -= dot * z[j];
        xn = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    }
    for (int j = 0; j < 3; ++j)
        x[j] /= xn;

    // y = z × x（右手系：x × y = z）
    double y[3];
    y[0] = z[1] * x[2] - z[2] * x[1];
    y[1] = z[2] * x[0] - z[0] * x[2];
    y[2] = z[0] * x[1] - z[1] * x[0];

    // 绕 z 轴（径向轴）滚转 roll
    const double c = std::cos(roll), s = std::sin(roll);
    double xr[3], yr[3];
    for (int j = 0; j < 3; ++j)
    {
        xr[j] = c * x[j] + s * y[j];
        yr[j] = -s * x[j] + c * y[j];
    }

    // 标准旋转矩阵约定：R[i][j] 的第 j 列 = 末端第 j 轴在根连杆系下的方向
    for (int i = 0; i < 3; ++i)
    {
        R[i][0] = xr[i]; // 第 0 列 = 末端 X
        R[i][1] = yr[i]; // 第 1 列 = 末端 Y
        R[i][2] = z[i];  // 第 2 列 = 末端 Z（径向）
    }
}

/** 在可达球内随机生成 count 个点到球心距离互不相等的点 */
void ArmBusiness::randomSpherePoints(const double shoulder[3], double max_radius, double min_z,
                                     int count, std::mt19937 &rng, const Transform &pose,
                                     std::vector<Transform> &out) const
{
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> azimuth(0.0, 2.0 * kPi);
    const double frac_lo = 0.35; // 不贴球心，避开腕部奇异位形
    const double band = (1.0 - frac_lo) / count;

    out.clear();
    out.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i)
    {
        // 分层半径：第 i 个半径落在互不相交的区间，天然两两不同
        const double r = max_radius * (frac_lo + band * (static_cast<double>(i) + unit(rng)));

        // 方向：cz 均匀采样，且保证 shoulder_z + r*cz >= min_z
        double cz_lo = (min_z - shoulder[2]) / r;
        cz_lo = std::max(-1.0, std::min(1.0, cz_lo));
        const double cz = cz_lo + (1.0 - cz_lo) * unit(rng);
        const double rho = std::sqrt(std::max(0.0, 1.0 - cz * cz));
        const double phi = azimuth(rng);

        Transform t = pose;
        t.p[0] = shoulder[0] + r * rho * std::cos(phi);
        t.p[1] = shoulder[1] + r * rho * std::sin(phi);
        t.p[2] = shoulder[2] + r * cz;
        out.push_back(t);
    }
}

/** 生成 8 个可逆解且整条臂不触地的随机点 */
bool ArmBusiness::generateTargets()
{
    const double min_ratio = 0.50;
    const double ratio_step = 0.02;
    const int num_points = 8;
    const int tries_per_ratio = 60; // 每个 ratio 档最多重新随机 60 批（化解偶发碰地/不可达）
    std::mt19937 point_rng(std::random_device{}());

    double used_ratio = 0.0;
    for (double ratio = reach_ratio_; ratio >= min_ratio - 1e-9; ratio -= ratio_step)
    {
        for (int attempt = 0; attempt < tries_per_ratio; ++attempt)
        {
            std::vector<Transform> trial;
            randomSpherePoints(shoulder_, max_reach_ * ratio, min_z_, num_points,
                               point_rng, center_, trial);

            std::vector<double> q_trial = q_;
            std::vector<std::vector<double>> solutions;
            bool all_ok = true;
            for (size_t i = 0; i < trial.size(); ++i)
            {
                // 选定该目标点逆解的参考旋转矩阵 ref（姿态误差以它为 0 基准）
                double cur_ref[3][3];
                if (orientation_mode_ == 1)
                {
                    radialToolRotation(trial[i].p, shoulder_, ref_R_, tool_roll_, cur_ref);
                }
                else
                {
                    for (int r = 0; r < 3; ++r)
                        for (int c = 0; c < 3; ++c)
                            cur_ref[r][c] = ref_R_[r][c];
                }

                int used_iterations = 0;
                double position_error = 0.0;
                if (!kinematics_.solveInverseKinematics(cur_ref, trial[i], q_trial, ik_epsilon_,
                                                        ik_max_iterations_, ik_damping_,
                                                        used_iterations, position_error))
                {
                    all_ok = false;
                    break;
                }
                // 地面碰撞检查：整条臂各连杆端点必须高于 ground_clearance_
                if (kinematics_.armMinZ(q_trial) < ground_clearance_)
                {
                    all_ok = false;
                    break;
                }
                solutions.push_back(q_trial);
            }

            if (all_ok)
            {
                targets_.swap(trial);
                q_targets_.swap(solutions);
                used_ratio = ratio;
                ROS_INFO("Found a collision-free 8-point set at ratio %.2f after %d random tries",
                         ratio, attempt + 1);
                ROS_INFO("Generated 8 random points inside a reach sphere of radius %.3f m "
                         "(%.0f%% of the nominal %.3f m), all with distinct distances to the shoulder",
                         max_reach_ * used_ratio, used_ratio * 100.0, max_reach_);

                // 逐个报告随机点位置、到肩距离与整条臂最低点高度
                double farthest = 0.0;
                for (size_t i = 0; i < targets_.size(); ++i)
                {
                    double d2 = 0.0;
                    for (int j = 0; j < 3; ++j)
                    {
                        const double delta = targets_[i].p[j] - shoulder_[j];
                        d2 += delta * delta;
                    }
                    const double distance = std::sqrt(d2);
                    farthest = std::max(farthest, distance);
                    const double arm_low = kinematics_.armMinZ(q_targets_[i]);
                    ROS_INFO("Point %d/8: target [%.3f, %.3f, %.3f], %.4f m from the shoulder, "
                             "lowest arm point %.4f m (ground clearance %.4f m)",
                             static_cast<int>(i + 1), targets_[i].p[0], targets_[i].p[1],
                             targets_[i].p[2], distance, arm_low, arm_low - link_radius_);
                }
                ROS_INFO("Farthest point is %.3f m from the shoulder (nominal reach %.3f m)",
                         farthest, max_reach_);
                return true;
            }
        }
    }
    return false;
}

int ArmBusiness::run()
{
    // 参数校验
    if (cube_side_ < 0.0 || bbox_margin_ < 0.0 || max_reach_ <= 0.0 || link_radius_ < 0.0 ||
        reach_ratio_ <= 0.0 || reach_ratio_ > 1.0 || min_z_ < 0.0 ||
        traverse_time_ <= 0.0 || hold_time_ < 0.0 ||
        ik_epsilon_ <= 0.0 || ik_damping_ <= 0.0 || ik_max_iterations_ <= 0)
    {
        ROS_FATAL("Invalid parameters: cube_side / bbox_margin / link_radius / min_z must be >= 0; "
                  "reach_ratio must be in (0, 1]; max_reach / traverse_time / ik_epsilon / ik_damping / "
                  "ik_max_iterations must be positive");
        return 1;
    }
    if (orientation_mode_ != 0 && orientation_mode_ != 1)
    {
        ROS_FATAL("Invalid parameter orientation_mode=%d; must be 0 (keep startup pose) or "
                  "1 (tool Z parallel to reach-sphere radius)",
                  orientation_mode_);
        return 1;
    }
    if (orientation_mode_ == 1)
    {
        ROS_INFO("End-effector orientation: tool Z axis parallel to reach-sphere radius "
                 "(radial, pointing outward), with roll angle %.3f rad (controllable via tool_roll)",
                 tool_roll_);
    }
    else
    {
        ROS_INFO("End-effector orientation: kept equal to the startup pose (fixed)");
    }
    if (random_order_)
        ROS_INFO("Traversal order: randomized each round");

    // 1. 等待 URDF 上参数服务器
    std::string xml;
    ros::Rate wait_rate(2.0);
    while (ros::ok() && !nh_.getParam(description_param_, xml))
    {
        ROS_INFO_THROTTLE(5.0, "Waiting for parameter %s ...", description_param_.c_str());
        wait_rate.sleep();
    }
    if (!ros::ok())
        return 0;

    urdf::Model model;
    bool parsed = false;
    try
    {
        parsed = model.initString(xml);
    }
    catch (const std::exception &ex)
    {
        ROS_FATAL("Failed to parse URDF: %s", ex.what());
        return 1;
    }
    if (!parsed)
    {
        ROS_FATAL("Failed to parse URDF, check parameter %s", description_param_.c_str());
        return 1;
    }

    if (!kinematics_.buildChain(model, tip_link_, joint_names_, root_link_))
    {
        ROS_FATAL("Failed to build kinematic chain -> %s", tip_link_.c_str());
        return 1;
    }
    dof_ = joint_names_.size();
    if (dof_ > 6)
    {
        ROS_FATAL("The IK solver supports at most 6 active joints, got %d", static_cast<int>(dof_));
        return 1;
    }
    ROS_INFO("Kinematic chain: %s -> %s, %d active joints", root_link_.c_str(), tip_link_.c_str(),
             static_cast<int>(dof_));
    for (size_t i = 0; i < dof_; ++i)
        ROS_INFO("  joint[%d] = %s", static_cast<int>(i), joint_names_[i].c_str());

    // 2. 读取当前关节角
    sensor_msgs::JointStateConstPtr state;
    while (ros::ok() && !state)
    {
        state = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_states_topic_, nh_, ros::Duration(2.0));
        if (!state)
            ROS_WARN("Waiting for %s ...", joint_states_topic_.c_str());
    }
    if (!ros::ok())
        return 0;

    q_.assign(dof_, 0.0);
    for (size_t i = 0; i < dof_; ++i)
    {
        bool found = false;
        for (size_t k = 0; k < state->name.size(); ++k)
        {
            if (state->name[k] == joint_names_[i] && k < state->position.size())
            {
                q_[i] = state->position[k];
                found = true;
                break;
            }
        }
        if (!found)
        {
            ROS_FATAL("Joint [%s] missing from %s", joint_names_[i].c_str(), joint_states_topic_.c_str());
            return 1;
        }
    }

    center_ = kinematics_.forwardKinematics(q_);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            ref_R_[i][j] = center_.R[i][j];

    // 2b. 工作空间包围盒（仅用于日志可视化）
    shoulder_[0] = shoulder_[1] = shoulder_[2] = 0.0;
    kinematics_.shoulderPosition(shoulder_);
    const double reach = max_reach_ + link_radius_;
    BoundingBox box = computeWorkspaceBox(shoulder_, reach, bbox_margin_, 0.0);
    if (cube_side_ > 0.0)
    {
        const double half_side = cube_side_ * 0.5;
        box.lo[0] = shoulder_[0] - half_side;
        box.hi[0] = shoulder_[0] + half_side;
        box.lo[1] = shoulder_[1] - half_side;
        box.hi[1] = shoulder_[1] + half_side;
        box.lo[2] = 0.0;
        box.hi[2] = cube_side_;
        ROS_INFO("Workspace box forced to a cube of side %.3f m with its bottom on the ground", cube_side_);
    }
    ROS_INFO("Reachable workspace box (relative to %s): x[%.3f, %.3f] y[%.3f, %.3f] z[%.3f, %.3f]",
             root_link_.c_str(), box.lo[0], box.hi[0], box.lo[1], box.hi[1], box.lo[2], box.hi[2]);
    ROS_INFO("The box is the AABB of the reach sphere: radius %.3f m about the shoulder [%.3f, %.3f, %.3f]",
             reach, shoulder_[0], shoulder_[1], shoulder_[2]);

    // 3. 随机点生成 + 逆解 + 碰地检查
    if (!generateTargets())
    {
        ROS_FATAL("No collision-free IK solution set found for the 8 random sphere points; "
                  "try lowering ground_clearance / raising min_z / increasing reach_ratio");
        return 1;
    }

    // 3b. 发布随机点圆点标记（latched）
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1, true);
    marker_pub_.publish(makeWorkspaceMarkers(targets_, root_link_));
    ROS_INFO("Published traversal point markers on %s", marker_topic_.c_str());

    // 4. 等待轨迹控制器 action server
    client_.reset(new TrajectoryClient(action_name_, true));
    while (ros::ok() && !client_->waitForServer(ros::Duration(2.0)))
        ROS_WARN_THROTTLE(5.0, "Waiting for action server %s ...", action_name_.c_str());
    if (!ros::ok())
        return 0;
    ROS_INFO("Connected to trajectory controller, starting random point traversal");

    // 5. 循环遍历
    int round = 0;
    std::vector<size_t> order(targets_.size());
    for (size_t i = 0; i < order.size(); ++i)
        order[i] = i;
    std::mt19937 rng(std::random_device{}());
    std::vector<double> q_prev = q_;

    while (ros::ok() && (loops_ <= 0 || round < loops_))
    {
        ++round;
        if (random_order_)
            std::shuffle(order.begin(), order.end(), rng);
        ROS_INFO("========== Traversal round %d (order: %s) ==========", round,
                 random_order_ ? "random" : "fixed");

        for (size_t k = 0; k < order.size() && ros::ok(); ++k)
        {
            const size_t idx = order[k];
            const std::vector<double> &q_goal = q_targets_[idx];

            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory.header.stamp = ros::Time(0);
            goal.trajectory.joint_names = joint_names_;

            // 第 0 个路点：当前实际所在位形，速度 0，保证与上一条轨迹连续
            trajectory_msgs::JointTrajectoryPoint start_point;
            start_point.positions = q_prev;
            start_point.velocities.assign(dof_, 0.0);
            start_point.time_from_start = ros::Duration(0.0);
            goal.trajectory.points.push_back(start_point);

            // 第 1 个路点：目标随机点，到达时静止
            trajectory_msgs::JointTrajectoryPoint end_point;
            end_point.positions = q_goal;
            end_point.velocities.assign(dof_, 0.0);
            end_point.time_from_start = ros::Duration(traverse_time_);
            goal.trajectory.points.push_back(end_point);
            goal.goal_time_tolerance = ros::Duration(1.0);

            // 自检：FK 位置误差 + 末端 Z 轴与径向夹角
            const Transform reached = kinematics_.forwardKinematics(q_goal);
            const double ex = reached.p[0] - targets_[idx].p[0];
            const double ey = reached.p[1] - targets_[idx].p[1];
            const double ez = reached.p[2] - targets_[idx].p[2];
            const double position_error = std::sqrt(ex * ex + ey * ey + ez * ez);

            const double tool_z[3] = {reached.R[0][2], reached.R[1][2], reached.R[2][2]};
            double rad[3] = {targets_[idx].p[0] - shoulder_[0],
                             targets_[idx].p[1] - shoulder_[1],
                             targets_[idx].p[2] - shoulder_[2]};
            double dot_tz = tool_z[0] * rad[0] + tool_z[1] * rad[1] + tool_z[2] * rad[2];
            const double n_tz = std::sqrt(tool_z[0] * tool_z[0] + tool_z[1] * tool_z[1] + tool_z[2] * tool_z[2]);
            const double n_rad = std::sqrt(rad[0] * rad[0] + rad[1] * rad[1] + rad[2] * rad[2]);
            double radial_angle_deg = -1.0;
            if (n_tz > 1e-9 && n_rad > 1e-9)
            {
                double c_ang = dot_tz / (n_tz * n_rad);
                c_ang = std::max(-1.0, std::min(1.0, c_ang));
                radial_angle_deg = std::acos(c_ang) * 180.0 / kPi;
            }

            ROS_INFO("Point %d/8: target [%.3f, %.3f, %.3f], FK position error %.2f mm, "
                     "tool-Z vs radial angle %.3f deg",
                     static_cast<int>(idx + 1), targets_[idx].p[0], targets_[idx].p[1],
                     targets_[idx].p[2], position_error * 1000.0, radial_angle_deg);

            if (!executeGoal(*client_, goal, traverse_time_ + 5.0))
            {
                ROS_ERROR("Point %d/8: execution failed, business node exiting", static_cast<int>(idx + 1));
                return 1;
            }
            q_prev = q_goal;
            ros::Duration(hold_time_).sleep();
        }
    }

    ROS_INFO("Random point traversal finished");
    return 0;
}

} // namespace arm_business
