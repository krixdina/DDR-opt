#include "front_end/jps_planner/jps_planner.h"

using namespace JPS;

JPSPlanner::JPSPlanner(std::shared_ptr<SDFmap> map, const ros::NodeHandle &nh): map_util_(map), nh_(nh) {
    // graph_search_ = std::make_shared<GraphSearch>(map_util_);
    path_pub_ = ros::Publisher(nh_.advertise<nav_msgs::Path>("jps_path", 1));
    init_path_pub_ = ros::Publisher(nh_.advertise<nav_msgs::Path>("init_jps_path", 1));
    normal_vector_pub_ = ros::Publisher(nh_.advertise<visualization_msgs::Marker>("normal_vector", 1));

    nh_.getParam(ros::this_node::getName()+ "/jps_safe_dis", safe_dis_);
    nh_.getParam(ros::this_node::getName()+ "/max_jps_dis", max_jps_dis_);
    nh_.getParam(ros::this_node::getName()+ "/jps_distance_weight", distance_weight_);
    nh_.getParam(ros::this_node::getName()+ "/jps_yaw_weight", yaw_weight_);
    nh_.getParam(ros::this_node::getName()+ "/trajCutLength", trajCutLength_);

    nh_.getParam(ros::this_node::getName()+ "/max_vel",max_vel_);
    nh_.getParam(ros::this_node::getName()+ "/max_acc",max_acc_);
    nh_.getParam(ros::this_node::getName()+ "/max_omega",max_omega_);
    nh_.getParam(ros::this_node::getName()+ "/max_domega",max_domega_);

    nh_.getParam(ros::this_node::getName()+ "/timeResolution",sampletime_);
    nh_.getParam(ros::this_node::getName()+ "/mintrajNum", mintrajNum_);

    nh_.getParam(ros::this_node::getName()+ "/jps_truncation_time", jps_truncation_time_);

    graph_search_ = std::make_shared<GraphSearch>(map_util_, safe_dis_);

}

bool JPSPlanner::plan(const Eigen::Vector3d &start, const Eigen::Vector3d &goal){
    
    start_state_ = start;
    end_state_ = goal;
    
    Eigen::Vector2i start_idx = map_util_->coord2gridIndex(start.head(2));
    Eigen::Vector2i goal_idx = map_util_->coord2gridIndex(goal.head(2));
    
    double start_dis = map_util_->getDistanceReal(map_util_->gridIndex2coordd(start_idx)) * 0.8;
    double goal_dis = map_util_->getDistanceReal(map_util_->gridIndex2coordd(goal_idx)) * 0.8;
    double safe_dis = std::max(std::min(safe_dis_, start_dis), 0.0);
    safe_dis = std::max(std::min(safe_dis, goal_dis), 0.0);

    // 将起点和终点的栅格坐标交给底层 GraphSearch 做前端图搜索。
    // 这里的 true 表示启用 JPS；若为 false，则退化为普通 A* 搜索。
    // 搜索结果不会直接返回世界坐标路径，而是先保存在 GraphSearch 内部，随后通过 getPath() 取出。
    graph_search_->plan(start_idx(0), start_idx(1), goal_idx(0), goal_idx(1), true, 1e10);

    const auto path = graph_search_->getPath();
    if (path.size() < 1) {
        std::cout << "Cannot find a path from " << start.transpose() <<" to " << goal.transpose() << " Abort!" << std::endl;
        status_ = -1;
        return false;
    }

    std::vector<Eigen::Vector2d> ps;
    for (const auto &it : path) {
        ps.push_back(map_util_->gridIndex2coordd(Eigen::Vector2i(it->x, it->y)));
    }
    pubPath(ps, init_path_pub_);

    raw_path_ = ps;
    std::reverse(std::begin(raw_path_), std::end(raw_path_));
    
    raw_path_.front() = start.head(2);
    raw_path_.back() = goal.head(2);


    return true;
}

// 在代码中被注释，未使用
void JPSPlanner::get_small_resolution_path_(){
    small_resolution_path_.clear();

    int path_size = path_.size();
    for(int i = 0; i < path_size - 1; i++){
        Eigen::Vector2i start = map_util_->coord2gridIndex(path_[i]);
        Eigen::Vector2i end = map_util_->coord2gridIndex(path_[i+1]);
        std::vector<Eigen::Vector2i> line = getGridsBetweenPoints2D(start, end);
        small_resolution_path_.insert(small_resolution_path_.end(), line.begin(), line.end());
    }

}

void JPSPlanner::pubPath(const std::vector<Eigen::Vector2d> &path, const ros::Publisher &pub){
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "world";
    for (const auto &it : path) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = it(0);
        pose.pose.position.y = it(1);
        pose.pose.position.z = 0;
        path_msg.poses.push_back(pose);
    }
    pub.publish(path_msg);
}

// 与 ST-opt-tools 中的路径简化逻辑完全一致
// 对 JPS 搜索得到的折线路径做一次简化：
// 若从“上一个保留点”可以无碰撞地直接连到更后面的点，
// 就删除中间拐点，从而去掉锯齿状的小折线段。
std::vector<Eigen::Vector2d> JPSPlanner::removeCornerPts(const std::vector<Eigen::Vector2d> &path) {
    if (path.size() < 2)
        return path;

    // optimized_path: 简化后的路径，只保留真正需要的转折点。
    std::vector<Eigen::Vector2d> optimized_path;
    Eigen::Vector2d pose1 = path[0];
    Eigen::Vector2d pose2 = path[1];
    // prev_pose: 当前已经确认保留在结果路径中的“前一个关键点”。
    Eigen::Vector2d prev_pose = pose1;
    optimized_path.push_back(pose1);
    // cost1: 当前方案中 prev_pose -> pose1 这段的代价
    // cost2: 当前方案中 pose1 -> pose2 这段的代价
    // cost3: 直接从 prev_pose -> pose2 跳过中间点的代价
    // 这里的“代价”就是线段长度；若直连碰撞，则记为无穷大。
    double cost1, cost2, cost3;

    if (!checkLineCollision(pose1, pose2))
        cost1 = (pose1 - pose2).norm();
    else
        cost1 = std::numeric_limits<double>::infinity();

    for (unsigned int i = 1; i < path.size() - 1; i++) {
        // 依次考察三点：prev_pose(上一个保留点)、pose1(候选中间点)、pose2(更后一个点)
        pose1 = path[i];
        pose2 = path[i + 1];
        if (!checkLineCollision(pose1, pose2))
            cost2 = (pose1 - pose2).norm();
        else
            cost2 = std::numeric_limits<double>::infinity();

        if (!checkLineCollision(prev_pose, pose2))
            cost3 = (prev_pose - pose2).norm();
        else
            cost3 = std::numeric_limits<double>::infinity();

        // 若 prev_pose 可以直接无碰撞连到 pose2，且更短，
        // 说明 pose1 只是多余拐点，可以跳过不保留。
        if (cost3 < cost1 + cost2)
            cost1 = cost3;
        else {
            // 否则说明 pose1 不能被安全且更优地跳过，需要保留为关键点。
            optimized_path.push_back(path[i]);
            cost1 = (pose1 - pose2).norm();
            prev_pose = pose1;
        }
    }

    // 终点始终保留。
    optimized_path.push_back(path.back());
    return optimized_path;
}

bool JPSPlanner::checkLineCollision(const Eigen::Vector2d &start, const Eigen::Vector2d &end){
    std::vector<Eigen::Vector2i> line = getGridsBetweenPoints2D(map_util_->coord2gridIndex(start), map_util_->coord2gridIndex(end));
    for(auto line_pt:line){
        if(map_util_->isOccWithSafeDis(line_pt, graph_search_->GetSafeDis())){
            return true;
        }
    }
    return false;
}

// 注意：这个函数返回的不是“连续几何直线”，而是“直线段在栅格地图中的离散表示”。
// 几何上，start 和 end 确定了一条直线段。但在栅格地图里，程序没法直接操作“连续直线”。
// 所以它要把这条连续直线，转换成一串离散格子点。这串格子点连起来，看起来就是一条“栅格化的直线”。
// 故给定 start 和 end 两个栅格点后，它会输出一串连续栅格，
// 表示这条线段从起点到终点时，依次经过了哪些栅格。该函数返回的是这条直线经过的离散栅格序列。
// 这里使用的是 Bresenham 风格的栅格直线遍历，用离散格子去逼近连续线段。
std::vector<Eigen::Vector2i> JPSPlanner::getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end){
    std::vector<Eigen::Vector2i> line;
    
    int dx = abs(end.x() - start.x());
    int dy = abs(end.y() - start.y());
    int sx = (start.x() < end.x()) ? 1 : -1;
    int sy = (start.y() < end.y()) ? 1 : -1;
    int err = dx - dy;

    double x0 = start.x();
    double y0 = start.y();

    while (true) {
        line.emplace_back(x0, y0);
        if (x0 == end.x() && y0 == end.y()) break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }

    return line;
}

// void JPSPlanner::getKinoNode(const Eigen::Vector3d &current_state_VAJ, const Eigen::Vector3d &current_state_OAJ){
//     current_state_VAJ_ = current_state_VAJ;
//     current_state_OAJ_ = current_state_OAJ;

//     get_small_resolution_path_();
//     pubPath(path_, path_pub_);
//     getSampleTraj();
//     getTrajsWithTime();
// }

void JPSPlanner::getKinoNodeWithStartPath(const std::vector<Eigen::Vector3d> &start_path, const bool if_forward, const Eigen::Vector3d &current_state_VAJ, const Eigen::Vector3d &current_state_OAJ){
    current_state_VAJ_ = current_state_VAJ;
    current_state_OAJ_ = current_state_OAJ;

    if(start_path.size() > 0){
      std::vector<Eigen::Vector2d> start_path_2d;
      for(auto pt:start_path){
        start_path_2d.push_back(pt.head(2));
        ROS_INFO_STREAM("start_path_3d: " << pt.transpose());
        start_path_2d.pop_back();
      }
      raw_path_.insert(raw_path_.begin(), start_path_2d.begin(), start_path_2d.end());
      start_state_ = start_path.front();
    }
    
    // get_small_resolution_path_();
    path_ = removeCornerPts(raw_path_);
    Unoccupied_path_ = path_;

    // for(auto pt:Unoccupied_path_){
    //   std::cout<<"Unoccupied_path_: "<<pt.transpose()<<std::endl;
    // }

    pubPath(path_, path_pub_);
    getSampleTraj();
    getTrajsWithTime();
}

// 将简化后的几何路径 Unoccupied_path_ 转换为一组离散“平坦状态”采样点。
// 每个采样点用 state5d = [x, y, theta, dtheta, ds] 表示：
// 1. x, y: 当前采样点位置
// 2. theta: 当前段采用的航向角
// 3. dtheta: 相对上一个采样状态的航向增量
// 4. ds: 相对上一个采样状态的弧长增量
// 这些离散状态随后会被 getTrajsWithTime() 进一步做时间分配，整理成 flat_traj_。
void JPSPlanner::getSampleTraj(){
    Unoccupied_sample_trajs_.clear();
    double cur_theta;

    // state5d: 当前构造出的 5 维采样状态 [x, y, theta, dtheta, ds]
    Eigen::VectorXd state5d;// x y theta dtheta ds
    state5d.resize(5);
    // 先压入起点本身，初始时不发生转角和位移增量。
    state5d << start_state_.x(), start_state_.y(), start_state_.z(), 0, 0;
    Unoccupied_sample_trajs_.push_back(state5d); 
    
    // 计算起点到第一段路径的朝向，并把角度调整到接近起始朝向的等价角。
    cur_theta = atan2(Unoccupied_path_[1].y() - Unoccupied_path_[0].y(), Unoccupied_path_[1].x() - Unoccupied_path_[0].x());
    normalizeAngle(start_state_.z(), cur_theta);
    // 在起点位置插入一个“转向后”的状态：位置不变，只记录朝向变化。
    state5d << start_state_.x(), start_state_.y(), cur_theta , cur_theta - start_state_.z(), 0;
    Unoccupied_sample_trajs_.push_back(state5d); 

    // 再插入一个等价朝向状态。这里通过反向向量 + pi 的写法重新得到同一几何方向，
    // 其目的通常是为后续角度连续化和状态拼接提供更稳定的候选表示。
    cur_theta = atan2(Unoccupied_path_[0].y() - Unoccupied_path_[1].y(), Unoccupied_path_[0].x() - Unoccupied_path_[1].x()) + M_PI;
    normalizeAngle(start_state_.z(), cur_theta);
    state5d << start_state_.x(), start_state_.y(), cur_theta , cur_theta - start_state_.z(), 0;
    Unoccupied_sample_trajs_.push_back(state5d); // 2
    
    int path_size = Unoccupied_path_.size();
    // pt: 当前处理的路径点（通常是中间拐点或终点）
    Eigen::VectorXd pt;
    for(int i = 1; i<path_size-1; i++){ 
        pt = Unoccupied_path_[i];

        // 先加入“走到该点但保持上一段航向”的状态。
        // Unoccupied_sample_trajs_.back()[2] 是上一采样状态的 theta。
        // ds 是从上一采样状态位置走到当前点的直线距离。
        state5d << pt.x(), pt.y(), Unoccupied_sample_trajs_.back()[2], 0, sqrt(pow(pt.x() - Unoccupied_sample_trajs_.back()[0], 2) + pow(pt.y() - Unoccupied_sample_trajs_.back()[1], 2));
        Unoccupied_sample_trajs_.push_back(state5d);

        // 再计算“从当前点指向下一点”的新航向，并插入一个原地转向状态。
        cur_theta = atan2(Unoccupied_path_[i+1].y() - Unoccupied_path_[i].y(), Unoccupied_path_[i+1].x() - Unoccupied_path_[i].x());
        normalizeAngle(Unoccupied_sample_trajs_.back()[2], cur_theta);
        state5d << pt.x(), pt.y(), cur_theta, cur_theta - Unoccupied_sample_trajs_.back()[2], 0;
        Unoccupied_sample_trajs_.push_back(state5d);

    }

    // 处理终点位置：先走到终点，但先保持上一段航向。
    pt = Unoccupied_path_.back();
    state5d << pt.x(), pt.y(), Unoccupied_sample_trajs_.back()[2], 0, sqrt(pow(pt.x() - Unoccupied_sample_trajs_.back()[0], 2) + pow(pt.y() - Unoccupied_sample_trajs_.back()[1], 2));
    Unoccupied_sample_trajs_.push_back(state5d);

    // 最后在终点位置补一个“转到目标最终朝向”的状态。
    cur_theta = end_state_.z();
    normalizeAngle(Unoccupied_sample_trajs_.back()[2], cur_theta);
    state5d << pt.x(), pt.y(), cur_theta, cur_theta - Unoccupied_sample_trajs_.back()[2], 0;
    Unoccupied_sample_trajs_.push_back(state5d);
}

// 在 getSampleTraj() 生成的离散状态基础上进一步加入时间信息，
// 最终构造出后端优化器使用的 flat_traj_。
// 主要分两步：
// 1. 按 trajCutLength_ 对前端轨迹做长度截断，得到 cut_Unoccupied_sample_trajs_
// 2. 依据加权路径长度和速度约束分配总时间，再按固定时间间隔进行插值采样
void JPSPlanner::getTrajsWithTime(){
    cut_Unoccupied_sample_trajs_.clear();


    // 下面三个数组是对截断后轨迹的辅助累计量：
    // Unoccupied_thetas: 每个采样状态对应的 theta
    // Unoccupied_pathlengths: 从起点累计的真实路径长度 s
    // Unoccupied_Weightpathlengths: 从起点累计的“加权路径长度”
    // 这里的加权长度 = yaw_weight_ * |dtheta| + distance_weight_ * |ds|
    std::vector<double> Unoccupied_thetas;
    std::vector<double> Unoccupied_pathlengths; 
    std::vector<double> Unoccupied_Weightpathlengths; 

    // 累计的加权路径长度，用于后面时间分配。
    double Unoccupied_AllWeightingPathLength_ = 0; 
    // 累计的真实路径长度，用于截断与平坦变量 s 的记录。
    double Unoccupied_AllPathLength = 0;

    // if_cut: 记录本次前端轨迹是否因为 trajCutLength_ 被截断。
    bool if_cut = false;
    // cut_state: 截断后的末端位姿；若未截断，则默认取原始采样序列的最后一个状态。
    // Unoccupied_sample_trajs_ 是在 getSampleTraj() 中被赋值的
    // cut_state = [最后一个状态的 x, y, theta]
    Eigen::Vector3d cut_state = Unoccupied_sample_trajs_.back().head(3);

    int PathNodeNum = Unoccupied_sample_trajs_.size();
    // 截断轨迹总是从第一个采样状态开始。
    cut_Unoccupied_sample_trajs_.push_back(Unoccupied_sample_trajs_[0]);
    Unoccupied_thetas.push_back(Unoccupied_sample_trajs_[0][2]);
    Unoccupied_pathlengths.push_back(0);
    Unoccupied_Weightpathlengths.push_back(0);
    
    // 逐段扫描 getSampleTraj() 生成的离散状态序列：
    // 1. 若累计真实路径长度尚未超过 trajCutLength_ ，则直接保留当前状态；
    // 2. 若当前这一段会使总长度超过 trajCutLength_，则在该段内部线性插值出截断点；
    // 3. 同时更新截断后轨迹的真实长度、加权长度、theta 等辅助数组，供后续时间分配与插值采样使用。
    int pathnodeindex = 1;
    for(; pathnodeindex<PathNodeNum&&!if_cut; pathnodeindex++){
        // pathnode = [x, y, theta, dtheta, ds]
        Eigen::VectorXd pathnode = Unoccupied_sample_trajs_[pathnodeindex];
        // 一旦下一段位移会让累计真实长度超过 trajCutLength_，就在该段内部插值截断。
        if(Unoccupied_AllPathLength + fabs(pathnode[4]) >= trajCutLength_ && pathnode[4] != 0){
            if_cut = true;
            
            // former_state: 截断段前一个已保留状态，用它和 pathnode 做线性插值求截断点。
            Eigen::Vector3d former_state = Unoccupied_sample_trajs_[pathnodeindex-1].head(3);
            cut_state = former_state + (pathnode.head(3) - former_state) * (trajCutLength_ - Unoccupied_AllPathLength) / fabs(pathnode[4]);
            Eigen::VectorXd state5d; state5d.resize(5);
            // 对截断点同步构造 [x, y, theta, dtheta, ds]。
            // 其中 dtheta、ds 都按当前段所占比例缩放。
            state5d<<cut_state.x(), cut_state.y(), cut_state.z(), (trajCutLength_ - Unoccupied_AllPathLength)/fabs(pathnode[4]) * pathnode[3], trajCutLength_ - Unoccupied_AllPathLength;
            cut_Unoccupied_sample_trajs_.push_back(state5d);
            Unoccupied_thetas.push_back(state5d[2]);
            Unoccupied_AllPathLength += state5d[4];
            Unoccupied_pathlengths.push_back(Unoccupied_AllPathLength); 
            Unoccupied_AllWeightingPathLength_ += yaw_weight_ * abs(state5d[3]) + distance_weight_ * abs(state5d[4]);
            Unoccupied_Weightpathlengths.push_back(Unoccupied_AllWeightingPathLength_);

            PathNodeNum = cut_Unoccupied_sample_trajs_.size();
            break;
        }
        // 若尚未达到截断长度，则直接保留当前采样状态，并继续累计长度。
        cut_Unoccupied_sample_trajs_.push_back(pathnode);
        Unoccupied_thetas.push_back(pathnode[2]);
        Unoccupied_AllPathLength += pathnode[4];
        Unoccupied_pathlengths.push_back(Unoccupied_AllPathLength); 
        Unoccupied_AllWeightingPathLength_ += yaw_weight_ * abs(pathnode[3]) + distance_weight_ * abs(pathnode[4]);
        Unoccupied_Weightpathlengths.push_back(Unoccupied_AllWeightingPathLength_);
    }

    // 基于总加权长度和当前线速度约束，估计整段前端轨迹的执行总时间。
    double totalTrajTime_ = evaluateDuration(Unoccupied_AllWeightingPathLength_, current_state_VAJ_.x(),0.0,max_vel_,max_acc_);
    // Unoccupied_traj_pts: 后端优化使用的平坦输出采样点，格式为 [yaw, s, t]
    std::vector<Eigen::Vector3d> Unoccupied_traj_pts; // Store the sampled coordinates yaw, s, t
    // Unoccupied_positions: 与上述时刻对应的几何位姿采样，格式为 [x, y, yaw]
    std::vector<Eigen::Vector3d> Unoccupied_positions; // Store the sampled coordinates x, y, yaw

    double Unoccupied_totalTrajTime_ = totalTrajTime_;
    double Unoccupied_sampletime;
    // Unoccupied_PathNodeIndex: 插值搜索的起始下标，避免每次都从头扫描。
    int Unoccupied_PathNodeIndex = 1;

    // 采样时间步长尽量接近 sampletime_，同时保证至少采 mintrajNum_ 个点。
    Unoccupied_sampletime = Unoccupied_totalTrajTime_ / std::max(int(Unoccupied_totalTrajTime_ / sampletime_ + 0.5), mintrajNum_);

    PathNodeNum = cut_Unoccupied_sample_trajs_.size();
    // tmparc: 当前轨迹节点对应的累计加权长度。
    double tmparc = 0;

    // 按时间均匀采样。每个 samplet 对应一段“应走到的加权弧长 arc”，
    // 再在 cut_Unoccupied_sample_trajs_ 上查找并插值出对应的 yaw / s / x / y。
    for(double samplet = Unoccupied_sampletime; samplet<Unoccupied_totalTrajTime_-1e-3; samplet+=Unoccupied_sampletime){
        // 计算当前时刻应走到的加权弧长 arc
        double arc = evaluateLength(samplet, Unoccupied_AllWeightingPathLength_, Unoccupied_totalTrajTime_, current_state_VAJ_.x(), 0.0, max_vel_, max_acc_);
        for (int k = Unoccupied_PathNodeIndex; k<PathNodeNum; k++){
            Eigen::VectorXd pathnode = cut_Unoccupied_sample_trajs_[k];
            Eigen::VectorXd prepathnode = cut_Unoccupied_sample_trajs_[k-1];
            // Unoccupied_Weightpathlengths[k] 表示“到第 k 个采样状态为止的累计加权路径长度”
            tmparc = Unoccupied_Weightpathlengths[k];
            
            // 如果累计加权长度大于当前时刻应走到的加权弧长，则进行插值
            if(tmparc >= arc){
                Unoccupied_PathNodeIndex = k; 
                // l: 当前这一段对应的加权长度增量
                // l1: 从当前段终点回退到目标 arc 还差多少
                double l1 = tmparc-arc;
                double l = Unoccupied_Weightpathlengths[k]-Unoccupied_Weightpathlengths[k-1];
                // interp_s / interp_yaw: 在当前段内线性插值得到的平坦变量
                double interp_s = Unoccupied_pathlengths[k-1] + (l-l1)/l*(pathnode[4]);
                double interp_yaw = cut_Unoccupied_sample_trajs_[k-1][2] + (l-l1)/l*(pathnode[3]);
                Unoccupied_traj_pts.emplace_back(interp_yaw, interp_s, samplet);

                // interp_x / interp_y: 在几何空间中对位置做线性插值。
                double interp_x = l1/l*prepathnode[0] + (l-l1)/l*(pathnode[0]);
                double interp_y = l1/l*prepathnode[1] + (l-l1)/l*(pathnode[1]);
                Unoccupied_positions.emplace_back(interp_x, interp_y, interp_yaw);
                break;
            }
        }
    }

    // 构造后端优化问题的起终状态。
    // 这里 flat output 采用两维变量 [yaw, s]：
    // startP / finalP 分别表示起终点的平坦位置。
    Eigen::MatrixXd startS;
    Eigen::MatrixXd endS;
    startS.resize(2,3);
    endS.resize(2,3);  
    Eigen::Vector2d startP(cut_Unoccupied_sample_trajs_[0][2],0);
    Eigen::Vector2d finalP(cut_Unoccupied_sample_trajs_[PathNodeNum-1][2],Unoccupied_pathlengths[PathNodeNum-1]);
    startS.col(0) = startP;
    startS.block(0,1,1,2) = current_state_OAJ_.transpose().head(2);
    startS.block(1,1,1,2) = current_state_VAJ_.transpose().head(2);
    endS.col(0) = finalP;
    endS.col(1).setZero();
    endS.col(2).setZero();

    // 将本函数得到的时间化前端轨迹写入 flat_traj_，供 back_end/optimizer 使用。
    flat_traj_.UnOccupied_traj_pts = Unoccupied_traj_pts;
    flat_traj_.UnOccupied_initT = Unoccupied_sampletime;
    flat_traj_.UnOccupied_positions = Unoccupied_positions;
  
    flat_traj_.start_state = startS;
    flat_traj_.final_state = endS;
    flat_traj_.start_state_XYTheta = start_state_;
    // flat_traj_.final_state_XYTheta = end_state_;
    flat_traj_.if_cut = if_cut;
    // 若轨迹被截断，则这里是截断点位姿；否则就是原始末端位姿。
    flat_traj_.final_state_XYTheta = cut_state;

    // flat_traj_.printFlatTrajData();
}

void JPSPlanner::normalizeAngle(const double &ref_angle, double &angle){
    while(ref_angle - angle > M_PI){
        angle += 2*M_PI;
    }
    while(ref_angle - angle < -M_PI){
        angle -= 2*M_PI;
    }
}

// 使用一维梯形/三角速度模型估计走完整段路径所需的总时间。
// 这里的 length 在本文件语境下通常不是单纯欧氏距离，
// 而是由距离项和航向变化项共同加权后的路径长度。
// 参数含义：
// 1. length: 待走完的总标量长度。
// 2. startV / endV: 该标量路径上的起止速度。
// 3. maxV / maxA: 允许的最大速度与最大加速度。
// 返回值：
// 1. 若路径足够长，可先加速到 maxV、再匀速、最后减速，返回梯形速度型总时长。
// 2. 若路径较短，达不到 maxV，则退化为三角速度型，返回对应总时长。
double JPSPlanner::evaluateDuration(const double &length, const double &startV, const double &endV, const double &maxV, const double &maxA){
  double critical_len; 
  double startv2 = pow(startV,2);  // 起始速度平方，用于位移公式 v^2 = v0^2 + 2as
  double endv2 = pow(endV,2);      // 终止速度平方
  double maxv2 = pow(maxV,2);      // 最大速度平方
  if(startV>maxV){
    startv2 = maxv2;
  }
  if(endV>max_vel_){
    endv2 = maxv2;
  }

  // critical_len 表示“恰好能加速到 maxV 再减速到 endV”所需的最短长度。
  // 若实际 length 更大，则存在中间匀速段；否则只能形成三角速度型。
  critical_len = (maxv2-startv2)/(2*maxA)+(maxv2-endv2)/(2*maxA);
  if(length>=critical_len){
    return (maxV-startV)/maxA+(maxV-endV)/maxA+(length-critical_len)/maxV;
  }
  else{
    // tmpv 是受路径长度限制时实际能达到的峰值速度，小于等于 maxV。
    double tmpv = sqrt(0.5*(startv2+endv2+2*maxA*length));
    return (tmpv-startV)/maxA + (tmpv-endV)/maxA;
  }
}



// Use trapezoidal velocity profile to get the distance at the curt timestamp
double JPSPlanner::evaluateLength(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV, const double &maxV, const double &maxA){
  // std::cout<<"curt: "<<curt<<"  locallength: "<<locallength<<"  localtime: "<<localtime<<"  startV: "<<startV<<"  endV: "<<endV<<"  maxV: "<<maxV<<"  maxA: "<<maxA<<std::endl;
  double critical_len; 
  double startv2 = pow(startV,2);
  double endv2 = pow(endV,2);
  double maxv2 = pow(maxV,2);
  if(startV>maxV){
    startv2 = maxv2;
  }
  if(endV>max_vel_){
    endv2 = maxv2;
  }

  critical_len = (maxv2-startv2)/(2*maxA)+(maxv2-endv2)/(2*maxA);
  // Get time from trapezoidal velocity profile
  if(locallength>=critical_len){// If locallength is greater than critical_len, accelerate to max speed and then decelerate
    double t1 = (maxV-startV)/maxA;
    double t2 = t1+(locallength-critical_len)/maxV;
    if(curt<=t1){
      return startV*curt + 0.5*maxA*pow(curt,2);
    }
    else if(curt<=t2){
      return startV*t1 + 0.5*maxA*pow(t1,2)+(curt-t1)*maxV;
    }
    else{
      return startV*t1 + 0.5*maxA*pow(t1,2) + (t2-t1)*maxV + maxV*(curt-t2)-0.5*maxA*pow(curt-t2,2);
    }
  }
  else{// If locallength is less than critical_len, decelerate without accelerating to max speed
    double tmpv = sqrt(0.5*(startv2+endv2+2*maxA*locallength));
    double tmpt = (tmpv-startV)/maxA;
    if(curt<=tmpt){
      return startV*curt+0.5*maxA*pow(curt,2);
    }
    else{
      return startV*tmpt+0.5*maxA*pow(tmpt,2) + tmpv*(curt-tmpt)-0.5*maxA*pow(curt-tmpt,2);
    }
  }
}

// 未被调用
double JPSPlanner::evaluateVel(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV, const double &maxV, const double &maxA){
  double critical_len; 
  double startv2 = pow(startV,2);
  double endv2 = pow(endV,2);
  double maxv2 = pow(maxV,2);
  if(startV>maxV){
    startv2 = maxv2;
  }
  if(endV>max_vel_){
    endv2 = maxv2;
  }

  critical_len = (maxv2-startv2)/(2*maxA)+(maxv2-endv2)/(2*maxA);
  // Get time from trapezoidal velocity profile
  if(locallength>=critical_len){// If locallength is greater than critical_len, accelerate to max speed and then decelerate
    double t1 = (maxV-startV)/maxA;
    double t2 = t1+(locallength-critical_len)/maxV;
    if(curt<=t1){
      return startV + maxA*curt;
    }
    else if(curt<=t2){
      return maxV;
    }
    else{
      return maxV - maxA*(curt-t2);
    }
  }
  else{// If locallength is less than critical_len, decelerate without accelerating to max speed
    double tmpv = sqrt(0.5*(startv2+endv2+2*maxA*locallength));
    double tmpt = (tmpv-startV)/maxA;
    if(curt<=tmpt){
      return startV + maxA*curt;
    }
    else{
      return tmpv - maxA * (curt - tmpt);
    }
  }
}

// 未被调用
double JPSPlanner::evaluteTimeOfPos(const double &pos, const double &locallength, const double &startV, const double &endV, const double &maxV, const double &maxA){
  double critical_len;
  double startv2 = pow(startV,2);
  double endv2 = pow(endV,2);
  double maxv2 = pow(maxV,2);
  double localpos = pos;
  if(startV>maxV){
    startv2 = maxv2;
  }
  if(endV>max_vel_){
    endv2 = maxv2;
  }
  if(pos>locallength){
    localpos = locallength;
  }

  critical_len = (maxv2-startv2)/(2*maxA)+(maxv2-endv2)/(2*maxA);
  if(locallength>=critical_len){
    double s1 = (maxv2-startv2)/maxA/2.0;
    if(localpos < s1){
      return (sqrt(startV*startV + 2*maxA*localpos)-startV)/maxA;
    }
    double s2 = locallength - (maxv2-endv2)/maxA/2.0;
    if(localpos < s2){
      return (maxV - startV)/maxA + (localpos-s1)/maxV;
    }
    else{
        return (maxV - startV)/maxA + (s2-s1)/maxV + (maxV - sqrt(maxv2-2.0*maxA*(localpos-s2)))/maxA;
    }
  }
  else{
    double v_m = sqrt(0.5*(startv2+endv2+2*maxA*locallength));
    if(localpos < (v_m*v_m - startv2)/2.0/maxA){
      return (sqrt(startV*startV + 2*maxA*localpos)-startV)/maxA;
    }
    else{
      double rest_s = pos - (v_m*v_m - startv2)/2.0/maxA;
      return (v_m - startV)/maxA + (v_m - sqrt(v_m*v_m - 2*maxA*rest_s))/maxA;
    }
  }
}

bool JPSPlanner::JPS_check_if_collision(const Eigen::Vector2d &pos){
  return map_util_->getDistanceReal(pos) > safe_dis_;
}
