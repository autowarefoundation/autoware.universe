#include "tracker.h"
#include "hungarian_bigraph_matcher.h"

Track::Track()
{
    // marker_pub = nh.advertise<visualization_msgs::MarkerArray>("track_info", 1);
    Tracked_object_.clear();
}

void Track::setNewObjects(std::vector<tracker> trackers,double frame_time)
{
    double new_time = 0.0;
    Current_Tracked_object_.clear();
    if (trackers.size() == 0)
    {
        return;
    }
    else
    {
        car_pose_(0, 0) = 1.;
        car_pose_(1, 1) = 1.;
        car_pose_(2, 2) = 1.;
        
        for (int i = 0; i < trackers.size(); ++i)
        {
            //激光雷达坐标系转到全局坐标系,此处为单位矩阵
            Eigen::Vector3f tmp_position;
            tmp_position << trackers[i].center[0], trackers[i].center[1], 1;
            trackers[i].center = car_pose_ * tmp_position;
        }
    }
    if (Tracked_object_.size() == 0)
    {
        createNewTrackers(trackers);
    }
    else{
        new_time = frame_time;
        double diff_time = new_time - last_frame_time_;
        //预测跟踪物的位置
        predictOldTrackers(diff_time);
        std::vector<std::vector<double> > cost;
        //计算关联矩阵
        computeAssociateMatrix(Tracked_object_, trackers, cost);
        std::vector<int> tracks_idx;
        std::vector<int> objects_idx;
        //匈牙利算法匹配
        HungarianOptimizer hungarian_optimizer(cost);
        hungarian_optimizer.minimize(&tracks_idx, &objects_idx);
        std::vector<std::pair<int, int> > local_assignments;
        std::vector<int> local_unassigned_tracks;
        std::vector<int> local_unassigned_objects;
        local_assignments.resize(trackers.size());
        local_unassigned_tracks.assign(Tracked_object_.size(), -1);
        local_unassigned_objects.assign(trackers.size(), -1);
        AssignObjectsToTracks(tracks_idx,objects_idx,cost,&local_assignments,&local_unassigned_tracks,&local_unassigned_objects);
        updateTrackers(trackers,local_assignments,local_unassigned_tracks,local_unassigned_objects,diff_time);
    }
    int box_count = 0;
    visualization_msgs::MarkerArray markers;
    markers.markers.clear();
    for(int i = 0;i<Current_Tracked_object_.size();++i)
    {
        Current_Tracked_object_[i].center = car_pose_.inverse()*Current_Tracked_object_[i].center;
        if(Current_Tracked_object_[i].center[0]>20||Current_Tracked_object_[i].center[0]<-20)
            continue;
        if(Current_Tracked_object_[i].center[1]>10||Current_Tracked_object_[i].center[1]<-10)
            continue;
        visualization_msgs::Marker marker1,marker2,marker3;
        draw_box(Current_Tracked_object_[i],box_count++,marker1);
        draw_text(Current_Tracked_object_[i],box_count++,marker2);
        draw_arrow(Current_Tracked_object_[i],box_count++,marker3);
        markers.markers.push_back(marker1);
        markers.markers.push_back(marker2);
        markers.markers.push_back(marker3);
    }
    /****需要发布markers，未移植******/
    // marker_pub.publish(markers);
    last_frame_time_ = new_time;
    std::cout<<"the size of tracks is "<<Tracked_object_.size()<<std::endl;
}

void Track::getObjects(std::vector<tracker> &trackers)
{
    trackers = Current_Tracked_object_;
}

void Track::getCarPose(Eigen::Matrix3f &carpose)
{
    carpose = car_pose_;
}

void Track::updateTrackers(std::vector<tracker> trackers, std::vector<std::pair<int, int> > assignments,
                           std::vector<int> unassigned_tracks, std::vector<int> unassigned_objects,double diff_time)
{
    for(int i=0;i<assignments.size();++i)
    {
        Tracked_object_[assignments[i].first].untracked_time = 0;
        Tracked_object_[assignments[i].first].center = trackers[assignments[i].second].center;
        Tracked_object_[assignments[i].first].vel_kalman(Tracked_object_[assignments[i].first].center[0],Tracked_object_[assignments[i].first].center[1],diff_time);
        Tracked_object_[assignments[i].first].num_points = trackers[assignments[i].second].num_points;
        Tracked_object_[assignments[i].first].corners[0] = trackers[assignments[i].second].corners[0];
        Tracked_object_[assignments[i].first].corners[1] = trackers[assignments[i].second].corners[1];
        Tracked_object_[assignments[i].first].corners[2] = trackers[assignments[i].second].corners[2];
        Tracked_object_[assignments[i].first].corners[3] = trackers[assignments[i].second].corners[3];
        Tracked_object_[assignments[i].first].corners[4] = trackers[assignments[i].second].corners[4];
        Tracked_object_[assignments[i].first].corners[5] = trackers[assignments[i].second].corners[5];
        Tracked_object_[assignments[i].first].corners[6] = trackers[assignments[i].second].corners[6];
        Tracked_object_[assignments[i].first].corners[7] = trackers[assignments[i].second].corners[7];
        Tracked_object_[assignments[i].first].latest_tracked_time = trackers[assignments[i].second].latest_tracked_time;
        Current_Tracked_object_.push_back(Tracked_object_[assignments[i].first]);
    }
    for(int i=0;i<unassigned_tracks.size();++i)
    {
        Tracked_object_[unassigned_tracks[i]].untracked_time++;
        Tracked_object_[unassigned_tracks[i]].velocity<<0,0,0;
    }
    size_t track_num = 0;
    std::vector<int> vec_track_id;//保留跟踪物的track_id，给后面新的tracker指定id
    for(int i=0;i<Tracked_object_.size();++i)
    {
        if(Tracked_object_[i].untracked_time>5)
            continue;
        if(i==track_num)
        {
            vec_track_id.push_back(Tracked_object_[i].track_id);
            track_num++;
        }
        else
        {
            tracker tmp_track = Tracked_object_[i];
            vec_track_id.push_back(Tracked_object_[i].track_id);
            Tracked_object_[track_num] = tmp_track;
            track_num++;
        }
        
    }
    //保留前track_num个元素
    Tracked_object_.resize(track_num);
    for(int i=0;i<unassigned_objects.size();++i)
    {
        int j = 0;
        std::vector<int>::iterator result;
        while(result!=vec_track_id.end())
        {
            result = find(vec_track_id.begin(), vec_track_id.end(), j);
            ++j;
        }
        vec_track_id.push_back(j-1);
        trackers[unassigned_objects[i]].track_id = j-1;
        Tracked_object_.push_back(trackers[unassigned_objects[i]]);
        Current_Tracked_object_.push_back(trackers[unassigned_objects[i]]);
    }
}

void Track::createNewTrackers(std::vector<tracker> trackers)
{
    for (int i = 0; i < trackers.size(); ++i)
    {
        trackers[i].track_id = i;
        Tracked_object_.push_back(trackers[i]);
    }
}

void Track::predictOldTrackers(double diff_time)
{
    for (int i = 0; i < Tracked_object_.size(); ++i)
    {
        // std::cout<<"the velocity of i is "<<Tracked_object_[i].velocity[0]<<","<<Tracked_object_[i].velocity[1]<<std::endl;
        Tracked_object_[i].center[0] += diff_time * Tracked_object_[i].velocity[0];
        Tracked_object_[i].center[1] += diff_time * Tracked_object_[i].velocity[1];
    }
}

void Track::computeAssociateMatrix(
    const std::vector<tracker> &tracks,
    const std::vector<tracker> &new_objects,
    std::vector<std::vector<double> > &cost)
{
    // Compute matrix of association distance
    Eigen::MatrixXf association_mat(tracks.size(), new_objects.size());
    int no_track = tracks.size();
    int no_object = new_objects.size();
    for (size_t i = 0; i < tracks.size(); ++i)
    {
        for (size_t j = 0; j < new_objects.size(); ++j)
        {
            float diff_points_num = fabs(new_objects[j].num_points - tracks[i].num_points);
            float num_cost = diff_points_num / 50.;
            float diff_position = (tracks[i].center - new_objects[j].center).norm();
            float position_cost = diff_position / 3.;
            float sum_cost = position_cost * 0.6 + num_cost * 0.4;
            // std::cout<<"the i and j is "<<num_cost<<","<<position_cost<<","<<sum_cost<<std::endl;
            association_mat(i, j) = sum_cost;
        }
    }
    cost.resize(tracks.size() + new_objects.size());
    for (int i = 0; i < no_track; ++i)
    {
        cost[i].resize(association_mat.cols());
        for (int j = 0; j < association_mat.cols(); ++j)
        {
            cost[i][j] = association_mat(i, j);
        }
    }
    for (int i = 0; i < no_object; ++i)
    {
        cost[i + no_track].resize(association_mat.cols());
        for (int j = 0; j < association_mat.cols(); ++j)
        {
            if (j == i)
            {
                cost[i + no_track][j] = 999 * 1.2f;
            }
            else
            {
                cost[i + no_track][j] = 999999.0f;
            }
        }
    }
}

void Track::AssignObjectsToTracks(
    std::vector<int> tracks_idx,
    std::vector<int> objects_idx,
    std::vector<std::vector<double> > cost_value,
    std::vector<std::pair<int, int> > * assignments,
    std::vector<int> *unassigned_tracks, std::vector<int> *unassigned_objects)
{
    int assignments_num = 0;
    int no_track = cost_value.size() - cost_value[0].size();
    int no_object = cost_value[0].size();
    std::vector<bool> tracks_used(no_track+no_object, false);
    std::vector<bool> objects_used(no_object, false);
    for (size_t i = 0; i < tracks_idx.size(); ++i)
    {
        if (tracks_idx[i] < 0 || tracks_idx[i] >= no_track || objects_idx[i] < 0 ||
            objects_idx[i] >= no_object)
        {
            continue;
        }
        if (cost_value[tracks_idx[i]][objects_idx[i]] <3.)
        {
            (*assignments)[assignments_num++] =
                std::make_pair(tracks_idx[i], objects_idx[i]);
            tracks_used[tracks_idx[i]] = true;
            objects_used[objects_idx[i]] = true;
        }
    }
    assignments->resize(assignments_num);
    unassigned_tracks->resize(no_track);
    int unassigned_tracks_num = 0;
    for (int i = 0; i < no_track; ++i)
    {
        if (tracks_used[i] == false)
        {
            (*unassigned_tracks)[unassigned_tracks_num++] = i;
        }
    }
    unassigned_tracks->resize(unassigned_tracks_num);
    unassigned_objects->resize(no_object);
    int unassigned_objects_num = 0;
    for (int i = 0; i < no_object; ++i)
    {
        if (objects_used[i] == false)
        {
            (*unassigned_objects)[unassigned_objects_num++] = i;
        }
    }
    unassigned_objects->resize(unassigned_objects_num);
}



void Track::draw_box(const tracker &tmp_tracker, const int &marker_id, visualization_msgs::Marker &marker)
{

    marker.id = marker_id;
    marker.type_ = 5;
	marker.action = 0;
    // marker.type_ = visualization_msgs::Marker::LINE_LIST;
    // marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "rslidar";
    std::vector<geometry_msgs::Point> cub_points;

    for (int i = 0; i < 8; ++i)
    {
        geometry_msgs::Point pts;
        pts.x = tmp_tracker.corners[i].x;
        pts.y = tmp_tracker.corners[i].y;
        pts.z = tmp_tracker.corners[i].z;
        cub_points.push_back(pts);
    }

    marker.scale.x = 0.1;
    marker.color.b = 1;
    marker.color.a = 1;
    // marker.lifetime = ros::Duration(0.1);

    marker.points.push_back(cub_points[0]);
    marker.points.push_back(cub_points[1]);
    marker.points.push_back(cub_points[1]);
    marker.points.push_back(cub_points[2]);
    marker.points.push_back(cub_points[2]);
    marker.points.push_back(cub_points[3]);
    marker.points.push_back(cub_points[3]);
    marker.points.push_back(cub_points[0]);
    // horizontal high poisnts for lines
    marker.points.push_back(cub_points[4]);
    marker.points.push_back(cub_points[5]);
    marker.points.push_back(cub_points[5]);
    marker.points.push_back(cub_points[6]);
    marker.points.push_back(cub_points[6]);
    marker.points.push_back(cub_points[7]);
    marker.points.push_back(cub_points[7]);
    marker.points.push_back(cub_points[4]);
    // vertical points for lines
    marker.points.push_back(cub_points[0]);
    marker.points.push_back(cub_points[4]);
    marker.points.push_back(cub_points[1]);
    marker.points.push_back(cub_points[5]);
    marker.points.push_back(cub_points[2]);
    marker.points.push_back(cub_points[6]);
    marker.points.push_back(cub_points[3]);
    marker.points.push_back(cub_points[7]);
}


void Track::draw_text(const tracker& pos, const int& marker_id, visualization_msgs::Marker& marker)
{
	//--------------标记跟踪信息----------
	marker.id = marker_id;
    marker.header.frame_id = "rslidar";

	// marker.type_ = visualization_msgs::Marker::TEXT_VIEW_FACING;
	// marker.action = visualization_msgs::Marker::ADD;
	marker.type_ = 5;
	marker.action = 0;
	marker.pose.position.x = pos.center[0];
	marker.pose.position.y = pos.center[1];
	marker.pose.position.z = 0.;
    marker.ns = "pos_info";
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
	marker.color.r = 1;
	marker.color.g = 1;
	marker.color.b = 0.;
	marker.color.a = 1.0;
    // marker.lifetime = ros::Duration(0.1);
    std::string info;
    std::stringstream ss;
    ss << pos.track_id;
    info = ss.str();
	marker.text = info;

}

void Track::draw_arrow(const tracker& pos, const int& marker_id, visualization_msgs::Marker& marker)
{
	//--------------标记跟踪信息----------
	marker.id = marker_id;
    marker.header.frame_id = "rslidar";
    marker.type_ = 5;
	marker.action = 0;
	// marker.type_ = visualization_msgs::Marker::ARROW;
	// marker.action = visualization_msgs::Marker::ADD;
	marker.type_ = 5;
	marker.action = 0;
	marker.pose.position.x = pos.center[0];
	marker.pose.position.y = pos.center[1];
    marker.pose.position.z = 0.;
    Eigen::Vector3f tmp_velocity(pos.velocity[0],pos.velocity[1],1);
    Eigen::Matrix3f world_to_car(Eigen::Matrix3f::Identity());
    world_to_car.block(0,0,2,2) = car_pose_.block(0,0,2,2);
    tmp_velocity = world_to_car.inverse()*tmp_velocity;

    /*欧拉角转化为四元素,未移植*/
    // tf::Quaternion qua = tf::createQuaternionFromYaw(atan2f(tmp_velocity[1],tmp_velocity[0]));
    // marker.pose.orientation.x = qua.x();
    // marker.pose.orientation.y = qua.y();
    // marker.pose.orientation.z = qua.z();
    // marker.pose.orientation.w = qua.w();
    marker.ns = "vel_info";
    float scale_x = pos.velocity.norm();
    if(scale_x <1e-5)
        scale_x = 0.1;
    //std::cout<<"the scale is "<<scale_x<<std::endl;
	marker.scale.x = scale_x;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;
	marker.color.r = 0;
	marker.color.g = 1;
	marker.color.b = 0.;
	marker.color.a = 1.0;
    // marker.lifetime = ros::Duration(0.1);
}