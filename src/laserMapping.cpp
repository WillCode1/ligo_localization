/*
 * BSD 3-Clause License

 *  Copyright (c) 2025, Dongjiao He
 *  All rights reserved.
 *
 *  Author: Dongjiao HE <hdj65822@connect.hku.hk>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

// #include <so3_math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "li_initialization.h"
#include <malloc.h>
// #include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "chi-square.h"
// #include <ros/console.h>
#include "backend_optimization/global_localization/Relocalization.hpp"


#define PUBFRAME_PERIOD     (20)

const float MOV_THRESHOLD = 1.5f;

string root_dir = ROOT_DIR;

int time_log_counter = 0; 

bool init_map = false, flg_first_scan = true;
std::vector<ObsPtr> gnss_cur;
nav_msgs::OdometryPtr nmea_cur;
Eigen::Vector3d first_pvt_anc, first_lla_anc;
Eigen::Vector3d first_pvt_used, first_lla_used;

bool  flg_reset = false, flg_exit = false;

//surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
std::deque<PointCloudXYZI::Ptr> depth_feats_world;
pcl::VoxelGrid<PointType> downSizeFilterSurf;
shared_ptr<Relocalization> relocalization;

V3D euler_cur;

std::string map_frame = "map";
std::string odom_frame = "odom";
nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::PoseStamped msg_body_pose;
ros::Publisher pubGlobalMap;

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    {
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void MapIncremental() {
    PointVector points_to_add;
    int cur_pts = feats_down_world->size();
    points_to_add.reserve(cur_pts);
    
    for (size_t i = 0; i < cur_pts; ++i) {
        /* decide if need add to map */
        PointType &point_world = feats_down_world->points[i];
        if (!Nearest_Points[i].empty()) {
            const PointVector &points_near = Nearest_Points[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) * filter_size_map_min;
            bool need_add = true;
            for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
                Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
                if (fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    }
    ivox_->AddPoints(points_to_add);
}

void publish_init_map(const ros::Publisher & pubLaserCloudFullRes)
{
    int size_init_map = init_feats_world->size();

    sensor_msgs::PointCloud2 laserCloudmsg;
                
    pcl::toROSMsg(*init_feats_world, laserCloudmsg);
        
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = map_frame;
    pubLaserCloudFullRes.publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFullRes)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body); // (points_num); // 
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));
        
        for (int i = 0; i < size; i++)
        {
            // if (i % 3 == 0)
            {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x; // updatedmap[i / 3](0); // 
            laserCloudWorld->points[i].y = feats_down_world->points[i].y; // updatedmap[i / 3](1); // 
            laserCloudWorld->points[i].z = feats_down_world->points[i].z; // updatedmap[i / 3](2); // 
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity; // feats_down_world->points[i].y; // updatedmap[i / 3](2); //feats_down_world->points[i].z; // 
            }
        }
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time); // (map_time); // 
        laserCloudmsg.header.frame_id = map_frame;
        pubLaserCloudFullRes.publish(laserCloudmsg);
        // publish_count -= PUBFRAME_PERIOD;
    }
    
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = points_num; // feats_down_world->points.size();
        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x; // updatedmap[i](0); //
            laserCloudWorld->points[i].y = feats_down_world->points[i].y; // updatedmap[i](1); //
            laserCloudWorld->points[i].z = feats_down_world->points[i].z; // updatedmap[i](2); //
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity; // updatedmap[i](2); //
        }

        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        pointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
}

template<typename T>
void set_posestamp(T & out)
{
    {
        out.position.x = kf_output.x_.pos(0);
        out.position.y = kf_output.x_.pos(1);
        out.position.z = kf_output.x_.pos(2);
        Eigen::Quaterniond q(kf_output.x_.rot);
        out.orientation.x = q.coeffs()[0];
        out.orientation.y = q.coeffs()[1];
        out.orientation.z = q.coeffs()[2];
        out.orientation.w = q.coeffs()[3];
    }
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = map_frame;
    odomAftMapped.child_frame_id = odom_frame;
    if (publish_odometry_without_downsample)
    {
        odomAftMapped.header.stamp = ros::Time().fromSec(time_current);
    }
    else
    {
        odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    }
    set_posestamp(odomAftMapped.pose.pose);
    
    pubOdomAftMapped.publish(odomAftMapped);

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, map_frame, odom_frame ) );
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose.pose);
    // msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = map_frame;
    static int jjj = 0;
    jjj++;
    // if (jjj % 2 == 0) // if path is too large, the rvis will crash
    {
        path.poses.emplace_back(msg_body_pose);
        pubPath.publish(path);
    }
}        

void load_parameters()
{
    relocalization = make_shared<Relocalization>();
    ros::param::param("/laserMapping/scan_context/lidar_height", relocalization->sc_manager->LIDAR_HEIGHT, 2.0);
    ros::param::param("/laserMapping/scan_context/sc_dist_thres", relocalization->sc_manager->SC_DIST_THRES, 0.5);

    V3D extrinT_eigen;
    M3D extrinR_eigen;
#if 0
    ros::param::param("/laserMapping/mapping/extrinsicT_imu2gnss", extrinT, vector<double>());
    ros::param::param("/laserMapping/mapping/extrinsicR_imu2gnss", extrinR, vector<double>());
    extrinT_eigen << VEC_FROM_ARRAY(extrinT);
    extrinR_eigen << MAT_FROM_ARRAY(extrinR);
    relocalization->set_extrinsic(extrinT_eigen, extrinR_eigen);
#endif

    ros::param::param("/laserMapping/relocalization_cfg/algorithm_type", relocalization->algorithm_type, std::string("UNKONW"));

    BnbOptions match_option;
    ros::param::param("/laserMapping/bnb3d/linear_xy_window_size", match_option.linear_xy_window_size, 10.);
    ros::param::param("/laserMapping/bnb3d/linear_z_window_size", match_option.linear_z_window_size, 1.);
    ros::param::param("/laserMapping/bnb3d/angular_search_window", match_option.angular_search_window, 30.);
    ros::param::param("/laserMapping/bnb3d/pc_resolutions", match_option.pc_resolutions, vector<double>());
    ros::param::param("/laserMapping/bnb3d/bnb_depth", match_option.bnb_depth, 5);
    ros::param::param("/laserMapping/bnb3d/min_score", match_option.min_score, 0.1);
    ros::param::param("/laserMapping/bnb3d/enough_score", match_option.enough_score, 0.8);
    ros::param::param("/laserMapping/bnb3d/min_xy_resolution", match_option.min_xy_resolution, 0.2);
    ros::param::param("/laserMapping/bnb3d/min_z_resolution", match_option.min_z_resolution, 0.1);
    ros::param::param("/laserMapping/bnb3d/min_angular_resolution", match_option.min_angular_resolution, 0.1);
    ros::param::param("/laserMapping/bnb3d/filter_size_scan", match_option.filter_size_scan, 0.1);
    ros::param::param("/laserMapping/bnb3d/debug_mode", match_option.debug_mode, false);

    ros::param::param("/laserMapping/mapping/extrinsic_T", extrinT, vector<double>());
    ros::param::param("/laserMapping/mapping/extrinsic_R", extrinR, vector<double>());
    extrinT_eigen << VEC_FROM_ARRAY(extrinT);
    extrinR_eigen << MAT_FROM_ARRAY(extrinR);
    V3D ext_rpy = EigenMath::RotationMatrix2RPY(extrinR_eigen);
    Pose lidar_extrinsic;
    lidar_extrinsic.x = extrinT_eigen.x();
    lidar_extrinsic.y = extrinT_eigen.y();
    lidar_extrinsic.z = extrinT_eigen.z();
    lidar_extrinsic.roll = ext_rpy.x();
    lidar_extrinsic.pitch = ext_rpy.y();
    lidar_extrinsic.yaw = ext_rpy.z();
    relocalization->set_bnb3d_param(match_option, lidar_extrinsic);

    double step_size, resolution;
    ros::param::param("/laserMapping/ndt/step_size", step_size, 0.1);
    ros::param::param("/laserMapping/ndt/resolution", resolution, 1.);
    relocalization->set_ndt_param(step_size, resolution);

    bool use_gicp;
    double gicp_downsample, filter_range, search_radius, teps, feps, fitness_score;
    ros::param::param("/laserMapping/gicp/use_gicp", use_gicp, true);
    ros::param::param("/laserMapping/gicp/filter_range", filter_range, 80.);
    ros::param::param("/laserMapping/gicp/gicp_downsample", gicp_downsample, 0.2);
    ros::param::param("/laserMapping/gicp/search_radius", search_radius, 0.5);
    ros::param::param("/laserMapping/gicp/teps", teps, 1e-3);
    ros::param::param("/laserMapping/gicp/feps", feps, 1e-3);
    ros::param::param("/laserMapping/gicp/fitness_score", fitness_score, 0.3);
    relocalization->set_gicp_param(use_gicp, filter_range, gicp_downsample, search_radius, teps, feps, fitness_score);
}

bool system_state_vaild = false;
PointCloudType::Ptr global_map;

bool run_relocalization(PointCloudType::Ptr scan, const double &lidar_beg_time)
{
    if (!system_state_vaild)
    {
        Eigen::Matrix4d imu_pose;
        if (relocalization->run(scan, imu_pose, lidar_beg_time))
        {
            kf_output.x_.rot = M3D(imu_pose.topLeftCorner(3, 3));
            kf_output.x_.pos = V3D(imu_pose.topRightCorner(3, 1));
            kf_output.x_.vel.setZero();
            kf_output.x_.ba.setZero();
            kf_output.x_.bg.setZero();
            system_state_vaild = true;
        }
        else
        {
#ifdef DEDUB_MODE
            kf_output.x_.rot = M3D(imu_pose.topLeftCorner(3, 3));
            kf_output.x_.pos = V3D(imu_pose.topRightCorner(3, 1));
            kf_output.x_.vel.setZero();
            kf_output.x_.ba.setZero();
            kf_output.x_.bg.setZero();
#endif
        }
    }
    return system_state_vaild;
}

void init_global_map(PointCloudType::Ptr &submap)
{
    if (!ivox_->grids_map_.empty())
    {
        LOG_ERROR("Error, ivox not null when initializing the map!");
        std::exit(100);
    }
    ivox_->AddPoints(submap->points);
}

void init_system_mode()
{
    string globalmap_path = PCD_FILE_DIR("globalmap.pcd");
    string trajectory_path = PCD_FILE_DIR("trajectory.pcd");
    string scd_path = PCD_FILE_DIR("scancontext/");

    /*** init localization mode ***/
    if (access(globalmap_path.c_str(), F_OK) != 0)
    {
        LOG_ERROR("File not exist! Please check the \"globalmap_path\".");
        std::exit(100);
    }

    Timer timer;
    global_map.reset(new PointCloudType());
    pcl::io::loadPCDFile(globalmap_path, *global_map);
    if (global_map->points.size() < 5000)
    {
        LOG_ERROR("Too few point clouds! Please check the map file.");
        std::exit(100);
    }
    LOG_WARN("Load pcd successfully! There are %lu points in map. Cost time %fms.", global_map->points.size(), timer.elapsedLast());

    if (!relocalization->load_prior_map(global_map))
    {
        std::exit(100);
    }

    pcl::io::loadPCDFile(trajectory_path, *relocalization->trajectory_poses);
    if (relocalization->trajectory_poses->points.size() < 10)
    {
        LOG_ERROR("Too few point clouds! Please check the trajectory file.");
        std::exit(100);
    }
    LOG_WARN("Load trajectory poses successfully! There are %lu poses.", relocalization->trajectory_poses->points.size());

    if (!relocalization->load_keyframe_descriptor(scd_path))
    {
        relocalization->algorithm_type = "manually_set";
        LOG_ERROR("Load keyframe descriptor failed, set algorithm_type to manually_set!");
    }
    else
        LOG_WARN("Load keyframe descriptor successfully! There are %lu descriptors.", relocalization->sc_manager->polarcontexts_.size());

    /*** initialize the map ivox ***/
    init_global_map(global_map);
}

void publish_global_map(const ros::TimerEvent &)
{
    if (pubGlobalMap.getNumSubscribers() != 0)
    {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*global_map, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = map_frame;
        pubGlobalMap.publish(cloud_msg);
    }
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    const geometry_msgs::Pose &pose = msg->pose.pose;
    const auto &ori = msg->pose.pose.orientation;
    Eigen::Quaterniond quat(ori.w, ori.x, ori.y, ori.z);
    auto rpy = EigenMath::Quaternion2RPY(quat);
    // prior pose in map(imu pose)
    Pose init_pose;
    init_pose.x = pose.position.x;
    init_pose.y = pose.position.y;
    init_pose.z = pose.position.z;
    // init_pose.roll = DEG2RAD(lidar_turnover_roll);
    // init_pose.pitch = DEG2RAD(lidar_turnover_pitch);
    init_pose.roll = 0;
    init_pose.pitch = 0;
    init_pose.yaw = rpy.z();
    relocalization->set_init_pose(init_pose);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(0);
    spinner.start();
    readParameters(nh);
    cout<<"lidar_type: "<<lidar_type<<endl;
    ivox_ = std::make_shared<IVoxType>(ivox_options_);
    ivox_last_ = std::make_shared<IVoxType>(ivox_options_); //(*ivox_);
#if 1
    load_parameters();
    init_system_mode();
#endif
    
    path.header.stamp    = ros::Time().fromSec(lidar_end_time);
    path.header.frame_id =map_frame;

    /*** variables definition for counting ***/
    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_propag = 0;

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    {
        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    }

    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;
    if (GNSS_ENABLE)
    {
        std::copy(default_gnss_iono_params.begin(), default_gnss_iono_params.end(), 
            std::back_inserter(p_gnss->p_assign->latest_gnss_iono_params));
        p_gnss->Tex_imu_r << VEC_FROM_ARRAY(extrinT_gnss);
        p_gnss->gnss_ready = false; // gnss_quick_init; // edit
        p_gnss->nolidar = nolidar; // edit
        p_gnss->pre_integration->setnoise();

        if (p_gnss->p_assign->ephem_from_rinex)
        {
            p_gnss->p_assign->Ephemfromrinex(LOCAL_FILE_DIR(ephem_fname));
        }
    }
    else if (NMEA_ENABLE)
    {
        p_nmea->Tex_imu_r << VEC_FROM_ARRAY(extrinT_gnss);
        p_nmea->Rex_imu_r << MAT_FROM_ARRAY(extrinR_gnss);
        p_nmea->nmea_ready = false; // gnss_quick_init; // edit
        p_nmea->nolidar = nolidar; // edit
        p_nmea->pre_integration->setnoise();
    }
    if (NMEA_ENABLE)
    {
        kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output, h_model_NMEA_output);
    }
    else
    {
        kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output, h_model_GNSS_output);
    }
    Eigen::Matrix<double, 24, 24> P_init_output; // = MD(24, 24)::Identity() * 0.01;
    reset_cov_output(P_init_output);
    kf_output.change_P(P_init_output);
    Eigen::Matrix<double, 24, 24> Q_output = process_noise_cov_output();
    open_file();

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);

    ros::Subscriber sub_ephem, sub_glo_ephem, sub_gnss_meas, sub_gnss_iono_params, sub_nmea_meas;
    ros::Subscriber sub_gnss_time_pluse_info, sub_local_trigger_info;
    ros::Subscriber sub_rtk_pvt_info, sub_rtk_lla_info;
    if (GNSS_ENABLE)
    {
        sub_ephem = nh.subscribe(gnss_ephem_topic, 10000, gnss_ephem_callback);
        sub_glo_ephem = nh.subscribe(gnss_glo_ephem_topic, 10000, gnss_glo_ephem_callback);
        if (p_gnss->p_assign->obs_from_rinex)
        {
            sub_gnss_meas = nh.subscribe("/gnss_preprocessor_node/GNSSPsrCarRov1", 200, gnss_meas_callback_urbannav);
            sub_rtk_pvt_info = nh.subscribe("/gnss_preprocessor_node/ECEFSolutionRTK", 500, rtklibOdomHandler); 
        }
        else
        {
            sub_gnss_meas = nh.subscribe(gnss_meas_topic, 10000, gnss_meas_callback);
            
            sub_rtk_lla_info = nh.subscribe(rtk_lla_topic, 1000, rtk_lla_callback); // not used
        }

        if (p_gnss->p_assign->pvt_is_gt)
        {
            sub_rtk_pvt_info = nh.subscribe(rtk_pvt_topic, 1000, rtk_pvt_callback); // 订阅rtk数据作为GT对比
        }
        else
        {
            std::vector<Eigen::Vector4d> gt_holder;
            if (gt_file_type == LIVOX)
            {
                GtfromTXT_LIVOX(LOCAL_FILE_DIR(gt_fname), gt_holder);
            }
            else if (gt_file_type == URBAN)
            {
                GtfromTXT_URBAN(LOCAL_FILE_DIR(gt_fname), gt_holder);
            }
            else if (gt_file_type == M2DGR)
            {
                GtfromTXT_M2DGR(LOCAL_FILE_DIR(gt_fname), gt_holder);
            }
            std::cout << "check gt size:" << gt_holder.size() << std::endl;
            if (gt_file_type == M2DGR)
            {
                for (size_t i = 0; i < gt_holder.size(); i++)
                {
                    inputpvt_ecef(gt_holder[i][0], gt_holder[i][1], gt_holder[i][2], gt_holder[i][3], p_gnss->first_lla_pvt, p_gnss->first_xyz_ecef_pvt, p_gnss->pvt_time, 
                            p_gnss->pvt_holder, p_gnss->diff_holder, p_gnss->float_holder); // 
                }
            }
            else
            {
                for (size_t i = 0; i < gt_holder.size(); i++)
                {
                    inputpvt_lla(gt_holder[i][0], gt_holder[i][1], gt_holder[i][2], gt_holder[i][3], p_gnss->first_lla_pvt, p_gnss->first_xyz_ecef_pvt, p_gnss->pvt_time, 
                            p_gnss->pvt_holder, p_gnss->diff_holder, p_gnss->float_holder); // 
                }
            }
        }
        sub_gnss_iono_params = nh.subscribe(gnss_iono_params_topic, 10000, gnss_iono_params_callback);

        if (gnss_local_online_sync)
        {
            sub_gnss_time_pluse_info = nh.subscribe(gnss_tp_info_topic, 100, 
                gnss_tp_info_callback);
            sub_local_trigger_info = nh.subscribe(local_trigger_info_topic, 100, 
                local_trigger_info_callback);
        }
        else
        {
            time_diff_gnss_local = gnss_local_time_diff; // 18.0
            p_gnss->inputGNSSTimeDiff(time_diff_gnss_local);
            time_diff_valid = true;
        }
    }
    else
    {
        if (!NMEA_ENABLE)
        {
            sub_rtk_pvt_info = nh.subscribe(rtk_pvt_topic, 100, rtk_pvt_callback);
        }
    }

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 1000);
    ros::Publisher pubLaserCloudFullRes_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 1000);
    ros::Publisher pubLaserCloudEffect  = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 1000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 1000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/aft_mapped_to_init", 1000);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", 1000);
    ros::Publisher plane_pub = nh.advertise<visualization_msgs::Marker>
            ("/planner_normal", 1000);
    // ros::Publisher pub_gnss_lla = nh.advertise<sensor_msgs::NavSatFix>("gnss_fused_lla", 1000);
    pubGlobalMap = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1);
    ros::Timer timer = nh.createTimer(ros::Duration(2.0), publish_global_map);
    ros::Subscriber sub_initpose = nh.subscribe("/initialpose", 1, initialPoseCallback);
    
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate loop_rate(500);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();
        if(sync_packages(Measures, p_gnss->gnss_msg, p_nmea->nmea_msg)) 
        {
#if 1
            if (!system_state_vaild)
            {
#if 0
                if (!p_gnss->gnss_msg.empty() && GNSS_ENABLE)
                {
                    gnss_cur = p_gnss->gnss_msg.front();
                    while (time2sec(gnss_cur[0]->time) - time_diff_gnss_local < Measures.lidar_beg_time)
                    {
                        p_gnss->gnss_msg.pop();
                        if (!p_gnss->gnss_msg.empty())
                            gnss_cur = p_gnss->gnss_msg.front();
                        else
                            break;
                    }

                    p_gnss->processGNSS(gnss_cur, kf_output.x_);
                    // p_gnss->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                    // update_gnss = p_gnss->Evaluate(kf_output.x_);
                    if (!p_gnss->gnss_ready)
                        continue;
                    else
                        system_state_vaild = true;
                }
                else
#endif
                // if (!run_relocalization(Measures.lidar, 0))
                //     continue;
                // else
                {
                    kf_output.x_.pos = V3D(-0.0199728, -0.0152043, 0.0133042);
                    kf_output.x_.rot << 0.998994, 0.000894, 0.044826, -0.001156, 0.999982, 0.005821, -0.044820, -0.005867, 0.998978;
                    p_gnss->anc_ecef = Eigen::Vector3d(-2169507.8742181729, 4385248.6699813260, 4078236.5512984213);
                    p_gnss->R_ecef_enu << -0.896309, 0.285046, -0.339674, -0.44343, -0.576167, 0.686587, 0, 0.766016, 0.642822;
                    p_gnss->SetLidarInit(kf_output.x_, p_gnss->anc_ecef, p_gnss->R_ecef_enu, Measures.lidar_last_time);
                    p_gnss->gnss_ready = true;
                    ROS_INFO("GNSS Initialization is done, by lidar!");
                    system_state_vaild = true;
                }
            }
#endif
            Timer timer;

            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
                if (first_imu_time < 1)
                {
                    first_imu_time = imu_next.header.stamp.toSec();
                    // printf("first imu time: %f acceleration: %f%f%f\n", first_imu_time, imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z);
                }
                time_current = 0.0;
                if(imu_en)
                {
                    kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
                    // kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
                    // kf_output.x_.acc *= -1; 

                    if (!nolidar && !imu_deque.empty())
                    {
                        while (Measures.lidar_beg_time > imu_next.header.stamp.toSec()) // if it is needed for the new map?
                        {
                            imu_deque.pop_front();
                            if (imu_deque.empty())
                            {
                                break;
                            }
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            // imu_deque.pop();
                        }
                    }
                }
                else
                {
                    kf_output.x_.gravity << VEC_FROM_ARRAY(gravity); //_init);
                    kf_output.x_.acc << VEC_FROM_ARRAY(gravity); //_init);
                    kf_output.x_.acc *= -1; 
                    p_imu->imu_need_init_ = false;
                    // p_imu->after_imu_init_ = true;
                }  
                G_m_s2 = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
                // if (GNSS_ENABLE)
                // {   
                //     // p_gnss->gnss_ready = true;
                //     // p_gnss->gtSAMgraphMade = true;
                //     set_gnss_offline_init(false);
                // }         
            }

            double t0, t5;
            t0 = omp_get_wtime();
            
            /*** downsample the feature points in a scan ***/
            p_imu->Process(Measures, feats_undistort);
            if(space_down_sample)
            {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list); 
            }
            else
            {
                feats_down_body = Measures.lidar;
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list); 
            }
            if (!nolidar)
            {
                time_seq = time_compressing<int>(feats_down_body);
                feats_down_size = feats_down_body->points.size();
            }
            else
            {
                time_seq.clear();
            }
         
#if 0
            if (!p_imu->after_imu_init_)
            {
                if (!p_imu->imu_need_init_)
                { 
                    V3D tmp_gravity;
                    if (init_with_imu && imu_en)
                    {
                        tmp_gravity = - p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
                    }
                    else
                    {   tmp_gravity << VEC_FROM_ARRAY(gravity_init);
                        p_imu->after_imu_init_ = true;
                    }
                    M3D rot_init;
                    p_imu->Set_init(tmp_gravity, rot_init);
                    // p_gnss->Rot_gnss_init = rot_init;  
                    kf_output.x_.rot = rot_init;
                    // kf_output.x_.rot; //.normalize();
                    kf_output.x_.acc = - rot_init.transpose() * kf_output.x_.gravity;
                }
                else{
                continue;}
            }
#endif

            /*** ICP and Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            Nearest_Points.resize(feats_down_size);
            // t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            crossmat_list.reserve(feats_down_size);
            pbody_list.reserve(feats_down_size);
            pimu_list.reserve(feats_down_size);
            // pbody_ext_list.reserve(feats_down_size);
                          
            for (size_t i = 0; i < feats_down_body->size(); i++)
            {
                V3D point_this(feats_down_body->points[i].x,
                            feats_down_body->points[i].y,
                            feats_down_body->points[i].z);
                pbody_list[i]=point_this;
                {
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                    pimu_list[i] = point_this;
                }
                M3D point_crossmat;
                point_crossmat << SKEW_SYM_MATRX(point_this);
                crossmat_list[i]=point_crossmat;
            }
            {     
                effct_feat_num = 0;
                /**** point by point update ****/
                // lidar,imu,gnss三种约束
                if (time_seq.size() > 0) // || (!GNSS_ENABLE && !NMEA_ENABLE) )
                {
                    if (GNSS_ENABLE)  
                    {p_gnss->p_assign->process_feat_num += time_seq.size();
                    p_gnss->nolidar_cur = false;}
                    if (NMEA_ENABLE)  
                    {p_nmea->p_assign->process_feat_num += time_seq.size();
                    p_nmea->nolidar_cur = false;}
                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++)
                {
                    PointType &point_body  = feats_down_body->points[idx+time_seq[k]];

                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                    if (time_current < time_predict_last_const)
                    {
                        continue;
                    }

                    if (is_first_frame)
                    {
                        if(imu_en && !imu_deque.empty())
                        {
                            while (time_current > imu_next.header.stamp.toSec())
                            {
                                imu_deque.pop_front();
                                if (imu_deque.empty()) break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                            angvel_avr<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            acc_avr   <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                            // if (imu_deque.empty()) break;
                        }
                        if (GNSS_ENABLE)
                        {
                            // std::vector<Eigen::Vector3d>().swap(p_gnss->norm_vec_holder);
                            p_gnss->p_assign->process_feat_num = 0;
                            p_gnss->norm_vec_num = 0;
                            // acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                            // p_gnss->pre_integration->repropagate(kf_output.x_.ba, kf_output.x_.bg);
                            // p_gnss->pre_integration->setacc0gyr0(acc_avr_norm, angvel_avr);
                        }
                        if (NMEA_ENABLE)
                        {
                            p_nmea->p_assign->process_feat_num = 0;
                            p_nmea->norm_vec_num = 0;
                        }
                        is_first_frame = false;
                        time_update_last = time_current;
                        time_predict_last_const = time_current;
                    }
                    if(imu_en && !imu_deque.empty())
                    {
                        bool last_imu = imu_next.header.stamp.toSec() == imu_deque.front()->header.stamp.toSec();
                        while (imu_next.header.stamp.toSec() < time_predict_last_const && !imu_deque.empty())
                        {
                            if (!last_imu)
                            {
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                                break;
                            }
                            else
                            {
                                imu_deque.pop_front();
                                if (imu_deque.empty()) break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                            if (imu_deque.empty()) break;
                        }
                        bool imu_comes = time_current >= imu_next.header.stamp.toSec();
                        while (imu_comes) 
                        {
                            if (!p_gnss->gnss_msg.empty() && GNSS_ENABLE)
                            {   
                                gnss_cur = p_gnss->gnss_msg.front();
                                // printf("%f, %f, %f\n", time2sec(gnss_cur[0]->time), time_diff_gnss_local, time_predict_last_const);
                                while (time2sec(gnss_cur[0]->time) - time_diff_gnss_local < time_predict_last_const)
                                {
                                    p_gnss->gnss_msg.pop();
                                    if(!p_gnss->gnss_msg.empty())
                                    {
                                        gnss_cur = p_gnss->gnss_msg.front();
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }
                                if (p_gnss->gnss_msg.empty()) break;
                                while ((imu_next.header.stamp.toSec() >= time2sec(gnss_cur[0]->time) - time_diff_gnss_local) && (time2sec(gnss_cur[0]->time) - time_diff_gnss_local >= time_predict_last_const))
                                {
                                    double dt = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_predict_last_const;
                                    double dt_cov = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_update_last;

                                    if (p_gnss->gnss_ready)
                                    {
                                        if (dt_cov > 0.0)
                                        {
                                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        }
                                        kf_output.predict(dt, Q_output, input_in, true, false);
                                        // p_gnss->pre_integration->push_back(dt, kf_output.x_.acc + kf_output.x_.ba, kf_output.x_.omg + kf_output.x_.bg); // acc_avr, angvel_avr); 
                                        // p_gnss->processIMUOutput(dt, kf_output.x_.acc, kf_output.x_.omg);
                                        time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                        time_update_last = time_predict_last_const;
                                        p_gnss->processGNSS(gnss_cur, kf_output.x_);
                                        p_gnss->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                        // p_gnss->sqrt_lidar *= 0.002;
                                        update_gnss = p_gnss->Evaluate(kf_output.x_);
                                        if (!p_gnss->gnss_ready)
                                        {
                                            flg_reset = true;
                                            p_gnss->gnss_msg.pop();
                                            if(!p_gnss->gnss_msg.empty())
                                            {
                                                gnss_cur = p_gnss->gnss_msg.front();
                                            }
                                            break; // ?
                                        }

                                        if (update_gnss)
                                        {
                                            state_output out_state = kf_output.x_;
                                            kf_output.update_iterated_dyn_share_GNSS();
                                            Eigen::Vector3d pos_enu;
                                            if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                            // sensor_msgs::NavSatFix gnss_lla_msg;
                                            // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                            // gnss_lla_msg.header.frame_id = "camera_init";
                                            // gnss_lla_msg.latitude = pos_enu(0);
                                            // gnss_lla_msg.longitude = pos_enu(1);
                                            // gnss_lla_msg.altitude = pos_enu(2);
                                            // pub_gnss_lla.publish(gnss_lla_msg);
                                            if ((out_state.pos - kf_output.x_.pos).norm() > 0.1 && pose_graph_key_pose.size() > 4)
                                            {                                                
                                                curvefitter::PoseData pose_data;
                                                pose_data.timestamp = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                                map_time = pose_data.timestamp;
                                                pose_data.orientation = Sophus::SO3d(Eigen::Quaterniond(kf_output.x_.rot).normalized().toRotationMatrix());
                                                pose_data.position = kf_output.x_.pos;
                                                if (map_time > pose_graph_key_pose.back().timestamp) // + 1e-9)
                                                {
                                                    pose_time_vector.push_back(pose_data.timestamp);
                                                    pose_graph_key_pose.emplace_back(pose_data);
                                                }
                                                else
                                                // else if (map_time == pose_time_vector.back())
                                                {
                                                    pose_data.timestamp = pose_graph_key_pose.back().timestamp;
                                                    pose_graph_key_pose.back() = pose_data;
                                                }
                                                // curvefitter::Trajectory<4> traj(0.1);
                                                // std::shared_ptr<curvefitter::Trajectory<4> > Traj_ptr = std::make_shared<curvefitter::Trajectory<4> >(traj);  
                                                traj_manager->SetTrajectory(std::make_shared<curvefitter::Trajectory<4> >(0.025));
                                                traj_manager->FitCurve(pose_graph_key_pose[0].orientation.unit_quaternion(), pose_graph_key_pose[0].position, pose_time_vector[0], pose_time_vector.back(), pose_graph_key_pose);
                                                updatedmap.resize(points_num);
                                                updatedmap = traj_manager->GetUpdatedMapPoints(pose_time_vector, LiDAR_points);
                                                ivox_last_->AddPoints(updatedmap);
                                                ivox_->grids_map_ = ivox_last_->grids_map_;
                                                // for (auto &t : ivox_last_->grids_map_)
                                                // {
                                                    // ivox_->grids_map_[t.first] = (t.second);
                                                // }
                                                // ivox_ = std::make_shared<IVoxType>(*ivox_last_);
                                            }
                                            else
                                            {
                                                ivox_last_->grids_map_ = ivox_->grids_map_;
                                            }
                                            // reset_cov_output(kf_output.P_);
                                            traj_manager->ResetTrajectory(pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                                        }
                                    }
                                    else
                                    {
                                        if (dt_cov > 0.0)
                                        {
                                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        }
                                        
                                        kf_output.predict(dt, Q_output, input_in, true, false);

                                        time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                        time_update_last = time_predict_last_const;
                                        state_out = kf_output.x_;
                                        // state_out.rot = state_out.rot; //.normalized().toRotationMatrix();
                                        // state_out.rot.normalize();
                                        // state_out.pos = state_out.pos;
                                        // state_out.vel = state_out.vel;
                                        p_gnss->processGNSS(gnss_cur, state_out);
                                        if (p_gnss->gnss_ready)
                                        {
                                            // printf("time gnss ready: %f \n", time_predict_last_const);
                                            Eigen::Vector3d pos_enu;
                                            if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                            // sensor_msgs::NavSatFix gnss_lla_msg;
                                            // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                            // gnss_lla_msg.header.frame_id = "camera_init";
                                            // gnss_lla_msg.latitude = pos_enu(0);
                                            // gnss_lla_msg.longitude = pos_enu(1);
                                            // gnss_lla_msg.altitude = pos_enu(2);
                                            // pub_gnss_lla.publish(gnss_lla_msg);
                                        }
                                    }
                                    p_gnss->gnss_msg.pop();
                                    if(!p_gnss->gnss_msg.empty())
                                    {
                                        gnss_cur = p_gnss->gnss_msg.front();
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }
                            }
                            if (!p_nmea->nmea_msg.empty() && NMEA_ENABLE)
                            {   
                                nmea_cur = p_nmea->nmea_msg.front();
                                while (nmea_cur->header.stamp.toSec() - time_diff_nmea_local < time_predict_last_const)
                                {
                                    p_nmea->nmea_msg.pop();
                                    if(!p_nmea->nmea_msg.empty())
                                    {
                                        nmea_cur = p_nmea->nmea_msg.front();
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }
                                if (p_nmea->nmea_msg.empty()) break;
                                while ((imu_next.header.stamp.toSec() >= nmea_cur->header.stamp.toSec() - time_diff_nmea_local) && (nmea_cur->header.stamp.toSec() - time_diff_nmea_local >= time_predict_last_const))
                                {
                                    double dt = nmea_cur->header.stamp.toSec() - time_diff_nmea_local - time_predict_last_const;
                                    double dt_cov = nmea_cur->header.stamp.toSec() - time_diff_nmea_local - time_update_last;

                                    if (p_nmea->nmea_ready)
                                    {
                                        if (dt_cov > 0.0)
                                        {
                                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        }
                                        kf_output.predict(dt, Q_output, input_in, true, false);
                                        time_predict_last_const = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                        time_update_last = time_predict_last_const;
                                        p_nmea->processNMEA(nmea_cur, kf_output.x_);
                                        p_nmea->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                        // p_gnss->sqrt_lidar *= 0.002;
                                        update_nmea = p_nmea->Evaluate(kf_output.x_);
                                        if (!p_nmea->nmea_ready)
                                        {
                                            flg_reset = true;
                                            p_nmea->nmea_msg.pop();
                                            if(!p_nmea->nmea_msg.empty())
                                            {
                                                nmea_cur = p_nmea->nmea_msg.front();
                                            }
                                            break; // ?
                                        }

                                        if (update_nmea)
                                        {
                                            kf_output.update_iterated_dyn_share_NMEA();
                                            if (!runtime_pos_log) cout_state_to_file_nmea();
                                        }
                                    }
                                    else
                                    {
                                        if (dt_cov > 0.0)
                                        {
                                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        }
                                        
                                        kf_output.predict(dt, Q_output, input_in, true, false);

                                        time_predict_last_const = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                        time_update_last = time_predict_last_const;
                                        state_out = kf_output.x_;
                                        // state_out.rot = state_out.rot; //.normalized().toRotationMatrix();
                                        // state_out.pos = state_out.pos;
                                        // state_out.vel = state_out.vel;
                                        p_nmea->processNMEA(nmea_cur, state_out);
                                    }
                                    p_nmea->nmea_msg.pop();
                                    if(!p_nmea->nmea_msg.empty())
                                    {
                                        nmea_cur = p_nmea->nmea_msg.front();
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }
                            }
                            if (flg_reset)
                            {
                                break;
                            }
                            angvel_avr<<imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                            acc_avr   <<imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

                            /*** covariance update ***/
                            double dt = imu_next.header.stamp.toSec() - time_predict_last_const;
                            time_predict_last_const = imu_next.header.stamp.toSec(); 
                            double dt_cov = imu_next.header.stamp.toSec() - time_update_last; 

                            if (dt_cov > 0.0)
                            {
                                time_update_last = imu_next.header.stamp.toSec();

                                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            }
                            kf_output.predict(dt, Q_output, input_in, true, false);
                            kf_output.update_iterated_dyn_share_IMU();
                            imu_deque.pop_front();
                            if (imu_deque.empty()) break;
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_comes = time_current >= imu_next.header.stamp.toSec();
                        }
                    }
                    if (flg_reset)
                    {
                        break;
                    }
                    if (!p_gnss->gnss_msg.empty() && GNSS_ENABLE)
                    {
                        gnss_cur = p_gnss->gnss_msg.front();
                        // printf("%f, %f, %f\n", time2sec(gnss_cur[0]->time), time_diff_gnss_local, time_predict_last_const);
                        while ( time2sec(gnss_cur[0]->time) - time_diff_gnss_local < time_predict_last_const)
                        {
                            p_gnss->gnss_msg.pop();
                            if(!p_gnss->gnss_msg.empty())
                            {
                                gnss_cur = p_gnss->gnss_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                        if (p_gnss->gnss_msg.empty()) break;
                        while (time_current >= time2sec(gnss_cur[0]->time) - time_diff_gnss_local && time2sec(gnss_cur[0]->time) - time_diff_gnss_local >= time_predict_last_const)
                        {
                            double dt = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_predict_last_const;
                            double dt_cov = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_update_last;
                            // cout << "check gnss ready:" << p_gnss->gnss_ready << endl;
                            if (p_gnss->gnss_ready)
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);

                                // p_gnss->pre_integration->push_back(dt, kf_output.x_.acc + kf_output.x_.ba, kf_output.x_.omg + kf_output.x_.bg); // acc_avr, angvel_avr); 
                                // p_gnss->processIMUOutput(dt, kf_output.x_.acc, kf_output.x_.omg);

                                time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                time_update_last = time_predict_last_const;
                                p_gnss->processGNSS(gnss_cur, kf_output.x_);
                                p_gnss->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                // p_gnss->sqrt_lidar *= 0.002;
                                update_gnss = p_gnss->Evaluate(kf_output.x_);
                                if (!p_gnss->gnss_ready)
                                {
                                    flg_reset = true;
                                    p_gnss->gnss_msg.pop();
                                    if(!p_gnss->gnss_msg.empty())
                                    {
                                        gnss_cur = p_gnss->gnss_msg.front();
                                    }
                                    break; // ?
                                }

                                if (update_gnss)
                                {
                                    state_output out_state = kf_output.x_;
                                    kf_output.update_iterated_dyn_share_GNSS();
                                    // reset_cov_output(kf_output.P_);
                                    Eigen::Vector3d pos_enu;
                                    if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                    // sensor_msgs::NavSatFix gnss_lla_msg;
                                    // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                    // gnss_lla_msg.header.frame_id = "camera_init";
                                    // gnss_lla_msg.latitude = pos_enu(0);
                                    // gnss_lla_msg.longitude = pos_enu(1);
                                    // gnss_lla_msg.altitude = pos_enu(2);
                                    // pub_gnss_lla.publish(gnss_lla_msg);
                                    if ((out_state.pos - kf_output.x_.pos).norm() > 0.1 && pose_graph_key_pose.size() > 4)
                                    {                                         
                                        curvefitter::PoseData pose_data;
                                        pose_data.timestamp = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                        map_time = pose_data.timestamp;
                                        // pose_time_vector.push_back(pose_data.timestamp);
                                        pose_data.orientation = Sophus::SO3d(Eigen::Quaterniond(kf_output.x_.rot).normalized().toRotationMatrix());
                                        pose_data.position = kf_output.x_.pos;
                                        if (map_time > pose_graph_key_pose.back().timestamp) // + 1e-9)
                                        {
                                            pose_time_vector.push_back(pose_data.timestamp);
                                            pose_graph_key_pose.emplace_back(pose_data);
                                        }
                                        else
                                        // else if (map_time == pose_time_vector.back())
                                        {
                                            pose_data.timestamp = pose_graph_key_pose.back().timestamp;
                                            pose_graph_key_pose.back() = pose_data;
                                        }
                                        // pose_graph_key_pose.emplace_back(pose_data);
                                        traj_manager->SetTrajectory(std::make_shared<curvefitter::Trajectory<4> >(0.025));
                                        traj_manager->FitCurve(pose_graph_key_pose[0].orientation.unit_quaternion(), pose_graph_key_pose[0].position, pose_time_vector[0], pose_time_vector.back(), pose_graph_key_pose);
                                        updatedmap.resize(points_num);
                                        updatedmap = traj_manager->GetUpdatedMapPoints(pose_time_vector, LiDAR_points);
                                        ivox_last_->AddPoints(updatedmap);
                                        ivox_->grids_map_ = ivox_last_->grids_map_;
                                    }
                                    else
                                    {
                                        ivox_last_->grids_map_ = ivox_->grids_map_;
                                    }
                                    traj_manager->ResetTrajectory(pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                                }
                            }
                            else
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                time_update_last = time_predict_last_const;
                                state_out = kf_output.x_;
                                // state_out.rot = state_out.rot; //.normalized().toRotationMatrix();
                                // state_out.rot.normalize();
                                // state_out.pos = state_out.pos;
                                // state_out.vel = state_out.vel;
                                p_gnss->processGNSS(gnss_cur, state_out);
                                if (p_gnss->gnss_ready)
                                {
                                    // printf("time gnss ready: %f \n", time_predict_last_const);
                                    Eigen::Vector3d pos_enu;
                                    if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                    // sensor_msgs::NavSatFix gnss_lla_msg;
                                    // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                    // gnss_lla_msg.header.frame_id = "camera_init";
                                    // gnss_lla_msg.latitude = pos_enu(0);
                                    // gnss_lla_msg.longitude = pos_enu(1);
                                    // gnss_lla_msg.altitude = pos_enu(2);
                                    // pub_gnss_lla.publish(gnss_lla_msg);
                                }
                            }
                            p_gnss->gnss_msg.pop();
                            if(!p_gnss->gnss_msg.empty())
                            {
                                gnss_cur = p_gnss->gnss_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                    if (!p_nmea->nmea_msg.empty() && NMEA_ENABLE)
                    {
                        nmea_cur = p_nmea->nmea_msg.front();
                        while ( nmea_cur->header.stamp.toSec() - time_diff_nmea_local < time_predict_last_const)
                        {
                            p_nmea->nmea_msg.pop();
                            if(!p_nmea->nmea_msg.empty())
                            {
                                nmea_cur = p_nmea->nmea_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                        if (p_nmea->nmea_msg.empty()) break;
                        while (time_current >= nmea_cur->header.stamp.toSec() - time_diff_nmea_local && nmea_cur->header.stamp.toSec() - time_diff_nmea_local >= time_predict_last_const)
                        {
                            double dt = nmea_cur->header.stamp.toSec() - time_diff_nmea_local - time_predict_last_const;
                            double dt_cov = nmea_cur->header.stamp.toSec() - time_diff_nmea_local - time_update_last;
                            if (p_nmea->nmea_ready)
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);

                                time_predict_last_const = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                time_update_last = time_predict_last_const;
                                p_nmea->processNMEA(nmea_cur, kf_output.x_);
                                p_nmea->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                // p_gnss->sqrt_lidar *= 0.002;
                                update_nmea = p_nmea->Evaluate(kf_output.x_);
                                if (!p_nmea->nmea_ready)
                                {
                                    flg_reset = true;
                                    p_nmea->nmea_msg.pop();
                                    if(!p_nmea->nmea_msg.empty())
                                    {
                                        nmea_cur = p_nmea->nmea_msg.front();
                                    }
                                    break; // ?
                                }

                                if (update_nmea)
                                {
                                    kf_output.update_iterated_dyn_share_NMEA();
                                    if (!runtime_pos_log) cout_state_to_file_nmea();
                                }
                            }
                            else
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                time_update_last = time_predict_last_const;
                                state_out = kf_output.x_;
                                // state_out.rot = state_out.rot; //.normalized().toRotationMatrix();
                                // state_out.pos = state_out.pos;
                                // state_out.vel = state_out.vel;
                                p_nmea->processNMEA(nmea_cur, state_out);
                            }
                            p_nmea->nmea_msg.pop();
                            if(!p_nmea->nmea_msg.empty())
                            {
                                nmea_cur = p_nmea->nmea_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                    if (flg_reset)
                    {
                        break;
                    }
                    double dt = time_current - time_predict_last_const;
                    // double propag_state_start = omp_get_wtime();
                    if(!prop_at_freq_of_imu)
                    {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            time_update_last = time_current;   
                        }
                    }
                    // if (dt > 0.0)
                    {
                    kf_output.predict(dt, Q_output, input_in, true, false);
                    time_predict_last_const = time_current;
                    if (feats_down_size < 1)
                    {
                        ROS_WARN("No point, skip this scan!\n");
                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_output.update_iterated_dyn_share_modified()) 
                    {
                        idx = idx+time_seq[k];
                        continue;
                    }
                    }
                    // else
                    // {
                    //     idx = idx+time_seq[k];
                    //     continue;
                    // }
                    
                    // solve_start = omp_get_wtime();
                        
                    if (publish_odometry_without_downsample)
                    {
                        /******* Publish odometry *******/
                        publish_odometry(pubOdomAftMapped);
                    }
                    std::vector<Eigen::Vector3d> lidarpoints;
                    for (int j = 0; j < time_seq[k]; j++)
                    {
                        PointType &point_body_j  = feats_down_body->points[idx+j+1];
                        PointType &point_world_j = feats_down_world->points[idx+j+1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    if (GNSS_ENABLE || NMEA_ENABLE)
                        lidarpoints.push_back(pimu_list[idx+j+1]); // (Eigen::Vector3d(point_body_j.x, point_body_j.y, point_body_j.z));
                    }
                    if (GNSS_ENABLE || NMEA_ENABLE)
                    {
                        if (pose_graph_key_pose.empty()){
                            traj_manager->AddGraphPose(Eigen::Quaterniond(kf_output.x_.rot).normalized(), kf_output.x_.pos, lidarpoints, time_current, pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                        }else
                        {
                            if (time_current > pose_graph_key_pose.back().timestamp && lidarpoints.size() > 0)
                                traj_manager->AddGraphPose(Eigen::Quaterniond(kf_output.x_.rot).normalized(), kf_output.x_.pos, lidarpoints, time_current, pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                        }
                    }
                    idx += time_seq[k];
                }
                }
                // lidar退化,此时使用pre_integration积分得到需要的相对位姿变换
                else
                {
                    if (GNSS_ENABLE)  p_gnss->nolidar_cur = true;
                    if (NMEA_ENABLE)  p_nmea->nolidar_cur = true;
                    if (!imu_deque.empty())
                    { 
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());

                    while (imu_next.header.stamp.toSec() > time_current && ((imu_next.header.stamp.toSec() < imu_first_time + lidar_time_inte && nolidar) || (imu_next.header.stamp.toSec() < Measures.lidar_beg_time + lidar_time_inte && !nolidar)))
                    { // >= ?
                        if (is_first_frame)
                        {
                            if (!nolidar && GNSS_ENABLE) //std::vector<Eigen::Vector3d>().swap(p_gnss->norm_vec_holder);
                            {p_gnss->p_assign->process_feat_num = 0;
                            p_gnss->norm_vec_num = 0;}
                            if (!nolidar && NMEA_ENABLE) //std::vector<Eigen::Vector3d>().swap(p_gnss->norm_vec_holder);
                            {p_nmea->p_assign->process_feat_num = 0;
                            p_nmea->norm_vec_num = 0;}

                            if (!p_gnss->gnss_msg.empty() && GNSS_ENABLE)
                            {
                                gnss_cur = p_gnss->gnss_msg.front();
                                double front_gnss_ts = time2sec(gnss_cur[0]->time); // take time
                                time_current = front_gnss_ts - time_diff_gnss_local;
                                while (imu_next.header.stamp.toSec() < time_current) // 0.05
                                {
                                    ROS_WARN("throw IMU, only should happen at the beginning 2510");
                                    imu_deque.pop_front();
                                    if (imu_deque.empty()) break;
                                    imu_last = imu_next;
                                    imu_next = *(imu_deque.front()); // could be used to initialize
                                }
                                if (imu_deque.empty()) break;
                            }
                            else if (!p_nmea->nmea_msg.empty() && NMEA_ENABLE)
                            {
                                nmea_cur = p_nmea->nmea_msg.front();
                                double front_nmea_ts = nmea_cur->header.stamp.toSec(); // take time
                                time_current = front_nmea_ts - time_diff_nmea_local;
                                while (imu_next.header.stamp.toSec() < time_current) // 0.05
                                {
                                    ROS_WARN("throw IMU, only should happen at the beginning 2510");
                                    imu_deque.pop_front();
                                    if (imu_deque.empty()) break;
                                    imu_last = imu_next;
                                    imu_next = *(imu_deque.front()); // could be used to initialize
                                }
                                if (imu_deque.empty()) break;
                            }
                            else
                            {
                                if (nolidar)
                                {
                                    while (imu_next.header.stamp.toSec() < imu_first_time + lidar_time_inte)
                                    {
                                        // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                                        imu_deque.pop_front();
                                        if(imu_deque.empty()) break;
                                        imu_last = imu_next;
                                        imu_next = *(imu_deque.front()); // could be used to initialize
                                    }
                                    // if (imu_deque.empty()) break;
                                }
                                else
                                {
                                    while (imu_next.header.stamp.toSec() < Measures.lidar_beg_time + lidar_time_inte)
                                    {
                                        // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                                        imu_deque.pop_front();
                                        if(imu_deque.empty()) break;
                                        imu_last = imu_next;
                                        imu_next = *(imu_deque.front());
                                    }
                                }
                                break;
                            }
                            angvel_avr<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            if (nolidar) kf_output.x_.omg = angvel_avr;
                                            
                            acc_avr   <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                            time_current = imu_next.header.stamp.toSec();

                            time_update_last = time_current;
                            time_predict_last_const = time_current;
                            acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                            if (GNSS_ENABLE)
                            {
                            p_gnss->pre_integration->repropagate(kf_output.x_.ba, kf_output.x_.bg);
                            p_gnss->pre_integration->setacc0gyr0(acc_avr_norm, angvel_avr); 
                            }
                            if (NMEA_ENABLE)
                            {
                            p_nmea->pre_integration->repropagate(kf_output.x_.ba, kf_output.x_.bg);
                            p_nmea->pre_integration->setacc0gyr0(acc_avr_norm, angvel_avr); 
                            }

                            {
                                is_first_frame = false;
                            }
                        }
                        time_current = imu_next.header.stamp.toSec();

                        if (!is_first_frame)
                        {
                        if (!p_gnss->gnss_msg.empty() && GNSS_ENABLE)
                        {
                            gnss_cur = p_gnss->gnss_msg.front();
                            while (time2sec(gnss_cur[0]->time) - time_diff_gnss_local <= time_predict_last_const)
                            {
                                p_gnss->gnss_msg.pop();
                                if(!p_gnss->gnss_msg.empty())
                                {
                                    gnss_cur = p_gnss->gnss_msg.front();
                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (p_gnss->gnss_msg.empty()) break;
                        while ((time_current > time2sec(gnss_cur[0]->time) - time_diff_gnss_local) && (time2sec(gnss_cur[0]->time) - time_diff_gnss_local > time_predict_last_const))
                        {
                            double dt = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_predict_last_const;
                            double dt_cov = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_update_last;

                            if (p_gnss->gnss_ready)
                            {
                                if (dt_cov > 0.0)
                                {
                                    // kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                    time_update_last = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                }
                                // kf_output.predict(dt, Q_output, input_in, true, false);
                                p_gnss->pre_integration->push_back(dt, acc_avr_norm, angvel_avr); //acc_avr_norm, angvel_avr); 
                                // change to state_const.omg and state_const.acc? 
                                time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                p_gnss->processGNSS(gnss_cur, kf_output.x_);
                                if (!nolidar)
                                {
                                    p_gnss->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                }
                                update_gnss = p_gnss->Evaluate(kf_output.x_); 
                                if (!p_gnss->gnss_ready)
                                {
                                    flg_reset = true;
                                    p_gnss->gnss_msg.pop();
                                    if(!p_gnss->gnss_msg.empty())
                                    {
                                        gnss_cur = p_gnss->gnss_msg.front();
                                    }
                                    break; // ?
                                }
                                if (update_gnss)
                                {
                                    if (!nolidar)
                                    {
                                        state_output out_state = kf_output.x_;
                                        kf_output.update_iterated_dyn_share_GNSS();
                                        // reset_cov_output(kf_output.P_);
                                        if ((out_state.pos - kf_output.x_.pos).norm() > 0.1 && pose_graph_key_pose.size() > 4)
                                        {                                    
                                            curvefitter::PoseData pose_data;
                                            pose_data.timestamp = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                            map_time = pose_data.timestamp;
                                            // pose_time_vector.push_back(pose_data.timestamp);
                                            pose_data.orientation = Sophus::SO3d(Eigen::Quaterniond(kf_output.x_.rot).normalized().toRotationMatrix());
                                            pose_data.position = kf_output.x_.pos;
                                            if (map_time > pose_graph_key_pose.back().timestamp) // + 1e-9)
                                            {
                                                pose_time_vector.push_back(pose_data.timestamp);
                                                pose_graph_key_pose.emplace_back(pose_data);
                                            }
                                            // else if (map_time == pose_time_vector.back())
                                            else
                                            {
                                                pose_data.timestamp = pose_graph_key_pose.back().timestamp;
                                                pose_graph_key_pose.back() = pose_data;
                                            }
                                            // pose_graph_key_pose.emplace_back(pose_data);
                                            // curvefitter::Trajectory<4> traj(0.1);
                                            // std::shared_ptr<curvefitter::Trajectory<4> > Traj_ptr = std::make_shared<curvefitter::Trajectory<4> >(traj);  
                                            traj_manager->SetTrajectory(std::make_shared<curvefitter::Trajectory<4> >(0.025));
                                            traj_manager->FitCurve(pose_graph_key_pose[0].orientation.unit_quaternion(), pose_graph_key_pose[0].position, pose_time_vector[0], pose_time_vector.back(), pose_graph_key_pose);
                                            updatedmap.resize(points_num);
                                            updatedmap = traj_manager->GetUpdatedMapPoints(pose_time_vector, LiDAR_points);
                                            ivox_last_->AddPoints(updatedmap);
                                            ivox_->grids_map_ = ivox_last_->grids_map_;
                                            // for (auto &t : ivox_last_->grids_map_)
                                            // {
                                                // (ivox_->grids_map_[t.first]) = (t.second);
                                            // }
                                            // ivox_ = std::make_shared<IVoxType>(*ivox_last_);
                                        }
                                        else
                                        {
                                            ivox_last_->grids_map_ = ivox_->grids_map_;
                                        }
                                        traj_manager->ResetTrajectory(pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                                    }
                                    Eigen::Vector3d pos_enu;
                                    if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                    // sensor_msgs::NavSatFix gnss_lla_msg;
                                    // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                    // gnss_lla_msg.header.frame_id = "camera_init";
                                    // gnss_lla_msg.latitude = pos_enu(0);
                                    // gnss_lla_msg.longitude = pos_enu(1);
                                    // gnss_lla_msg.altitude = pos_enu(2);
                                    // pub_gnss_lla.publish(gnss_lla_msg);
                                }
                            }
                            else
                            {
                                if (dt_cov > 0.0)
                                {
                                    // kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                    time_update_last = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                }
                                // kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                p_gnss->processGNSS(gnss_cur, kf_output.x_);
                                if (p_gnss->gnss_ready)
                                {
                                    Eigen::Vector3d pos_enu;
                                    if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                    // printf("time gnss ready: %f \n", time_predict_last_const);
                                    // sensor_msgs::NavSatFix gnss_lla_msg;
                                    // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                    // gnss_lla_msg.header.frame_id = "camera_init";
                                    // gnss_lla_msg.latitude = pos_enu(0);
                                    // gnss_lla_msg.longitude = pos_enu(1);
                                    // gnss_lla_msg.altitude = pos_enu(2);
                                    // pub_gnss_lla.publish(gnss_lla_msg);
                                    if (nolidar)
                                    {
                                        // Eigen::Matrix3d R_enu_local_;
                                        // R_enu_local_ = Eigen::AngleAxisd(p_gnss->yaw_enu_local, Eigen::Vector3d::UnitZ());
                                        kf_output.x_.pos = p_gnss->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_gnss->frame_num-1)).segment<3>(0); // p_gnss->anc_ecef - p_gnss->R_ecef_enu * R_enu_local_ * state_const.rot_end * p_gnss->Tex_imu_r;
                                        kf_output.x_.rot = p_gnss->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(p_gnss->frame_num-1)).matrix(); // p_gnss->R_ecef_enu * R_enu_local_ * state_const.rot_end;
                                        // kf_output.x_.rot.normalize();
                                        kf_output.x_.vel = p_gnss->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_gnss->frame_num-1)).segment<3>(3); // p_gnss->R_ecef_enu * R_enu_local_ * state_const.vel_end; // Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.ba = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.bg = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.omg = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.gravity = p_gnss->R_ecef_enu * kf_output.x_.gravity; // * R_enu_local_ 
                                        kf_output.x_.acc = kf_output.x_.rot.transpose() * (-kf_output.x_.gravity); // R_ecef_enu * state.vel_end;.conjugate().normalized()
                                        
                                        kf_output.P_ = MD(24,24)::Identity() * INIT_COV;
                                    }
                                }
                            }
                            p_gnss->gnss_msg.pop();
                            if(!p_gnss->gnss_msg.empty())
                            {
                                gnss_cur = p_gnss->gnss_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                        }
                        if (!p_nmea->nmea_msg.empty() && NMEA_ENABLE)
                        {
                            nmea_cur = p_nmea->nmea_msg.front();
                            while ( nmea_cur->header.stamp.toSec() - time_diff_nmea_local < time_predict_last_const)
                            {
                                p_nmea->nmea_msg.pop();
                                if(!p_nmea->nmea_msg.empty())
                                {
                                    nmea_cur = p_nmea->nmea_msg.front();
                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (p_nmea->nmea_msg.empty()) break;
                        while ((time_current > nmea_cur->header.stamp.toSec() - time_diff_nmea_local) && (nmea_cur->header.stamp.toSec() - time_diff_nmea_local >= time_predict_last_const))
                        {
                            double dt = nmea_cur->header.stamp.toSec() - time_diff_nmea_local - time_predict_last_const;
                            double dt_cov = nmea_cur->header.stamp.toSec() - time_diff_nmea_local - time_update_last;

                            if (p_nmea->nmea_ready)
                            {
                                if (dt_cov > 0.0)
                                {
                                    // kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                    time_update_last = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                }
                                // kf_output.predict(dt, Q_output, input_in, true, false);
                                p_nmea->pre_integration->push_back(dt, acc_avr_norm, angvel_avr); //acc_avr_norm, angvel_avr); 
                                time_predict_last_const = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                p_nmea->processNMEA(nmea_cur, kf_output.x_);
                                if (!nolidar)
                                {
                                    p_nmea->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                }
                                update_nmea = p_nmea->Evaluate(kf_output.x_); 
                                if (!p_nmea->nmea_ready)
                                {
                                    flg_reset = true;
                                    p_nmea->nmea_msg.pop();
                                    if(!p_nmea->nmea_msg.empty())
                                    {
                                        nmea_cur = p_nmea->nmea_msg.front();
                                    }
                                    break; // ?
                                }
                                if (update_nmea)
                                {
                                    if (!nolidar)
                                    {
                                        kf_output.update_iterated_dyn_share_NMEA();
                                        // reset_cov_output(kf_output.P_);
                                    }
                                    if (!runtime_pos_log) cout_state_to_file_nmea();
                                }
                            }
                            else
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                    time_update_last = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = nmea_cur->header.stamp.toSec() - time_diff_nmea_local;
                                p_nmea->processNMEA(nmea_cur, kf_output.x_);
                                if (p_nmea->nmea_ready)
                                {
                                    if (nolidar)
                                    {
                                        Eigen::Matrix3d R_enu_local;
                                        R_enu_local = Eigen::AngleAxisd(p_nmea->yaw_enu_local, Eigen::Vector3d::UnitZ()); 
                                        kf_output.x_.pos = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_nmea->frame_num-1)).segment<3>(0); // p_gnss->anc_ecef - p_gnss->R_ecef_enu * R_enu_local_ * state_const.rot_end * p_gnss->Tex_imu_r;
                                        kf_output.x_.rot = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(p_nmea->frame_num-1)).matrix(); // p_gnss->R_ecef_enu * R_enu_local_ * state_const.rot_end;
                                        kf_output.x_.vel = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_nmea->frame_num-1)).segment<3>(3); // p_gnss->R_ecef_enu * R_enu_local_ * state_const.vel_end; // Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.ba = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.bg = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.omg = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.gravity = R_enu_local * kf_output.x_.gravity; // * R_enu_local_ 
                                        kf_output.x_.acc = kf_output.x_.rot.transpose() * (-kf_output.x_.gravity); // R_ecef_enu * state.vel_end;.conjugate().normalized()
                                        
                                        kf_output.P_ = MD(24,24)::Identity() * INIT_COV;
                                    }
                                }
                            }
                            p_nmea->nmea_msg.pop();
                            if(!p_nmea->nmea_msg.empty())
                            {
                                nmea_cur = p_nmea->nmea_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                        }
                        if (flg_reset)
                        {
                            break;
                        }
                        double dt = time_current - time_predict_last_const;
                        {
                            double dt_cov = time_current - time_update_last;
                            if (dt_cov > 0.0)
                            {
                                // kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                time_update_last = time_current;
                            }
                            // kf_output.predict(dt, Q_output, input_in, true, false);
                            if (GNSS_ENABLE)   p_gnss->pre_integration->push_back(dt, acc_avr_norm, angvel_avr); // acc_avr_norm, angvel_avr); // 
                            if (NMEA_ENABLE)   p_nmea->pre_integration->push_back(dt, acc_avr_norm, angvel_avr); // acc_avr_norm, angvel_avr); // 
                        }

                        time_predict_last_const = time_current;

                        angvel_avr<<imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                        if (nolidar) kf_output.x_.omg = angvel_avr;
                        acc_avr   <<imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z; 
                        acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                        kf_output.update_iterated_dyn_share_IMU();
                        imu_deque.pop_front();
                        if (imu_deque.empty()) break;
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                    }
                    else
                    {
                        imu_deque.pop_front();
                        if (imu_deque.empty()) break;
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                    }
                    }
                    }
                }
            }
            
            /******* Publish odometry downsample *******/
            if (!publish_odometry_without_downsample)
            {
                publish_odometry(pubOdomAftMapped);
            }

#if 1
            LOG_INFO("location valid. feats_down = %lu, cost time = %.1fms.", feats_down_world->size(), timer.elapsedLast());
            std::cout << std::fixed << std::setprecision(10);
            std::cout << "pos = " << kf_output.x_.pos << std::endl;
            printf("rot = %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                   kf_output.x_.rot(0, 0), kf_output.x_.rot(0, 1), kf_output.x_.rot(0, 2),
                   kf_output.x_.rot(1, 0), kf_output.x_.rot(1, 1), kf_output.x_.rot(1, 2),
                   kf_output.x_.rot(2, 0), kf_output.x_.rot(2, 1), kf_output.x_.rot(2, 2));
            Eigen::Vector3d anc_cur;
            Eigen::Matrix3d R_enu_local_;
            anc_cur = p_gnss->p_assign->isamCurrentEstimate.at<gtsam::Vector3>(E(0));
            R_enu_local_ = p_gnss->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix();
            std::cout << "anc_cur = " << anc_cur.transpose() << std::endl;
            printf("R_enu_local = %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                   R_enu_local_(0, 0), R_enu_local_(0, 1), R_enu_local_(0, 2),
                   R_enu_local_(1, 0), R_enu_local_(1, 1), R_enu_local_(1, 2),
                   R_enu_local_(2, 0), R_enu_local_(2, 1), R_enu_local_(2, 2));
#endif
            t5 = omp_get_wtime();
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFullRes);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFullRes_body);
        }
        status = ros::ok();
        loop_rate.sleep();
    }
    
    return 0;
}
