bool check_vector(const std::vector<int>& vec, int &ob_count)
{
    if (vec.size() < 4) {
        return false;
    }

    int zero_count = 0;
    int index = vec.size() - 1;

    // Count consecutive zeros at the end of the vector
    while (index >= 0 && vec[index] == 0)
    {
        ++zero_count;
        --index;

        if (zero_count >= 4)
        {
            return false;
        }
    }

    if (zero_count >= 1) {
        int count = 0;

        // Count consecutive ones before zeros
        while (index >= 0 && vec[index] == 1) {
            ++count;
            --index;
        }

        ob_count = count;
        if (count >= 3) {
            return true;
        }
    }

    return false;
}



void VoxelGridLayer::updateLaser(const std::vector<cv::Point2f> &laser_scan, const long long & time_stamp,
                                 const SensorPosition& laser_position, const Pose& robot_pose)
{
    std::vector<cv::Point2f> robot_points,world_points;
    std::vector<cv::Point3f> laser_points;
    //保证更新measure_map过程中不更新原点，防止数据错乱
    std::string sensor_name = SensorInfo::getSensorName(laser_position, SensorType::LASER);
    //1) prase data
    LOGD(TAG) << " robot_info_.height:"<< robot_info_.height_ ;
    if(sensor_dict.count(sensor_name) == 0)

    {
        LOGD(TAG) << sensor_name << "RETURN!" << LOGEND;
       // lidar_update_ = false;
        return;
    }
    else
    {
        LOGD(TAG) << sensor_name <<time_stamp<< LOGEND;
    }

    if(lidar_update_ == true)
    {
        LOGD(TAG) << "is updating, return!" << std::endl;
        return ;
    }
    lidar_update_=true;
    newest_timestamp_ = time_stamp;
    Pose robot_pose_new = robot_pose;
    int if_find = 0;
   // auto robot_pose_get_ts = getPoseBaseTs (time_stamp, robot_pose, sensor_type_for_pose::laser, if_find );
    //robot_pose_new = *robot_pose_get_ts;
    robot_pose_new = robot_pose;
    cv::Point2f robot_pos_world = cv::Point2f(robot_pose_new[0], robot_pose_new[1]);
    cv::Point robot_pose_map = transformFromWorldToMap(robot_pos_world, resolution_, origin_point_world_);
    current_bound_ = getBound(robot_pose_map);
    cv::Rect patch_roi = bound2Rect(current_bound_);
    points_robot_3d_.clear();
    points_world_3d_.clear();
    points_world_2d_.clear();
    points_robot_2d_.clear();
    std::vector<cv::Point3f> tmp_robot_points_3d, tmp_world_points_3d;
    std::vector<cv::Point2f> tmp_robot_points_2d, tmp_world_points_2d;

    if(1)
    {
        // 3) update measure_map
        float min_x = robot_pose_new[0] - 2.0;
        float max_x = robot_pose_new[0] + 2.0;
        float min_y = robot_pose_new[1] - 2.0;
        float max_y = robot_pose_new[1] + 2.0;
        tmp_world_points_2d = voxel_grid_manager_->getStaticPointInWorld(min_x, max_x, min_y, max_y, 0.95);

        #if __ANDROID__
        std::vector<unsigned char> world_points_2d(tmp_world_points_2d.size()*8);
        for(int i =0;i<tmp_world_points_2d.size();i++ )
        {
            float x_ = tmp_world_points_2d[i].x;
            float y_ = tmp_world_points_2d[i].y;

          //unsigned short x = 1234;
          // unsigned short y = 4567;
            //  x_=12.34;
            //  y_=45.78;
            memcpy(world_points_2d.data()+i*8, &x_, 4);
            memcpy(world_points_2d.data()+i*8+4, &y_, 4);
        }
        std::string code_data = cppcodec::base64_rfc4648::encode(world_points_2d);
        LOGD(TAG) << "voxel_world_points:"<< code_data;
        #endif
        {
            std::scoped_lock lk(measure_mutex_);
            cleanBoundInt(measure_map_, current_bound_, FREE_SPACE);
            for(int i = 0;i < tmp_world_points_2d.size();i++)
            {
                cv::Point pt_m = transformFromWorldToMap(tmp_world_points_2d[i], resolution_, origin_point_world_);
                if (checkInBound(pt_m, current_bound_))
                {
                    measure_map_.at<uchar>(pt_m) = LETHAL_OBSTACLE;
                }
            }
            //not need update free
        }

        points_robot_3d_.set(tmp_robot_points_3d);
        points_world_3d_.set(tmp_world_points_3d);
        points_world_2d_.set(tmp_world_points_2d);
        points_robot_2d_.set(tmp_robot_points_2d);

        auto ex = sensor_dict[sensor_name].extrinsic;
        auto T_rc = getTFMatrix(ex.tx, ex.ty, ex.tz, ex.roll, ex.pitch, ex.yaw, ROTATE_ORDER::RZ_RY_RX);
        auto T_wr = getTFMatrix(Eigen::Vector3f(0, 0, robot_pose_new[2]), Eigen::Vector3f(robot_pose_new[0], robot_pose_new[1], 0.0));
        auto range = sensor_dict[sensor_name].range_max;
        float x, y, d, angular;
        float max_range_S = range*range;
        float min_range_S = 0.02*0.02;
        Eigen::Vector3f point_camera, point_robot, point_world;
        cv::Point2f pt_r;
        for (const auto &p : laser_scan)
        {
            //  auto p = laser_scan[i];
            // trans lasser from laser coord to robot_coord
            x = p.x;
            y = p.y;
            d = x*x +y*y;
            if (d > max_range_S || d < min_range_S)
            {
                if(d < 0.001)
                    continue;
                d = range;
                angular = atan(y/(x+0.000001));
                x = d * cos(angular);
                y = d * sin(angular);
            }
            // if (p.angular > max_angular || p.angular < min_angular || std::isnan(p.angular))
            //   continue;

            point_camera.x() = x;
            point_camera.y() = y;
            point_camera.z() = 0;
            point_robot = T_rc * point_camera;
            laser_points.push_back(cv::Point3f(point_world.x(), point_world.y(), 0));
            pt_r.x = point_robot[0];
            pt_r.y = point_robot[1];
            robot_points.push_back(pt_r);
        }
        cv::Rect2f rect_clear(0.40,-0.38,0.55,0.76);
        int detect_num =0 ;
        world_points.clear();
        for(int i = 0;i< robot_points.size();i++)
        {
//           if(rect_clear.contains(robot_points[i]))
            if(inRectBound(robot_points[i],rect_clear))
           {
               Eigen::Vector3f robot_points_3d(robot_points[i].x, robot_points[i].y, 0.);
               detect_num ++;
               point_world = T_wr * robot_points_3d;
               world_points.push_back(cv::Point2f(point_world[0],point_world[1]));
           }
        }

        if(lidar_near_points_consecutive_frames.size()>=10){
            lidar_near_points_consecutive_frames.erase(lidar_near_points_consecutive_frames.begin());
        }
        lidar_near_points_consecutive_frames.push_back(world_points);

        std::cout<<"resolution_  "<<resolution_<<std::endl;
        std::cout<<"origin_point_world_  "<<origin_point_world_.x<<"   "<<origin_point_world_.y<<std::endl;
        // 此处不是拿lidar_near_points_consecutive_frames 从头到尾的点
        float resolution_dynamic = 0.1;
        cv::Point2f origin_world_dynamic (0.0,0.0);

        if(if_points_exist_.size()>=10){
            if_points_exist_.erase(if_points_exist_.begin());
        }
        if(detect_num>10){
            if_points_exist_.push_back(1);
        }
        else
        {
            if_points_exist_.push_back(0);
        }
        int obstacle_count = 0 ;
        bool if_lidar_points = check_vector(if_points_exist_, obstacle_count);
        bool if_lidar_dynamic = false;

        if(if_lidar_points == true)
        {
            cv::Mat dynamic_measure_map = cv::Mat(cv::Size(800,800), CV_8UC1, cv::Scalar::all(0));

            //取出对应帧数的点云，投影到超低精度的local map，每个地图点按照投影频率进行次数统计
            for(int i = lidar_near_points_consecutive_frames.size()-2; i>lidar_near_points_consecutive_frames.size()-2-obstacle_count; i--)
            {
                cv::Mat dynamic_measure_map_count = cv::Mat(cv::Size(800,800), CV_8UC1, cv::Scalar::all(0));
                for(int j = 0 ;j<lidar_near_points_consecutive_frames[i].size();j++)
                {
                    cv::Point point_map = transformFromWorldToMap(lidar_near_points_consecutive_frames[i][j], resolution_dynamic, origin_world_dynamic);
                    if(dynamic_measure_map_count.at<uchar>(point_map.x, point_map.y) == 0)
                    {
                        dynamic_measure_map.at<uchar>(point_map.x, point_map.y) = dynamic_measure_map.at<uchar>(point_map.x, point_map.y) + 1;
                        dynamic_measure_map_count.at<uchar>(point_map.x, point_map.y) = 1;
                    }
                }

            }
            int no_zero = 0 ;
            int static_num =0;
            for(int i = 0;i<dynamic_measure_map.rows;i++)
                for(int j = 0;j<dynamic_measure_map.cols;j++)
                {
                    if(dynamic_measure_map.at<uchar>(j,i)!=0)
                    {
                        no_zero++;
                        std::cout<<"no_zero position  " << j <<"  "<<i<<"    "<<(int)dynamic_measure_map.at<uchar>(j,i)<<std::endl;
                    }
                    if(dynamic_measure_map.at<uchar>(j,i)>2)
                    {
                        static_num++;
                          std::cout<<"static_num position  " << j <<"  "<<i<<"  "<<(int)dynamic_measure_map.at<uchar>(j,i)<<std::endl;
                    }

                }

            std::cout<<"static_num   " << static_num <<"  "<<"  no_zero  "<<no_zero<<std::endl;
             if((float)static_num/(float)no_zero<0.5)
             {
                 if_lidar_dynamic = true;
             }
        }



        std::cout<<"if_lidar_dynamic   " << if_lidar_dynamic <<std::endl;
        voxel_grid_manager_->setLaserConsecutiveDetect(if_lidar_dynamic);
        // 4) update prop_map
        std::scoped_lock lk(measure_mutex_, prop_mutex_, prop_copy_mutex_, measure_copy_mutex_);
        {
            LOGD(TAG) << "update prop map" << LOGEND;
            updateMapWithMeasureGrip(measure_map_, prop_map_, current_bound_, update_policy_);
            if (patch_roi.area() > 0)
            {
                cleanBoundFloat(prop_map_copy_, last_bound_, current_bound_, 0.5);
                cv::Mat(prop_map_, patch_roi).copyTo(cv::Mat(prop_map_copy_, patch_roi));
                cleanBoundInt(measure_map_copy_, last_bound_, NO_INFORMATION);
                cv::Mat(measure_map_, patch_roi).copyTo(cv::Mat(measure_map_copy_, patch_roi));
            }
        }

        //cv::imshow("static_pro_map", prop_map_);
        last_bound_ = current_bound_;
#ifdef __ANDROID__
        std::vector<unsigned char> costmap_storage = getLocalMapAsLogFormat(measure_map_, robot_pose_new);
        dumpMapToLog("static_map", robot_pos_world, costmap_storage, resolution_, patch_size_);
#else
        cv::Mat show_mat = getShowMat(measure_map_);
        cv::imshow("voxel_measure_map", show_mat);
        cv::imshow("voxel_prop_map", prop_map_);
#endif
    }
    lidar_update_ = false;
}




