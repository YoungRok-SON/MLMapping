#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <utils/include/all_utils.h>
#include <map_awareness.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rviz_vis.h>
#include <msg_awareness2local.h>
#include <msg_awareness.h>
#include <chrono>
#include <numeric>
#include <iostream>
namespace mlmapping_ns
{

    using namespace std;

    class AwarenessMapNodeletClass : public nodelet::Nodelet
    {
    public:
        AwarenessMapNodeletClass() { ; }
        ~AwarenessMapNodeletClass() { ; }

    private:
        // Subscribes
        // Main Inputs
        message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
        message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

        // TIme synchronizer 관련 내용은 아래 링크 참고
        // https://velog.io/@zzziito/ROS-Message-filter-%EB%A9%94%EC%8B%9C%EC%A7%80-%ED%95%84%ED%84%B0
        // Exact time policies with pose msgs or odom msgs
        typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> ExactSyncPolicy;
        typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> ExactSyncPolicyOdom;
        // Synchronizer with ExactTime polices with pose msgs and odom msgs
        message_filters::Synchronizer<ExactSyncPolicy> *exactSync_;
        message_filters::Synchronizer<ExactSyncPolicyOdom> *OdomexactSync_;
        // ApproximateTime Policies with pose msgs and odom msgs
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> ApproxSyncPolicy;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> ApproxSyncPolicyOdom;
        // Synchronizer with Approximate Polices with pose msgs and odom msgs
        message_filters::Synchronizer<ApproxSyncPolicy> *approxSync_;
        message_filters::Synchronizer<ApproxSyncPolicyOdom> *OdomapproxSync_;
        
        // Publishes
        msg_awareness2local *a2w_pub;       // Awareness to world topics
        msg_awareness *awarenessmap_pub;    // Awareness frame topics
        tf2_ros::TransformBroadcaster br;

        // Timer
        ros::Timer timer_;
        int count = 0;
        double time_total = 0;

        // variable
        awareness_map_cylindrical *awareness_map;
        int pc_sample_cnt;
        bool publish_T_wb;
        bool publish_T_bs;
        geometry_msgs::TransformStamped transformStamped_T_wb; // body to worl
        geometry_msgs::TransformStamped transformStamped_T_wa; // awareness to world
        geometry_msgs::TransformStamped transformStamped_T_bs; // sensor to body

        // Brief
        // 얘들은 포인트 클라우드랑 Odom or Pose가 둘다 들어와야 콜백이 실행됨
        // 저기 message filter 라는 놈이 두개 다 받아서 어느정도 비슷하게 들어온 애들만 골라서 이걸 실행 시킴
        void pc_odom_input_callback(const sensor_msgs::PointCloud2::ConstPtr &pc_Ptr,
                                    const nav_msgs::Odometry::ConstPtr &pose_Ptr)
        {
            //        tic_toc_ros tt;
            //        static double sum_t = 0;
            //        static int count = 0;
            double time_gap = (pc_Ptr->header.stamp - pose_Ptr->header.stamp).toSec();
            cout<<"Time gap between pcl and odom (ms): "<<time_gap*1000<<endl;
            auto t1 = std::chrono::system_clock::now();
            SE3 T_wb(SO3(Quaterniond(pose_Ptr->pose.pose.orientation.w,
                                     pose_Ptr->pose.pose.orientation.x,
                                     pose_Ptr->pose.pose.orientation.y,
                                     pose_Ptr->pose.pose.orientation.z)),
                     Vec3(pose_Ptr->pose.pose.position.x,
                          pose_Ptr->pose.pose.position.y,
                          pose_Ptr->pose.pose.position.z));
            if (publish_T_wb)
            {
                transformStamped_T_wb.header.stamp = pose_Ptr->header.stamp;
                transformStamped_T_wb.transform.translation.x = T_wb.translation().x();
                transformStamped_T_wb.transform.translation.y = T_wb.translation().y();
                transformStamped_T_wb.transform.translation.z = T_wb.translation().z();
                transformStamped_T_wb.transform.rotation.x = T_wb.so3().unit_quaternion().x();
                transformStamped_T_wb.transform.rotation.y = T_wb.so3().unit_quaternion().y();
                transformStamped_T_wb.transform.rotation.z = T_wb.so3().unit_quaternion().z();
                transformStamped_T_wb.transform.rotation.w = T_wb.so3().unit_quaternion().w();
                br.sendTransform(transformStamped_T_wb);
                transformStamped_T_wa.header.stamp = pose_Ptr->header.stamp;
                transformStamped_T_wa.transform.translation = transformStamped_T_wb.transform.translation;
                br.sendTransform(transformStamped_T_wa);
            }
            if (publish_T_bs)
            {
                transformStamped_T_bs.header.stamp = pose_Ptr->header.stamp;
                br.sendTransform(transformStamped_T_bs);
            }
            auto t10 = std::chrono::system_clock::now();
            PointCloudP_ptr cloud(new PointCloudP);
            pcl::fromROSMsg(*pc_Ptr, *cloud);
            
            if (pc_Ptr->is_dense)
            {
            }
            else
            { // remove invalid pts
                vector<int> index;
                pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
            }

            int pcsize = static_cast<int>(cloud->size());
            vector<Vec3> pc_eigen;
            if (pcsize > pc_sample_cnt)
            {
                for (int i = 0; i < pc_sample_cnt; i++)
                {
                    size_t rand_idx = static_cast<size_t>(rand() % pcsize); // 
                    PointP pt = cloud->at(rand_idx);
                    pc_eigen.push_back(Vec3(static_cast<double>(pt.x),
                                            static_cast<double>(pt.y),
                                            static_cast<double>(pt.z)));
                }
            }
            else
            {
                for (int i = 0; i < pcsize; i++)
                {
                    PointP pt = cloud->at(i);
                    pc_eigen.push_back(Vec3(static_cast<double>(pt.x),
                                            static_cast<double>(pt.y),
                                            static_cast<double>(pt.z)));
                }
            }
            auto t11 = std::chrono::system_clock::now();
            awareness_map->input_pc_pose(pc_eigen, T_wb);
            
            awarenessmap_pub->pub(awareness_map, pose_Ptr->header.stamp);

            a2w_pub->pub_a2l(awareness_map,
                         pose_Ptr->header.stamp);
            auto t2 = std::chrono::system_clock::now();
            std::chrono::duration<double> diff = t2 - t1;
            // std::chrono::duration<double> diff = t11 - t10;
            time_total += diff.count() * 1000;
            printf("Awareness map single time: %.8f ms, ave-time cost: %.8f ms\n", diff.count() * 1000, time_total / (++count));

            //        sum_t+=tt.dT_ms();
            //        count++;
            //    cout << awareness_map->l2g_msg_hit_pts_l.size()<<" "<<awareness_map->l2g_msg_miss_pts_l.size() << endl;
        }

        void pc_pose_input_callback(const sensor_msgs::PointCloud2::ConstPtr &pc_Ptr,
                                    const geometry_msgs::PoseStamped::ConstPtr &pose_Ptr)
        {
            //        tic_toc_ros tt;
            //        static double sum_t = 0;
            //        static int count = 0;

            // Pose 데이터 받아서 body to world transformation 변수에 삽입
            SE3 T_wb(SO3(Quaterniond(pose_Ptr->pose.orientation.w,
                                     pose_Ptr->pose.orientation.x,
                                     pose_Ptr->pose.orientation.y,
                                     pose_Ptr->pose.orientation.z)),
                     Vec3(pose_Ptr->pose.position.x,
                          pose_Ptr->pose.position.y,
                          pose_Ptr->pose.position.z));
            
            // body to world T를 pub할거면 아래 실행
            if (publish_T_wb)
            {
                transformStamped_T_wb.header.stamp = pose_Ptr->header.stamp;
                transformStamped_T_wb.transform.translation.x = T_wb.translation().x();
                transformStamped_T_wb.transform.translation.y = T_wb.translation().y();
                transformStamped_T_wb.transform.translation.z = T_wb.translation().z();
                transformStamped_T_wb.transform.rotation.x = T_wb.so3().unit_quaternion().x();
                transformStamped_T_wb.transform.rotation.y = T_wb.so3().unit_quaternion().y();
                transformStamped_T_wb.transform.rotation.z = T_wb.so3().unit_quaternion().z();
                transformStamped_T_wb.transform.rotation.w = T_wb.so3().unit_quaternion().w();
                br.sendTransform(transformStamped_T_wb);

                transformStamped_T_wa.header.stamp = pose_Ptr->header.stamp; // pose 내용 담아서 pub
                transformStamped_T_wa.transform.translation = transformStamped_T_wb.transform.translation;
                br.sendTransform(transformStamped_T_wa);
            }
            
            // sensor to body frame pub
            if (publish_T_bs)
            {
                transformStamped_T_bs.header.stamp = pose_Ptr->header.stamp;
                br.sendTransform(transformStamped_T_bs);
            }

            //pcl::PointCloud<pcl::PointCloudXYZ>::Ptr == PointCloudP_ptr 
            PointCloudP_ptr cloud(new PointCloudP);
            // ROS msg에 담긴걸 pcl로 옮겨
            pcl::fromROSMsg(*pc_Ptr, *cloud);
            if (pc_Ptr->is_dense)
            {
            }
            else // remove invalid pts
            { 
                vector<int> index;
                pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
            }
            // 사이즈 구해
            int pcsize = static_cast<int>(cloud->size());
            vector<Vec3> pc_eigen;
            // 포인트 클라우드의 크기가 내가 지정한 크기보다 많다면?
            if (pcsize > pc_sample_cnt)
            {
                for (int i = 0; i < pc_sample_cnt; i++)
                {
                    size_t rand_idx = static_cast<size_t>(rand() % pcsize); // 진짜 인덱스 랜덤으로 골라서 뽑아버림;
                    PointP pt = cloud->at(rand_idx);
                    pc_eigen.push_back(Vec3(static_cast<double>(pt.x),
                                            static_cast<double>(pt.y),
                                            static_cast<double>(pt.z)));
                }
            }
            else // 적거나 같으면 복사 ..
            {
                for (int i = 0; i < pcsize; i++)
                {
                    PointP pt = cloud->at(i);
                    pc_eigen.push_back(Vec3(static_cast<double>(pt.x),
                                            static_cast<double>(pt.y),
                                            static_cast<double>(pt.z)));
                }
            }


            // 여기가 진짜네 
            awareness_map->input_pc_pose(pc_eigen, T_wb);
            awarenessmap_pub->pub(awareness_map, pose_Ptr->header.stamp);

            a2w_pub->pub_a2l(awareness_map,
                         pose_Ptr->header.stamp);

            //        sum_t+=tt.dT_ms();
            //        count++;
            //        cout << (sum_t/count) << endl;
        }

        //  Brief
        // 일정 시간마다(여기서는 0.2초) 아래 내용들을 퍼블리쉬함
        // Published topics: transformStamped_T_wa, transformStamped_T_wb, transformStamped_T_bs
        void timerCb(const ros::TimerEvent &event)
        {
            
            transformStamped_T_wa.header.stamp = ros::Time::now();
            br.sendTransform(transformStamped_T_wa);
            if (publish_T_wb)
            {
                transformStamped_T_wb.header.stamp = ros::Time::now();
                br.sendTransform(transformStamped_T_wb);
            }
            if (publish_T_bs)
            {
                transformStamped_T_bs.header.stamp = ros::Time::now();
                br.sendTransform(transformStamped_T_bs);
            }
        }


        // Nodelet은 main문 대신 OnInit에서 main 처럼 돌아감
        // 모든 쓰레드 관리는 자동으로 이뤄짐
        virtual void onInit()
        {
            cout << "-------------------------------------------------" << endl;
            cout << "Nodelet Name: " << getName() << endl;
            
            // Get the private node handle with the Multi Threaded callback queue. 
            // (provides this nodelets custom remappings in its private namespace)
            ros::NodeHandle &nh = getMTPrivateNodeHandle();
            string configFilePath;
            nh.getParam("/mlmapping_configfile", configFilePath); // "/mlmapping_configfile" passed by launch file.
            cout << "config file path: " << configFilePath << endl;

            // Init map
            pc_sample_cnt = getIntVariableFromYaml(configFilePath, "mlmapping_sample_cnt");
            // Create Awareness map class as object
            awareness_map = new awareness_map_cylindrical();
            // Read transformation for sensor frame to body frame.
            Mat4x4 T_bs_mat = Mat44FromYaml(configFilePath, "T_B_S");
            // Declare transformation
            SE3 T_bs = SE3(T_bs_mat.topLeftCorner(3, 3),
                           T_bs_mat.topRightCorner(3, 1));
            // Pass the SE3 T_bs to class member variable
            awareness_map->setTbs(T_bs);
            // Set a parameters to generate Cylindrical coordinate
            awareness_map->init_map(getDoubleVariableFromYaml(configFilePath, "mlmapping_am_d_Rho"),
                                    getDoubleVariableFromYaml(configFilePath, "mlmapping_am_d_Phi_deg"),
                                    getDoubleVariableFromYaml(configFilePath, "mlmapping_am_d_Z"),
                                    getIntVariableFromYaml(configFilePath, "mlmapping_am_n_Rho"),
                                    getIntVariableFromYaml(configFilePath, "mlmapping_am_n_Z_below"),
                                    getIntVariableFromYaml(configFilePath, "mlmapping_am_n_Z_over"),
                                    getBoolVariableFromYaml(configFilePath, "mlmapping_use_raycasting"));

            // transformation 초기화
            // transformStamped_T_wb ( body to world )
            publish_T_wb = getBoolVariableFromYaml(configFilePath, "publish_T_wb");
            transformStamped_T_wb.header.frame_id = getStringFromYaml(configFilePath, "world_frame_id");
            transformStamped_T_wb.child_frame_id = getStringFromYaml(configFilePath, "body_frame_id");
            transformStamped_T_wb.transform.translation.x = 0;
            transformStamped_T_wb.transform.translation.y = 0;
            transformStamped_T_wb.transform.translation.z = 0;
            transformStamped_T_wb.transform.rotation.x = 0;
            transformStamped_T_wb.transform.rotation.y = 0;
            transformStamped_T_wb.transform.rotation.z = 0;
            transformStamped_T_wb.transform.rotation.w = 1;
            // transformStamped_T_wa ( Awareness to World )
            transformStamped_T_wa = transformStamped_T_wb;
            transformStamped_T_wa.child_frame_id = getStringFromYaml(configFilePath, "awareness_frame_id");
            // transformStamped_T_bs ( Sensor to Body )
            publish_T_bs = getBoolVariableFromYaml(configFilePath, "publish_T_bs");
            transformStamped_T_bs.header.frame_id = getStringFromYaml(configFilePath, "body_frame_id");
            transformStamped_T_bs.child_frame_id = getStringFromYaml(configFilePath, "sensor_frame_id");
            transformStamped_T_bs.transform.translation.x = T_bs.translation().x();
            transformStamped_T_bs.transform.translation.y = T_bs.translation().y();
            transformStamped_T_bs.transform.translation.z = T_bs.translation().z();
            transformStamped_T_bs.transform.rotation.x = T_bs.so3().unit_quaternion().x();
            transformStamped_T_bs.transform.rotation.y = T_bs.so3().unit_quaternion().y();
            transformStamped_T_bs.transform.rotation.z = T_bs.so3().unit_quaternion().z();
            transformStamped_T_bs.transform.rotation.w = T_bs.so3().unit_quaternion().w();

            // Initialization for Publishers
            awarenessmap_pub = new msg_awareness(nh, "/mlmapping_awareness"); // 메세지 받아서 퍼블리쉬만 하는 클래스를 만들어버림
            a2w_pub = new msg_awareness2local(nh, "/awareness2local", 2);     // 여기도 마찬가지로 Awareness map에서 world로 바꿔서 퍼블리쉬하는 클래스를 만들어버림
            bool use_exactsync = getBoolVariableFromYaml(configFilePath, "use_exactsync");
            bool use_odom = getBoolVariableFromYaml(configFilePath, "use_odom");

            if (use_exactsync)
            {
                // pc_sub.subscribe(nh, "/camera1/depth/color/points", 10);
                pc_sub.subscribe(nh, "/mlmapping/pc", 10);
                if (use_odom)
                {
                    odom_sub.subscribe(nh, "/mlmapping/odom", 10);
                    // 저기 5라고 적혀 있는 queue는 무엇에 대한 큐인가..
                    // 예상을 하건데 A와 B 메세지가 있을 때, A 메세지의 주기가 더 빠르다고 치면
                    // A 메세지가 5개 쌓일동안 B 메세지가 안들어오면 A 메세지를 다 날려버리는...?
                    // 많이 찾아봤는데 잘 모르겠다

                    OdomexactSync_ = new message_filters::Synchronizer<ExactSyncPolicyOdom>(ExactSyncPolicyOdom(5), pc_sub, odom_sub);
                    // 여기서 _1,_2는 자리표(Placeholder)로 bind에 의해 생성된 함수자가 받을 인수를 의미
                    OdomexactSync_->registerCallback(boost::bind(&AwarenessMapNodeletClass::pc_odom_input_callback, this, _1, _2));
                }
                else

                {
                    pose_sub.subscribe(nh, "/mlmapping/pose", 10);
                    exactSync_ = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(5), pc_sub, pose_sub);
                    exactSync_->registerCallback(boost::bind(&AwarenessMapNodeletClass::pc_pose_input_callback, this, _1, _2));
                }
                cout << "ExactSyncPolicy" << endl;
            }
            else
            {
                pc_sub.subscribe(nh, "/mlmapping/pc", 1);
                if (use_odom)
                {
                    odom_sub.subscribe(nh, "/mlmapping/odom", 1);
                    OdomapproxSync_ = new message_filters::Synchronizer<ApproxSyncPolicyOdom>(ApproxSyncPolicyOdom(100), pc_sub, odom_sub);
                    OdomapproxSync_->registerCallback(boost::bind(&AwarenessMapNodeletClass::pc_odom_input_callback, this, _1, _2));
                }
                else

                {
                    pose_sub.subscribe(nh, "/mlmapping/pose", 1);
                    approxSync_ = new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(100), pc_sub, pose_sub);
                    approxSync_->registerCallback(boost::bind(&AwarenessMapNodeletClass::pc_pose_input_callback, this, _1, _2));
                }
                cout << "ApproxSyncPolicy" << endl;
            }
            // 일정 시간(0.2초)이 지나면 timerCB를 호출하여 돌아가도록 함
            timer_ = nh.createTimer(ros::Duration(0.2), boost::bind(&AwarenessMapNodeletClass::timerCb, this, _1));
        }
    }; // class AwarenessMapNodeletClass
} // namespace mlmapping_ns

PLUGINLIB_EXPORT_CLASS(mlmapping_ns::AwarenessMapNodeletClass, nodelet::Nodelet)
