#include "msg_awareness.h"

msg_awareness::msg_awareness()
{

}

// Input  : 노드 핸들러, 토픽명, 버퍼 사이즈
// Output : void
// Brief
// 노드 핸들러랑, 토픽 이름, 버퍼 사이즈를 받아서 msg_awareness 클래스 안에 있는 awarenessmap_pub
// 변수에 토픽 이름과 버퍼 사이즈를 저장해둠
// 왜 이렇게 번거로운 짓을 하는 거지?
msg_awareness::msg_awareness(ros::NodeHandle& nh, string topic_name, int buffersize)
{
    this->awarenessmap_pub = nh.advertise<mlmapping::awareness>(topic_name,buffersize);
}


// Input  : 맵 포인터 
// Output : void
// Brief
// 함수 인자 받아서 
void msg_awareness::pub(awareness_map_cylindrical *map, ros::Time stamp)
{
    mlmapping::awareness msg; // mlmapping::awareness는 메세지 타입
    msg.header.stamp = stamp;
    msg.occupied_cell_count = static_cast<unsigned int>(map->occupied_cell_idx.size()); // 같은 사이즈 할당
    for(auto cell: map->occupied_cell_idx)
    {
        msg.occupied_cell_idx.push_back(cell); // 전부 복사
    }
    Vec3 t = map->T_wa.translation();
    Quaterniond uq= map->T_wa.unit_quaternion();
    msg.T_w_a.rotation.w=uq.w();
    msg.T_w_a.rotation.x=uq.x();
    msg.T_w_a.rotation.y=uq.y();
    msg.T_w_a.rotation.z=uq.z();
    msg.T_w_a.translation.x = t(0);
    msg.T_w_a.translation.y = t(1);
    msg.T_w_a.translation.z = t(2);
    awarenessmap_pub.publish(msg);

}

// Input  : msg_ptr(뭔지 모름)
// Output : awareness_map_cylindrical 타입의 map_output 포인터를 넘겨줌
// Brief
// 호출되면 msg의 포인터를 받아서 안에 있는 데이터 map_output 포인터로 복사해줌
void msg_awareness::unpack(mlmapping::awarenessConstPtr msg_ptr,
                          awareness_map_cylindrical *map_output)
{
    // SE(쿼터니언, 평행이동) 변환 관계에 대한 행렬 만드는 부분
    map_output->T_wa = SE3(Quaterniond(msg_ptr->T_w_a.rotation.w,
                                msg_ptr->T_w_a.rotation.x,
                                msg_ptr->T_w_a.rotation.y,
                                msg_ptr->T_w_a.rotation.z),
                    Vector3d(msg_ptr->T_w_a.translation.x,
                             msg_ptr->T_w_a.translation.y,
                             msg_ptr->T_w_a.translation.z));
                             
    // msg_ptr로부터 들어온 데이터를 받기 위해 map_output 포인터에 접근해서 안에 있는 애들 수동으로 초기화 
    // 그리고나서 데이터 직접 다 옮김
    // 왜 이런 귀찮은 일을 해줘야했는지..
    // 작성자 옆에 앉혀서 심문하고 싶다

    map_output->clear_map(); // map에 있는 cell에 occupied를 전부 false, odd 값을 0으로 초기화, occupied_cell_idx_map도 초기화
    size_t cnt = msg_ptr->occupied_cell_count; 
    for(size_t i=0; i<cnt; i++) // 카운트 있는 만큼 
    {
        map_output->occupied_cell_idx.push_back(msg_ptr->occupied_cell_idx.at(i));
        map_output->map->at(i).is_occupied=true;
    }
}
