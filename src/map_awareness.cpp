#include "map_awareness.h"
#define deg2rad M_PI / 180
inline double awareness_map_cylindrical::fast_atan2(double y, double x)
{
    // 1. map the input to 0-1
    double input = y / x;
    double a_input = abs(input);
    double res;
    //   int sign = a_input/input;
    if (a_input > 1) // 이 1의 의미가 뭐지..? --> y=x를 기준으로 나눠서 
    {
        res = copysign(deg2rad * (90 - fast_atan(1 / a_input)), input);
    }
    else // 부호 바꾸는 부분
    {
        res = copysign(deg2rad * fast_atan(a_input), input);
    }

    // 어느 사분면에 있는지 확인하여 부호를 바꿔줌
    if (x > 0)
    {
        return res;
    }
    else if (y >= 0)
    {
        return res + M_PI;
    }
    else
    {
        return res - M_PI;
    }
}

// 숫자 다 줄여가지고 atan 함수를 만듦
inline double awareness_map_cylindrical::fast_atan(double x)
{
    return x * (45 - (x - 1) * (14 + 3.83 * x));
}

awareness_map_cylindrical::awareness_map_cylindrical()
{
}

size_t awareness_map_cylindrical::mapIdx_out(Vec3I Rho_Phi_z)  //for using this function outside this class (not inline)
{
    return this->mapIdx(Rho_Phi_z(0), Rho_Phi_z(1), Rho_Phi_z(2));
}

inline size_t awareness_map_cylindrical::mapIdx(Vec3I Rho_Phi_z)
{
    return this->mapIdx(Rho_Phi_z(0), Rho_Phi_z(1), Rho_Phi_z(2));
}
inline size_t awareness_map_cylindrical::mapIdx(int Rho, int Phi, int z)
{
    return static_cast<size_t>(z * this->nRho_x_nPhi + Phi * this->map_nRho + Rho);
}

void awareness_map_cylindrical::setTbs(SE3 T_bs_in)
{
    this->T_bs = T_bs_in;
}


// Input  : 로, 파이, 높이에 대한 간격 값, 개수, 래이캐스팅 유무
// Output : none
// Brief
// 1. 변수를 클래스 내부 변수에 저장
// 2. 
void awareness_map_cylindrical::init_map(double d_Rho, double d_Phi_deg, double d_Z, int n_Rho, int n_z_below, int n_z_over, bool apply_raycasting)
{
    this->map_dRho = d_Rho;
    this->map_dPhi = d_Phi_deg * M_PI / 180;
    this->map_dZ = d_Z;

    this->map_nRho = n_Rho;
    this->map_nPhi = static_cast<int>(360 / d_Phi_deg);
    this->map_nZ = n_z_below + n_z_over + 1;
    cout << "localmap discretization to n_Rho: " << this->map_nRho << "  n_Phi:" << this->map_nPhi << "  n_Z:" << this->map_nZ << endl;
    this->map_center_z_idx = n_z_below;
    this->z_border_min = -(n_z_below * d_Z) - 0.5 * d_Z;
    this->nRho_x_nPhi = map_nRho * map_nPhi;
    this->map = std::unique_ptr<vector<CYLINDRICAL_CELL>>(new vector<CYLINDRICAL_CELL>());
    // this->map_tmp = std::unique_ptr<vector<CYLINDRICAL_CELL>>(new vector<CYLINDRICAL_CELL>());
    int idx_in_order = 0;
    // 하드코딩 이거 뭐야 ㅋㅋㅋㅋ 
    diff_range = 10;
    for (int diff = -diff_range; diff < diff_range + 1; diff++)
    {
        vector<float> line;
        for (int r = 0; r < n_Rho; r++) // ㄱ값이 n_Rho에 따라 달라짐... n_Rho 값에 따라 ...
        {
            line.emplace_back(get_odds(diff, r));
            //   cout<<"odd: "<<line.back() << " at r: "<<r<<" at diff: "<<diff<<endl;
        }
        get_odds_table.emplace_back(line);
    }
    for (int z = 0; z < this->map_nZ; z++)
    {
        for (int phi = 0; phi < this->map_nPhi; phi++)
        {
            for (int rho = 0; rho < this->map_nRho; rho++)
            {
                struct CYLINDRICAL_CELL cell;
                cell.idx_rho = rho;
                cell.idx_phi = phi;
                cell.idx_z = z;
                double center_z = this->z_border_min + (this->map_dZ / 2) + (z * this->map_dZ);
                double center_rho = this->map_dRho / 2 + (rho * this->map_dRho);
                double center_phi = this->map_dPhi / 2 + (phi * this->map_dPhi);
                double center_x = center_rho * cos(center_phi);
                double center_y = center_rho * sin(center_phi);
                cell.center_pt = Vec3(center_x, center_y, center_z);
                cell.is_occupied = false;
                if (rho > 0)
                {
                    cell.raycasting_z_over_rho = (z - map_center_z_idx) / (rho * 1.0);
                }
                else
                {
                    cell.raycasting_z_over_rho = 0;
                }
                cell.idx = idx_in_order;
                idx_in_order++;
                this->map->emplace_back(cell);
                // this->map_tmp->emplace_back(cell);
            }
        }
    }
    cout << "awareness map contain " << map->size() << " cells" << endl;
    first_input = true;
    visibility_check = apply_raycasting;
}

// Input  : 점의 xyz 위치( Global Coordinate )
// Output : 로, 파이, 높이 값에 대응되는 원통형 셀의 인덱스, 캐스트 할 수 있는지
// Brief 

bool awareness_map_cylindrical::xyz2RhoPhiZwithBoderCheck(Vec3 xyz_l, Vec3I &rhophiz, bool &can_do_cast)
{
    // 원통형 좌표계에서 X, Y 평면 상의 거리 = rho(로)
    double rho = sqrt(pow(xyz_l(0), 2) + pow(xyz_l(1), 2));
    // 로에 맞는 인덱스를 찾음 (거리/델타 로(간격))
    int rho_idx = static_cast<int>(rho / this->map_dRho);
    // double phi0 = atan2(xyz_l(1),xyz_l(0));
    
    // 안쪽에서 경우에 따라 atan2를 구현해 버림.. 기존 펑션이 좀 느렸었나봐
    double phi = fast_atan2(xyz_l(1), xyz_l(0)); // 각도 찾는 부분 

    // fast_atan2 안쪽에서는 사분면에 대한 부호 판단만 하기 때문에
    // 전체 크기에 대한 파이 값은 여기서 확인해주는 것 같음
    if (phi < 0)
        phi += 2 * M_PI;
    
    // 파이값 구하면 델타 값으로 나눠서 인덱스 구함
    int phi_idx = static_cast<int>(phi / this->map_dPhi);

    // 최 하단 값을 빼줌 -> (-) 값을 빼주니까 결과적으로는 위로 올려주는 결과
    double z = xyz_l(2) - this->z_border_min;
    // 똑같이 인덱스 구해줌
    int z_idx = static_cast<int>(floor(z / this->map_dZ));
    rhophiz = Vec3I(rho_idx, phi_idx, z_idx); // 값 반환하기 위해 넣어주고
    // 인덱스가 모두 0 보다 커야하고, 파이의 인덱스가 우리가 지정한 각도에 대한 값보다 작아야함 --> 캐스팅이 되는지 확인..
    can_do_cast = (rho_idx >= 0 && phi_idx >= 0 && phi_idx < this->map_nPhi);
    // 결과적으로 구한 인덱스 값들이 옳은 값을 가지고 있고 캐스팅이 가능한지에 대해 검사
    if (can_do_cast && z_idx >= 0 && rho_idx < this->map_nRho && z_idx < this->map_nZ)
    {
        return true;
    }

    return false;
}

void awareness_map_cylindrical::clear_map()
{
    for (auto &cell : *this->map)
    {
        cell.is_occupied = false;
    }
    this->occupied_cell_idx.clear();
}

inline float awareness_map_cylindrical::sigma_in_dr(size_t x)
{
    float dis = (x * this->map_dRho);
    return 0.00375 * dis * dis / this->map_dRho; // for realsense d435i
}

inline float awareness_map_cylindrical::standard_ND(float x)
{
    double a1 = 0.254829592;
    double a2 = -0.284496736;
    double a3 = 1.421413741;
    double a4 = -1.453152027;
    double a5 = 1.061405429;
    double p = 0.3275911;

    // Save the sign of x
    int sign = 1;
    if (x < 0)
        sign = -1;
    x = fabs(x) / sqrt(2.0);

    // A&S formula 7.1.26
    double t = 1.0 / (1.0 + p * x);
    double y = 1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * exp(-x * x);
    // cout<<"y: "<<y<<" x: "<<x<<" exp: "<<exp(-x * x)<<endl;
    return 0.5 * (1.0 + sign * y); // a close representation for the cumulative density function of a standard normal distribution, see http://www.johndcook.com/blog/cpp_phi/
}

float awareness_map_cylindrical::get_odds(int diff, size_t r)
{
    if (r == 0)
    {
        r = 1;
    }
    float up = standard_ND(static_cast<float>(diff + 0.5) / sigma_in_dr(r));
    float down = standard_ND(static_cast<float>(diff - 0.5) / sigma_in_dr(r));
    // cout<<"get odds: "<<up<<" "<<down<<endl;
    float res = up - down < 0.001 ? 0.001 : up - down;
    res = res >= 1 ? 0.999 : res;
    return res / (1 - res);
}

inline void awareness_map_cylindrical::update_odds_hashmap(Vec3I rpz_idx, float odd)
{
    if (hit_idx_odds_hashmap.find(rpz_idx) == hit_idx_odds_hashmap.end())

        hit_idx_odds_hashmap[rpz_idx] = odd;
    else
        hit_idx_odds_hashmap[rpz_idx] = 1 - (1 - hit_idx_odds_hashmap[rpz_idx]) * (1 - odd);
}
void awareness_map_cylindrical::update_hits(Vec3 p_l, Vec3I rpz_idx, size_t map_idx)
{


    vector<Vec3I> rpz_idx_l;
    int raycasting_z;
    double raycasting_rate = map->at(map_idx).raycasting_z_over_rho; // 이 래이캐스팅 레이트는 뭐하는 거지...
    // 로가 커지고, z 인덱스가 낮으면 ... 낮은 층에 바깥쪽 셀 -> 그냥 기울기
    float odd;
    Vec3I neighbor;

    // 특정 포인트의 인덱스가 들어오면 거기에 해당되는 get_odds_table에 있는 값을 수정하는 듯 함
    update_odds_hashmap(rpz_idx, get_odds_table[0 + diff_range][rpz_idx[0]]);
    // Q1. 이 hit update라는 놈은 하나의 스캔 측정값에 대해 업데이트를 진행하는 건데
    //      raycast를 통해서 그 부분이 없다는걸 인지하는 부분은 어디 있는 거지?
    // Q2. 이전에 차있던 부분을 이번에 래이캐스팅 해서 없어졌다는 걸 인지한다면, 해쉬맵은
    //     각 셀마다 고유한 아이디가 있는건가?

    // 
    for (auto diff_r = 1; diff_r < 3 * sigma_in_dr(rpz_idx[0]) && (rpz_idx[0] + diff_r < this->map_nRho); diff_r++)
    {
        raycasting_z = static_cast<int>(round(rpz_idx[2] + (diff_r * raycasting_rate)));
        odd = get_odds_table[diff_r + diff_range][rpz_idx[0]];
        if (0 <= raycasting_z && raycasting_z < map_nZ)
        {
            // l2g_msg_hit_pts_l.emplace_back(map->at(this->mapIdx(Vec3I(rpz_idx[0] + diff_r, rpz_idx[1], raycasting_z))).center_pt);
            // l2g_msg_hit_odds_l.emplace_back(odd);
            neighbor = {rpz_idx[0] + diff_r, rpz_idx[1], raycasting_z};
            update_odds_hashmap(neighbor, odd);
        }
        odd = get_odds_table[-diff_r + diff_range][rpz_idx[0]];
        raycasting_z = static_cast<int>(round(rpz_idx[2] - (diff_r * raycasting_rate)));
        if (0 <= raycasting_z && raycasting_z < map_nZ)
        {
            // l2g_msg_hit_pts_l.emplace_back(map->at(this->mapIdx(Vec3I(rpz_idx[0] - diff_r, rpz_idx[1], raycasting_z))).center_pt);
            // l2g_msg_hit_odds_l.emplace_back(odd);
            neighbor = {rpz_idx[0] - diff_r, rpz_idx[1], raycasting_z};
            update_odds_hashmap(neighbor, odd);
        }
        // cout<<"added odds other: "<<l2g_msg_hit_odds_l.back()<<endl;
    }
}

void awareness_map_cylindrical::input_pc_pose(vector<Vec3> PC_s, SE3 T_wb)
{
    // this->l2g_msg_hit_pts_l.clear();
    // this->l2g_msg_miss_pts_l.clear();
    // this->l2g_msg_hit_odds_l.clear();
    this->hit_idx_odds_hashmap.clear();
    this->miss_idx_set.clear();
    // this->occupied_cell_idx.clear();

    // STEP 1: transfer from previous map
    // Frame [w]orld, [s]ensor, [b]ody, [l]ocalmap; [g]lobalmap
    T_wa = SE3(SO3(Quaterniond(1, 0, 0, 0)), T_wb.translation()); // awareness == body
    // Awareness랑 world랑은 X축 일치하게 둬서 방향은 안바꾸고 평행이동 값만 넣어줌
    SE3 T_ws = T_wb * this->T_bs; // b2w * s2b --> s to b to w
    SE3 T_ls = T_wa.inverse() * T_ws; // s2w -> w2a --> 결국 sensor to awareness..
    // 왜 이름이 T_ls일까 ls는 결국 sensor to awareness
    
    /* // 그냥 디버깅용 코드인가봄
    // only for visualization, not used.
    // if(first_input)
    // {
    //     first_input = false;
    // }else
    // {
    //     //map_tmp = map;
    //     this->map.swap(this->map_tmp);
    //     for(auto& cell:*this->map)
    //     {
    //         cell.is_occupied = false;
    //     }
    //     Vec3 t_diff = -(T_wa.translation()-this->last_T_wa.translation());
    //     for(auto& cell:*this->map_tmp)
    //     {
    //         if(cell.is_occupied)
    //         {
    //             Vec3 transfered_pt_l = cell.sampled_xyz + t_diff;
    //             Vec3I rpz_idx;
    //             if(xyz2RhoPhiZwithBoderCheck(transfered_pt_l, ))
    //             {//set observerable
    //                 size_t map_idx = mapIdx(rpz_idx);
    //                 if(map->at(map_idx).is_occupied == false)
    //                 {
    //                     map->at(map_idx).is_occupied = true;
    //                     map->at(map_idx).sampled_xyz = transfered_pt_l;
    //                     this->occupied_cell_idx.emplace_back(map_idx);
    //                 }
    //             }
    //         }
    //     }
    // } */
    
    // STEP 2: Add measurement
    for (auto p_s : PC_s) //PC_s는 egien matrix로 xyz 데이터만 담아서 사용
    {
        // Awareness frame으로 위치 변환
        auto p_l = T_ls * p_s;
        //  Eigen::Matrix<int, 3, 1> == Vec3I
        Vec3I rpz_idx; // 로, 파이, 높이에 대한 원통좌표계 인덱스 저장
        bool can_do_cast;
        size_t map_idx;
        
        // 어떤 점의 rho, phi, z에 대한 인덱스와 캐스팅 가능 여부를 검사 --> 항상 true로 나오는거 아닌가?
        bool inside_range = xyz2RhoPhiZwithBoderCheck(p_l, rpz_idx, can_do_cast);
        if (inside_range) // 레인지 검사하는 부분이 없었는데 왜? 이렇게? 해놨찌?--> 최대 개수도 검사를 하니까 거기서 레인지 검사가 이미 된거
        {
            // l2g_msg_hit_pts_l.emplace_back(p_l);

            // set observerable
            map_idx = mapIdx(rpz_idx); // 전체 원통 좌표계에서 어떤 셀에 들어가 있는지 확인
            update_hits(p_l, rpz_idx, map_idx);
            // if(map->at(map_idx).is_occupied == false)
            // {
            //     map->at(map_idx).is_occupied = true;
            //     map->at(map_idx).sampled_xyz = p_l;
            //     this->occupied_cell_idx.emplace_back(map_idx);
            // }
        }

        // 
        if (can_do_cast && visibility_check) // 한 점에 대해서 래이캐스팅이 된다면
        {
            double raycasting_rate;
            if (inside_range)
            {
                raycasting_rate = map->at(map_idx).raycasting_z_over_rho;
            }
            else
            {
                // calculate the raycasting rate again
                if (rpz_idx[0] > 0)
                {
                    raycasting_rate = (rpz_idx[2] - this->map_center_z_idx) / (rpz_idx[0] * 1.0);
                }
                else
                {
                    raycasting_rate = 0;
                }
            }
            if (rpz_idx[0] >= map_nRho)
            {
                rpz_idx[2] = static_cast<int>(round(rpz_idx[2] - ((rpz_idx[0] - map_nRho + 1) * raycasting_rate)));
                rpz_idx[0] = map_nRho - 1;
            }
            for (int r = rpz_idx[0] - 4; r > 0; r--)
            {
                int diff_r = rpz_idx[0] - r;
                int raycasting_z = static_cast<int>(round(rpz_idx[2] - (diff_r * raycasting_rate)));
                // map->at(this->mapIdx(Vec3I(r,rpz_idx[1],raycasting_z))).is_occupied = false;
                if (0 <= raycasting_z && raycasting_z < map_nZ)
                    miss_idx_set.emplace(this->mapIdx(Vec3I(r, rpz_idx[1], raycasting_z)));
                    // l2g_msg_miss_pts_l.emplace_back(map->at(this->mapIdx(Vec3I(r, rpz_idx[1], raycasting_z))).center_pt);
            }
        }

        else
            cout << "point out range: " << p_l.transpose() << ", " << rpz_idx.transpose() << endl;
    }
    // cout<<"miss nodes size: "<<l2g_msg_miss_pts_l.size()<<endl;
    // this->last_T_wa = T_wa;
}
