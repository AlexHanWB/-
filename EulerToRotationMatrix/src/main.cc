//#include "pch.h"
#include <iostream>
#include "Eigen/Dense"
#include <deque>
#include <algorithm>
using namespace std;

const int GLOBAL_POSE_SIZE=700;

Eigen::Matrix3d zyxToRotationMatrix(Eigen::Vector3d zyx, bool is_inner_rotate)
{
    // 计算旋转矩阵的X分量
    Eigen::Matrix3d R_x;
    R_x << 1,            0,             0,
           0,  cos(zyx[2]),  -sin(zyx[2]),
           0,  sin(zyx[2]),   cos(zyx[2]);

    // 计算旋转矩阵的Y分量
    Eigen::Matrix3d R_y;
    R_y << cos(zyx[1]),  0,  sin(zyx[1]),
                     0,  1,            0,
          -sin(zyx[1]),  0,  cos(zyx[1]);

    // 计算旋转矩阵的Z分量
    Eigen::Matrix3d R_z;
    R_z << cos(zyx[0]),  -sin(zyx[0]),  0,
           sin(zyx[0]),   cos(zyx[0]),  0,
                     0,             0,  1;

    // 依次左乘，合并
    // Eigen::Matrix3d R = R_z*R_y*R_x;
    if(is_inner_rotate)
    {
        return R_z*R_y*R_x;
    }
    else
    {
        return R_x*R_y*R_z;
    }
    
}
int main(){
    //ZYX旋转顺序 内部旋转or外部旋转
    bool is_inner_rotate = true;
    Eigen::Vector3d w_p(1,1,0), c_p;
    //【2-1】: 欧拉角(机体坐标系旋转)  
    Eigen::Vector3d euler_zyx(M_PI/2, M_PI/6, 0);     
    Eigen::Matrix3d Rwc, Rcw;

    Eigen::AngleAxisd r_z ( euler_zyx[0], Eigen::Vector3d ( 0,0,1 ) ); //沿 Z 轴旋转
    Eigen::AngleAxisd r_y ( euler_zyx[1], Eigen::Vector3d ( 0,1,0 ) ); //沿 Y 轴旋转
    Eigen::AngleAxisd r_x ( euler_zyx[2], Eigen::Vector3d ( 1,0,0 ) ); //沿 X 轴旋转
    Eigen::Quaterniond q_zyx = is_inner_rotate ? r_z*r_y*r_x : r_x*r_y*r_z; //（绕旋转后的轴接着旋转）or绕固轴旋转）
    //【1-2】: 四元数-->>旋转矩阵                                  
    Rwc = q_zyx.toRotationMatrix(); //如果是从w系按照ZYX旋转得到b系，那么此旋转矩阵是Rwc，不是Rcw
    Rcw = Rwc.transpose();
    std::cout<<Rwc.transpose()<<"\n\n"<<Rwc.inverse()<<endl;
    c_p =Rcw * w_p;
    cout << "四元数:\nRcw:\n"<<Rcw<<"\nc_p:\n"<<c_p << endl;                     
    
    //【2-2】: 欧拉角-->>旋转矩阵
    Rwc = zyxToRotationMatrix(euler_zyx, is_inner_rotate); //此转换结果与【1-1】相同
    Rcw = Rwc.transpose();
    c_p =Rcw * w_p;
    cout << "欧拉角:\nRcw:\n"<<Rcw<<"\nc_p:\n"<<c_p << endl;
    return 0;
}
