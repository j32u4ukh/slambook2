#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

/// 本程序演示sophus的基本用法

int main(int argc, char **argv) {

  // 沿Z軸轉90度的旋轉矩陣
    AngleAxisd angle_axisd = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1));
    cout << "angle_axisd: " << angle_axisd.angle() << "\n" << angle_axisd.axis() << endl;
    
    Matrix3d R = angle_axisd.toRotationMatrix();
    // 或者四元數
    Quaterniond q(R);
    Sophus::SO3d SO3_R(R);              // Sophus::SO3d可以直接從旋轉矩陣構造
    Sophus::SO3d SO3_q(q);              // 也可以通過四元數構造
    // 二者是等價的
    cout << "SO(3) from matrix:\n" << SO3_R.matrix() << endl;
    cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << endl;
    cout << "they are equal" << endl;

    // 使用對數映射獲得它的李代數
    Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;
    // hat 為向量到反對稱矩陣
    cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << endl;
    // 相對的，vee為反對稱到向量
    cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

    // 增量擾動模型的更新
    Vector3d update_so3(1e-4, 0, 0); //假設更新量為這麽多
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;

    cout << "*******************************" << endl;
    // 對SE(3)操作大同小異
    Vector3d t(1, 0, 0);           // 沿X軸平移1
    Sophus::SE3d SE3_Rt(R, t);           // 從R,t構造SE(3)
    Sophus::SE3d SE3_qt(q, t);            // 從q,t構造SE(3)
    cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << endl;
    cout << "SE3 from q,t= \n" << SE3_qt.matrix() << endl;
    // 李代數se(3) 是一個六維向量，方便起見先typedef一下
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    cout << "se3 = " << se3.transpose() << endl;
    // 觀察輸出，會发現在Sophus中，se(3)的平移在前，旋轉在後.
    // 同樣的，有hat和vee兩個算符
    cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << endl;
    cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;

    // 最後，演示一下更新
    Vector6d update_se3; //更新量
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;

    return 0;
}
