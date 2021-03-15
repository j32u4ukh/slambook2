#include <iostream>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#define MATRIX_SIZE 50

int main(int argc, char** argv){
    // Matrix3d 實質上是 Eigen::Matrix<double, 3, 3>
    // 初始化為全零的矩陣
    Matrix3d matrix_33 = Matrix3d::Zero();
    
    // 如果不確定矩陣大小，可以使用動態大小的矩陣（本例中未使用）
    Matrix<double, Dynamic, Dynamic> matrix_dynamic;
    
    // 更簡單的版本（本例中未使用）
    MatrixXd matrix_x;
    
    // ＝＝＝＝＝ 對 Eigen 進行操作 ＝＝＝＝＝
    // 輸入資料（初始化）
    Matrix<float, 2, 3> matrix_23;
    matrix_23 << 1, 2, 3, 4, 5, 6;
    
    // 輸出矩陣
    cout << "matrix_23 from 1 to 6:\n" << matrix_23 << endl;
    
    // 利用（）存取矩陣中的元素
    cout << "print matrix_23:" << endl;
    for(int i = 0; i < 2; i++){
        
        for(int j = 0; j < 3; j++){            
            cout << matrix_23(i, j) << "\t";
        }
        
        cout << endl;
    }
    
    Vector3d v_3d;
    Matrix<float, 3, 1> vd_3d;
    
    v_3d << 3, 2, 1;
    vd_3d << 4, 5, 6;
    
    // 矩陣與向量相乘（實際上仍是矩陣和矩陣）
    // 利用 cast<double> 做顯性轉換是必須的
    // Eigen 的 * 已重載為矩陣乘法
    Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    cout << "[1,2,3;4,5,6]*[3,2,1] = " << result.transpose() << endl;

    // 矩陣和矩陣相乘
    Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    cout << "[1,2,3;4,5,6]*[4,5,6]: " << result2.transpose() << endl;
    
    // 一些矩陣運算
    // 產生一個隨機數矩陣
    matrix_33 = Matrix3d::Random();     
    cout << "random matrix: \n" << matrix_33 << endl;
    cout << "轉置 transpose: \n" << matrix_33.transpose() << endl;
    
    cout << "各元素和 sum: " << matrix_33.sum() << endl;
    cout << "跡 trace: " << matrix_33.trace() << endl;
    cout << "數乘 times 10: \n" << 10 * matrix_33 << endl;
    cout << "逆 inverse: \n" << matrix_33.inverse() << endl;
    cout << "行列式 det: " << matrix_33.determinant() << endl;

    // 特征值
    // 實對稱矩陣可以保證對角化成功
    SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    cout << "特征值 Eigen values = \n" << eigen_solver.eigenvalues() << endl;
    cout << "特征向量 Eigen vectors = \n" << eigen_solver.eigenvectors() << endl;

    // 解方程
    // 我們求解 matrix_NN * x = v_Nd 這個方程
    // N的大小在前邊的宏里定義，矩陣數值由隨機數生成
    // 直接求逆自然是最直接的，但是求逆運算量大

    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    
    // 保證半正定
    matrix_NN = matrix_NN * matrix_NN.transpose();  
    
    Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);

    // 計時
    clock_t time_stt = clock();
    
    // 直接求逆
    Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    
    cout << "time of normal inverse is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
        
    cout << "x = " << x.transpose() << endl;

    
    time_stt = clock();
    
    // 通常用矩陣分解來求，例如 QR 分解，速度會快很多
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    
    cout << "time of QR decomposition is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;

    
    time_stt = clock();
    
    // 對於正定矩陣，還可以用 cholesky 分解來解方程
    x = matrix_NN.ldlt().solve(v_Nd);
    
    cout << "time of ldlt decomposition is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;
}

















