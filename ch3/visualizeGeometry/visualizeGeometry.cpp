#include <iostream>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>

using namespace Eigen;
using namespace std;

struct RotationMatrix {
    Matrix3d matrix = Matrix3d::Identity();
};

struct TranslationVector {
    Vector3d trans = Vector3d(0, 0, 0);
};

struct QuaternionDraw {
    Quaterniond q;
};

ostream &operator<<(ostream &out, const RotationMatrix &r);
istream &operator>>(istream &in, RotationMatrix &r);

ostream &operator<<(ostream &out, const TranslationVector &t);
istream &operator>>(istream &in, TranslationVector &t);

ostream &operator<<(ostream &out, const QuaternionDraw quat);
istream &operator>>(istream &in, const QuaternionDraw quat);

int main(int argc, char **argv) {
    pangolin::CreateWindowAndBind("visualize geometry", 1000, 600);
    glEnable(GL_DEPTH_TEST);
    
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1000, 600, 420, 420, 500, 300, 0.1, 1000),
        pangolin::ModelViewLookAt(3, 3, 3, 0, 0, 0, pangolin::AxisY)
    );
    
    const int UI_WIDTH = 500;
    pangolin::View &d_cam = pangolin::CreateDisplay().
    SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -1000.0f / 600.0f).
    SetHandler(new pangolin::Handler3D(s_cam));
    
    // ui
    pangolin::Var<RotationMatrix> rotation_matrix("ui.R", RotationMatrix());
    pangolin::Var<TranslationVector> translation_vector("ui.t", TranslationVector());
    pangolin::Var<TranslationVector> euler_angles("ui.rpy", TranslationVector());
    pangolin::Var<QuaternionDraw> quaternion("ui.q", QuaternionDraw());
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        pangolin::OpenGlMatrix matrix = s_cam.GetModelViewMatrix();
        Matrix<double, 4, 4> m = matrix;
        RotationMatrix R; 
        
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                R.matrix(i, j) = m(j, i);                
            }            
        }
        
        // 呈現旋轉矩陣
        rotation_matrix = R;

        TranslationVector t;
        t.trans = Vector3d(m(0, 3), m(1, 3), m(2, 3));
        t.trans = -R.matrix * t.trans;
        
        // 呈現平移向量
        translation_vector = t;

        TranslationVector euler;
        
        // 旋轉矩陣便換成歐拉角（須指定旋轉順序）
        euler.trans = R.matrix.eulerAngles(2, 1, 0);
        
        // 呈現歐拉角
        euler_angles = euler;

        QuaternionDraw quat;
        quat.q = Quaterniond(R.matrix);
        
        // 呈現四元數
        quaternion = quat;

        // 清空畫面(白色)
        glColor3f(1.0, 1.0, 1.0);

        // 畫出有色立方體
        pangolin::glDrawColouredCube();
        
        // draw the original axis
        // 指定描繪形式： 線
        glBegin(GL_LINES);   
        glLineWidth(3);
        
        // X 軸
        glColor3f(0.8f, 0.f, 0.f);
        glVertex3f(0, 0, 0);
        glVertex3f(10, 0, 0);
        
        // Y 軸
        glColor3f(0.f, 0.8f, 0.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 10, 0);
        
        // Z 軸
        glColor3f(0.2f, 0.2f, 1.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 10);
        
        // 結束：指定描繪形式
        glEnd();

        pangolin::FinishFrame();
  }
}

ostream &operator<<(ostream &out, const RotationMatrix &r) {
    out.setf(ios::fixed);
    Matrix3d matrix = r.matrix;
    out << '=';
    out << "[" << setprecision(2) << matrix(0, 0) << "," << matrix(0, 1) << "," << matrix(0, 2) << "],"
        << "[" << matrix(1, 0) << "," << matrix(1, 1) << "," << matrix(1, 2) << "],"
        << "[" << matrix(2, 0) << "," << matrix(2, 1) << "," << matrix(2, 2) << "]";
    return out;
}

istream &operator>>(istream &in, RotationMatrix &r) {
    return in;
}

ostream &operator<<(ostream &out, const TranslationVector &t) {
    out << "=[" << t.trans(0) << ',' << t.trans(1) << ',' << t.trans(2) << "]";
    return out;
}

istream &operator>>(istream &in, TranslationVector &t) {
    return in;
}

ostream &operator<<(ostream &out, const QuaternionDraw quat) {
    auto c = quat.q.coeffs();
    out << "=[" << c[0] << "," << c[1] << "," << c[2] << "," << c[3] << "]";
    return out;
}

istream &operator>>(istream &in, const QuaternionDraw quat) {
    return in;
}

