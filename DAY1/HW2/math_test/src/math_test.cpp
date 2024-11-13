#include <iostream>
#include <Eigen/Dense>
#include <cmath>

#define PI 3.141592653589793238463

Eigen::Matrix4d createHomogeneousMatrix(double x, double y, double theta) {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity(); // 단위 행렬 생성

    double rad = theta * PI / 180.0; // 각도를 라디안으로 변환

    // 회전 부분 설정
    matrix(0,0) = cos(rad);
    matrix(0,1) = -sin(rad);
    matrix(1,0) = sin(rad);
    matrix(1,1) = cos(rad);

    // 평행이동 부분 설정
    matrix(0,3) = x;
    matrix(1,3) = y;

    return matrix; // 변환 행렬 반환
}

void printTransform(const Eigen::Matrix4d& T, const std::string& name) {
    std::cout << name << ":\n";
    std::cout << "Matrix:\n" << T << "\n";
    std::cout << "Position (x,y): " << T(0,3) << ", " << T(1,3) << "\n";
    double theta = atan2(T(1,0), T(0,0)) * 180.0 / PI;
    std::cout << "Rotation (degrees): " << theta << "\n\n";
}

int main() {
    // 포즈 생성
    Eigen::Matrix4d T_A = createHomogeneousMatrix(3, 4, 45);
    Eigen::Matrix4d T_B = createHomogeneousMatrix(-6, 7, -60);
    Eigen::Matrix4d T_C = createHomogeneousMatrix(10, 2, 135);

    Eigen::Matrix4d T_AB = T_B.inverse() * T_A;  // A -> B
    Eigen::Matrix4d T_CB = T_B.inverse() * T_C;  // C -> B

    std::cout << "Given transforms:\n";
    printTransform(T_AB, "T_AB (A to B transform)");
    printTransform(T_CB, "T_CB (C to B transform)");

    Eigen::Matrix4d T_AC = T_CB.inverse() * T_AB;

    std::cout << "\nCalculated transform:\n";
    printTransform(T_AC, "T_AC (A to C transform)");

    Eigen::Vector4d p_A(1, 0, 0, 1); // A 좌표계의 테스트 점 (1, 0)
    Eigen::Vector4d origin_A(0, 0, 0, 1); // A 좌표계의 원점

    Eigen::Vector4d p_C = T_AC * p_A;
    Eigen::Vector4d origin_C = T_AC * origin_A;

    std::cout << "Points in A's frame:\n";
    std::cout << "Origin: " << origin_A.transpose() << "\n";
    std::cout << "Test point: " << p_A.transpose() << "\n\n";

    std::cout << "Points in C's frame:\n";
    std::cout << "A's origin: " << origin_C.transpose() << "\n";
    std::cout << "Test point: " << p_C.transpose() << "\n";

    return 0;
}
