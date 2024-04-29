#ifndef POINT_TF
#define POINT_TF

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

typedef Matrix<double, 3, 1> Matrix31d;
typedef Matrix<double, 4, 1> Matrix41d;

class PointTF
{
public:
    static geometry_msgs::Point Transform(geometry_msgs::Point point_, geometry_msgs::Transform tf_, bool reverse);
private:
    static Matrix4d Quaternion2Mat4(geometry_msgs::Quaternion quat_in);
    static geometry_msgs::Quaternion Mat32Quaternion(Matrix3d matrix_in);
    static Matrix3d Quaternion2Mat3(geometry_msgs::Quaternion quat_in);
    static geometry_msgs::Transform ReverseTF(geometry_msgs::Transform tf_);
};

#endif
