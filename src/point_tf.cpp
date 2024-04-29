#include <point_tf/point_tf.h>

using namespace std;
using namespace Eigen;

geometry_msgs::Point PointTF::Transform(geometry_msgs::Point point_, geometry_msgs::Transform tf_, bool reverse){
    geometry_msgs::Point transformed_point;

    geometry_msgs::Transform tf = tf_;
    if (reverse){
        tf = ReverseTF(tf_);
    }
    
    Matrix4d tf_mat_t = Matrix4d::Zero();

    Matrix41d point_t_in = Matrix41d::Zero();

    Matrix41d point_t_out = Matrix41d::Zero();

    // transformation matrix for position(4x4)
    tf_mat_t = Quaternion2Mat4(tf.rotation);

    tf_mat_t(0,3) = tf.translation.x;
    tf_mat_t(1,3) = tf.translation.y;
    tf_mat_t(2,3) = tf.translation.z;
    tf_mat_t(3,3) = 1.0;

    // position matrix of the point (4x1)
    point_t_in(0,0) = point_.x;
    point_t_in(1,0) = point_.y;
    point_t_in(2,0) = point_.z;
    point_t_in(3,0) = 1.0;

    // transformation
    point_t_out = tf_mat_t * point_t_in;

    // point out
    transformed_point.x = point_t_out(0,0);
    transformed_point.y = point_t_out(1,0);
    transformed_point.z = point_t_out(2,0);

    return transformed_point;
}

Matrix4d PointTF::Quaternion2Mat4(geometry_msgs::Quaternion quat_in){
    Matrix4d rotation_matrix = Matrix4d::Zero();
    double qx = quat_in.x;
    double qy = quat_in.y;
    double qz = quat_in.z;
    double qw = quat_in.w;
    double r = sqrt(pow(qx,2.0) + pow(qy,2.0) + pow(qz,2.0) + pow(qw,2.0));
    if (r!=0){
        qx = qx/r;
        qy = qy/r;
        qz = qz/r;
        qw = qw/r;
    }

    rotation_matrix(0,0) = 1 - 2*pow(qy,2) - 2*pow(qz,2);
    rotation_matrix(0,1) = 2*qx*qy - 2*qz*qw;
    rotation_matrix(0,2) = 2*qx*qz + 2*qy*qw;
    
    rotation_matrix(1,0) = 2*qx*qy + 2*qz*qw;
    rotation_matrix(1,1) = 1 - 2*pow(qx,2) - 2*pow(qz,2);
    rotation_matrix(1,2) = 2*qy*qz - 2*qx*qw;

    rotation_matrix(2,0) = 2*qx*qz - 2*qy*qw;
    rotation_matrix(2,1) = 2*qy*qz + 2*qx*qw;
    rotation_matrix(2,2) = 1 - 2*pow(qx,2) - 2*pow(qy,2);
    
    return rotation_matrix;
}

geometry_msgs::Quaternion PointTF::Mat32Quaternion(Matrix3d matrix_in){
    geometry_msgs::Quaternion quaternion;

	double r11 = matrix_in(0, 0);
	double r12 = matrix_in(0, 1);
	double r13 = matrix_in(0, 2);
	double r21 = matrix_in(1, 0);
	double r22 = matrix_in(1, 1);
	double r23 = matrix_in(1, 2);
	double r31 = matrix_in(2, 0);
	double r32 = matrix_in(2, 1);
	double r33 = matrix_in(2, 2);
	double q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
	double q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
	double q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
	double q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
	if (q0 < 0.0) {
		q0 = 0.0;
	}
	if (q1 < 0.0) {
		q1 = 0.0;
	}
	if (q2 < 0.0) {
		q2 = 0.0;
	}
	if (q3 < 0.0) {
		q3 = 0.0;
	}
	q0 = sqrt(q0);
	q1 = sqrt(q1);
	q2 = sqrt(q2);
	q3 = sqrt(q3);
	if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
		q0 *= +1.0f;
		q1 *= ((r32 - r23) >= 0.0) ? +1.0 : -1.0;
		q2 *= ((r13 - r31) >= 0.0) ? +1.0 : -1.0;
		q3 *= ((r21 - r12) >= 0.0) ? +1.0 : -1.0;

	}
	else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
		q1 *= +1.0f;
		q0 *= ((r32 - r23) >= 0.0f) ? +1.0f : -1.0f;
		q2 *= ((r21 + r12) >= 0.0f) ? +1.0f : -1.0f;
		q3 *= ((r13 + r31) >= 0.0f) ? +1.0f : -1.0f;
	}
	else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
		q2 *= +1.0f;
		q0 *= ((r13 - r31) >= 0.0f) ? +1.0f : -1.0f;
		q1 *= ((r21 + r12) >= 0.0f) ? +1.0f : -1.0f;
		q3 *= ((r32 + r23) >= 0.0f) ? +1.0f : -1.0f;

	}
	else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
		q3 *= +1.0f;
		q0 *= ((r21 - r12) >= 0.0f) ? +1.0f : -1.0f;
		q1 *= ((r31 + r13) >= 0.0f) ? +1.0f : -1.0f;
		q2 *= ((r32 + r23) >= 0.0f) ? +1.0f : -1.0f;
	}

    double r = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q0 * q0); 

    quaternion.w = q0/r;
    quaternion.x = q1/r;
    quaternion.y = q2/r;
    quaternion.z = q3/r;

    return quaternion;
}

Matrix3d PointTF::Quaternion2Mat3(geometry_msgs::Quaternion quat_in){
    Matrix3d rotation_matrix = Matrix3d::Zero();
    double qx = quat_in.x;
    double qy = quat_in.y;
    double qz = quat_in.z;
    double qw = quat_in.w;

    double r = sqrt(pow(qx,2.0) + pow(qy,2.0) + pow(qz,2.0) + pow(qw,2.0));
    if (r!=0){
        qx = qx/r;
        qy = qy/r;
        qz = qz/r;
        qw = qw/r;
    }
    
    rotation_matrix(0,0) = 1 - 2*pow(qy,2) - 2*pow(qz,2);
    rotation_matrix(0,1) = 2*qx*qy - 2*qz*qw;
    rotation_matrix(0,2) = 2*qx*qz + 2*qy*qw;
    
    rotation_matrix(1,0) = 2*qx*qy + 2*qz*qw;
    rotation_matrix(1,1) = 1 - 2*pow(qx,2) - 2*pow(qz,2);
    rotation_matrix(1,2) = 2*qy*qz - 2*qx*qw;

    rotation_matrix(2,0) = 2*qx*qz - 2*qy*qw;
    rotation_matrix(2,1) = 2*qy*qz + 2*qx*qw;
    rotation_matrix(2,2) = 1 - 2*pow(qx,2) - 2*pow(qy,2);
    
    return rotation_matrix;
}

geometry_msgs::Transform PointTF::ReverseTF(geometry_msgs::Transform tf_)
{
    Matrix31d tf_mat_t = Matrix31d::Zero();
    tf_mat_t(0, 0) = tf_.translation.x;
    tf_mat_t(1, 0) = tf_.translation.y;
    tf_mat_t(2, 0) = tf_.translation.z;
    Matrix3d tf_mat_r = Matrix3d::Zero();
    tf_mat_r = Quaternion2Mat3(tf_.rotation);

    Matrix3d tf_mat_r_reverse = tf_mat_r.transpose();
    Matrix31d tf_mat_t_reverse = (-1) * tf_mat_r_reverse * tf_mat_t;

    geometry_msgs::Transform reverse_tf;
    reverse_tf.translation.x = tf_mat_t_reverse(0, 0);
    reverse_tf.translation.y = tf_mat_t_reverse(1, 0);
    reverse_tf.translation.z = tf_mat_t_reverse(2, 0);
    reverse_tf.rotation = Mat32Quaternion(tf_mat_r_reverse);
    
    return reverse_tf;
}