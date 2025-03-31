#ifndef ROBOTARM_KINEMATIC_H
#define ROBOTARM_KINEMATIC_H

#include <Eigen/Eigen>
#include <vector>

using namespace std;
using namespace Eigen;

class robotarm_kinematic
{
public:

    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    using Vector6 = Eigen::Matrix<double, 1, 6>;
    using Tranform44 = Eigen::Isometry3d;

    /**
     * @param robotarm 机械臂型号
    **/
    robotarm_kinematic(int robotarm);

    /**
     * @brief tilde_matrix 求向量的叉乘矩阵
     * @param v 叉乘矩阵对应的向量
     * @return 叉乘矩阵
     */
    Matrix3d tilde_matrix(Vector3d v);

    /**
     * @brief rotation_with_fixed_axis 求绕固定轴旋转的旋转矩阵
     * @param axis 轴向量
     * @param theta 旋转角度，单位rad
     * @return 旋转对应的旋转矩阵
     */
    Matrix3d rotation_with_fixed_axis(Vector3d axis, double theta);

    /**
     * @brief rotation_with_x_axis 求绕x轴旋转的旋转矩阵
     * @param theta 旋转角度，单位rad
     * @return 旋转对应的旋转矩阵
     */
    Matrix3d rotation_with_x_axis(double theta);

    /**
     * @brief rotation_with_y_axis 求绕y轴旋转的旋转矩阵
     * @param theta 旋转角度，单位rad
     * @return 旋转对应的旋转矩阵
     */
    Matrix3d rotation_with_y_axis(double theta);

    /**
     * @brief rotation_with_z_axis 求绕z轴旋转的旋转矩阵
     * @param theta 旋转角度，单位rad
     * @return 旋转对应的旋转矩阵
     */
    Matrix3d rotation_with_z_axis(double theta);

    /**
     * @brief rotation_eular_angle 求欧拉角描述的旋转矩阵
     * @param theta double[3],旋转角度，单位rad
     * @param axis int[3]，每个旋转角度对应的旋转轴，1为x轴，2为y轴，3为z轴
     * @param rotation_method 内旋（自身坐标系）或者外旋（世界坐标系），0为内旋，1为外旋，默认内旋
     * @return 旋转对应的旋转矩阵
     */
    Matrix3d rotation_eular_angle(double *theta, int *axis, int rotation_method=0);

    Vector3d eular_angle_from_rotation_matrix(Matrix3d rotation_matrix);

    /**
     * @brief to_transform_matrix 将旋转矩阵和平移向量变换为变换矩阵
     * @param rotation_matrix 旋转矩阵
     * @param translation 平移向量
     * @return 变换矩阵
     */
    Matrix4d to_transform_matrix(Matrix3d rotation_matrix, Vector3d translation);

    /**
     * @brief to_rotation_matrix 从变换矩阵中得到旋转矩阵
     * @param transform_matrix 变换矩阵
     * @return 旋转矩阵
     */
    Matrix3d to_rotation_matrix(Matrix4d transform_matrix);

    /**
     * @brief to_translation 从变换矩阵中得到平移向量
     * @param transform_matrix 变换矩阵
     * @return 平移向量
     */
    Vector3d to_translation(Matrix4d transform_matrix);

    /**
     * @brief 运动学正解
     * @param q double[6]数组，各关节角度，单位rad
    **/
    Isometry3d baseTend(const double *q);

    Isometry3d baseTend(const Vector6 &q);

//    void isometry3dTpose()


    /**
     * @brief 关节在机械臂基座下的位置
     * @param q double[6]数组，各关节角度，单位rad
     * @param joint 关节1为1,以此类推
     * @param joint_position_related_to_DH 关节i在第i坐标系下的坐标
     * @return
     */
    Isometry3d baseTJoint(const double *q, int joint, const Vector3d joint_position_related_to_DH);

    /**
     * @brief 运动学逆解，已知末端到基座坐标系的转换矩阵和各关节当前位置
     * @param transform3D 末端到基座坐标系的转换矩阵
     * @param cur_Q double[6]数组，各关节当前角度，单位rad
     * @param targetQ double[6]数组，各关节求出的目标角度，单位rad
     * @param threahold 关节角变化的最大幅度，超过该幅度会报错，默认值6.28
    **/
    int getQ(const Isometry3d& transform3D, double *cur_Q, double *targetQ, double threahold = 6.28);

    /**
     * @brief 运动学逆解，已知末端在基座坐标系下的坐标和欧拉角，以及各关节当前位置
     * @param xyzrpy double[6]数组，分别是x，y，z，roll，pitch，yaw
     * 其他参数见上
    **/
    int getQ(double * xyzrpy, double *cur_Q, double *targetQ, double threahold = 6.28);

    /**
     * @brief 改变属具长度
     * @param tool_length 属具的长度，单位m
    **/
    void set_tool_length(double tool_length);

    /**
     * @brief 改变各关节角比重
     * @param set_ratio int[6]数组，数值越大关节越不容易运动
    **/
    void set_ratio(int * set_ratio);

    /**
     * @brief 改变各关节角比重
     * @param set_ratio vector<int>，数值越大关节越不容易运动
    **/
    void set_ratio(vector<int> set_ratio);

    /**
     * @brief 改变各关节角度最大和最小限制
     * @param min double[6]数组，各关节最小角度限制
     * @param max double[6]数组，各关节最大角度限制
    **/
    void set_min_max(double * min, double * max);

    /**
     * @brief 设置关节旋转正方向
     * @param direciton z轴由盖指向谐波为true，由谐波指向盖为false
     */
    void set_joint_positive_direction(bool direciton);

    /**
     * @brief Numerical Inverse kinematics 根据笛卡尔空间位姿和当前关节角度值求解关节角度值，使用数值解迭代的方法求解
     * @param transform3D 笛卡尔空间的目标位姿 in transformation matrix
     * @param q 机械臂当前的关节角度值
     * @param[out] targetQ 返回目标关节角度
     * @param threshold 阈值，和当前的关节角度最大差值
     * @return ERRORCODE 错误码，正确时，返回为0，错误时为非零值
     */
    bool getQ_num(const Eigen::Isometry3d &transform3D, const Vector6 &q, Vector6 &targetQ, int & Errorcode, double threshold = 6.28);

    bool getQ_num(const Eigen::Isometry3d &transform3D, double * q, double * target_q, int & Errorcode, double threshold = 6.28);


    /**
     * @brief 获取基于当前关节角度值的几何雅可比矩阵[反应关节速度和笛卡尔空间速度的映射关系]
     * @param q 机械臂当前的关节角度值
     * @param[out] jacobian 6*6的几何雅可比矩阵
     */
    void getGeometricJacobian(const Vector6 &q, Eigen::Matrix<double, 6, 6> &jacobian);

    /**
     * @brief 获取基于当前关节角度值的几何雅可比矩阵[反应关节速度和笛卡尔空间速度的映射关系]
     * @param q 机械臂当前的关节角度值
     * @return 6*6的几何雅可比矩阵
     */
    Eigen::Matrix<double, 6, 6> getGeometricJacobian(const Vector6 &q);

    /**
     * @brief 根据关节角度值和关节速度值计算末端笛卡尔空间速度值
     * @param q 机械臂当前的关节角度值
     * @param qdot 机械臂当前的关节速度值
     * @param cartvel[out] 机械臂当前笛卡尔空间速度值
     */
    void JntVeltoCartVel(const Vector6 &q, const Vector6 &qdot, Vector6 &cartvel);

    /**
     * @brief 根据关节角度值和关节速度值计算末端笛卡尔空间速度值
     * @param q 机械臂当前的关节角度值 rad
     * @param qdot 机械臂当前的关节速度值
     * @return 机械臂当前笛卡尔空间速度值
     */
    Vector6 JntVeltoCartVel(const Vector6 &q, const Vector6 &qdot);

    /**
     * @brief 根据关节角度值和笛卡尔空间速度值计算关节速度值
     * @param q 机械臂当前的关节角度值 rad
     * @param cartvel 机械臂当前笛卡尔空间速度值
     * @param[out] qdot 机械臂当前的关节速度值
     */
    void CartVeltoJntVel(const Vector6 &q, const Vector6 &cartvel, Vector6 &qdot);

    /**
     * @brief 根据关节角度值和笛卡尔空间速度值计算关节速度值
     * @param q 机械臂当前的关节角度值 rad
     * @param cartvel 机械臂当前笛卡尔空间速度值
     * @return 机械臂当前的关节速度值
     */
    Vector6 CartVeltoJntVel(const Vector6 &q, const Vector6 &cartvel);

    /**
     * @brief 判断目标点位是否在机械臂可达范围内
     * @param target
     * @return 可达为true,不可达为false
     */
    bool innerWorkspace(const Eigen::Isometry3d &target);


private:
    Vector7d _a;
    Vector7d _alpha;
    Vector7d _d;
    Vector7d _theta;

    Matrix3d r;
    Vector3d p;

    double _eps =  1e-6;
    double _tool_length;

    double ratio[6] = {1, 1, 1, 1, 1, 1};

    double min_limit[6] = {-M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI};
    double max_limit[6] = {M_PI, M_PI, M_PI, M_PI, M_PI, M_PI};

    bool _direction = true;

    Isometry3d Transform3D_DH(double a,double alpha,double d, double theta);

    double solveQ0(bool positive);
    double solveQ4(double q0,bool positive);
    double solveQ5(double q0,double q4);
    double solveQ123(double q0,double q4);
    double solveQ2(double q0,double q4,double q123,bool positive);
    double solveQ1(double q0,double q4,double q123,double q2);
    double solveQ3(double q123,double q2,double q1);
    double M(double q0, double q4,double q123);
    double N(double q0, double q4,double q123);

    const std::string prefix = "IK error: ";

    int solve(const Isometry3d& transform3D, std::vector<Vector6d>& joints, double eps=1e-10);

    double norm2(Vector6d q1, double * q2);

    int filter_joints(const double *cur_Q, const double *targetQ, double threahold = 6.28);
    int filter_joints(const Vector6 &cQ, const Vector6 &tQ, double threahold = 6.28);

    /**
     * @brief 计算从基座到各关节的转换矩阵存于成员变量 _chained_transformations
     * @param q current joint values
     */
    void calculateKinematicChain(const Vector6 &q);

    /**
     * @brief  从关节i-1到关节i的转换矩阵 存于成员变量 _transformations
     * @param q current joint values
     */
    void calculateKinematicTransformations(const Vector6 &q);

    /**
     * @brief calculate geometric Jacobian stored in member variable _jacobian
     * @param q current joint values
     */
    void calculateGeometricJacobian(const Vector6 &q);

    /**
     * @brief calculate inverse geometric Jacobian stored in member variable _invJacobian
     * @param q current joint values
     */
    void calculateInverseGeometricJacobian(const Vector6 &q);

    void performIKAStep(const Vector6 &q, const Vector6 &error_vector, Vector6 &joint_vel, double lambda, const Vector6 &target_vel = Eigen::Matrix<double, 1, 6>::Zero(1, 6));

    double findOptimalLambda(const Vector6 &initialState, const Eigen::Isometry3d &targetTform, const Vector6 &jointVel, double lambdaFactor, double dt, double lambda);

    void calculateTaskSpaceError(const Vector6 &q, const Eigen::Isometry3d &transform3D, Vector6 &error_vector);

//    vector<Isometry3d> _chained_transformations; //从基座到各关节的转换矩阵
//    vector<Isometry3d> _transformations; // 从关节i-1到关节i的转换矩阵
//    Eigen::Matrix<double, 6, 6> _jacobian, _invJacobian; //几何雅可比矩阵和其逆矩阵
//    Eigen::Matrix<double, 6, 6> _weight_matrix; //对角矩阵，求解权重系数；

    std::vector<Tranform44> _chained_transformations; //从基座到各关节的转换矩阵
    std::vector<Tranform44> _transformations; // 从关节i-1到关节i的转换矩阵
    Eigen::Matrix<double, 6, 6> _jacobian, _invJacobian; //几何雅可比矩阵和其逆矩阵
    Eigen::Matrix<double, 6, 6> _weight_matrix; //对角矩阵，求解权重系数；

    template<typename T>
    static int sgn(const T val) {
        return (T(0) < val) - (val < T(0));
    };
};

#endif
