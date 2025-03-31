#include "robotarm_kinematic.h"
#include "robotarm_parameters.h"
#include <iostream>
#include <math.h>

using namespace std;

robotarm_kinematic::robotarm_kinematic(int robotarm)
{
    double default_ratio[6] = {1, 1, 1, 1, 1, 1};
    double default_min[6] = {-M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI};
    double default_max[6] = {M_PI, M_PI, M_PI, M_PI, M_PI, M_PI};

    for (int i=0; i<6; i++)
    {
        ratio[i] = default_ratio[i];
        min_limit[i] = default_min[i];
        max_limit[i] = default_max[i];
    }

    _direction = true;

    _tool_length = 0;

    robotarm_parameters::robotarm_parameters _arm_type;

    if (robotarm == 1)
    {
        _arm_type = robotarm_parameters::AR5_PARAMETERS;
    }
    else if (robotarm == 2)
    {
        _arm_type = robotarm_parameters::AR10_PARAMETERS;
    }
    else if (robotarm == 3)
    {
        _arm_type = robotarm_parameters::ZEROERR_PARAMETERS;
    }
    else if (robotarm == 4)
    {
        _arm_type = robotarm_parameters::UR5_PARAMETERS;
    }
    else if (robotarm == 5)
    {
        _arm_type = robotarm_parameters::INNFOS_GLUON_2L6_4L3_PARAMETERS;
    }
    else if (robotarm == 6)
    {
        _arm_type = robotarm_parameters::AR5_20231220_PARAMETERS;
    }
    else if (robotarm == 7)
    {
        _arm_type = robotarm_parameters::UNDERWATER_025_PARAMETERS;
    }
    else if (robotarm == 8)
    {
        _arm_type = robotarm_parameters::UR5_PARAMETERS;
    }
    else
    {
        cout << "Error: WRONG PARAMETER FILE" << endl;
    }

    for (int i=0; i<6; i++)
    {
        _a[i] = _arm_type.a[i];
        _alpha[i] = _arm_type.alpha[i];
        _d[i] = _arm_type.d[i];
        _theta[i] = _arm_type.theta[i];
    }

    _a[6] = _arm_type.a[6];
    _alpha[6] = _arm_type.alpha[6];
    _d[6] = _arm_type.d[6]+_tool_length;
    _theta[6] = _arm_type.theta[6];
}

void robotarm_kinematic::set_tool_length(double tool_length)
{
    _d[6] = _d[6]-_tool_length+tool_length;
    return;
}

void robotarm_kinematic::set_ratio(int * set_ratio)
{
    for (int i=0; i<6; i++)
    {
        ratio[i] = set_ratio[i];
    }
    return;
}

void robotarm_kinematic::set_ratio(vector<int> set_ratio)
{
    for (int i=0; i<6; i++)
    {
        ratio[i] = set_ratio[i];
    }
    return;
}

void robotarm_kinematic::set_min_max(double * min, double * max)
{
    for (int i=0; i<6; i++)
    {
        min_limit[i] = *(min+i);
        max_limit[i] = *(max+i);
        //cout << min_limit[i] << "; " << max_limit[i] << endl;
    }
    return;
}

Matrix3d robotarm_kinematic::tilde_matrix(Vector3d v)
{
    Matrix3d res;
    res <<  0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return res;
}

Matrix3d robotarm_kinematic::rotation_with_fixed_axis(Vector3d axis, double theta)
{
    Matrix3d res;

    res = cos(theta)*Matrix3d::Identity() + (1-cos(theta))*axis*axis.transpose() + sin(theta)*tilde_matrix(axis);

    return res;
}

Matrix3d robotarm_kinematic::rotation_with_x_axis(double theta)
{
    Matrix3d res;

    res <<  1, 0, 0,
            0, cos(theta), -sin(theta),
            0, sin(theta), cos(theta);

    return res;
}

Matrix3d robotarm_kinematic::rotation_with_y_axis(double theta)
{
    Matrix3d res;

    res <<  cos(theta), 0, sin(theta),
            0, 1, 0,
            -sin(theta), 0, cos(theta);

    return res;
}

Matrix3d robotarm_kinematic::rotation_with_z_axis(double theta)
{
    Matrix3d res;

    res <<  cos(theta), -sin(theta), 0,
            sin(theta), cos(theta), 0,
            0, 0, 1;

    return res;
}

Matrix3d robotarm_kinematic::rotation_eular_angle(double *theta, int *axis, int rotation_method)
{
    Matrix3d res = Matrix3d::Identity();

    if (rotation_method == 0)
    {
        for (int i=0; i<3; i++)
        {
            if (axis[i] == 1)
            {
                res = res * rotation_with_x_axis(theta[i]);
            }
            else if (axis[i] == 2)
            {
                res = res * rotation_with_y_axis(theta[i]);
            }
            else if (axis[i] == 3)
            {
                res = res * rotation_with_z_axis(theta[i]);
            }
            else
            {
                cout << "Error: wrong rotation axis!" << endl;
                return Matrix3d::Identity();
            }
        }
    }
    else if (rotation_method == 1)
    {
        for (int i=2; i>-1; i--)
        {
            if (axis[i] == 1)
            {
                res = res * rotation_with_x_axis(theta[i]);
            }
            else if (axis[i] == 2)
            {
                res = res * rotation_with_y_axis(theta[i]);
            }
            else if (axis[i] == 3)
            {
                res = res * rotation_with_z_axis(theta[i]);
            }
            else
            {
                cout << "Error: wrong rotation axis!" << endl;
                return Matrix3d::Identity();
            }
        }
    }
    else
    {
        cout << "Error: wrong rotation method!" << endl;
        return Matrix3d::Identity();
    }

    return res;
}

Vector3d robotarm_kinematic::eular_angle_from_rotation_matrix(Matrix3d rotation_matrix)
{
    return rotation_matrix.eulerAngles(2,1,0);
}

Matrix4d robotarm_kinematic::to_transform_matrix(Matrix3d rotation_matrix, Vector3d translation)
{
    Matrix4d res;

    res << rotation_matrix, translation, 0, 0, 0, 1;

    return res;
}

Matrix3d robotarm_kinematic::to_rotation_matrix(Matrix4d transform_matrix)
{
    Matrix3d res;

    res = transform_matrix.block(0, 0, 3, 3);

    return res;
}

Vector3d robotarm_kinematic::to_translation(Matrix4d transform_matrix)
{
    Vector3d res;

    res << transform_matrix(0,3), transform_matrix(1,3), transform_matrix(2,3);

    return res;
}

Isometry3d robotarm_kinematic::baseTend(const Vector6 &q)
{
    Eigen::Isometry3d transTCPInBase = Transform3D_DH(_a(0), _alpha(0), _d(0), _theta(0));
    for(int ii = 1; ii < 7; ii++)
        transTCPInBase = transTCPInBase * Transform3D_DH(_a(ii), _alpha(ii), _d(ii), _theta(ii) + q(ii-1));

    return transTCPInBase;
}

Isometry3d robotarm_kinematic::baseTend(const double *q){
    Isometry3d transTCPInBase=Transform3D_DH(_a(0),_alpha(0),_d(0),_theta(0));
    for(int ii=1;ii<7;ii++){
        transTCPInBase=transTCPInBase*Transform3D_DH(_a(ii),_alpha(ii),_d(ii),_theta(ii) + *(q+ii-1));
    }
    return transTCPInBase;
}

Isometry3d robotarm_kinematic::baseTJoint(const double *q, int joint, const Vector3d joint_position_related_to_DH)
{
    Vector6 q_vector;
    q_vector << q[0], q[1], q[2], q[3], q[4], q[5];
    Eigen::Isometry3d transTCPInBase = Transform3D_DH(_a(0), _alpha(0), _d(0), _theta(0));
    for(int ii = 1; ii < joint; ii++)
        transTCPInBase = transTCPInBase * Transform3D_DH(_a(ii), _alpha(ii), _d(ii), _theta(ii) + q_vector(ii-1));
    Eigen::Isometry3d JointInBase;
    Eigen::Matrix4d joint_position_tf;
    joint_position_tf << 1, 0, 0, joint_position_related_to_DH(0), 0, 1, 0, joint_position_related_to_DH(1), 0, 0, 1, joint_position_related_to_DH(2), 0, 0, 0, 1;
    JointInBase = transTCPInBase.matrix() * joint_position_tf;

    return JointInBase;
}

Isometry3d robotarm_kinematic::Transform3D_DH(double a,double alpha,double d, double theta){
    Isometry3d tranDH;
    tranDH(0, 0) = cos(theta);
    tranDH(1, 0) = sin(theta);
    tranDH(2, 0) = 0;
    tranDH(3, 0) = 0;

    tranDH(0, 1) = -sin(theta)*cos(alpha);
    tranDH(1, 1) = cos(theta)*cos(alpha);
    tranDH(2, 1) = sin(alpha);
    tranDH(3, 1) = 0;

    tranDH(0, 2) = sin(theta)*sin(alpha);
    tranDH(1, 2) = -cos(theta)*sin(alpha);
    tranDH(2, 2) = cos(alpha);
    tranDH(3, 2) = 0;

    tranDH(0, 3) = a*cos(theta);
    tranDH(1, 3) = a*sin(theta);
    tranDH(2, 3) = d;
    tranDH(3, 3) = 1;
    return tranDH;
}

double robotarm_kinematic::solveQ0(bool positive){
    double A=r(0,2)*_d[6]-p(0);
    double B=p(1)-r(1,2)*_d[6];
    double C=_d[4];
    double q;
    double sq = A*A+B*B-C*C;
    if(fabs(sq) < _eps){
        sq = 0.0;
    }
    if(positive){
        q=atan2(C,sqrt(sq))-atan2(A,B);
    }else{
        q=atan2(C,-sqrt(sq))-atan2(A,B);
    }
    if(q>M_PI){
        q-=M_PI*2;
    }
    if(q<-M_PI){
        q+=M_PI*2;
    }
    return q;
}

double robotarm_kinematic::solveQ4(double q0,bool positive){
    double b6=-cos(q0)*r(0,2)+sin(q0)*r(1,2);
    double q;
    double sq = 1-b6*b6;
    if(fabs(sq) < _eps){
        sq = 0.0;
    }
    if(positive){
        q=atan2(sqrt(sq),b6);
    }else{
        q=atan2(-sqrt(sq),b6);
    }
    if(q>M_PI){
        q-=M_PI*2;
    }
    if(q<-M_PI){
        q+=M_PI*2;
    }
    return q;
}

double robotarm_kinematic::solveQ5(double q0,double q4){
    double A6=-(cos(q0)*r(0,1)-sin(q0)*r(1,1))/sin(q4);
    double B6=(cos(q0)*r(0,0)-sin(q0)*r(1,0))/sin(q4);
    double q=atan2(A6,B6);
    if(q>M_PI){
        q-=M_PI*2;
    }
    if(q<-M_PI){
        q+=M_PI*2;
    }
    return q;
}

double robotarm_kinematic::solveQ123(double q0,double q4){
    double A345=-r(2,2)/sin(q4);
    double B345=(-sin(q0)*r(0,2)-cos(q0)*r(1,2))/sin(q4);
    double q=atan2(A345,B345);
    if(q>M_PI){
        q-=M_PI*2;
    }
    if(q<-M_PI){
        q+=M_PI*2;
    }
    return q;
}

double robotarm_kinematic::solveQ2(double q0,double q4,double q123,bool positive){
    double m = M(q0,q4,q123);
    double n = N(q0,q4,q123);
    double b=(m*m+n*n-_a[2]*_a[2]-_a[3]*_a[3])/(2.0*_a[2]*_a[3]);
    double sq = 1 - b*b;
    if(fabs(sq) < _eps){
        sq = 0.0;
    }
    double q;
    if(positive){
        q=atan2(sqrt(sq),b);
    }else{
        q=atan2(-sqrt(sq),b);
    }
    if(q>M_PI){
        q-=M_PI*2;
    }
    if(q<-M_PI){
        q+=M_PI*2;
    }
    return q;
}

double robotarm_kinematic::solveQ1(double q0,double q4,double q123,double q2){
    double m = M(q0,q4,q123);
    double A = _a[3]*sin(q2);
    double B = _a[3]*cos(q2)+_a[2];
    double C = m;
    double sq = A*A+B*B-C*C;
    if(fabs(sq) < _eps){
        sq = 0.0;
    }
    //TODO 我不知道为什么这里不是同时存在两种情况都有解，而是只有一种情况是有解的。
    double qp=atan2(C,sqrt(sq))-atan2(A,B);
    double qn=atan2(C,-sqrt(sq))-atan2(A,B);
    double q;
    double pzp=_a[2]*cos(qp)+_a[3]*cos(q2+qp)-_d[1]+_d[5]*cos(q123)-_d[6]*sin(q4)*sin(q123);
    double pzn=_a[2]*cos(qn)+_a[3]*cos(q2+qn)-_d[1]+_d[5]*cos(q123)-_d[6]*sin(q4)*sin(q123);
    if(fabs(pzp-p(2))<_eps){
        q = qp;
    } else if(fabs(pzn-p(2))<_eps){
        q = qn;
    } else {
        q = NAN;
        return q;
    }

    if(q>M_PI){
        q-=M_PI*2;
    }
    if(q<-M_PI){
        q+=M_PI*2;
    }
    return q;
}

double robotarm_kinematic::solveQ3(double q123,double q2,double q1){
    double q = q1+q2-q123;
    if(q>M_PI){
        q-=M_PI*2;
    }
    if(q<-M_PI){
        q+=M_PI*2;
    }
    return q;
}

double robotarm_kinematic::M(double q0, double q4,double q123){
    double m=-p(0)*sin(q0)-p(1)*cos(q0)-_d[5]*sin(q123)-_d[6]*sin(q4)*cos(q123);
    return m;
}

double robotarm_kinematic::N(double q0, double q4,double q123){
    double n=-_d[5]*cos(q123)+_d[6]*sin(q4)*sin(q123)+_d[1]+p(2);
    return n;
}

int robotarm_kinematic::solve(const Isometry3d& transform3D, vector<Vector6d> &joints, double eps){
    _eps =eps;
    Matrix<double, 8, 6> solutions;

    r=transform3D.rotation();
    p=transform3D.translation();

    double q0_p=solveQ0(true);
    double q0_n=solveQ0(false);

    double q4_pp=solveQ4(q0_p,true);
    double q4_pn=solveQ4(q0_p,false);
    double q4_np=solveQ4(q0_n,true);
    double q4_nn=solveQ4(q0_n,false);

    if(fabs(sin(q4_pp))<eps){
        cout << "Robot is in singular state q4_pp, joint0 = "
                  << q0_p/M_PI*180.0
                  << " or " << q0_n/M_PI*180.0
                  << ",joint4 = 0, 关节2,3,4,6平行"<<"\n";
        return -1;
    }
    if(fabs(sin(q4_pn))<eps){
        cout << "Robot is in singular state q4_pn, joint0 = "
                  << q0_p/M_PI*180.0
                  << " or " << q0_n/M_PI*180.0
                  << ",joint4 = 0, 关节2,3,4,6平行"<<"\n";
        return -1;
    }
    if(fabs(sin(q4_np))<eps){
        cout << "Robot is in singular state q4_np, joint0 = "
                  << q0_p/M_PI*180.0
                  << " or " << q0_n/M_PI*180.0
                  << ",joint4 = 0, 关节2,3,4,6平行"<<"\n";
        return -1;
    }
    if(fabs(sin(q4_nn))<eps){
        cout << "Robot is in singular state q4_nn, joint0 = "
                  << q0_p/M_PI*180.0
                  << " or " << q0_n/M_PI*180.0
                  << ",joint4 = 0, 关节2,3,4,6平行"<<"\n";
        return -1;
    }

    double q5_pp=solveQ5(q0_p,q4_pp);
    double q5_pn=solveQ5(q0_p,q4_pn);
    double q5_np=solveQ5(q0_n,q4_np);
    double q5_nn=solveQ5(q0_n,q4_nn);

    double q123_pp=solveQ123(q0_p,q4_pp);
    double q123_pn=solveQ123(q0_p,q4_pn);
    double q123_np=solveQ123(q0_n,q4_np);
    double q123_nn=solveQ123(q0_n,q4_nn);

    double q2_ppp=solveQ2(q0_p,q4_pp,q123_pp, true);
    double q2_ppn=solveQ2(q0_p,q4_pp,q123_pp, false);
    double q2_pnp=solveQ2(q0_p,q4_pn,q123_pn, true);
    double q2_pnn=solveQ2(q0_p,q4_pn,q123_pn, false);
    double q2_npp=solveQ2(q0_n,q4_np,q123_np, true);
    double q2_npn=solveQ2(q0_n,q4_np,q123_np, false);
    double q2_nnp=solveQ2(q0_n,q4_nn,q123_nn, true);
    double q2_nnn=solveQ2(q0_n,q4_nn,q123_nn, false);

    double q1_ppp=solveQ1(q0_p,q4_pp,q123_pp,q2_ppp);
    double q1_ppn=solveQ1(q0_p,q4_pp,q123_pp,q2_ppn);
    double q1_pnp=solveQ1(q0_p,q4_pn,q123_pn,q2_pnp);
    double q1_pnn=solveQ1(q0_p,q4_pn,q123_pn,q2_pnn);
    double q1_npp=solveQ1(q0_n,q4_np,q123_np,q2_npp);
    double q1_npn=solveQ1(q0_n,q4_np,q123_np,q2_npn);
    double q1_nnp=solveQ1(q0_n,q4_nn,q123_nn,q2_nnp);
    double q1_nnn=solveQ1(q0_n,q4_nn,q123_nn,q2_nnn);

    double q3_ppp=solveQ3(q123_pp,q2_ppp,q1_ppp);
    double q3_ppn=solveQ3(q123_pp,q2_ppn,q1_ppn);
    double q3_pnp=solveQ3(q123_pn,q2_pnp,q1_pnp);
    double q3_pnn=solveQ3(q123_pn,q2_pnn,q1_pnn);
    double q3_npp=solveQ3(q123_np,q2_npp,q1_npp);
    double q3_npn=solveQ3(q123_np,q2_npn,q1_npn);
    double q3_nnp=solveQ3(q123_nn,q2_nnp,q1_nnp);
    double q3_nnn=solveQ3(q123_nn,q2_nnn,q1_nnn);

    solutions <<
        q0_p,q1_ppp,q2_ppp,q3_ppp,q4_pp,q5_pp,
        q0_p,q1_ppn,q2_ppn,q3_ppn,q4_pp,q5_pp,
        q0_p,q1_pnp,q2_pnp,q3_pnp,q4_pn,q5_pn,
        q0_p,q1_pnn,q2_pnn,q3_pnn,q4_pn,q5_pn,
        q0_n,q1_npp,q2_npp,q3_npp,q4_np,q5_np,
        q0_n,q1_npn,q2_npn,q3_npn,q4_np,q5_np,
        q0_n,q1_nnp,q2_nnp,q3_nnp,q4_nn,q5_nn,
        q0_n,q1_nnn,q2_nnn,q3_nnn,q4_nn,q5_nn;
//    cout << "ik solutions: " << endl;
//    cout << solutions << endl;

    for(int ii=0;ii<8;ii++){
         if(!isnan(solutions(ii,0)) &&
            !isnan(solutions(ii,1)) &&
            !isnan(solutions(ii,2)) &&
            !isnan(solutions(ii,3)) &&
            !isnan(solutions(ii,4)) &&
            !isnan(solutions(ii,5)) )
         {
             joints.push_back(solutions.block(ii,0,1,6).transpose());
             //joints.push_back(solutionsPlus.block(ii,0,1,6));
             //joints.push_back(solutionsMinus.block(ii,0,1,6));
             //auto q = solutions.block(ii,0,1,6);
             //LOG_INFO << "Index "<<ii <<": "<<q/M_PI*180.0<<"\n";
             //baseTend(q);
         }
    }

    for (int i=1; i<=10; i++)
    {
        //cout << i << ": " << endl;
        Matrix<double, 8, 6> solutionsPlus, solutionsMinus;
        solutionsPlus <<
            q0_p,q1_ppp,q2_ppp,q3_ppp,q4_pp,q5_pp + 2*i*M_PI,
            q0_p,q1_ppn,q2_ppn,q3_ppn,q4_pp,q5_pp + 2*i*M_PI,
            q0_p,q1_pnp,q2_pnp,q3_pnp,q4_pn,q5_pn + 2*i*M_PI,
            q0_p,q1_pnn,q2_pnn,q3_pnn,q4_pn,q5_pn + 2*i*M_PI,
            q0_n,q1_npp,q2_npp,q3_npp,q4_np,q5_np + 2*i*M_PI,
            q0_n,q1_npn,q2_npn,q3_npn,q4_np,q5_np + 2*i*M_PI,
            q0_n,q1_nnp,q2_nnp,q3_nnp,q4_nn,q5_nn + 2*i*M_PI,
            q0_n,q1_nnn,q2_nnn,q3_nnn,q4_nn,q5_nn + 2*i*M_PI;
        solutionsMinus <<
            q0_p,q1_ppp,q2_ppp,q3_ppp,q4_pp,q5_pp - 2*i*M_PI,
            q0_p,q1_ppn,q2_ppn,q3_ppn,q4_pp,q5_pp - 2*i*M_PI,
            q0_p,q1_pnp,q2_pnp,q3_pnp,q4_pn,q5_pn - 2*i*M_PI,
            q0_p,q1_pnn,q2_pnn,q3_pnn,q4_pn,q5_pn - 2*i*M_PI,
            q0_n,q1_npp,q2_npp,q3_npp,q4_np,q5_np - 2*i*M_PI,
            q0_n,q1_npn,q2_npn,q3_npn,q4_np,q5_np - 2*i*M_PI,
            q0_n,q1_nnp,q2_nnp,q3_nnp,q4_nn,q5_nn - 2*i*M_PI,
            q0_n,q1_nnn,q2_nnn,q3_nnn,q4_nn,q5_nn - 2*i*M_PI;

        for(int ii=0;ii<8;ii++)
        {
             if(!isnan(solutionsPlus(ii,0)) &&
                !isnan(solutionsPlus(ii,1)) &&
                !isnan(solutionsPlus(ii,2)) &&
                !isnan(solutionsPlus(ii,3)) &&
                !isnan(solutionsPlus(ii,4)) &&
                !isnan(solutionsPlus(ii,5)) )
             {
                 joints.push_back(solutionsPlus.block(ii,0,1,6).transpose());
                 //joints.push_back(solutionsMinus.block(ii,0,1,6));
                 //auto q = solutions.block(ii,0,1,6);
                 //LOG_INFO << "Index "<<ii <<": "<<q/M_PI*180.0<<"\n";
                 //baseTend(q);
             }

             if(!isnan(solutionsMinus(ii,0)) &&
                !isnan(solutionsMinus(ii,1)) &&
                !isnan(solutionsMinus(ii,2)) &&
                !isnan(solutionsMinus(ii,3)) &&
                !isnan(solutionsMinus(ii,4)) &&
                !isnan(solutionsMinus(ii,5)) )
             {
                 //joints.push_back(solutionsPlus.block(ii,0,1,6));
                 joints.push_back(solutionsMinus.block(ii,0,1,6).transpose());
                 //auto q = solutions.block(ii,0,1,6);
                 //LOG_INFO << "Index "<<ii <<": "<<q/M_PI*180.0<<"\n";
                 //baseTend(q);
             }
        }
    }

    //LOG(INFO) <<"\nSolutions:\n\n"<<solutions/M_PI*180.0;


    //Solutions are empty.
    if(joints.empty()){
        return -2;
    }

    return 0;
}

int robotarm_kinematic::getQ(const Isometry3d& transform3D, double *cur_Q, double *targetQ, double threahold)
{
    double norm = 2 * sqrt(ratio[0]+ratio[1]+ratio[2]+ratio[3]+ratio[4]+ratio[5]) * M_PI, _temnorm = 0;
    int choseNum = 0;
    double target_save[6] = {0, 0, 0, 0, 0, 0};

    //Get all solutions.
    vector<Vector6d> solutions;
    if(solve(transform3D, solutions) < 0)
    {
        cout << prefix << "NO IK solutions." << endl;
        return -1;
    }
    //Find bast solution.
    for(int i = 0, solNum = solutions.size(); i < solNum; ++i){
        if(norm > (_temnorm = norm2(solutions[i], cur_Q))){
            //cout << i << ": " << solutions[i] << "; " << _temnorm << endl;
            norm = _temnorm;
            choseNum = i;
        }
    }
    if(norm > 2 * sqrt(ratio[0]+ratio[1]+ratio[2]+ratio[3]+ratio[4]+ratio[5]) * M_PI)
    {
        cout << prefix << "Solution is out of joint limit." << endl;
        return -1;
    }

    for (int i=0; i<6; i++)
    {
        *(target_save+i) = solutions[choseNum][i];
    }
    //cout << "Target: " << solutions[choseNum] << endl;

    //Deal with the case when got a discontinuous solution.
    if(filter_joints(cur_Q, target_save, threahold)){
        cout << prefix << "Get discontinuous solution(threshold " << threahold << ")." <<  endl;
        return -2;
    }

    for (int i=0; i<6; i++)
    {
        *(targetQ+i) = solutions[choseNum][i];
    }
    //cout << "Target: " << solutions[choseNum] << endl;

    return 0;
}

int robotarm_kinematic::getQ(double * xyzrpy, double *cur_Q, double *targetQ, double threahold)
{
    double x = *xyzrpy;
    double y = *(xyzrpy+1);
    double z = *(xyzrpy+2);
    double roll = *(xyzrpy+3);
    double pitch = *(xyzrpy+4);
    double yaw = *(xyzrpy+5);

    Isometry3d tm = Isometry3d::Identity();

    tm.translate(Vector3d(x,y,z));
    tm.rotate(AngleAxisd(yaw, Vector3d::UnitZ()) * AngleAxisd(pitch, Vector3d::UnitY()) * AngleAxisd(roll, Vector3d::UnitX()));

    return getQ(tm, cur_Q, targetQ, threahold);
}

int robotarm_kinematic::filter_joints(const double *cQ, const double *tQ, double threahold)
{
    Vector6d offset;
    for (int i=0; i<6; i++)
    {
        offset[i] = *(cQ+i) - *(tQ+i);
    }

    for(int i = 0; i < 6; ++i)
    {
        //cout << i << ": " << offset[i] << endl;
        if(fabs(offset[i]) >= threahold)
        {
            return -1;
        }
    }
    return 0;
}

int robotarm_kinematic::filter_joints(const Vector6 &cQ, const Vector6 &tQ, double threshold)
{
    Vector6 offset = cQ - tQ;
    for(int i = 0; i < 6; ++i){
        if(fabs(offset[i]) >= threshold){
            std::cout << "Current: " << cQ << " Target: " << tQ << std::endl;
            return -1;
        }
    }

    return 0;
}

double robotarm_kinematic::norm2(Vector6d q1, double * q2)
//inline double AR10_2_kinematics::norm2(Vector6d q1, Vector6d q2)
{
    double q[6];
    for (int i=0; i<6; i++)
    {
        if (q1[i] >= min_limit[i] and q1[i] <= max_limit[i])
        {
            q[i] = *(q2+i) - q1[i];
            //cout << i << ": " << q1[i] << "; (" << min_limit[i] << ", " << max_limit[i] << ")" << endl;
        }
        else
        {
            q[i] = 3 * sqrt(ratio[0]+ratio[1]+ratio[2]+ratio[3]+ratio[4]+ratio[5]) * M_PI;
            //cout << i << ": " << q[i] << endl;
        }
    }

    double res = 0;
    for (int i=0; i < 6; i++)
    {
        res = pow(q[i],2) * ratio[i] + res;
    }
    return sqrt(res);
}

bool robotarm_kinematic::getQ_num(const Eigen::Isometry3d &transform3D, const Vector6 &q, Vector6 &targetQ, int & Errorcode,
                                  double threshold)
{
    /**
     * inverse kinematics algorithm with damped least-squares method
     */
    Vector6 errorVector;
    Vector6 state = q;
    Vector6 nextState = q;
    Vector6 jntVel = Vector6::Zero(jntVel.rows(),jntVel.cols());
    double lambda = 0.1; //init lambda
    double lambdaFactor = 10;
    int maxIterations = 100;
    double dt=1.0;

//    std::fstream outstream,staterecord;
//    outstream.open("errorvector.txt",std::ios::out);
//    staterecord.open("state.txt",std::ios::out);

    calculateTaskSpaceError(q,transform3D,errorVector);
//    outstream << errorVector[0] << " " <<  errorVector[1] << " " << errorVector[2] << " " << errorVector[3] << " " << errorVector[4] << " " << errorVector[5] << " " << errorVector.norm() << endl;
    if(errorVector.norm() < _eps){
        targetQ = q;
        //Errorcode =ROBOT_NO_ERROR;
        return true;
    }

    double lasterrornorm = 1000000;
    for (int i = 0; i < maxIterations; ++i) {
        if(lambdaFactor > 1){
            lambda = findOptimalLambda(state,transform3D,jntVel,lambdaFactor,dt,lambda);
        }

        performIKAStep(state,errorVector,jntVel, lambda);
        nextState+= jntVel*dt;
        state = nextState;

        calculateTaskSpaceError(state,transform3D,errorVector);

//        outstream << errorVector[0] << " " <<  errorVector[1] << " " << errorVector[2] << " " << errorVector[3] << " " << errorVector[4] << " " << errorVector[5] << " " << errorVector.norm() << " " << dt <<" " << lambda << endl;
//        cout << "state: " << state[0] << " " <<  state[1] << " " << state[2] << " " << state[3] << " " << state[4] << " " << state[5] << endl;

        if (fabs(lasterrornorm - errorVector.norm()) <= 1e-9){
            dt = dt*0.95;
//            if (dt<0.5) dt = (rand()%1000+1)/1000.0;
            state = q;
            nextState = q;
            lambdaFactor = 10;
            lambda = 0.1;
            jntVel = Vector6::Zero(jntVel.rows(),jntVel.cols());
        }
        lasterrornorm = errorVector.norm();

        if(errorVector.norm() < 1e-06){

            double min_p[6] = {-M_PI, -M_PI, -M_PI*2, -M_PI, -M_PI*3, -M_PI*2};
            double max_p[6] = {M_PI, M_PI, M_PI*2, M_PI, M_PI*3, M_PI*2};
            for (int j = 0; j < 6; ++j) {
                while(state(j) > max_p[j] or state(j) < min_p[j])
                {
                    if (state(j) > max_p[j]){
                        state(j) += -2*M_PI;
                    }

                    if (state(j) < min_p[j]) {
                        state(j) += 2 * M_PI;
                    }
                }
            }

//            cout << "j_state: " << state[0] << "," << state[1] << "," << state[2] << "," << state[3] << "," << state[4] << "," << state[5] << endl;
            if(filter_joints(q, state, threshold)){
                //Errorcode = ROBOT_KINEMATICS_DISCOUNTINUOUS;
                cout << "threshold " << threshold << " error: " << state[0] << "," << state[1] << "," << state[2] << "," << state[3] << "," << state[4] << "," << state[5] << endl;
                return false;
            }

            targetQ = state;
            //Errorcode = ROBOT_NO_ERROR;
            return true;
        }
    }

    targetQ = q;
    //Errorcode = ROBOT_KINEMATICS_NONUMSOLUTION;
    return false;
}

bool robotarm_kinematic::getQ_num(const Eigen::Isometry3d &transform3D, double *q, double *target_q, int &Errorcode, double threshold)
{
    Vector6 q_vector, target_q_vector;
    q_vector << q[0], q[1], q[2], q[3], q[4], q[5];
    bool res = getQ_num(transform3D, q_vector, target_q_vector, Errorcode, threshold);
    for (int i=0; i<6; i++)
    {
        target_q[i] = target_q_vector(i);
    }
    return res;
}

void robotarm_kinematic::performIKAStep(const Vector6 &q, const Vector6 &error_vector, Vector6 &joint_vel, double lambda,
                                    const Vector6 &target_vel)
{
    /*
     * target: update joint velocities to update next joint states.
     */
    Eigen::Matrix<double,6,6> currentjacobian, inversejacobian, identitymatrix;
    getGeometricJacobian(q,currentjacobian);


    Eigen::Matrix<double, 6, 1> scaled_error_vector;
    scaled_error_vector = _weight_matrix * (error_vector.transpose());

    identitymatrix.setIdentity();

    // 使用迭代法求得的lambda
    inversejacobian = currentjacobian.transpose() * (((currentjacobian * currentjacobian.transpose()) + lambda * lambda * identitymatrix).inverse());
    //使用误差作为lambda
//    double l = 0.5*scaled_error_vector.transpose()*scaled_error_vector;
//    inversejacobian = currentjacobian.transpose() * (((currentjacobian * currentjacobian.transpose()) + l * identitymatrix).inverse());
    //直接求解几何雅可比的逆
//    calculateInverseGeometricJacobian(q);
//    inversejacobian = _invJacobian;

    joint_vel = inversejacobian * (target_vel.transpose() + scaled_error_vector);
}

double robotarm_kinematic::findOptimalLambda(const Vector6 &initialState, const Eigen::Isometry3d &targetTform,
                                              const Vector6 &jointVel, double lambdaFactor, double dt, double lambda)
{
    if (lambdaFactor <= 1){
       // std::cerr << "lambda factor must be higher than 1 to determine optimal lambda. Returning provided lambda" << std::endl;
        return lambda;
    }

    Vector6 lambdaState = initialState;
    Vector6 lambdaStateVel = initialState;
    Vector6 initialError;
    Vector6 lambdaStateError;

    calculateTaskSpaceError(initialState,targetTform,initialError);
    double initError = initialError.norm();
    performIKAStep(initialState,initialError,lambdaStateVel,lambda);
    lambdaState = initialState + lambdaStateVel*dt;

    calculateTaskSpaceError(lambdaState, targetTform, lambdaStateError);
    double lambdaError = lambdaStateError.norm();

    if (lambda <= 1e-6){
        if(lambdaError <= initError){
            return lambda;
        }
        else{ // lambdaError > initError
            for (int i = 0; i < 100; ++i) {

                lambda *= lambdaFactor;

                performIKAStep(initialState, initialError, lambdaStateVel, lambda);
                lambdaState = initialState + lambdaStateVel*dt;

                calculateTaskSpaceError(lambdaState, targetTform, lambdaStateError);

                if(lambdaStateError.norm() <= initError) return lambda;
            }
        }
    }
    else{
        double factorLambda = lambda/lambdaFactor;

        performIKAStep(initialState,initialError, lambdaStateVel, factorLambda);
        lambdaState = initialState + lambdaStateVel*dt;

        calculateTaskSpaceError(lambdaState, targetTform, lambdaStateError);
        double factorLambdaError = lambdaStateError.norm();

        if(factorLambdaError <= initError){
            return factorLambda;
        }
        else if( factorLambdaError > initError && lambdaError <= initError ){
            return lambda;
        }
        else{ // factorLambdaError > initError && lambdaError > initError
            for (int i = 0; i < 100; ++i) {

                lambda *= lambdaFactor;

                performIKAStep(initialState, initialError, lambdaStateVel, lambda);
                lambdaState = initialState + lambdaStateVel*dt;

                calculateTaskSpaceError(lambdaState, targetTform, lambdaStateError);
                double testerror = lambdaStateError.norm();

                if(lambdaStateError.norm() <= initError) return lambda;
            }
        }
    }

    return lambda;
}

Eigen::Matrix<double, 6, 6> robotarm_kinematic::getGeometricJacobian(const Vector6 &q)
{
    calculateGeometricJacobian(q);
    return _jacobian;
}

void robotarm_kinematic::getGeometricJacobian(const Vector6 &q, Eigen::Matrix<double, 6, 6> &jacobian)
{
    calculateGeometricJacobian(q);
    jacobian = _jacobian;
}

robotarm_kinematic::Vector6 robotarm_kinematic::JntVeltoCartVel(const Vector6 &q, const Vector6 &qdot)
{
    Vector6 cartvel;
    JntVeltoCartVel(q, qdot, cartvel);

    return cartvel;
}

void robotarm_kinematic::JntVeltoCartVel(const Vector6 &q, const Vector6 &qdot, Vector6 &cartvel)
{
    calculateGeometricJacobian(q);

    Eigen::Matrix<double, 6, 1> vel;
    vel = _jacobian * (qdot.transpose());

    cartvel = vel.transpose();
}

robotarm_kinematic::Vector6 robotarm_kinematic::CartVeltoJntVel(const Vector6 &q, const Vector6 &cartvel)
{
    Vector6 qdot;
    CartVeltoJntVel(q, cartvel, qdot);
    return qdot;
}

void robotarm_kinematic::CartVeltoJntVel(const Vector6 &q, const Vector6 &cartvel, Vector6 &qdot)
{
    calculateInverseGeometricJacobian(q);
    Eigen::Matrix<double, 6, 1> qvel;
    qvel = _invJacobian*(cartvel.transpose());
    qdot = qvel.transpose();
}

void robotarm_kinematic::calculateKinematicChain(const Vector6 &q)
{
    calculateKinematicTransformations(q);

    _chained_transformations[0] = _transformations[0];

    for (int i = 1; i < (int)_transformations.size(); ++i)
    {
        _chained_transformations[i] = _chained_transformations[i - 1] * _transformations[i];
    }
}

void robotarm_kinematic::calculateKinematicTransformations(const Vector6 &q)
{
    for (int i = 0; i < 6; ++i)
    {
        _transformations[i] = Transform3D_DH(_a(i), _alpha(i), _d(i), _theta(i));

        cout << 1;

        _transformations[i].rotate(Eigen::AngleAxisd(q(i),Eigen::Vector3d::UnitZ()));
    }
}

void robotarm_kinematic::calculateGeometricJacobian(const Vector6 &q)
{
    calculateKinematicTransformations(q);
    calculateKinematicChain(q);

    _jacobian.setZero();

    Eigen::Vector3d ee_position, axis;
    ee_position = _chained_transformations.back().translation();
    int k =0;

    std::vector<bool> jointaxis = {false, false, true};

    for (int i = 0; i < (int)_transformations.size(); ++i)
    {
        Eigen::Matrix<double, 4, 4> tformmatrix;
        tformmatrix.setIdentity();
        tformmatrix.block<3,3>(0,0) = _chained_transformations[i].rotation();
        tformmatrix.block<3,1>(0,3) = _chained_transformations[i].translation();

        axis = tformmatrix.block<3, 1>(0, 0) * jointaxis[0] + tformmatrix.block<3, 1>(0, 1) * jointaxis[1] +
               tformmatrix.block<3, 1>(0, 2) * jointaxis[2];

        _jacobian.block<3, 1>(0, k) = axis.cross(ee_position - tformmatrix.block<3, 1>(0, 3));  // J_Li
        _jacobian.block<3, 1>(3, k) = axis;                                                                     // J_Oi
        ++k;
    }
}

void robotarm_kinematic::calculateInverseGeometricJacobian(const Vector6 &q)
{
    calculateGeometricJacobian(q);
    _invJacobian.setZero();
    bool useSVD = true;
    Eigen::MatrixXd jacobian, inverse_jacobian;
    jacobian = _jacobian;
    if (useSVD)
    {
        Eigen::MatrixXd v  = Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd u  = Eigen::MatrixXd::Identity(6, 6);
        Eigen::VectorXd id = Eigen::VectorXd::Zero(6);
        Eigen::MatrixXd ident  = Eigen::MatrixXd::Identity(6, 6);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
        u=svd.matrixU();
        v=svd.matrixV();
        id=svd.singularValues();
        ident=id.asDiagonal().inverse();
        for (int i = 0; i < 6; ++i) {
            if(id(i)<0.05)
            {
                ident(i,i)=0;
            }
        }

        inverse_jacobian= v*ident*u.transpose();
    }
    else
    {
        double jacobianLambda=0.05;
        Eigen::MatrixXd identity_matrix  = Eigen::MatrixXd::Identity(6, 6);
        inverse_jacobian = jacobian.transpose() * (jacobian * jacobian.transpose() + jacobianLambda * jacobianLambda * identity_matrix).inverse();
    }
    _invJacobian = inverse_jacobian;
}

void robotarm_kinematic::calculateTaskSpaceError(const Vector6 &q, const Eigen::Isometry3d &transform3D,
                                                  Vector6 &error_vector)
{
    Eigen::Isometry3d tform = baseTend(q);
    Eigen::Quaterniond current_quaternion(tform.rotation());
    Eigen::Quaterniond target_quaternion(transform3D.rotation());

    Eigen::Quaterniond rotated_quaternion;
    Eigen::Vector3d position_error;

    position_error = transform3D.translation() - tform.translation();
    rotated_quaternion = target_quaternion * current_quaternion.inverse();

    error_vector.block<1,3>(0,0) = position_error;
    error_vector.block<1,3>(0,3) = sgn<double>(rotated_quaternion.w()) * rotated_quaternion.vec();
}

bool robotarm_kinematic::innerWorkspace(const Eigen::Isometry3d &target)
{
    // outer workspace
    Eigen::Vector3d point = target.translation();
    double boundary = (_a(2)+_a(3)+_d(5))*(_a(2)+_a(3)+_d(5))+(_d(4)+_d(6))*(_d(4)+_d(6));
    double dist = (point[2]-_d(1))*(point[2]-_d(1)) + point[0]*point[0] + point[1]*point[1];
    if ((sqrt(dist) - sqrt(boundary)) > 0.01) return false;


    // inner limit
    double dist2 = point[0]*point[0] + point[1]*point[1];
    if ( (_d(4)-_d(6))*(_d(4)-_d(6)) > dist2) return false;

    return true;
}
