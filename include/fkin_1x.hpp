#ifndef FKIN_H_INCLUDED
#define FKIN_H_INCLUDED

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/QR>    
#include <algorithm>    // std::max
#include <random>

//#define M_PI 3.14159265
#define MAX_ITER 5000

class PlanarLink{
public:
    PlanarLink(float linkLength, float angle);
    Eigen::Matrix3f getTransform(float angle);
    void setAngle(float angle);    
    float getAngle();
    float getLength();
protected:
    float linkLength,angle;

};


class Planar3Robot{
public:
    Planar3Robot();
    int addLink(PlanarLink in);
    int forwardKin(std::vector<float> &angles, Eigen::Matrix3f &T);
    int inverseKin(std::vector<float> &angles, Eigen::Vector3f des);
    int getDOF();
    void TMat2pos(Eigen::Matrix3f T, float *pos);
    Eigen::Vector3f TMat2vec(Eigen::Matrix3f T);
    std::vector<std::vector<float > > getMultipleSolutions(Eigen::Vector3f desired_pose,int n_solutions);    
    int insertIfUnique(std::vector<std::vector<float > > &sols, std::vector<float> s);
protected:
    Eigen::Matrix3Xf getNumericalJacobian(std::vector<float> angles);
    std::vector<PlanarLink> links;
    float constrainAngle(float x);    
};
#endif