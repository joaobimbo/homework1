#include <fkin_1x.hpp>
#include <gtest/gtest.h>

#define TOL 1e-5f
#define TOL2 1e-2f

TEST(FwdKinTest,Zeros){
    Planar3Robot robot;
    Eigen::Matrix3f T;

    robot.addLink(PlanarLink(1,0));
    robot.addLink(PlanarLink(1,0));
    robot.addLink(PlanarLink(1,0));
    std::vector<float> angles = {0,0,0};    
  
    robot.forwardKin(angles, T);
    float pos[3];    
    robot.TMat2pos(T,pos);
    EXPECT_NEAR(pos[0],3,TOL);
    EXPECT_NEAR(pos[1],0,TOL);
    EXPECT_NEAR(pos[2],0,TOL);   
}


TEST(FwdKinTest,Pis){
    Planar3Robot robot;
    Eigen::Matrix3f T;

    robot.addLink(PlanarLink(1,0));
    robot.addLink(PlanarLink(1,0));
    robot.addLink(PlanarLink(1,0));
    std::vector<float> angles = {M_PI/2,M_PI/2,M_PI/2};    
  
    robot.forwardKin(angles, T);
    float pos[3];    
    robot.TMat2pos(T,pos);
    EXPECT_NEAR(pos[0],-1,TOL);
    EXPECT_NEAR(pos[1],0,TOL);
    EXPECT_NEAR(pos[2],-M_PI/2,TOL);   
}


TEST(InvKinTest,Stretch){
    Planar3Robot robot;
    Eigen::Matrix3f T;
    Eigen::Vector3f v;

    robot.addLink(PlanarLink(1,0));
    robot.addLink(PlanarLink(1,0));
    robot.addLink(PlanarLink(1,0));
    std::vector<float> angles = {0,0,0};    
  
    //robot.forwardKin(angles, T);
    v << 3,0,0; 
    robot.inverseKin(angles,v);
    EXPECT_NEAR(angles[0],0,TOL);
    EXPECT_NEAR(angles[1],0,TOL);
    EXPECT_NEAR(angles[2],0,TOL);   
}



TEST(InvKinTest,Elbow){
    Planar3Robot robot;
    Eigen::Matrix3f T;
    Eigen::Vector3f v;

    robot.addLink(PlanarLink(1,0));
    robot.addLink(PlanarLink(1,0));
    robot.addLink(PlanarLink(1,0));
    std::vector<float> angles = {0,0,0};    
  
    //robot.forwardKin(angles, T);
    v << 2,1,M_PI/2; 
    robot.inverseKin(angles,v);

    robot.forwardKin(angles, T);
    float pos[3];    
    robot.TMat2pos(T,pos);
    EXPECT_NEAR(pos[0],v[0],TOL2);
    EXPECT_NEAR(pos[1],v[1],TOL2);
    EXPECT_NEAR(pos[2],v[2],TOL2);   

}

TEST(InvKinTest,Unreachable){
    Planar3Robot robot;
    Eigen::Matrix3f T;
    Eigen::Vector3f v;

    robot.addLink(PlanarLink(1,0));
    robot.addLink(PlanarLink(1,0));
    robot.addLink(PlanarLink(1,0));
    std::vector<float> angles = {0,0,0};    
    v << 0,5,M_PI/2; 
    robot.inverseKin(angles,v);

    EXPECT_NEAR(angles[0],M_PI/2,TOL2);
    EXPECT_NEAR(angles[1],0,TOL2);
    EXPECT_NEAR(angles[2],0,TOL2);   

}




TEST(InvKinTest,MultipleSolutions){

    Planar3Robot robot;
    Eigen::Matrix3f T;
    Eigen::Vector3f v;

    robot.addLink(PlanarLink(1,0));
    robot.addLink(PlanarLink(1,0));
    robot.addLink(PlanarLink(1,0));
    std::vector<float> angles = {0,0,0};    

    v << 1,1,M_PI/2; 
    std::vector<std::vector<float> > solutions = robot.getMultipleSolutions(v,2);
    EXPECT_EQ(solutions.size(),2);

}



TEST(InvKinTest,MultipleSolutionsRedundant){

    Planar3Robot robot;
    Eigen::Matrix3f T;
    Eigen::Vector3f v;

    robot.addLink(PlanarLink(1,0));
    robot.addLink(PlanarLink(1,0));
    robot.addLink(PlanarLink(1,0));
    robot.addLink(PlanarLink(1,0));
    std::vector<float> angles = {0,0,0};    

    v << 1,1,M_PI/2; 
    std::vector<std::vector<float> > solutions = robot.getMultipleSolutions(v,8);
    EXPECT_EQ(solutions.size(),8);

}




int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

