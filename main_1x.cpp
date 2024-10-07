#include <fkin_1x.hpp>


Eigen::Matrix3f getForwardKinematics(Planar3Robot robot,std::vector<float> angles){
    Eigen::Matrix3f T;
    robot.forwardKin(angles, T);
    float pos[3];    
    robot.TMat2pos(T,pos);
    printf("Forward Kinematics: (%.2f,%.2f,%.2f)\n",pos[0],pos[1],pos[2]);
    return T;
}

int getInverseKinematics(Planar3Robot robot,std::vector<float> &angles_ik, Eigen::Vector3f desired_pose){
    std::cout <<  "Starting angles: (";
    for(int i=0;i<angles_ik.size();i++){printf("%.3f ",angles_ik[i]);} printf(")\n");

    std::cout << "Desired Pose: " << desired_pose.transpose() << std::endl;
    int iterations=robot.inverseKin(angles_ik,desired_pose);

    //robot.forwardKin(angles_ik, T);
    if (iterations<MAX_ITER){
        std::cout <<"Inverse kinematics: ";
        for(int i=0;i<angles_ik.size();i++){printf("%.3f ",angles_ik[i]);} printf("\n");
    }
    else{
        std::cerr << "Couldn't reach " << desired_pose.transpose() << std::endl;
    }
    return iterations;
}


int main(){

Planar3Robot robot;
Eigen::Matrix3f T;
Eigen::Vector3f v;

robot.addLink(PlanarLink(1,0));
robot.addLink(PlanarLink(1,0));
//robot.addLink(PlanarLink(2,0)); //To experiment with 4 links
int n=robot.addLink(PlanarLink(1,M_PI/2));
printf("Created planar robot with %d links\n",n);


std::vector<float> angs = {0,0,M_PI/2};    
T= getForwardKinematics(robot,angs);


//v=robot.TMat2vec(T);
v << 3,0.0,0.0;
std::vector<float> angles_ik;
for(int j=0;j<robot.getDOF();j++) {angles_ik.push_back((float) M_PI * rand()/RAND_MAX);}
getInverseKinematics(robot,angles_ik,v);
getForwardKinematics(robot,angles_ik);

std::vector<std::vector<float> > solutions = robot.getMultipleSolutions(v,2);
for (int i=0;i<solutions.size();i++){
    printf("Sol %d: ",i);
    for(int j=0;j<robot.getDOF();j++){printf("%f ",solutions[i][j]);
    }
    printf("\n");
}

return 0;
}