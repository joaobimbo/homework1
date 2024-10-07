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

int insertIfUnique(std::vector<std::vector<float > > &sols, std::vector<float> s){    
    int unique=1;
    for(int i=0; i<sols.size();i++){
        float dist=0;
        for(int j=0; j<sols[i].size();j++){
            dist+=(sols[i][j]-s[j])*(sols[i][j]-s[j]); //get the squared difference
        }
        if(dist < 0.1){
            unique=0;           
        }
    }    
    if (unique==1 || sols.size()==0){
        sols.push_back(s);
        return 1;
    }
    else{
        return 0;
    }    
}

std::vector<std::vector<float > > getMultipleSolutions(Planar3Robot robot,Eigen::Vector3f desired_pose,int n_solutions){
    int trials=0;
    std::vector<std::vector<float > > solutions;
    while (solutions.size()<n_solutions){
        std::vector<float> angles_ik;
        //Create random initial positions
        for(int i=0;i<robot.getDOF();i++){angles_ik.push_back(2.0f*M_PI*rand()/RAND_MAX);}
        //Get IK solution
        getInverseKinematics(robot,angles_ik,desired_pose);
        //Add to solutions if it's new:
        insertIfUnique(solutions,angles_ik);        
    }
    return solutions;
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

std::vector<std::vector<float> > solutions = getMultipleSolutions(robot,v,2);
for (int i=0;i<solutions.size();i++){
    printf("Sol %d: ",i);
    for(int j=0;j<robot.getDOF();j++){printf("%f ",solutions[i][j]);
    }
    printf("\n");
}

return 0;
}