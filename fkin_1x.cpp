#include <fkin_1x.hpp>

PlanarLink::PlanarLink(float linkLength, float angle):linkLength(linkLength),angle(angle){};

void PlanarLink::setAngle(float angle){
    this->angle=angle;
}

float PlanarLink::getAngle(){
    return this->angle;
}

float PlanarLink::getLength(){
    return this->linkLength;
}

//This gets the homogenous transform from base to tip of the link
Eigen::Matrix3f PlanarLink::getTransform(float angle){
    this->setAngle(angle);
    Eigen::Matrix3f T;
    Eigen::Vector3f v;
    float c_a=cos(angle);
    float s_a=sin(angle);
    T << c_a , -s_a , 0,
         s_a ,  c_a , 0,
          0  ,   0  , 1;
    
    v << this->getLength(),0,1;    
    T.block<3,1>(0,2) = T*v;
    return T;
}


Planar3Robot::Planar3Robot(){}

int Planar3Robot::getDOF(){
    return this->links.size();    
}
int Planar3Robot::addLink(PlanarLink in){
    this->links.push_back(in);
    return this->links.size();    
}

int Planar3Robot::forwardKin(std::vector<float> &angles, Eigen::Matrix3f &T){
    T=Eigen::MatrixX3f::Identity(3,3);
    //Checks if the input is correct
    if(angles.size()!=this->getDOF()){
        std::cerr << "Wrong number of joints (" << angles.size() << " vs. " << this->getDOF() << std::endl;
        return -1;
    }
    
    //Multiplies each Homogenous matrix from base to tip 
    for (size_t i=0; i<this->links.size(); i++){
        T*=this->links[i].getTransform(angles[i]);
    }
    return 0;
}


Eigen::Matrix3Xf Planar3Robot::getNumericalJacobian(std::vector<float> angles){
    Eigen::Matrix3Xf J(3,angles.size());
    Eigen::Matrix3f T;
    Eigen::Vector3f v_i;
    float eps=0.001;    
    
    //initial position
    this->forwardKin(angles,T);
    v_i=TMat2vec(T);


    //fill in each column of the jacobian (disturbing each DoF)
    for (size_t i=0;i<angles.size();i++) {
        Eigen::Matrix3f T_eps;
        std::vector<float> angles_eps=angles;
        Eigen::Vector3f v_eps;
        angles_eps[i]+=eps;
        this->forwardKin(angles_eps,T_eps);
        v_eps=TMat2vec(T_eps);
        J.block<3,1>(0,i)=v_eps-v_i;
    } 

    return J;       
}

//Gets a (x,y,th) from the Transform  matrix
void Planar3Robot::TMat2pos(Eigen::Matrix3f T, float *pos){
    pos[0] = T(0,2);
    pos[1] = T(1,2);
    pos[2] = atan2(T(1,0),T(0,0));
}

//Gets a (x,y,th) (eigen) from the Transform  matrix
Eigen::Vector3f Planar3Robot::TMat2vec(Eigen::Matrix3f T){
    Eigen::Vector3f pos;
    pos[0] = T(0,2);
    pos[1] = T(1,2);
    pos[2] = atan2(T(1,0),T(0,0));
    return pos;
}
// Wrap angle to [-180,180]: https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code
float Planar3Robot::constrainAngle(float x){
    x = fmod(x + M_PI,M_PI*2);
    if (x < 0)
        x += M_PI*2;
    return x - M_PI;
}


//calculates inverse kinematics through a simple gradient descent
int Planar3Robot::inverseKin(std::vector<float> &angles, Eigen::Vector3f des){   
    Eigen::Matrix3Xf J;
    float lambda=0.1;
    Eigen::Matrix3f T;
    Eigen::VectorXf delta_q(angles.size());
    int iterations=0;

    //Initial values of the angles
    std::vector<float> cur_ang=angles;

    //finds the current position error
    this->forwardKin(cur_ang,T);
    Eigen::Vector3f current=TMat2vec(T);
    Eigen::Vector3f error = des-current;
    
    //saves the prev_error
    float prev_error=error.norm();

    while (error.norm()>0.01 && iterations<=MAX_ITER){           
            J = this->getNumericalJacobian(cur_ang);
            //This is a gradient descent x_{n+1} = x_n + lambda* \grad_err
            delta_q= lambda*J.transpose()*error;
            for (size_t i=0;i<cur_ang.size();i++) cur_ang[i]+=delta_q[i];

            //finds the new error
            this->forwardKin(cur_ang,T);
            current=TMat2vec(T);
            error = des-current;
            //std::cout << Debug: (distance) <<  error.transpose() << std::endl;

            //Trick to speed up the convergence
            if(error.norm()>=prev_error){
                //Bad step, cancel and reduce lambda
                for (size_t i=0;i<cur_ang.size();i++) cur_ang[i]-=delta_q[i];
                lambda=std::max(lambda/1.1f,0.00001f);
            }
            else{
                //Good step, increase lambda
                lambda=std::min(lambda*1.1f,1000.0f);
                prev_error=error.norm();
            }
            iterations++;            
    }
    //Wrap the angles to ]-180, 180]    
    for (size_t i=0;i<cur_ang.size();i++) {        
        cur_ang[i]=constrainAngle(cur_ang[i]);
        //printf("%f ",cur_ang[i]);
    }    
    //This gets out by reference:
    angles=cur_ang;   
    //return to check if it converged 
    return iterations;
}


int Planar3Robot::insertIfUnique(std::vector<std::vector<float > > &sols, std::vector<float> s){    
    int unique=1;
    for(int i=0; i<sols.size();i++){
        float dist=0;
        //for each existing solution
        for(int j=0; j<sols[i].size();j++){
            //get the squared difference to the new
            dist+=(sols[i][j]-s[j])*(sols[i][j]-s[j]); 
        }
        if(dist < 0.1){
            //exists already, skip (could break)
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

std::vector<std::vector<float > > Planar3Robot::getMultipleSolutions(Eigen::Vector3f desired_pose,int n_solutions){
    int trials=0;
    std::vector<std::vector<float > > solutions;
    while (solutions.size()<n_solutions){
        std::vector<float> angles_ik;
        //Create random initial positions
        for(int i=0;i<this->getDOF();i++){angles_ik.push_back(2.0f*M_PI*rand()/RAND_MAX);}
        //Get IK solution        
        inverseKin(angles_ik,desired_pose);
        //Add to solutions if it's new:
        insertIfUnique(solutions,angles_ik);        
    }
    return solutions;
}
