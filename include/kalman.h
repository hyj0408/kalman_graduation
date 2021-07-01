//
// Created by huangyujun on 4/16/21.
//

#ifndef SRC_KALMAN_H
#define SRC_KALMAN_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>


class Kalman {
public:
    Eigen::MatrixXd A_trans, B_input, Q_cov;
    Eigen::MatrixXd C_obs, R_noise;
    Eigen::MatrixXd P_cov_0;
    Eigen::MatrixXd L_obs;
    double imu_rate = 50;
    double imu_dt=1/imu_rate;
    Eigen::VectorXd last_y;
    bool update_flag=true;

    Kalman() {
        A_trans=Eigen::Matrix<double, 4, 4>::Identity();
        A_trans.block(0,2,2,2)=imu_dt*Eigen::Matrix<double, 2, 2>::Identity();
        B_input=Eigen::Matrix<double, 4, 2>::Zero();
        B_input.block(0,0,2,2)=0.5*imu_dt*imu_dt*Eigen::Matrix<double, 2, 2>::Identity();
        B_input.block(2,0,2,2)=imu_dt*Eigen::Matrix<double, 2, 2>::Identity();
        C_obs=Eigen::Matrix<double, 2, 4>::Zero();
        C_obs.block(0,0,2,2)=Eigen::Matrix<double, 2, 2>::Identity();
        Q_cov = 0.001 * Eigen::Matrix<double, 4,4>::Identity();//要调参
        R_noise = 0.1 * Eigen::Matrix<double, 2,2>::Identity();//要调参
        P_cov_0=0.2*Eigen::Matrix<double, 4, 4>::Identity();

        //额外的观测器
        L_obs=Eigen::Matrix<double, 4, 2>::Zero();
        L_obs.block(0,0,2,2)=Eigen::Matrix<double, 2, 2>::Identity();

    }

    struct State
    {
        int index=0;
        //uint32_t sys_time;
        std_msgs::Header head;
        Eigen::Vector4d x;
        Eigen::MatrixXd A, B, P_cov;
        Eigen::VectorXd u; //use input u to calculate x
        Eigen::Vector2d y;
        State(){
//            x<<0,0,0,0;
//            y<<0,0;
            x(0)=0;
            x(1)=0;
            x(2)=0;
            x(3)=0;
            y(0)=0;
            y(1)=0;
            P_cov=0.2 *Eigen::Matrix<double, 4, 4>::Identity();
        }
        State(Eigen::VectorXd x, Eigen::MatrixXd P_cov, Eigen::VectorXd u):
             x(x), P_cov(P_cov), u(u){}
        ~State()
        {
        }
    };

    //int state_index=0;


    std::deque<State> states;
    //State state;
    void predict(Eigen::VectorXd u,std_msgs::Header head)
    {
        //std::cout<<"go into predict"<<std::endl;
        State state_now;
        //std::cout<<"state_now success"<<std::endl;
        state_now.u=u;
        state_now.head=head;
        int state_index=0;
        //std::cout<<"state_now.u=u success"<<std::endl;
        //std::cout<<states.size()<<std::endl;
        //std::cout<<1<<' ';
        //std::cout<<states.size()<<' ';
        if (states.size()==0){
            state_now.index=0;
            states.push_back(state_now);
            //states[0]=state_now;
            state_index=0;
            std::cout<<states.size()<<' ';

        }
        else{
            state_index= states.back().index;
            state_now.index=state_index+1;
            states.push_back(state_now);
            state_index++;
            predict(state_index);
        }

//        std::cout<<"states.size="<<states.size()<<' ';
//        std::cout<<"state_index="<<state_index<<std::endl;

        //predict(state_index);
        //update(state_index);
    }

    void update(Eigen::VectorXd y,std_msgs::Header head)
    {
        while (update_flag)
        {
            last_y=y;
            update_flag=0;
        }
        //std::cout<<"go into update"<<std::endl;
        int &state_index= states.back().index;
        states[state_index].head=head;

        if (state_index==0)
        {
            states[state_index].y=y;
            last_y=y;
        } else
        {
            double dx=abs(last_y(0)-y(0));
            std::cout<<dx<<std::endl;
            double dy=abs(last_y(1)-y(1));
            std::cout<<dy;
            if (dx>=50) states[state_index].y(0)=last_y(0);
                else states[state_index].y(0)=y(0);
            if (dy>=100) states[state_index].y(1)=last_y(1);
                else states[state_index].y(1)=y(1);
            last_y=states[state_index].y;
        }
        //states[state_index].y=y;
        update(state_index);
    }

    std_msgs::Header getheader()
    {
        std::cout<<"into getheader()"<<std::endl;
        std_msgs::Header head1;
        if(states.size()!=0)
        {
            head1=states.back().head;
        }
        return head1;
    }


    geometry_msgs::Pose getpose()
    {
        //std::cout<<states.size()<<std::endl;
        geometry_msgs::Pose pose;
        if(states.size()!=0)
        {   //std::cout<<states.back().x<<std::endl;
            pose.position.x=states.back().x[0];
            pose.position.y=states.back().x[1];
        }
        return pose;
    }

private:
    void predict(int state_index)
    {
        //std::cout<<"private predict"<<std::endl;
        //std::cout<<B_input<<std::endl;

//    states[state_index].x=states[state_index].A*states[state_index-1].x + states[state_index-1].B*states[state_index-1].u;
//    states[state_index].P_cov=states[state_index].A*states[state_index-1].P_cov*states[state_index-1].A.transpose()+Q_cov;
    states[state_index].x=A_trans*states[state_index-1].x + B_input*states[state_index-1].u;
//    std::cout<<states[state_index].x<<std::endl;
//    std::cout<<states.back().x<<std::endl;
        states[state_index].P_cov=A_trans*states[state_index-1].P_cov*A_trans.transpose()+Q_cov;
    }

    void update(int & state_index)
    {
        //std::cout<<"private update"<<std::endl;
//额外加一个观测器

    states[state_index].x=A_trans*states[state_index-1].x + B_input*states[state_index-1].u+L_obs*(states[state_index-1].y-C_obs*states[state_index-1].x);
    states[state_index].P_cov=A_trans*states[state_index-1].P_cov*A_trans.transpose()+Q_cov;



//end

    Eigen::MatrixXd K_gain;
//    K_gain=states[state_index].P_cov*C_obs.transpose()*(C_obs*states[state_index].P_cov*C_obs.transpose()+R_noise).inverse();
//    states[state_index].x=states[state_index].x+K_gain*(states[state_index].y-C_obs*states[state_index].x);
//    states[state_index].P_cov=states[state_index].P_cov-K_gain*C_obs*states[state_index].P_cov;
    K_gain=states[state_index].P_cov*C_obs.transpose()*(C_obs*states[state_index].P_cov*C_obs.transpose()+R_noise).inverse();
    states[state_index].x=states[state_index].x+K_gain*(states[state_index].y-C_obs*states[state_index].x);
    states[state_index].P_cov=states[state_index].P_cov-K_gain*C_obs*states[state_index].P_cov;
//    std::cout<<states[state_index].x<<std::endl;
//    std::cout<<states.back().x<<std::endl;
    //state_index++;
    }

};

#endif //SRC_KALMAN_H
