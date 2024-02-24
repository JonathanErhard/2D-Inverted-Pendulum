/**
 * Simulation of one axis of an inverted pendulum.
 */

#define dp(x) std::cout << #x<<':' << x << std::endl

#include <iostream>
#include <signal.h>
#include <fstream>
#include <random>
#include <cmath>
#include "linux_src/DataPuffer.h"
#include "linux_src/Regulator.h"

#define L 0.3
#define inv_L 1 / L

#define small_g 9.80665

#define CAMERA_DT 0.0333 // Time between Camera inputs [s]
#define CAMERA_QUEUE_SIZE 4 // size of Camera Buffer  [1]

#define LIDAR_DT 0.1 // Time between Lidar inputs [s]
#define LIDAR_QUEUE_SIZE 0 // size of Lidar Buffer [1]

class Pencil_Sim
{
    //maximum stepsize
    double dt_max;

    //RingBuffer als Queue verwendet
    RingBuffer<double> Camera_Sensor_Buffer{CAMERA_QUEUE_SIZE};
    RingBuffer<double> LIDAR_Sensor_Buffer{LIDAR_QUEUE_SIZE};

    Regulator* regulator;

    double next_lidar_time = LIDAR_DT;   // time of the next Lidar data
    double next_camera_time = CAMERA_DT; // time of the next camera data

    // Which sensor will be updated in the current cicle
    bool update_lidar;
    bool update_camera;

    double x[4] = {0, 0, 0.1, 0}; // state
    double dx[4];                 // derivative of the state with noise
    double y[2] = {0, 0};       // last recorded Measurements with measurement noise

    //double Q[4] = {0, 0, 0, 0}; // no process noise
    //double R[2] = {0, 0}; // no measurement noise
    double Q[4] = {1e-4, 1e-4, 1e-7, 1e-7}; // process noise (gets added to state)
    double R[2] = {5e-4, 5e-2}; // measurement noise (gets added to sensor measurements)

    std::default_random_engine random_engine[6];

    // initialize noise distributions
    std::normal_distribution<double> normal_Q0;
    std::normal_distribution<double> normal_Q1;
    std::normal_distribution<double> normal_Q2;
    std::normal_distribution<double> normal_Q3;
    std::normal_distribution<double> normal_R0;
    std::normal_distribution<double> normal_R1;

    // 0, 1-4,    5-6
    // dt,x[0-3],y[0-1]
    std::array<double,5> line_x;
    std::array<double,5> line_x_hat;
    std::array<double,2> line_measured_p;
    std::array<double,2> line_measured_theta;
    std::array<double,2> line_e;
    std::array<double,2> line_u;
    std::vector<std::array<double, 5>> save_vector_x;
    std::vector<std::array<double, 5>> save_vector_x_hat;
    std::vector<std::array<double, 2>> save_vector_measured_p;
    std::vector<std::array<double, 2>> save_vector_measured_theta;
    std::vector<std::array<double, 2>> save_vector_e;
    std::vector<std::array<double, 2>> save_vector_u;

    template<size_t T>
    void save_vector(std::vector<std::array<double,T>> save_vector, std::string filename){
        std::stringstream ss;
        std::ofstream of;
        of.open(filename, std::ios::out);
        for (int i = 0; i < save_vector.size(); i++)
        {
            for (size_t j = 0; j < T-1; j++)
            {
                ss << save_vector.at(i)[j] << ',';
            }
            ss << save_vector.at(i)[T-1] << std::endl;
        }
        of << ss.str();
        of.close();
    }

    template<size_t T>
    void dump_vector_to_file(std::vector<std::array<double,T>> save_vector, std::string filename)
    {
        std::ofstream of;
        of.open(filename, std::ios::out | std::ios::binary);
        const char *ptr = (const char *)&(save_vector);
        int size = sizeof(save_vector);
        of.write(ptr, size);
        of.close();
    }

public:
    int iteration = 0;
    double simulation_time = 0;
    double J = 0;
    double time_since_last_measurement;

    // set standard deviation and initialize sensorBuffers and random engines
    Pencil_Sim(double dt_max, Regulator * regulator, int seed = 1) : normal_Q0(0, sqrt(Q[0])), normal_Q1(0, sqrt(Q[1])), normal_Q2(0, sqrt(Q[2])), normal_Q3(0, sqrt(Q[3])), normal_R0(0, sqrt(R[0])), normal_R1(0, sqrt(R[1]))
    {
        if(dt_max <= 0){
            std::cerr << "please select a reasonable timeinterval :)\n";
        }

        this->regulator = regulator;

        this->dt_max = dt_max;
        for (int i = 0; i < 6; i++)
            random_engine[i].seed(i + seed);
    }

    //for some reason I cant capture [this] in a lamda passed as default arguemntwhy but this works i guess...
    void next_measurement(double u)
    {
        //default:100hz
        next_measurement(u, [this](int iteration){return ((iteration%(int)(1/(this->dt_max*100)))==0);});
    }

    bool next_state(double u, double* next_state){
            dx[0] = x[1] + normal_Q0(random_engine[0]);
            dx[1] = u + normal_Q1(random_engine[1]);
            dx[2] = x[3] + normal_Q2(random_engine[2]);
            dx[3] = 1.5 * (small_g * sin(x[2]) + u * cos(x[2])) * inv_L + normal_Q3(random_engine[3]);

            x[0] += dt_max * dx[0];
            x[1] += dt_max * dx[1];
            x[2] += dt_max * dx[2];
            x[3] += dt_max * dx[3];

            if(iteration++%1000==0){
                line_u[0] = simulation_time;
                line_u[1] = u;
                save_vector_u.push_back(line_u);
                line_x[0] = simulation_time;
                line_x[1] = x[0];
                line_x[2] = x[1];
                line_x[3] = x[2];
                line_x[4] = x[3];
                save_vector_x.push_back(line_x);
            }

            if(x[2] <= -M_PI) x[2] += 2*M_PI;
            if(x[2] > M_PI) x[2] -= 2*M_PI;

            next_state = x;

            simulation_time += dt_max;

            regulator->recieve_state(x);

            return x[2]>-0.5*M_PI&&x[2]<0.5*M_PI;
    }

    void next_measurement(double u, std::function<bool(int)> save_x_or_nah)
    {
        // set time and save which sensor recieves new values
        double dt;
        double time_after_next_step;
        update_camera = false;
        update_lidar = false;
        while(!(update_camera||update_lidar)){
            //this variable is not set in stone for the next cycle! it might change depending on when the sensors are supposed to measure :)
            time_after_next_step = simulation_time + dt_max;
            /**calculate dt:
             * if there is not supposed to be a measurement in ]simulation_time;simulation_time+dt_max]
             * dt = dt_max
             * Otherwise dt is calculated such that the next measurement occurs at simulation_time + dt
             */

            //no updates by default to make the if statement more readable
            update_camera = false;
            update_lidar = false;

            //check which sensor has is measuring next (could be both)
            if(next_camera_time == next_lidar_time){
                if(next_camera_time <= time_after_next_step){ //both update. could compare to next_lidar_time as well since they are equal
                    time_after_next_step = next_camera_time;
                    update_camera = true;
                    update_lidar = true;
                }
            }else if(next_camera_time < next_lidar_time){
                if(next_camera_time <= time_after_next_step){//camera updates
                    time_after_next_step = next_camera_time;
                    update_camera = true;
                }
            }else{
                if(next_lidar_time <= time_after_next_step){//lidar update
                    time_after_next_step = next_lidar_time;
                    update_lidar = true;
                }
            };
            dt = time_after_next_step - simulation_time;

            // apply Euler method
            dx[0] = x[1] + normal_Q0(random_engine[0]);
            dx[1] = u + normal_Q1(random_engine[1]);
            dx[2] = x[3] + normal_Q2(random_engine[2]);
            dx[3] = 1.5 * (small_g * sin(x[2]) + u * cos(x[2])) * inv_L + normal_Q3(random_engine[3]);

            x[0] += dt * dx[0];
            x[1] += dt * dx[1];
            x[2] += dt * dx[2];
            x[3] += dt * dx[3];

            simulation_time = time_after_next_step;

            //check which sensors measure the state
            //kinda ugly with all that copied code might wanna do that properly... but it works i guess
            if (update_lidar)
            {
                //set next measurement time
                next_lidar_time += LIDAR_DT;

                //no Queue -> cant set and get anything :)
                if(LIDAR_QUEUE_SIZE!=0){
                    //retrieve oldest value
                    if(LIDAR_Sensor_Buffer.get(y[0],LIDAR_QUEUE_SIZE-1)==-1) //if the queue is not full, no measurement should be taken
                        update_lidar = false;
                    else{
                        line_measured_p[0] = simulation_time;
                        line_measured_p[1] = y[0];
                        save_vector_measured_p.push_back(line_measured_p);
                        regulator->recieve_pos(y[0]);
                        regulator->get_x_hat(&line_x_hat[1]);
                        line_x_hat[0]=simulation_time;
                        save_vector_x_hat.push_back(line_x_hat);
                    }
                    //write new value into buffer
                    LIDAR_Sensor_Buffer.set(x[0] + normal_R0(random_engine[4]));
                    
                } else{
                    y[0] = x[0] + normal_R0(random_engine[4]);
                    line_measured_p[0] = simulation_time;
                    line_measured_p[1] = y[0];
                    save_vector_measured_p.push_back(line_measured_p);
                    regulator->recieve_pos(y[0]);
                    regulator->get_x_hat(&line_x_hat[1]);
                    line_x_hat[0]=simulation_time;
                    save_vector_x_hat.push_back(line_x_hat);
                }
            }

            if (update_camera)
            {
                //set next measurement time
                next_camera_time += CAMERA_DT;

                if(CAMERA_QUEUE_SIZE!=0){
                    //retrieve oldest value
                    if(Camera_Sensor_Buffer.get(y[1],CAMERA_QUEUE_SIZE-1)==-1)
                        update_camera = false;
                    else{
                        line_measured_theta[0] = simulation_time;
                        line_measured_theta[1] = y[1];
                        save_vector_measured_theta.push_back(line_measured_theta);
                        regulator->recieve_angle(y[1]);
                        regulator->get_x_hat(&line_x_hat[1]);
                        line_x_hat[0]=simulation_time;
                        save_vector_x_hat.push_back(line_x_hat);
                        
                    }
                    //write new value into buffer
                    Camera_Sensor_Buffer.set(x[2] + normal_R1(random_engine[5]));
                }else {
                    y[1] = x[2] + normal_R1(random_engine[5]);
                    line_measured_theta[0] = simulation_time;
                    line_measured_theta[1] = y[1];
                    save_vector_measured_theta.push_back(line_measured_theta);
                    regulator->recieve_angle(y[1]);
                    regulator->get_x_hat(&line_x_hat[1]);
                    line_x_hat[0]=simulation_time;
                    save_vector_x_hat.push_back(line_x_hat);
                }
            }
            line_x[0] = simulation_time;
            line_x[1] = x[0];
            line_x[2] = x[1];
            line_x[3] = x[2];
            line_x[4] = x[3];
            if(save_x_or_nah(iteration++))
                save_vector_x.push_back(line_x);
        }//exit if measuerement was taken
        //square difference of x and x_hat 
        double e = 0.0;
        for(int i = 1;i<5;i++)
            e += (line_x[i]-line_x_hat[i])*(line_x[i]-line_x_hat[i]);
        line_e[0] = simulation_time;
        line_e[1] = e;
        save_vector_e.push_back(line_e);
        J += e*(simulation_time-time_since_last_measurement);
        time_since_last_measurement = simulation_time;
    }

    void save(std::string filename_state, std::string filename_measurement_p, std::string filename_measurement_theta, std::string filename_state_hat, std::string filename_e)
    {
        save_vector(save_vector_x, filename_state);
        save_vector(save_vector_measured_p, filename_measurement_p);
        save_vector(save_vector_measured_theta, filename_measurement_theta);
        save_vector(save_vector_x_hat, filename_state_hat);
        save_vector(save_vector_e,filename_e);
    }

    //writes the bits into a file. data_size.csv contains information about the length
    void dump_to_file(std::string filename_state, std::string filename_measurement_p, std::string filename_measurement_theta, std::string filename_x_hat, std::string filename_size = "data_size.csv")
    {
        dump_vector_to_file(save_vector_x, filename_state);
        dump_vector_to_file(save_vector_measured_p, filename_measurement_p);
        dump_vector_to_file(save_vector_measured_theta, filename_measurement_theta);
        dump_vector_to_file(save_vector_x_hat, filename_x_hat);
        
        /**
         * format:
         * save_vector_x.size()
         * save_vector_x_hat.size()
         * save_vector_measured_p.size()
         * save_vector_measured_theta.size()
        */

        std::stringstream ss;
        std::ofstream of;
        of.open(filename_size, std::ios::out);
        ss << save_vector_x.size() << '\n';
        ss << save_vector_x_hat.size() << '\n';
        ss << save_vector_measured_p.size() << '\n';
        ss << save_vector_measured_theta.size() << '\n';
        of << ss.str();
        of.close();
    }
};