#include <signal.h>
#include "Pencil_Sim.h"
#include "linux_src/Regulator.h"
#include "linux_src/PD_Controller.h"
#include "linux_src/Linear_regression_filter.h"


//plot current pen_sim
//$ g++ simulate.cpp linux_src/Linear_regression_filter.cpp linux_src/Eigenbewegung.cpp -o simulate.out && ./simulate.out && gnuplot data/plot_x_y.plg

double k[] = {1,2,-19.5,-10};
PD_Controller controller{k};

class Sim_Filter: public Linear_regression_filter{
    public:
    Pencil_Sim* sim;
    protected:
    using Linear_regression_filter::Linear_regression_filter;
    double get_time(){
        return sim->simulation_time;
    }
} filter{10,5};


Regulator ralf_der_regler(&filter, &controller, (FileWriter*)0, Identifier::X, true);

Pencil_Sim* pensim;



int main(int argc, char** argv){

    int seed = 1;
    bool save_data = true;

    if(argc>1){
        seed = std::stoi(argv[1]);
        save_data = false;
    }

    pensim = new Pencil_Sim(1e-5,&ralf_der_regler,seed);

    filter.sim = pensim;

    //ralf_der_regler.set_filter(&filter);

    double x[4];
    
    //sometimes in while condition
    int measurement_count = 0;

    while (pensim->simulation_time < 10)
    {
        measurement_count++;
        //simulate until the next measurement
        pensim->next_measurement(0.05);
        //std::cout << ralf_der_regler.u << '\n';
    }

    std::cout << seed << '&' << pensim->J << "\\\\\n";

    //if(save_data)
        pensim->save("data_x.csv","data_y0.csv","data_y1.csv","data_xhat.csv","data_J.csv");
}