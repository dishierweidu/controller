#include "./motorsolver/motorsolver.hpp"

int main(int argc,char* argv[])
{
    const string motor_config_path = "/home/dishierweidu/Documents/auto/controller/params/demo.yaml";
    CanPort canport;
    MotorSolver motorsolver;

    std::vector<int> x, y, z;

    motorsolver.loadParam("./params/demo.yaml","Motor_ID_1");
	canport.initCanPort();

    if(canport.initCanPort())
    {
        int i=0;
        while (1)
        {
            canport.receive();
    		canport.set_voltage = motorsolver.pid_solver(canport.motor_info,40000);
			canport.send();

            x.push_back(i);
	    	y.push_back(canport.set_voltage);
            z.push_back(canport.motor_info[0].rotor_speed);

            if (i % 10 == 0) 
            {
                // Clear previous plot
                plt::clf();
                // Plot line from given x and y data. Color is selected automatically.
                plt::plot(x, y);
                // Plot a line whose name will show up as "log(x)" in the legend.
                plt::named_plot("rotor_speed", x, z);

                // Set x-axis to interval [0,1000000]
                plt::xlim(0, 100000);

                // Add graph title
                plt::title("Sample figure");
                // Enable legend.
                plt::legend();
                // Display plot continuously
                plt::pause(0.01);
		    }

            i++;
			// usleep(1000);
        }    
    }
    return 0;
}