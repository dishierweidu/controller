#include "./motorsolver/motorsolver.hpp"

int main(int argc,char* argv[])
{
    const string motor_config_path = "/home/dishierweidu/Documents/auto/controller/params/demo.yaml";
    CanPort canport;
    MotorSolver motorsolver;

    motorsolver.loadParam("./params/demo.yaml","Motor_ID_1");
	canport.initCanPort();

    if(canport.initCanPort())
    {
        while (1)
        {
            canport.receive();
    		canport.set_voltage = motorsolver.pid_solver(canport.motor_info,30000);
			canport.send();
        }    
    }
    return 0;
}