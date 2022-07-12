#include "./motorsolver/motorsolver.hpp"

int main(int argc,char* argv[])
{
    const string motor_config_path = "/home/dishierweidu/Documents/auto/controller/params/demo.yaml";
    CanPort canport;
    MotorSolver motorsolver;

    motorsolver.loadParam(motor_config_path,"Motor_ID_1");
    motorsolver.pid_solver();
    if(canport.initCanPort())
    {
        while (1)
        {
            canport.receive();
        }    
    }
    return 0;
}