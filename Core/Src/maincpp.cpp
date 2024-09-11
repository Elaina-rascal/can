#include "maincpp.h"
uint8_t common_buffer[8] = {0};
Motor::MotorInterface_t motor;
int main_cpp()
{
    // Configure_Filter();
    motor.bind_pin(1, &hcan1, common_buffer, true);
    while (1)
    {
        motor.ControlOutput(400);
        HAL_Delay(10);
    }
    return 0;
}