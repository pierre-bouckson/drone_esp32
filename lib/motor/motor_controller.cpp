#include "motor_controller.h"
#include "pid.h"
#include "imu.h"


void motor_controller::motor_init() {

    ledcSetup(ledChannel, freq, resolution);
    ledcSetup(ledChanne2, freq, resolution);
    ledcSetup(ledChanne3, freq, resolution);
    ledcSetup(ledChanne4, freq, resolution);

    ledcAttachPin(PIN_motor_1, ledChannel);
    ledcAttachPin(PIN_motor_2, ledChanne2);
    ledcAttachPin(PIN_motor_3, ledChanne3);
    ledcAttachPin(PIN_motor_4, ledChanne4);

    ledcWrite(ledChannel, 0);
    ledcWrite(ledChanne2, 0);
    ledcWrite(ledChanne3, 0);
    ledcWrite(ledChanne4, 0);
}


void motor_controller::cmd_vel(int x, int y, int h, int yaw) {
    
  
}

motor_cmd motor_controller::stability() {


    const data_imu orientation = imu_.get_orientation();
    commande_.motor_1_duty = static_cast<int>(pid_orientation_.PID_output(orientation));
    // commande.motor_1_duty = static_cast<int>(pid_orientation_.PID_output(data_imu imu.get_orientation()));
    // commande.motor_2_duty = PID_orientation_y + PID_accelero_y;
    // commande.motor_3_duty = PID_orientation_x + PID_accelero_x;
    // commande.motor_4_duty = PID_orientation_y + PID_accelero_y;

    return commande_;
}

void motor_controller::send_cmd(motor_cmd commande) {

    ledcWrite(ledChannel, commande.motor_1_duty);
    ledcWrite(ledChanne2, commande.motor_2_duty);
    ledcWrite(ledChanne3, commande.motor_3_duty);
    ledcWrite(ledChanne4, commande.motor_4_duty);
}