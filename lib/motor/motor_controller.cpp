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


motor_cmd motor_controller::cmd_vel() {

    commande_rc_.motor_1_duty = msg_rc_.up * 2.5 - msg_rc_.forward * 0.5 - msg_rc_.left * 0.5;
    commande_rc_.motor_2_duty = msg_rc_.up * 2.5 + msg_rc_.forward * 0.5 - msg_rc_.left * 0.5;
    commande_rc_.motor_3_duty = msg_rc_.up * 2.5 + msg_rc_.forward * 0.5 + msg_rc_.left * 0.5;
    commande_rc_.motor_4_duty = msg_rc_.up * 2.5 - msg_rc_.forward * 0.5 + msg_rc_.left * 0.5;



    return commande_rc_;
}

motor_cmd motor_controller::stability() {

    const data_imu orientation = imu_.get_orientation();
    commande_stability_ = pid_orientation_.PID_output(orientation);
    // commande_stability_.motor_1_duty = PID_output(orientation).motor_1_duty;


    // commande_stability_.motor_2_duty = static_cast<int>(pid_orientation_.PID_output(orientation));
    // commande_stability_.motor_3_duty = static_cast<int>(pid_orientation_.PID_output(orientation));
    // commande_stability_.motor_4_duty = static_cast<int>(pid_orientation_.PID_output(orientation));

    return commande_stability_;
}

void motor_controller::send_cmd() {

    commande_final.motor_1_duty = -stability().motor_1_duty * 5 + cmd_vel().motor_1_duty;
    commande_final.motor_2_duty = -stability().motor_2_duty * 5 + cmd_vel().motor_2_duty;
    commande_final.motor_3_duty = -stability().motor_3_duty * 5 + cmd_vel().motor_3_duty;
    commande_final.motor_4_duty = -stability().motor_4_duty * 5 + cmd_vel().motor_4_duty;

    Serial.print("      duty 1 : ");
    Serial.println(commande_final.motor_1_duty);



    ledcWrite(ledChannel, commande_final.motor_1_duty);
    ledcWrite(ledChanne2, commande_final.motor_2_duty);
    ledcWrite(ledChanne3, commande_final.motor_3_duty);
    ledcWrite(ledChanne4, commande_final.motor_4_duty);
}