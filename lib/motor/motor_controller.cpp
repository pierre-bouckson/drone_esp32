#include "motor_controller.h"
#include "pid.h"
#include "imu.h"

int motor1 = 0;
int motor2 = 0;

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

    data_imu gyro = imu_.get_gyro();
    commande_stability_ = pid_.trad_motor(gyro);

    return commande_stability_;
}

void motor_controller::send_cmd() {


    while(emergency){

        digitalWrite(2, LOW);

        ledcWrite(ledChannel, 0);
        ledcWrite(ledChanne2, 0);
        ledcWrite(ledChanne3, 0);
        ledcWrite(ledChanne4, 0);


    }



    //Verifier Coeherence angle (deg rad, where forward, ...)

    data_imu orientation = imu_.get_orientation();

    unsigned long now = micros();

    if (last_time == 0) { 
        last_time = now; 
        return;              // on attend le prochain tour pour avoir un vrai dt
    }

    
    float dt = (now - last_time) / 1000000.0f;
    last_time = now;
    if (dt <= 0 || dt > 0.05f) {   // 50 ms
        return; // skip ce tour
    }

    erreur_pitch = msg_rc_.left/20 - (orientation.pitch_deg * (180/PI));
    erreur_roll = -msg_rc_.forward/20 - (orientation.roll_deg * (180/PI));



    erreur_rate.roll_deg = pid_.pid_rate_roll(erreur_roll, coef_udp, dt);
    erreur_rate.pitch_deg = pid_.pid_rate_pitch(erreur_pitch, coef_udp, dt);



    cmd_motor_rate = pid_.trad_motor(erreur_rate);



    trottle = msg_rc_.up;

    commande_final.motor_1_duty = trottle * 2.5 + cmd_motor_rate.motor_1_duty;
    commande_final.motor_2_duty = trottle * 2.5 + cmd_motor_rate.motor_2_duty;
    commande_final.motor_3_duty = trottle * 2.5 + cmd_motor_rate.motor_3_duty;
    commande_final.motor_4_duty = trottle * 2.5 + cmd_motor_rate.motor_4_duty;

    

    if(commande_final.motor_1_duty > 255) commande_final.motor_1_duty = 255;
    if(commande_final.motor_2_duty > 255) commande_final.motor_2_duty = 255;
    if(commande_final.motor_3_duty > 255) commande_final.motor_3_duty = 255;
    if(commande_final.motor_4_duty > 255) commande_final.motor_4_duty = 255;

    if(commande_final.motor_1_duty < 0) commande_final.motor_1_duty = 0;
    if(commande_final.motor_2_duty < 0) commande_final.motor_2_duty = 0;
    if(commande_final.motor_3_duty < 0) commande_final.motor_3_duty = 0;
    if(commande_final.motor_4_duty < 0) commande_final.motor_4_duty = 0;

    motor1 = commande_final.motor_1_duty;
    motor2 = commande_final.motor_2_duty;

    // my_connect.answer_values(commande_final.motor_1_duty, commande_final.motor_2_duty, commande_final.motor_3_duty, commande_final.motor_4_duty, 8895);

    ledcWrite(ledChannel, commande_final.motor_1_duty);
    ledcWrite(ledChanne2, commande_final.motor_2_duty);
    ledcWrite(ledChanne3, commande_final.motor_3_duty);
    ledcWrite(ledChanne4, commande_final.motor_4_duty);

    

}


    // erreur_pitch = msg_rc_.forward/10 - orientation.pitch_deg;
    // erreur_roll = msg_rc_.left/10 - orientation.roll_deg;

    // rate_sp_pitch = pid_.pi_attitude_pitch(erreur_pitch, 5, 0.2, dt);
    // rate_sp_roll  = pid_.pi_attitude_roll(erreur_roll, 5, 0.2, dt);

    // erreur_rate_pitch = rate_sp_pitch - gyro.pitch_deg;
    // erreur_rate_roll = rate_sp_roll - gyro.roll_deg;

    // erreur_rate.pitch_deg = pid_.pid_rate_pitch(erreur_rate_pitch, 0.02, 0.04, 0.001,dt);
    // erreur_rate.roll_deg = pid_.pid_rate_roll(erreur_rate_roll, 0.02, 0.04, 0.001,dt);