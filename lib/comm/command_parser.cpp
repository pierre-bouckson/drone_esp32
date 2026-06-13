#include "command_parser.h"

CommandParser::Kind CommandParser::parse(const char* msg) {
    if (strcmp(msg, "command") == 0) {
        return Kind::Ping;
    }

    if (strncmp(msg, "rc", 2) == 0) {
        Serial.println("msg rc detecter");
        if (sscanf(msg, "rc %d %d %d %d", &rc_.left, &rc_.forward, &rc_.up, &rc_.yaw) == 4) {
            Serial.print("CMD RECU : ");
            Serial.println(rc_.up);
            return Kind::Rc;
        }
    }

    if (strncmp(msg, "gain", 4) == 0) {
        Serial.println("msg gain detecter");
        if (sscanf(msg, "gain %d", &gain_) == 1) {
            Serial.print("GAIN RECU : ");
            Serial.println(gain_);
            return Kind::Gain;
        }
    }

    if (strncmp(msg, "pid", 3) == 0) {
        Serial.println("msg pid detecter");
        if (sscanf(msg, "pid %f %f %f", &coef_.kp, &coef_.ki, &coef_.kd) == 3) {
            Serial.print("PID RECU : ");
            Serial.println(coef_.kp);
            return Kind::Pid;
        }
    }

    return Kind::None;
}
