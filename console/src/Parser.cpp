#include "../include/simulation/Parser.hpp"
#include <iostream>

typedef enum {
    BEGINCHAR,
    SERVO_NR,
    PWM,
    TIME
} currentState;

Parser::Parser(std::string command)
{
    parseCommand(command);
}

Parser::~Parser()
{
}

void Parser::parseCommand(std::string& command) {
    currentState state = BEGINCHAR;
    std::string servoNr = "";
    std::string pwm = "";
    std::string time = "";

    for (size_t i = 0; i < command.length(); ++i) {
        switch (state) {
            case BEGINCHAR:
                if (command[i] == '#') {
                    state = SERVO_NR;
                }
                break;
            case SERVO_NR:
                if (command[i] == 'P') {
                    state = PWM;
                } else if (isDigit(command[i])) {
                    servoNr += command[i];
                } else {
                    state = BEGINCHAR;
                    this->errorHandler();
                    break;
                }
                break;
            case PWM:
                if (command[i] == 'T') {
                    state = TIME;
                } else if (isDigit(command[i])) {
                    pwm += command[i];
                } else {
                    state = BEGINCHAR;
                    this->errorHandler();
                    break;
                }
                break;
            case TIME:
                if (command[i] == '\r') {
                    state = BEGINCHAR;
                } else if (isDigit(command[i])) {
                    time += command[i];
                } else {
                    state = BEGINCHAR;
                    this->errorHandler();
                    break;
                }
                break;
        }
    }

    if (!this->error) {
        std::cout << "ServoNr: " << servoNr << std::endl;
        std::cout << "PWM: " << pwm << std::endl;
        std::cout << "Time: " << time << std::endl;


        this->time = std::stoi(time);
        this->pwm = std::stoi(pwm);
        this->servoNr = std::stoi(servoNr);
    }
}

bool Parser::isDigit(char ch) const {
    int v = ch; // ASCII Val converted
    if (!(v >= 48 && v <= 57) && v != 45) { // checks if it is a number or a minus sign
        return false;
    }
    return true;
}

void Parser::errorHandler() {
    std::cout << "Error: Invalid command" << std::endl;
    this->error = true;
}

uint8_t Parser::getServoNr() const {
    return this->servoNr;
}

uint16_t Parser::getPwm() const {
    return this->pwm;
}

uint16_t Parser::getTime() const {
    return this->time;
}

bool Parser::getError() const {
    return this->error;
}