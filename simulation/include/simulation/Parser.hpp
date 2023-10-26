#ifndef PARER_HPP__
#define PARER_HPP__

#include <string>
#include <stdint.h>


class Parser
{
public:
    Parser(std::string command);
    uint8_t getServoNr() const;
    uint16_t getPwm() const;
    uint16_t getTime() const;
    bool getError() const;
    bool getStop() const;
    ~Parser();
private:
    void parseCommand(std::string& command);
    bool isDigit(char ch) const;
    void errorHandler();

    // attributes
    bool error;
    uint8_t servoNr;
    uint16_t pwm;
    uint16_t time;
    bool stop;
};


#endif // PARER_HPP__