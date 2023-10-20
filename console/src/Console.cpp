#include "../include/console/Console.hpp"
#include "../include/console/Parser.hpp"

Console::Console() : Node("console") {
    this->commandPub = this->create_publisher<robot_arm_interface::msg::Command>("command", 10);
    AskForCommand();
}

Console::~Console() {
}

void Console::askForCommand() {
    std::cout << "Enter command (#<servo_nr>P<PWM>T<time>\\r): ";
    std::string command;
    std::cin >> command;
    std::cout << "Got command: " << command << std::endl;

    Parser parser(command);

    if (parser.getError()) {
        askForCommand();
    }

    auto msg = std::make_unique<robot_arm_interface::msg::Command>();

    msg->servo_nr = parser.getServoNr();
    msg->pwm = parser.getPwm();
    msg->time = parser.getTime();

    this->commandPub->publish(std::move(msg));

    std::cout << "Published command" << std::endl;

    askForCommand();
}