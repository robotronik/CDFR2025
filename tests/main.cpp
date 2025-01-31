#include <iostream>
#include "navigation/highways.h"
#include "defs/tableState.hpp"
#include "i2c/Asserv.hpp"
#include "i2c/Arduino.hpp"

bool testLogger();
bool test_lidar_opponent();
bool test_lidar_beacons();

int runAllTests();

int main() {
    return runAllTests();
}

//Runs every test
int runAllTests() {
    std::cout << "Running tests" << std::endl;
    int numPassed = 0;
    int numTests = 0;

    //Runs the logger tests
    std::cout << "Running logger tests" << std::endl;
    numTests++;
    if(testLogger())
        numPassed++;

    std::cout << "Running highway tests" << std::endl;
    numTests++;
    init_highways();
    if (unit_tests())
        numPassed++;

    //Runs the lidar tests
    std::cout << "Running lidar tests" << std::endl;
    numTests++;
    if(test_lidar_opponent())
        numPassed++;
    numTests++;
    if(test_lidar_beacons())
        numPassed++;


    std::cout << numPassed << "/" << numTests << " tests passed" << std::endl;
    return (numTests == numPassed) ? 0 : 1;
}