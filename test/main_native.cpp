#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"

auto main(int argc, char* argv[]) -> int {
    return CommandLineTestRunner::RunAllTests(argc, argv);
}
