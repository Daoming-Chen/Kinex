#include <gtest/gtest.h>
#include <kinex/logging.h>

int main(int argc, char** argv) {
    // Initialize logging for tests
    kinex::setLogLevel(spdlog::level::warn);
    
    // Initialize Google Test
    ::testing::InitGoogleTest(&argc, argv);
    
    // Run tests
    return RUN_ALL_TESTS();
}
