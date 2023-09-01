// #include "../../src/turret/hr8825_driver.hpp"  // Include the header file you
// want to test #include <gtest/gtest.h>

// // Mock GPIO class for testing
// class MockGPIO {
//  public:
//   static void setmode(int) {}
//   static void setup(int, int, int) {}
//   static void output(int, int) {}
// };

// // Replace HR8825_driver::GPIO with MockGPIO for testing
// namespace driver {
// using GPIO = MockGPIO;
// MOTOR Motor;
// }  // namespace driver

// // Define a test fixture for your tests
// class HR8825DriverTest : public ::testing::Test {
//  protected:
//   void SetUp() override {
//     // Initialize any necessary resources or objects before each test
//   }

//   void TearDown() override {
//     // Clean up any resources or objects after each test
//   }
// };

// // Define your test cases
// TEST_F(HR8825DriverTest, InitDriverPins) {
//   // Test the init_driver_pins function
//   EXPECT_EQ(driver::init_driver_pins(), 0);
// }

// TEST_F(HR8825DriverTest, SelectMotor) {
//   // Test the select_motor function
//   driver::select_motor(driver::MOTOR1);
//   EXPECT_EQ(driver::Motor.name, driver::MOTOR1);
//   // Add more assertions as needed
// }

// TEST_F(HR8825DriverTest, TurnMotor) {
//   // Test the turn_motor function
//   driver::select_motor(driver::MOTOR1);
//   driver::turn_motor(driver::FORWARD, 10, 100);  // Example values
//   // Add assertions to verify the motor behavior
// }