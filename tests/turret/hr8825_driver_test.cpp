#include "../../src/turret/hr8825_driver.cpp"
#include <gtest/gtest.h>

// // Mock GPIO class for testing
// class MockGPIO {
//  public:
//   static void setmode(int) {}
//   static void setup(int, int, int) {}
//   static void output(int, int) {}
// };

// // Replace HR8825_driver::GPIO with MockGPIO for testing
// namespace driver {
// // using GPIO = MockGPIO;
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

TEST(HR8825DriverTest, SelectMotor) {
  // Test the select_motor function
  driver::MOTOR Motor;
  driver::select_motor(MOTOR1);
  EXPECT_EQ(Motor.name, MOTOR1);
  // Add more assertions as needed
}

// TEST_F(HR8825DriverTest, TurnMotor) {
//   // Test the turn_motor function
//   driver::select_motor(driver::MOTOR1);
//   driver::turn_motor(driver::FORWARD, 10, 100);  // Example values
//   // Add assertions to verify the motor behavior
// }

// TEST(HR8825DriverTest, SelectMotor) {
//   // Test the select_motor function
//   driver::MOTOR Motor;
//   driver::select_motor(MOTOR1);
//   EXPECT_EQ(Motor.name, MOTOR1);
//   // Add more assertions as needed
// }