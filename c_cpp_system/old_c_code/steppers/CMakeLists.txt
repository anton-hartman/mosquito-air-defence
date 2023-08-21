# Create a library (or executable) for the steppers code
# Replace 'stepper_files.c' with your actual C files
add_library(steppers_lib controller.c HR8825_driver.c)
