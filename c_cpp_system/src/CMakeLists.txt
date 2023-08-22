# List your source files here
set(SOURCES
    main.cpp
    gpio/gpio.cpp
    detection/detection.cpp
    motors/controller.cpp
    utilities/debug.cpp
)

add_executable(MosquitoAirDefence ${SOURCES})
