#include <iostream>
#include <Carbon/Carbon.h>

bool exitLoop = false; // Flag to indicate whether to exit the loop

int main() {
    // Main loop
    while (!exitLoop) {
        
        // Check if the Up arrow key is pressed
        bool upKeyPressed = CGEventSourceKeyState(kCGEventSourceStateHIDSystemState, kVK_UpArrow);
        // Check if the Down arrow key is pressed
        bool downKeyPressed = CGEventSourceKeyState(kCGEventSourceStateHIDSystemState, kVK_DownArrow);
        if (upKeyPressed) {
            std::cout << "Up key is pressed" << std::endl;
        }
        else if (downKeyPressed) {
            std::cout << "Down key is pressed" << std::endl;
        }

        // Check if the Left arrow key is pressed
        bool leftKeyPressed = CGEventSourceKeyState(kCGEventSourceStateHIDSystemState, kVK_LeftArrow);
        // Check if the Right arrow key is pressed
        bool rightKeyPressed = CGEventSourceKeyState(kCGEventSourceStateHIDSystemState, kVK_RightArrow);
        if (leftKeyPressed) {
            std::cout << "Left key is pressed" << std::endl;
        }
        else if (rightKeyPressed) {
            std::cout << "Right key is pressed" << std::endl;
        }

        // Check if the Escape key is pressed
        bool escapeKeyPressed = CGEventSourceKeyState(kCGEventSourceStateHIDSystemState, kVK_Escape);
        if (escapeKeyPressed) {
            std::cout << "Escape key pressed. Exiting loop." << std::endl;
            exitLoop = true; // Set the flag to exit the loop
        }

        // Add a short delay to avoid busy-waiting
        usleep(10000); // Sleep for 10 milliseconds (10000 microseconds)
    }

    return 0;
}
