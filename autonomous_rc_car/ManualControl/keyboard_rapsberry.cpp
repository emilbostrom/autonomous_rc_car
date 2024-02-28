#include <iostream>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

bool exitLoop = false; // Flag to indicate whether to exit the loop

int main() {
    Display* display = XOpenDisplay(NULL);
    if (display == NULL) {
        std::cerr << "Error: Unable to open display." << std::endl;
        return 1;
    }

    Window rootWindow = DefaultRootWindow(display);
    XSelectInput(display, rootWindow, KeyPressMask | KeyReleaseMask);

    // Main loop
    while (!exitLoop) {
        XEvent event;
        XNextEvent(display, &event);
        if (event.type == KeyPress) {
            KeySym keySym;
            char buffer[1];
            XLookupString(&event.xkey, buffer, 1, &keySym, NULL);
            if (keySym == XK_Up) {
                std::cout << "Up key is pressed" << std::endl;
            } else if (keySym == XK_Down) {
                std::cout << "Down key is pressed" << std::endl;
            } else if (keySym == XK_Left) {
                std::cout << "Left key is pressed" << std::endl;
            } else if (keySym == XK_Right) {
                std::cout << "Right key is pressed" << std::endl;
            } else if (keySym == XK_Escape) {
                std::cout << "Escape key pressed. Exiting loop." << std::endl;
                exitLoop = true; // Set the flag to exit the loop
            }
        }
    }

    XCloseDisplay(display);

    return 0;
}
