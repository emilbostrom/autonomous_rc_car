#include <iostream>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

bool exitLoop = false; // Flag to indicate whether to exit the loop

int main() {
    // Add this line at the beginning of main()
    XSetErrorHandler([](Display* display, XErrorEvent* errorEvent) {
        char errorText[256];
        XGetErrorText(display, errorEvent->error_code, errorText, sizeof(errorText));
        std::cerr << "X11 Error: " << errorText << std::endl;
        return 0;
    });


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
            KeyCode keyCode = event.xkey.keycode;
            std::cout << "Keycode: " << (int)keyCode << std::endl;
        }
    }

    XCloseDisplay(display);

    return 0;
}
