import curses

def main(stdscr):
    # Clear screen
    stdscr.clear()
    # Turn off echoing of keys, and enter cbreak mode,
    # where no buffering is performed on keyboard input
    curses.cbreak()
    stdscr.keypad(True)

    while True:
        # Get the user's input
        c = stdscr.getch()
        stdscr.addch(c)
        stdscr.refresh()
        # Exit on 'q' key
        if c == ord('q'):
            break

# Run the application
curses.wrapper(main)
