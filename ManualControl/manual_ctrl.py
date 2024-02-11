import sys
import termios
import tty
import select

# Dictionary mapping keys to actions
key_actions = {
    'w': "Move Forward",
    'a': "Move Left",
    's': "Move Backward",
    'd': "Move Right"
}

# Set to store currently pressed keys
pressed_keys = set()

def main():
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)

    try:
        while True:
            if select.select([sys.stdin], [], [], 0)[0]:  # Check if there's input available
                key = sys.stdin.read(1)
                handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)

def handle_key(key):
    if key in key_actions:
        pressed_keys.add(key)
        check_combination(pressed_keys)
    elif key == chr(27):  # ESC
        print("Exiting...")
        sys.exit()

def handle_key_release(key):
    if key in key_actions:
        pressed_keys.remove(key)

def check_combination(keys):
    combination = "".join(sorted(keys))
    if combination in key_actions:
        action = key_actions[combination]
        print("You pressed combination:", combination, ":", action)

if __name__ == "__main__":
    main()
