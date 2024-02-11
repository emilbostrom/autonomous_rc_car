import keyboard

def on_key_event(event):
    print(f"Key {event.name} {'pressed' if event.event_type == 'down' else 'released'}")

# Subscribe to key events
keyboard.hook(on_key_event)

# Block the program from exiting
keyboard.wait('esc')
