import signal
from xbox360controller import Xbox360Controller


def on_button_pressed(button):
    print(button)
    print('Button {0} was pressed'.format(button.name))


def on_button_released(button):
    print(button)
    print('Button {0} was released'.format(button.name))


def on_axis_moved(axis):
    print(axis)
    if "Raw" in str(axis):
        print(f"Axis {axis.name} moved to {axis.value}")
    else:
        print(f"Axis {axis.name} moved to {axis.x} {axis.y}")    

try:
    with Xbox360Controller(0, axis_threshold=0.2) as controller:
        controller.info()
          
        # Button A events
        controller.button_a.when_pressed = on_button_pressed
        controller.button_a.when_released = on_button_released
                
        # Left and right axis move event
        controller.axis_l.when_moved = on_axis_moved
        controller.axis_r.when_moved = on_axis_moved

        controller.trigger_l.when_moved = on_axis_moved
        controller.trigger_r.when_moved = on_axis_moved
        
        signal.pause()
except KeyboardInterrupt:
    pass