from pynput import keyboard
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('key_controller', String, queue_size=10)

def on_press(key):
    try:
        print('key {0} pressed'.format(
            key.char))
        if key.char == 'w':
            pub.publish('w')
            print('w')
    except AttributeError:
        print('key {0} pressed'.format(
            key))

def on_release(key):
    print('key {0} release'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False
    
if __name__ == '__main__':
    rospy.init_node('key_controller')
    print("keyboard controler start")
    # Collect events until released
    with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()