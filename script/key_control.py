from pynput import keyboard
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('key_controller', String, queue_size=10)

# 每次按下按键，会发送：按键名p，松开按键会发送：按键名r
# 例如：kp，kr，wp，wr
def send_key(str):
    pub.publish(str)
    print(str)

def on_press(key):
    try:
        print('key {0} pressed'.format(key.char))
        send_key(key.char + 'p')
    except AttributeError:
        print('key {0} pressed'.format(
            key))

def on_release(key):
    print('key {0} release'.format(key))
    send_key(key.char + 'r')
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