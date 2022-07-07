#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
import tty, sys, termios
from pynput.keyboard import Key, Listener


msg = Int8()
msg.data = 0

# def on_press(key):
#     if key.char  == "0":
#         print("Stop all");
#         msg.data = 0
#     elif key.char  == "1":
#         print("Start left");
#         msg.data = 1
#     elif key.char == "2":
#         print("Start right");
#         msg.data = 2
#     elif key.char  == "3":
#         print("Stop left");
#         msg.data = 3
#     elif key.char  == "4":
#         print("Stop right");
#         msg.data = 4
#     else:
#         return
#
# def on_release(key):
#     #print('{0} release'.format(
#     #    key))
#     msg.data = 0
#     if key == Key.esc:
#         # Stop listener
#         return False

def main():
    rospy.init_node('enable_teleop', anonymous=True)
    pub = rospy.Publisher('enable_teleop', Int8, queue_size=1)

    filedescriptors = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)
    print("[0] Stop all")
    print("[1] Start left")
    print("[2] Start right")
    print("[3] Stop left")
    print("[4] Stop right")

    while not rospy.is_shutdown():
        # listener = Listener(
        #     on_press=on_press,
        #     on_release=on_release)
        #
        # listener.start()

        key = sys.stdin.read(1)[0]
        if key  == "0" or key == " ":
            print("Stop all");
            msg.data = 0
        elif key  == "1":
            print("Start left");
            msg.data = 1
        elif key  == "2":
            print("Start right");
            msg.data = 2
        elif key  == "3":
            print("Stop left");
            msg.data = 3
        elif key  == "4":
            print("Stop right");
            msg.data = 4
        else:
            continue
        pub.publish(msg)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN,filedescriptors)
    
if __name__ == '__main__':
    main()
