from tkinter import Tk

import rclpy
from rclpy.node import Node


class BlankScreenNode(Node):

    def __init__(self):
        super().__init__('blank_screen_node')

        self.window = Tk()
        self.window.attributes('-fullscreen', True)
        self.window.configure(background="#000000")


def main(args=None):

    try:
        rclpy.init(args=args)

        bs = BlankScreenNode()

        while rclpy.ok():
            bs.window.update()
            rclpy.spin_once(bs)

    except KeyboardInterrupt:
        bs.window.destroy()
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()