#!/usr/bin/env python2.7
import sys
from naoqi import ALProxy


def main(args):
    try:
        leds_proxy = ALProxy("ALLeds", args[1], 9559)

        # Turning all eyes leds off
        leds_proxy.off("FaceLeds")

        # Setting the colors for the happy expression
        happy_color1 = 0x00ff00ff  # magenta, decimal: 16711935
        happy_color2 = 0x00969600  # yellow, decimal: 9868800
        happy_color3 = 0x0000ff00  # green, decimal: 65280
        happy_color4 = 0x00ff7800  # orange, decimal: 16742400

        # Duration of the expression and number of steps
        total_duration = int(args[2])
        happy_time = 0.21
        steps = int(total_duration/(happy_time*4))

        # Creating the colors and times arrays
        colors1 = []
        colors2 = []
        colors3 = []
        colors4 = []
        times = []
        for i in range(0, steps):
            colors1.append(happy_color1)
            colors1.append(happy_color2)
            colors1.append(happy_color3)
            colors1.append(happy_color4)

            colors2.append(happy_color4)
            colors2.append(happy_color1)
            colors2.append(happy_color2)
            colors2.append(happy_color3)

            colors3.append(happy_color3)
            colors3.append(happy_color4)
            colors3.append(happy_color1)
            colors3.append(happy_color2)

            colors4.append(happy_color2)
            colors4.append(happy_color3)
            colors4.append(happy_color4)
            colors4.append(happy_color1)

            times.append(happy_time)
            times.append(happy_time)
            times.append(happy_time)
            times.append(happy_time)

        # Executing the pattern
        leds_proxy.post.fadeListRGB("happy_leds1", colors1, times)
        leds_proxy.post.fadeListRGB("happy_leds2", colors2, times)
        leds_proxy.post.fadeListRGB("happy_leds3", colors3, times)
        leds_proxy.fadeListRGB("happy_leds4", colors4, times)

        # Turning all eyes leds off
        leds_proxy.off("FaceLeds")

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main(sys.argv)