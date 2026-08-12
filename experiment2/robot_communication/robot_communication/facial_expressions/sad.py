#!/usr/bin/env python2.7
import sys
from naoqi import ALProxy


def main(args):
    try:
        leds_proxy = ALProxy("ALLeds", args[1], 9559)

        # Turning all eyes leds off
        leds_proxy.off("FaceLeds")

        # Setting the colors for the happy expression
        sad_color1 = 0x00000046  # decimal: 70
        sad_color2 = 0x00000064  # decimal: 100
        # sad_color1 = 0x000000b3  # decimal: 179
        # sad_color2 = 0x000000ff  # decimal: 255
        # sad_color1 = 0x00000000  # decimal: 0
        # sad_color2 = 0x000000ff  # decimal: 255

        # Duration of the expression and number of steps
        total_duration = int(args[2])
        sad_time = 0.3
        steps = int(total_duration/(sad_time*2))

        # Creating the colors and times arrays
        colors = []
        times = []
        for i in range(0, steps):
            colors.append(sad_color1)
            times.append(sad_time)
            colors.append(sad_color2)
            times.append(sad_time)

        # Executing the pattern
        leds_proxy.fadeListRGB("sad_leds", colors, times)

        # Turning all eyes leds off
        leds_proxy.off("FaceLeds")

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main(sys.argv)