#!/usr/bin/env python2.7
import sys
from naoqi import ALProxy


def main(args):
    try:
        leds_proxy = ALProxy("ALLeds", args[1], 9559)

        # LED group for sad expression
        sad_leds = ["FaceLedLeft1", "FaceLedLeft2",
                    "FaceLedLeft3", "FaceLedLeft4",
                    "FaceLedRight1", "FaceLedRight2",
                    "FaceLedRight3", "FaceLedRight4"]
        leds_proxy.createGroup("sad_leds", sad_leds)

        # LED groups for happy expression
        happy_leds1 = ["FaceLedLeft0", "FaceLedLeft1",
                       "FaceLedRight0", "FaceLedRight1"]
        leds_proxy.createGroup("happy_leds1", happy_leds1)

        happy_leds2 = ["FaceLedLeft2", "FaceLedLeft3",
                       "FaceLedRight2", "FaceLedRight3"]
        leds_proxy.createGroup("happy_leds2", happy_leds2)

        happy_leds3 = ["FaceLedLeft4", "FaceLedLeft5",
                       "FaceLedRight4", "FaceLedRight5"]
        leds_proxy.createGroup("happy_leds3", happy_leds3)

        happy_leds4 = ["FaceLedLeft6", "FaceLedLeft7",
                       "FaceLedRight6", "FaceLedRight7"]
        leds_proxy.createGroup("happy_leds4", happy_leds4)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main(sys.argv)