import argparse
from math import fabs, sin, sqrt, asin, atan, tan, pi
from textwrap import dedent


degrees_to_radians = (2 * pi) / 360
radians_to_degrees = 360 / (2 * pi)


def sgn(x: float) -> int:
    return 1 if x > 0 else -1 if x < 0 else 0


def inverse_snake_drive(width: float, length: float, theta: float) -> float:
    """
    Performs the inverse snake drive calculation: given a wheelbase of length
    x width meters and a front outer swerve module angle of theta degrees,
    returns the necessary turning radius in meters.  This can be useful for
    determining what the minimum and maximum turning radii should be during
    snake drive.
    """
    theta *= degrees_to_radians
    x = length / (2 * fabs(tan(theta))) - width

    print(f"x:      {x:,} meters")

    radius = sqrt(x**2 + (length / 2)**2)

    phi = asin((width * sin(theta)) / radius)
    print(f"phi:    {radians_to_degrees * phi:.6}°")
    print(f"Front outer wheel angle: {radians_to_degrees * theta:}°")
    print(f"front inner wheel angle: {radians_to_degrees * (theta + phi)}°")

    return radius * sgn(theta)


def snake_drive(width: float, length: float, radius: float) -> (float, float, float, float):
    """
    Performs the snake drive calculation: given a wheelbase of length x width
    meters and a(n inner) turning radius in meters, calculates the angles of
    the four swerve modules, in degrees, as a 4-tuple.

    As in the 2020bot Java code, the angles are returned in the following
    order:

      1. Front left
      2. Back left
      3. Back right
      4. Front right

    The turning radius can be positive (indicating a clockwise turn) or
    negative (indicating a counterclockwise turn.)  Positive angles indicate
    that the module is turning clockwise.
    """
    x = sqrt(radius**2 - (length / 2)**2)
    theta = atan(length / (2 * (x + width)))
    phi = asin((width * sin(theta)) / radius)

    theta *= radians_to_degrees
    theta *= sgn(radius)
    phi *= radians_to_degrees

    print(f"theta:  {theta:.6f}°\nphi:    {phi:.6f}°\nx:      {x} meters")

    front_left, back_left, back_right, front_right = 0, 0, 0, 0
    if radius < 0:
        # Turning left.
        front_left = theta + phi
        front_right = theta
    else:
        front_left = theta
        front_right = theta + phi

    back_left = -front_left
    back_right = -front_right

    return front_left, back_left, back_right, front_right


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=dedent("""
    A simple calculator for the forward and inverse snake drive formulas.
    """), formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument("width",
                        metavar="WIDTH",
                        type=float,
                        help=dedent("""
                        The width of the wheelbase (i.e., the distance from
                        the bottom of a wheel on the left to the bottom of the
                        wheel across from it), in meters."""))

    parser.add_argument("length",
                        metavar="LENGTH",
                        type=float,
                        help=dedent("""
                        The length of the wheelbase (i.e., the distance from
                        the bottom of a wheel on the front to the bottom of
                        the wheel behind it), in meters."""))

    parser.add_argument("radius-or-angle",
                        metavar="RADIUS-OR-ANGLE",
                        type=float,
                        help=dedent("""See the --mode argument."""))

    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument("-m",
                            "--mode",
                            choices=["normal", "inverse"],
                            default="normal",
                            help=dedent("""
                            This flag determines how we will interpret the
                            third argument.

                            If the mode is 'normal' (the default), the third
                            argument is interpreted as a(n inner) turning
                            radius, in meters.  This script will then
                            calculate the angles of the four swerve wheels.

                            If the mode is 'inverse', the third argument is
                            interpreted as the swerve angle of the front outer
                            wheel, in degrees.  This script will then
                            determine the (inner) turn radius needed to get
                            the front outer wheel to adopt that angle.

                            Either way, the third argument and the results are
                            both signed: a negative radius or negative angle
                            swerves counterclockwise, and a positive radius or
                            angle swerves clockwise.  """))

    mode_group.add_argument("-i",
                        "--inverse",
                            action="store_true",
                            help=dedent("""Short for --mode=inverse."""))
    mode_group.add_argument("-n",
                            "--normal",
                            action="store_true",
                            help=dedent("""Short for --mode=normal."""))

    mode = "normal"
    args = parser.parse_args()
    if args.inverse:
        mode = "inverse"
    elif args.normal:
        mode = "normal"
    elif args.mode:
        mode = args.mode

    print(f"Mode:   {mode}")
    print(f"Length: {args.length:} meters")
    print(f"Width:  {args.width:} meters")

    if mode == "normal":
        radius = vars(args)['radius-or-angle']
        print(f"Radius: {radius:} meters")
        if fabs(radius) < args.length / 2:
            print(f"Error: Turning radius cannot be smaller than half the length ({args.length / 2} meters).")
        else:
            front_left, back_left, back_right, front_right = snake_drive(args.width, args.length, radius)

            print(f"\n{'Turn Direction':^26}")
            print(f"{'-' * 14:^26}")
            print(f"{'    Clockwise --->' if radius > 0 else '<--- Counterclockwise':^26}")

            print(dedent(f"""
            {'FRONT_LEFT':<13}  {'FRONT_RIGHT':<13}
            -------------  -------------
            {front_left:>12.6f}°  {front_right:>12.6f}°

            {'BACK_LEFT':<13}  {'BACK_RIGHT':<13}
            -------------  -------------
            {back_left:>12.6f}°  {back_right:>12.6f}°
            """))
    else:
        front_outer_angle = vars(args)['radius-or-angle']
        radius = inverse_snake_drive(args.width, args.length, front_outer_angle)
        print(f"\nRadius: {radius:,.6} meters ({'counter' if radius < 0 else ''}clockwise)")
