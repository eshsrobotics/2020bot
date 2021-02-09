- Human Drive Schemes
    1. Crab Drive
        - drive(arg 1: x, arg 2: y) - returns array of 4 swerve module states
            - x and y directly control direction vectors of all 4 swerve wheels
            - magnitude of resultant vector determines speed of swerve wheels
            - vector is relative to FRONT OF ROBOT
        - turn(arg 1: turnSpeed) - returns array of 4 swerve module states
            - set the swerve drive wheel orientation to the crab rotation angles
            - sets the swerve drive speeds to the turn speed

    2. Field-Oriented Swerve Drive
        - drive(arg 1: x, arg 2: y) - returns array of 4 swerve module states
            - x and y directly control direction vectors of all 4 swerve wheels
            - magnitude of resultant vector determines speed of swerve wheels
            - vector is relative to FIELD, not robot
        - turn(arg 1: turnSpeed) - returns array of 4 swerve module states
            - set the swerve drive wheel orientation to the crab rotation angles
            - sets the swerve drive speeds to the turn speed
        - driveAndTurn(arg 1: x1, arg 2: y1, arg 3: x2,) - returns array of 4 swerve module states
            - makes drive() and turn() obsolete for ONLY Field-Oriented Swerve
            - theoretical as of now (unsure as to implementation)

    3. Snake Drive
        - drive(arg 1: x, arg 2: y) - returns array of 4 swerve module states
            - sign of y determines robot's direction (forward or backward)
            - maginitude of y determines speed
            - sign of x controls turn direction
            - magnitude of x controls radius of turn


- Computer Drive Schemes
    1. Trajectory Follower
        - getSwerveModuleStates(arg 1: list of SwerveModuleState[4])
            - use the kinematics code to get SwerveModuleStates- use JSON trajectories

    2. Vision Tracker
        - getSwerveModuleStates(arg 1: list of SwerveModuleState[4])
            - go into crab rotation mode; turn left or right until goal angle is 0