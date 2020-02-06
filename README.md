# 2020bot
This is the source code for the ESHS Potatoes' 2020 FRC Infinite Recharge
robot.

# Subsystem breakdown

## Drive subsystem
**Swerve drive**
- Motors:
  * 4x Neo
  * 4x Neo 550
- Controllers:
  * 8x Spark Max
- Sensors:
  * Gyro
  * Accelerometer

## Climbing subsystem
- Motors: 2x Andymark gear motors
- Controllers: 2x standard Spark
- Other:
  * 2x tape measure belts with hooks at the end

## "Wheel of Fortune" subsystem
- Motors: 1x Neo 550
- Controllers: 1x Spark Max
- Other:
  * Pliant wheel
  * Pneumatics (to raise and lower the wheel)
  * Color sensor (to control the wheel's final color)

## Intake subsystem
- Motors: 1x Neo 550
- Controllers: 1x Spark Max
- Other:
  * Pneumatics (to raise and lower the entire intake chamber)
  * Uses right-handed Mecanum wheels, left-handed Mecanum wheels, and Omniwheels
  in the center to direct the power cells.

## Belt subsystem
- Motors: 2x Mini-CIM
- Controllers: 2x standard Spark
- Sensors: **???** (for ball detection)
- Other:
  * Smooth belts
  * Pneumatic indexer, to temporarily stop the balls from entering the shooter
    (this allows shot separation)

## Shooting subsystem
- Motors: 2x Neo
- Controller: 2x Spark Max
- Other:
  * 2x flywheels (oriented vertically)
  * 1x Vision camera (preferably Limelight)

## Total
- Motors: 16 (6 Neo, 6 Neo 550, 2 Mini-CIM, 2 Andymark gear)
- Motor controllers: 16 (12 Spark Max, 4 standard Spark)
