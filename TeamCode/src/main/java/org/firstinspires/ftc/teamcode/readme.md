# RobotHardware #
This class serves as an initializer and access point for all of the hardware components on the robot, such as
[DriveTrain][1], [LinearSlides][2], [Manipulator][3], and [Vision][4] system. Its purpose is to simplify the process
of creating and modifying robot hardware in OpModes. Instead of manually redeclaring hardware variables for each
OpMode, you can simply just create a RobotHardware object to quickly set up all hardware components. This approach
expedites the initialization process and facilitates easy modifications, allowing changes across multiple
OpModes by simply adjusting the HardwareMap class, rather than having to modify individual OpMode variables.

## Fields ##
### myOpMode ###
`private LinearOpMode myOpMode`

This value is the OpMode that the RobotHardware was created in. It is passed to the object when it is initially created
via the object's constructor. This is beneficiary as it allows the RobotHardware object to access the Telemetry and
HardwareMap of the OpMode it was created in, which allows the RobotHardware to actually initialize the robot's hardware.

### driveTrain ###
`public DriveTrain driveTrain`

An object used to store all of the DriveTrain's hardware and methods. This value is null until `init()` is called,  
since that is where this value is initialized.  
(See [DriveTrain](#driveTrainClassRef) for more info)

### linearSlide ###
`public LinearSlide linearSlide`

An object used to store all of the LinearSlide's hardware and methods. This value is null until `init()` is called,  
since that is where this value is initialized.  
(See [LinearSlide](#linearSlideClassRef) for more info)

### manipulator ###
`public Manipulator manipulator`

An object used to store all of the Manipulator's hardware and methods. This value is null until `init()` is called,  
since that is where this value is initialized.  
(See [Manipulator](#manipulatorClassReff) for more info)

### aprilTagDetection ###
`public Vision aprilTagDetection`

An object used to store all of the  hardware and methods. This value is null until `init()` is called,  
since that is where this value is initialized.  
(See [Vision](#visionClassRef) for more info)
## Methods ##
### init() ###
`public init()`

This method calls all of the initialization methods of all of the hardware on the robot, which gives the OpMode
access to all of the above listed classes. Additionally, this method will tell the user via the telemetry that
the hardware has been successfully initialized viw the Telemetry on the DriverHub.

<br/>
<br/>
<br/>

<div id="driveTrainClassRef"></div>

# DriveTrain #
This class serves as a container for all of the DriveTrain's hardware and methods. 

## Fields ##
### cordinateSystem ###
`public CoordinateSystem cordinateSystem`

An object used to keep track of the robot's position on the field. This value is null until `init()` is called,  
since that is where this value is initialized.
(See [CoordinateSystem]["#coordinateSystemClassReference] for more info)

### cordinateSystem ###
`public CoordinateSystem cordinateSystem`

An object used to keep track of the robot's position on the field. This value is null until `init()` is called,  
since that is where this value is initialized.
<br/>
<br/>
<br/>

<div id="coordinateSystemClassReference"></div>

# CoordinateSystem #
This class is used by the [DriveTrain][1] class in order to keep track of the robot's position the field.
The reason that a coordinate system is useful in the first place is because it makes writing autonomous code
significantly easier. The reason we don't just make this part of the [DriveTrain][1] class is because it makes the
code cleaner and overall easier to work with.

## Methods ##
### updateRobotPosition() ###
`public void updateRobotPosition(double currentRightFrontPosition, double currentRightBackPosition,
&nbsp; double currentLeftFrontPosition, double currentLeftBackPosition)`

This method determines where the robot moved to on the field and then updates the robot's position so that 
it's equal to where it moved to on the field. This is accomplished by finding the difference between the current 
encoder counts of the wheels and what the encoder counts of the wheels were the last time this function was called. 
By getting these values, we can then use the below listed equation in order to figure out how much the robot moved in each direction.

*xChange = $\frac{(leftFrontChange + rightBackChange) - (leftBackChange + rightFrontChange)}{4}$*

*yChange = $\frac{rightFrontChange + rightBackChange + leftFrontChange + leftBackChange}{4}$*

Once we calculate how far right/left (X) and forwards/backwards (Y) the robot moved, we proceed to convert 
these values to inches to make it easier to debug and measure the degree to which slippage is an issue with your wheels. 
After this, we then

### getPosition() ###
`public double[] getPosition()`

This method returns an array containing the robot's X and Y coordinates (in inches) and the robot's 
\rotation (in radians)

[1]: RobotSystems/Subsystems/DriveTrain.java
[1.1]: RobotSystems/Subsystems/SubsystemEnums/DriveMode.java
[2]: RobotSystems/Subsystems/LinearSlide.java
[2.1]: RobotSystems/Subsystems/SubsystemEnums/LinearSlideStage.java
[3]: RobotSystems/Subsystems/Manipulator.java
[4]: RobotSystems/Subsystems/Vision.java