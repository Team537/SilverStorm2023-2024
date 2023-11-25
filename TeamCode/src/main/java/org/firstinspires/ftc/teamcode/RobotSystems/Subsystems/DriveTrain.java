package org.firstinspires.ftc.teamcode.RobotSystems.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.SubsystemEnums.DriveMode;
import org.firstinspires.ftc.teamcode.Utility.InputController;
import org.firstinspires.ftc.teamcode.Utility.PIDController;
import org.firstinspires.ftc.teamcode.Utility.TurnToPIDController;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

public class DriveTrain {
    private LinearOpMode myOpMode = null;
    public CoordinateSystem coordinateSystem = null;
    public PIDController pidController = null; // Public so that it can be tuned within an OpMode.
    private DriveMode current_drive_mode = DriveMode.DEFAULT_DRIVE;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private boolean fieldCentric = false;
    public static final double STRAFE_OFFSET = 1.1;
    public static final double maxAutonomousSpeed = 0.8;
    public DriveTrain(LinearOpMode opMode) {myOpMode = opMode; }

    /**
     * Initializes all of the driveTrain's hardware.
     */
    public void init() {

        // Initialize hardware values
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "right_back_drive");
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "left_back_drive");

        // Set the ZeroPowerBehaviour to BRAKE so that the robot stops faster.
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run all motors using RUN_WITH_ENCODER so that we can use encoder related methods.
        // Commented out for bug testing purposes.
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Note: Most motors have one side reverse.
        // Due to this, we need to reverse one side of the motors so that the robot can drive straight.
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Coordinate System
        coordinateSystem = new CoordinateSystem();
        coordinateSystem.initializeImu(myOpMode.hardwareMap);

        // Initialize PID Controller
        pidController = new PIDController(0.01, 0, 0.003);

        // Tell the user that this subsystem has been successfully initialized.
        myOpMode.telemetry.addData("->", "DriveTrain successfully initialized");
    }

    /**
     * Calculates the powers for the motors to achieve the requested
     * motion: Drive front and back, and robot rotation.
     *
     * @param driveFrontBack Determines how far forwards or backwards the robot has to move.
     * @param driveLeftRight Determines how far left or right the robot moves.
     * @param rotation Determines how much the robot should turn.
     **/
    public void driveRobot(double driveFrontBack, double driveLeftRight, double rotation) {

        // Declare variables so that their value can be calculated differently based on whether or not
        // field centric is enabled.
        double newDriveFrontBack;
        double newDriveLeftRight;
        double newRotation;

        /*
        If field centric is enabled then rotate the provided values so that the robot can drive
        independent form it's rotation. Otherwise, just set the values to whatever was provided by
        the user.
         */
        if (fieldCentric) {
            // Get the robot's rotation so the below calculations will be accurate.
            double robotRotation = coordinateSystem.getPosition().rotation;

            // Rotate the values so that the robot can drive in a field centric way.
            newDriveFrontBack = driveLeftRight * Math.cos(robotRotation) - driveFrontBack * Math.sin(robotRotation);
            newDriveLeftRight = driveLeftRight * Math.sin(robotRotation) + driveFrontBack * Math.cos(robotRotation);
        } else {
            newDriveFrontBack = driveFrontBack;
            newDriveLeftRight = driveLeftRight;
        }
        newRotation = rotation;

        // Get the robots speed multiplier
        double speedMultiplier = current_drive_mode.speedMultiplier();

        // Recalculate the values to adjust for the speed multiplier and strafe offset.
        newDriveFrontBack *= speedMultiplier;
        newDriveLeftRight *= STRAFE_OFFSET * speedMultiplier;
        newRotation *= speedMultiplier;

        // Calculate the value that all o the values need to be divided by in order for the robot's
        // wheels to maintain a consistent ratio. This is required because motor power is capped at 1.
        double denominator =  Math.max(Math.abs(newDriveFrontBack) + Math.abs(newDriveLeftRight) + Math.abs(newRotation), 1);

        // Calculate the power levels for each motor.
        double rightFrontPower = (newDriveFrontBack - newDriveLeftRight - newRotation) / denominator;
        double rightBackPower = (newDriveFrontBack + newDriveLeftRight - newRotation)/ denominator;
        double leftFrontPower = (newDriveFrontBack + newDriveLeftRight + newRotation) / denominator;
        double leftBackPower = (newDriveFrontBack - newDriveLeftRight + newRotation) / denominator;

        // Use the pre-existing function to apply the power to the wheels.
        setDrivePower(rightFrontPower, rightBackPower, leftFrontPower, leftBackPower);
    }

    /**
     * Apply the specified power levels to their associated motors. Allows the robot to move and rotate.
     *
     * @param rightFrontPower The power that will be applied to the right front motor.
     * @param rightBackPower The power that will be applied to the right back motor.
     * @param leftFrontPower The power that will be applied to the left front  motor.
     * @param leftBackPower The power that will be applied to the left back  motor.
     */
    public void setDrivePower(double rightFrontPower, double rightBackPower, double leftFrontPower, double leftBackPower) {

        // Apply the power levels to the motors.
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);
        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);

        // Update coordinate position
        coordinateSystem.updateRobotPosition(
                rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition(),
                leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition());

       // Tell the user what position the robot is located at.
        displayRobotPosition();
    }

    /**
     * Drives the robot to the specified position and rotation in the coordinate system.
     *
     * @param targetPosition The position on the field you want the robot to drive to.
     */
    public void driveRobotToPosition(RobotPosition targetPosition) {

        // Convert the target position's rotation to radians.
        targetPosition.rotation = Math.toRadians(targetPosition.rotation);

        // Get distance to the target from the robot's coordinate system and rotate it relative
        // to the robot's rotation.
        RobotPosition targetPositionDistance = coordinateSystem.getDistanceToPosition(targetPosition);

        // Convert the distance to the target position from inches and radians to encoder counts.
        targetPositionDistance.multiplyBy(CoordinateSystem.TICKS_PER_INCH);

        // Move robot to the rotated position.
        driveToRelativePosition(targetPositionDistance);
    }

    /**
     * Moves the robot a specified amount of inches in any direction relative to the robot.
     *
     * @param targetPosition The position you want the robot to move to.
     */
    public void driveToRelativePosition(RobotPosition targetPosition) {

        // Get the robot's current position
        RobotPosition robotPosition = coordinateSystem.getPosition();

        // Helps prevent slippage through acceleration and deceleration
        InputController inputController = new InputController(.2, 1);
        pidController.reset();

        // Create a PIDController for each individual motor. (Gives us greater control over each motor)
        PIDController rightFrontController = new PIDController(pidController);
        PIDController rightBackController = new PIDController(pidController);
        PIDController leftFrontController = new PIDController(pidController);
        PIDController leftBackController = new PIDController(pidController);

        // Calculate encoder changes for each individual motor
        int rightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (targetPosition.y - targetPosition.x);
        int rightBackTarget = rightBackDrive.getCurrentPosition() + (int) (targetPosition.y + targetPosition.x);
        int leftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (targetPosition.y + targetPosition.x);
        int leftBackTarget = leftBackDrive.getCurrentPosition() + (int) (targetPosition.y - targetPosition.x);

        // Keep track of each motor's last encoder count in case we have to rotate while moving.
        int lastRightFront = rightFrontDrive.getCurrentPosition();
        int lastRightBack = rightBackDrive.getCurrentPosition();
        int lastLeftFront = leftFrontDrive.getCurrentPosition();
        int lastLeftBack = leftBackDrive.getCurrentPosition();

        // Keep moving until each motor is within 6 encoder ticks of their desired position.
        while (myOpMode.opModeIsActive() &&
                Math.abs(rightFrontTarget - rightFrontDrive.getCurrentPosition()) > 6 ||
                Math.abs(rightBackTarget - rightBackDrive.getCurrentPosition()) > 6 ||
                Math.abs(leftFrontTarget - leftFrontDrive.getCurrentPosition()) > 6 ||
                Math.abs(leftBackTarget - leftBackDrive.getCurrentPosition()) > 6) {

            // Gives the robot time to accelerate to full speed.
            double smoothingFactor = inputController.smoothInput(maxAutonomousSpeed);

            // Calculate each motor's speed based on how close they are to their target position.
            double rightFrontPower = rightFrontController.update(rightFrontTarget, rightFrontDrive.getCurrentPosition());
            double rightBackPower = rightBackController.update(rightBackTarget, rightBackDrive.getCurrentPosition());
            double leftFrontPower = leftFrontController.update(leftFrontTarget, leftFrontDrive.getCurrentPosition());
            double leftBackPower = leftBackController.update(leftBackTarget, leftBackDrive.getCurrentPosition());

            // Multiply each power by smoothingFactor to allow for gentle acceleration.
            rightFrontPower *= smoothingFactor;
            rightBackPower *= smoothingFactor;
            leftFrontPower *= smoothingFactor;
            leftBackPower *= smoothingFactor;

            // Apply the powers to the motors.
            setDrivePower(rightFrontPower, rightBackPower, leftFrontPower, leftBackPower);

            // Update last encoder position.
            lastRightFront = rightFrontDrive.getCurrentPosition();
            lastRightBack = rightBackDrive.getCurrentPosition();
            lastLeftFront = leftFrontDrive.getCurrentPosition();
            lastLeftBack = leftBackDrive.getCurrentPosition();

            // Make sure the robot is facing within 1 degree of the direction it was facing when it started moving.
            if (Math.abs(coordinateSystem.getDistanceToRotation(robotPosition.rotation)) < Math.toRadians(1)) {

                // Rotate the robot back into position.
                rotateTo(robotPosition.rotation);

                // Account for the change in the encoder counts caused by the rotation.
                rightFrontTarget += rightFrontDrive.getCurrentPosition() - lastRightFront;
                rightBackTarget += rightBackDrive.getCurrentPosition() - lastRightBack;
                leftFrontTarget += leftFrontDrive.getCurrentPosition() - lastLeftFront;
                leftBackTarget += leftBackDrive.getCurrentPosition() - lastLeftBack;
            }

            // Update robot's position
            coordinateSystem.updateRobotPosition(
                    rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition(),
                    leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition());

            // Show the driver the robot's current position
            displayRobotPosition();
        }

        // Stop the robot from moving //
        setDrivePower(0, 0, 0, 0);

        // Rotate the robot to the desired position if the robot is not already within 1 degree of
        // the target's rotation.
        if (Math.abs(coordinateSystem.getDistanceToRotation(targetPosition.rotation)) < Math.toRadians(1)) {
            rotateTo(targetPosition.rotation);
        }
    }

    /**
     * Rotates the robot so that its facing the provided direction. Accounts for any disturbances that
     * might disrupt this process by using a PID controller, which allows us to respond to these disturbances.
     *
     * @param targetAnge The angle (in radians) you want the robot to be facing.
     */
    public void rotateTo(double targetAnge) {

        // Create a TurnToPIDController to handle the robot's rotation.
        TurnToPIDController rotationController = new TurnToPIDController(0.0135, 0, 0.003);

        // Calculate the difference between the robot's current
        double error = coordinateSystem.getDistanceToRotation(targetAnge);

        // Continues rotating until the robot is within 1 degree of it's targetAngle or until
        // the opMode is stopped.
        while (myOpMode.opModeIsActive() && Math.abs(error) > Math.toRadians(1)) {

            // Update the error (Difference between targetPosition and currentPosition)
            error = coordinateSystem.getDistanceToRotation(targetAnge);

            // Calculate the power required to turn the robot.
            double targetPower = rotationController.update(error);

            // Apply motor powers to each motor.
            setDrivePower(targetPower, targetPower, -targetPower, -targetPower);

            // Update robot's position
            coordinateSystem.updateRobotPosition(
                    rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition(),
                    leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition());

            // Debug //
            myOpMode.telemetry.addLine("---PID Related Values---");
            myOpMode.telemetry.addData("Error (Radians) ", error);
            myOpMode.telemetry.addData("Error (Degrees) ", Math.toDegrees(error));

            // Show the driver the robot's current position
            displayRobotPosition();
        }

        // Stop the robot from moving
        setDrivePower(0, 0, 0, 0);
    }

    /**
     * Displays what position the robot is currently located at on the field on the Telemetry.
     */
    private void displayRobotPosition() {

        // Get the robot's position
        RobotPosition robotPosition = coordinateSystem.getPosition();

        // Convert the robot's rotation to degrees to make it easier for a human to understand.
        double robotRotationDegrees = Math.toDegrees(robotPosition.rotation);

        // Tell the user their current position.
        myOpMode.telemetry.addLine("---Settings---");
        myOpMode.telemetry.addData("DriveMode: ", current_drive_mode);
        myOpMode.telemetry.addData("Speed Multiplier", current_drive_mode.speedMultiplier());
        myOpMode.telemetry.addData("FieldCentricEnabled", fieldCentric);

        myOpMode.telemetry.addLine("---PID Coefficients---");
        myOpMode.telemetry.addData("Kp ", pidController.getProportionalTerm());
        myOpMode.telemetry.addData("Ki ", pidController.getIntegralTerm());
        myOpMode.telemetry.addData("kd ", pidController.getDerivativeTerm());

        myOpMode.telemetry.addLine("---Robot Position---");
        myOpMode.telemetry.addData("Robot X", robotPosition.x);
        myOpMode.telemetry.addData("Robot Y", robotPosition.y);
        myOpMode.telemetry.addData("Robot Rotation (Radians)", robotPosition.rotation);
        myOpMode.telemetry.addData("Robot Rotation (Degrees)", robotRotationDegrees);

        myOpMode.telemetry.addLine("---Experimental Values---");
        myOpMode.telemetry.addData("Encoder-Based rotation in radians ", coordinateSystem.rotationTest);
        myOpMode.telemetry.addData("Encoder-Based rotation in radians ", Math.toDegrees(coordinateSystem.rotationTest));

        myOpMode.telemetry.update();
    }

    /**
     * Switch the robot from one mode to another, (allowing for the user to go slower or faster)
     *
     * @param drive_mode The drive mode the user wants to change the robot to.
     */
    public void setDriveMode(DriveMode drive_mode) {
        current_drive_mode = drive_mode;
    }

    /**
     * Toggles field centric drive on and off.
     */
    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
    }

    /**
     * Resets the IMU to it's original position. This helps counteract IMU drift.
     */
    public void resetIMU() {
        coordinateSystem.resetIMU();
    }

}