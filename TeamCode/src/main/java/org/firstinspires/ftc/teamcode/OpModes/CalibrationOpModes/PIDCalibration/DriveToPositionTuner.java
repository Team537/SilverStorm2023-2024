package org.firstinspires.ftc.teamcode.OpModes.CalibrationOpModes.PIDCalibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Utility.PIDController;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

@TeleOp (name = "\uD83D\uDFE9 Drive To Position Tuner", group = "Calibration Op Modes")
public class DriveToPositionTuner extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    @Override
    public void runOpMode() {

        // Initialize all of the robot's hardware.
        robot.init();

        PIDController pidController = robot.driveTrain.pidController;

        // Wait for the op mode to be started
        waitForStart();

        // Runs while the OpMode is active
        while (opModeIsActive()) {

            // Drive To Positions
            if (gamepad1.dpad_up) {
                robot.driveTrain.driveRobotToPosition(new RobotPosition(0, 24, 0));
            }
            if (gamepad1.dpad_down) {
                robot.driveTrain.driveRobotToPosition(new RobotPosition(0, -24, 0));
            }
            if (gamepad1.dpad_left) {
                robot.driveTrain.driveRobotToPosition(new RobotPosition(-24, 0, 0));
            }
            if (gamepad1.dpad_right) {
                robot.driveTrain.driveRobotToPosition(new RobotPosition(24, 0, 0));
            }

            // Change PID Coefficients //
            // Proportional Term
            if (gamepad1.x) {
                pidController.setKp(pidController.getProportionalTerm() + 0.01);
            }
            if (gamepad1.a) {
                pidController.setKp(pidController.getProportionalTerm() - 0.01);
            }
            // Integral Term
            if (gamepad1.start) {
                pidController.setKi(pidController.getIntegralTerm() + 0.01);
            }
            if (gamepad1.back) {
                pidController.setKi(pidController.getIntegralTerm() - 0.01);
            }
            // Derivative Term
            if (gamepad1.y) {
                pidController.setKd(pidController.getDerivativeTerm() + 0.01);
            }
            if (gamepad1.b) {
                pidController.setKd(pidController.getDerivativeTerm() - 0.01);
            }
        }
    }
}
