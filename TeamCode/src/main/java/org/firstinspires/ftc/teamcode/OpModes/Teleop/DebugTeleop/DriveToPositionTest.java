package org.firstinspires.ftc.teamcode.OpModes.Teleop.DebugTeleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.PositionDataTypes.RobotPosition;

@TeleOp (name = "Drive To Position Test", group = "Debug OpModes")
public class DriveToPositionTest extends LinearOpMode {

    private RobotHardware robotHardware = new RobotHardware(this);

    @Override
    public void runOpMode() {

        robotHardware.init();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                robotHardware.driveTrain.driveRobotToPosition(new RobotPosition(0, 14, 0));
            }
            if (gamepad1.dpad_right) {
                robotHardware.driveTrain.driveRobotToPosition(new RobotPosition(14, 0, 0));
            }
            if (gamepad1.dpad_down) {
                robotHardware.driveTrain.driveRobotToPosition(new RobotPosition(0, -14, 0));
            }
            if (gamepad1.dpad_left) {
                robotHardware.driveTrain.driveRobotToPosition(new RobotPosition(-14,0 ,0));
            }

            if (gamepad1.a) {
                robotHardware.driveTrain.driveRobotToPosition(new RobotPosition(0, 0, 0));
            }
        }
    }
}
