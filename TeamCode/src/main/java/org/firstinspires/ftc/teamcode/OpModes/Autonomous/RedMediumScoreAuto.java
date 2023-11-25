package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystems.CommandScheduler;
import org.firstinspires.ftc.teamcode.RobotSystems.Commands.ManipulatorAndSlideCommands.CloseManipulator;
import org.firstinspires.ftc.teamcode.RobotSystems.Commands.ManipulatorAndSlideCommands.MoveLinearSlides;
import org.firstinspires.ftc.teamcode.RobotSystems.Commands.ManipulatorAndSlideCommands.OpenManipulator;
import org.firstinspires.ftc.teamcode.RobotSystems.Commands.PositionCommands.DriveToStack;
import org.firstinspires.ftc.teamcode.RobotSystems.Commands.PositionCommands.DriveToTeamBackdrop;
import org.firstinspires.ftc.teamcode.RobotSystems.Commands.CalibrationCommands.SetupHardware;
import org.firstinspires.ftc.teamcode.RobotSystems.Commands.WaitCommand;
import org.firstinspires.ftc.teamcode.RobotSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotSystems.Subsystems.SubsystemEnums.LinearSlideStage;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutoParams;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousEnums.PixelStack;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AutonomousEnums.StartingLocation;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Red-MediumScore-Autonomous", group = "Red", preselectTeleOp = "Robot Centric Mecanum")
public class RedMediumScoreAuto extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);;
    AutoParams autoParams = new AutoParams(StartingLocation.RED_BACKDROP, PixelStack.RED_SIDE_STACK_1);
    CommandScheduler commandList = new CommandScheduler(robot, telemetry, autoParams);

    @Override
    public void runOpMode() {

        // Initialize RobotHardware
        robot.init();

        // initialize commands
        commandList.addCommand(new SetupHardware());
        commandList.addCommand(new OpenManipulator());
        commandList.addCommand(new DriveToStack(1));
        commandList.addCommand(new MoveLinearSlides(LinearSlideStage.GROUND_STAGE));
        commandList.addCommand(new WaitCommand(2000, TimeUnit.MILLISECONDS));
        commandList.addCommand(new CloseManipulator());
        commandList.addCommand(new DriveToTeamBackdrop(1)); // Drives through stage door
        commandList.addCommand(new MoveLinearSlides(LinearSlideStage.HIGH_STAGE));
        commandList.addCommand(new WaitCommand(2000, TimeUnit.MILLISECONDS));
        commandList.addCommand(new OpenManipulator());

        // Wait for the OpMode to be started.
        waitForStart();

        // Run the commands
        commandList.run();
    }
}
