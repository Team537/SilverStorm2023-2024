package org.firstinspires.ftc.teamcode.RobotSystems.Commands;

import org.firstinspires.ftc.teamcode.Utility.Timer;

import java.util.concurrent.TimeUnit;

/**
 * WaitCommand waits for a specified duration  in the provided TimeUnit (e.g., 1000 milliseconds,
 * 100 microseconds, 1 second, etc.).
 */
public class WaitCommand extends Commands {
    private Timer timer = null;

    /**
     * Initializes the wait command with the specified duration and time unit.
     *
     * @param time The duration to wait for.
     * @param timeUnit The unit of time for the specified duration.
     */
    public WaitCommand(int time, TimeUnit timeUnit) {
        timer = new Timer(time, timeUnit);
    }

    /**
     * Starts the timer when the command is executed.
     */
    @Override
    public void run() {
        timer.start();
    }

    /**
     * Checks if the wait duration has elapsed.
     *
     * @return True if the specified time has passed since the command started.
     */
    @Override
    public boolean isCommandFinished() {
        return timer.done();
    }
}
