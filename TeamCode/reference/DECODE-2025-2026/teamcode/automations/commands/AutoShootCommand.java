package org.firstinspires.ftc.teamcode.automations.commands;

import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.ShooterCalculations;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class AutoShootCommand extends CommandBase {
    private final Robot robot;

    private Timer overrideTimer;

    private int shootingState;

    CommandBase progressTransferCommand = new InstantCommand();

    public AutoShootCommand(Robot robot) {
        this.robot = robot;
        overrideTimer = new Timer();
    }

    @Override
    public void initialize() {
        setShootingState(0);
        overrideTimer.resetTimer();
    }

    @Override
    public void execute() {
        if (robot.drive.follower.getCurrentTValue() < 0.995) overrideTimer.resetTimer();

        if (shootingState == 0) {
            if ((Math.abs(robot.shooter.getError()) < 200 && Math.abs(robot.drive.follower.getHeadingError()) < 0.1 && !robot.drive.follower.isBusy() && robot.drive.follower.getTranslationalError().getMagnitude() < 5) || overrideTimer.getElapsedTime() > 3000) {
                progressTransferCommand = new ProgressTransferCommand(robot, true);
                CommandScheduler.getInstance().schedule(progressTransferCommand);

                setShootingState(-1);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return (shootingState == -1 && progressTransferCommand.isFinished());
    }

    public void setShootingState(int shootingState) {
        this.shootingState = shootingState;
    }
}
