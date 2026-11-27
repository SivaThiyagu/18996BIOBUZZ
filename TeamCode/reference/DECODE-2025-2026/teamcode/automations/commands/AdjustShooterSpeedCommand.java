package org.firstinspires.ftc.teamcode.automations.commands;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.ShooterCalculations;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class AdjustShooterSpeedCommand extends CommandBase {
    private Robot robot;
    private Pose pose;

    public AdjustShooterSpeedCommand(Robot robot) {
        this.robot = robot;
    }

    public AdjustShooterSpeedCommand(Robot robot, Pose pose) {
        this.robot = robot;
        this.pose = pose;
    }

    @Override
    public void initialize() {
        robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.ACTIVE);
    }

    @Override
    public void execute() {
        if (pose == null) {
            robot.shooter.setTargetTps(ShooterCalculations.getSpeed(robot.localizer.getDistanceToGoal()));
        } else {
            robot.shooter.setTargetTps(ShooterCalculations.getSpeed(robot.localizer.getDistanceToGoal(pose)));
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
