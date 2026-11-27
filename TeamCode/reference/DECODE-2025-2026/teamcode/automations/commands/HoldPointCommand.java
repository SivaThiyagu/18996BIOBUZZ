package org.firstinspires.ftc.teamcode.automations.commands;

import static java.lang.Math.atan2;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.ShooterCalculations;
import org.firstinspires.ftc.teamcode.automations.drive.Drive;
import org.firstinspires.ftc.teamcode.automations.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class HoldPointCommand extends CommandBase {
    private final Robot robot;
    private Drive.DriveMode previousDriveMode;

    public HoldPointCommand(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        previousDriveMode = robot.drive.getDriveMode();

        if (previousDriveMode == Drive.DriveMode.HOLD_POINT) end(true);

        // This monstrosity keeps the same point but changes the heading to point toward goal

        // since the path was already followed, it should be the same
        robot.drive.setHoldPoint(
                robot.localizer.getPose().setHeading(
                        atan2(
                                GlobalDataStorage.goalPose.minus(robot.localizer.getPose()).getY(),
                                GlobalDataStorage.goalPose.minus(robot.localizer.getPose()).getX()
                        )
                )
        );
        robot.drive.setDriveMode(Drive.DriveMode.HOLD_POINT);

    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) robot.drive.setDriveMode(previousDriveMode);
    }
}
