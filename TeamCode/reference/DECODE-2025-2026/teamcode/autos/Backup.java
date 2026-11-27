package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.commands.AdjustShooterSpeedCommand;
import org.firstinspires.ftc.teamcode.automations.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.automations.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous
public class Backup extends OpMode {
    private Robot robot;

    private Timer pathTimer, autoTimer;
    private int pathState;

    private Pose startPose, shootPose, cornerPose, cornerSetPose, secondCornerPose, intakeMark3StartPose, intakeMark3EndPose, parkPose;

    public PathChain shootPreload, intakeCorner, reverseCorner, secondIntakeCorner, shootCorner, prepareMark3, intakeMark3, shootMark3, park;

    private Command autoShootCommand = new InstantCommand();
    private Command adjustShooterSpeedCommand = new InstantCommand();
    private Command holdPointCommand = new InstantCommand();

    private double number = 0;

    @Override
    public void init() {
        pathTimer = new Timer();
        autoTimer = new Timer();

        CommandScheduler.getInstance().reset();

        GlobalDataStorage.staticLocalizer = null;
        robot = new Robot(hardwareMap, GlobalDataStorage.autoTeam, true);

        buildPoses(GlobalDataStorage.autoTeam);
        buildPaths();

        robot.drive.setDriveMode(Drive.DriveMode.AUTO);
        robot.localizer.setPose(startPose);

        robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD);

        telemetry.addData("x", robot.localizer.getPose().getX());
        telemetry.addData("y", robot.localizer.getPose().getY());
        telemetry.addData("heading", robot.localizer.getPose().getHeading());
        telemetry.addData("Pose", startPose);
        telemetry.addData("Transfer", robot.transfer.getTransferState());
        telemetry.addData("Power", robot.transfer.transferMotor.get());
        telemetry.addData("Transfer Error", robot.transfer.getError());
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        robot.update();
        CommandScheduler.getInstance().run();

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", robot.localizer.getPose().getX());
        telemetry.addData("y", robot.localizer.getPose().getY());
        telemetry.addData("heading", robot.localizer.getPose().getHeading());
        telemetry.addData("busy", robot.drive.follower.isBusy());
        telemetry.addData("error", robot.shooter.getError());
        telemetry.addData("Transfer", robot.transfer.getTransferState());
        telemetry.addData("Transfer Error", robot.transfer.getError());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.drive.follower.followPath(shootPreload, true);
                robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                adjustShooterSpeedCommand = new AdjustShooterSpeedCommand(robot, shootPose);
                adjustShooterSpeedCommand.schedule();

                autoShootCommand = new AutoShootCommand(robot);
                autoShootCommand.schedule();

                setPathState(2);
                break;
            case 2:
                if (autoShootCommand.isFinished()) {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                    robot.drive.follower.setMaxPower(0.7);
                    robot.drive.follower.followPath(intakeCorner, false);

                    number ++;

                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 2000) {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                    robot.drive.follower.setMaxPower(0.5);
                    robot.drive.follower.followPath(reverseCorner, false);
                    setPathState(4);
                }
                break;
            case 4:
                if (!robot.drive.follower.isBusy()) {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                    robot.drive.follower.setMaxPower(0.8);
                    robot.drive.follower.followPath(secondIntakeCorner, false);

                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTime() > 1000) {

                    robot.drive.follower.setMaxPower(0.8);
                    robot.drive.follower.followPath(shootCorner, true);

                    autoShootCommand = new AutoShootCommand(robot);
                    autoShootCommand.schedule();

                    if (number < 2) setPathState(2);
                    else setPathState(10);
                }
                break;
            case 10:
                if (autoShootCommand.isFinished()) {
                    robot.drive.follower.followPath(park);
                    robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.IDLE);

                    setPathState(-1);
                }
                break;
        }
    }

    public void buildPaths() {
        shootPreload = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                shootPose
                        )
                ).setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .addParametricCallback(0.3, ()-> robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD))
                .build();

        intakeCorner = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPose,
                                cornerPose
                        )
                ).setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        reverseCorner = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                cornerPose,
                                cornerSetPose
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        secondIntakeCorner = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                cornerSetPose,
                                secondCornerPose
                        )
                )
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        shootCorner = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                secondCornerPose,
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(secondCornerPose.getHeading(), shootPose.getHeading())
                .build();

        park = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPose,
                                parkPose
                        )
                ).setConstantHeadingInterpolation(parkPose.getHeading())
                .build();
    }

    public void buildPoses(Robot.Team team) {
        // BUILT FOR BLUE
        startPose = new Pose(56.9, 10.3, Math.toRadians(90));
        // 56.1 9.4

        shootPose = new Pose(61.2, 20.2, Math.toRadians(111.5));

        // 16 11.7 -164.3

        cornerPose = new Pose(12, 11.7, Math.toRadians(-172));

        cornerSetPose = new Pose(19, 11, Math.toRadians(180));

        secondCornerPose = new Pose(12, 10, Math.toRadians(180));

        parkPose = new Pose(42, 14, Math.toRadians(180));

        intakeMark3StartPose = new Pose(52, 36, Math.toRadians(180));

        intakeMark3EndPose = new Pose(12, 36, Math.toRadians(180));

        if (team == Robot.Team.RED) {
            startPose = startPose.mirror();

            shootPose = shootPose.mirror();

            cornerPose = cornerPose.mirror();
            cornerSetPose = cornerSetPose.mirror();

            secondCornerPose = secondCornerPose.mirror();

            intakeMark3StartPose = intakeMark3StartPose.mirror();
            intakeMark3EndPose = intakeMark3EndPose.mirror();

            parkPose = parkPose.mirror();
        }

        // built for red mirrors
        // startPose.mirror();
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }
}
