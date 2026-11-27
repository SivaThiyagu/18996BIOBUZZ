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

@Autonomous
public class TwelveBallNoGate extends OpMode {
    private Robot robot;

    private Timer pathTimer, autoTimer;
    private int pathState;
    private int gateCycles = 0;

    private Pose startPose, shootPose, intakeMark1StartPose, intakeMark1EndPose, intakeMark2StartPose, intakeMark2EndPose, intakeMark3StartPose, intakeMark3EndPose, parkPose, gateContactPose;

    public PathChain shootPreload, shootMark1, shootMark2, shootMark3, park, gateContact, gateShoot;

    private Command autoShootCommand = new InstantCommand();
    private Command adjustShooterSpeedCommand = new InstantCommand();
    private Command holdPointCommand = new InstantCommand();

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
        telemetry.addData("Team", GlobalDataStorage.autoTeam);
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

        GlobalDataStorage.robotPose = robot.localizer.getPose();

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

                setPathState(1);
                break;
            case 1:
                if(autoShootCommand.isFinished()) {

                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                    robot.drive.follower.followPath(shootMark2, true);

                    autoShootCommand = new AutoShootCommand(robot);
                    autoShootCommand.schedule();

                    setPathState(2);
                }
                break;
            case 2:
                if(autoShootCommand.isFinished()) {

                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                    robot.drive.follower.followPath(gateContact, true);

                    setPathState(3);
                }
                break;
            case 3:
                if(!robot.drive.follower.isBusy() && pathTimer.getElapsedTime() > 2600) {

                    robot.drive.follower.followPath(gateShoot, true);
                    gateCycles++;

                    autoShootCommand = new AutoShootCommand(robot);
                    autoShootCommand.schedule();

                    if (gateCycles < 3) setPathState(2);
                    else setPathState(5);
                }
                break;
            case 4:
                if(autoShootCommand.isFinished()) {

                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                    robot.drive.follower.followPath(shootMark3, true);

                    autoShootCommand = new AutoShootCommand(robot);
                    autoShootCommand.schedule();

                    setPathState(5);
                }
                break;
            case 5:
                if(autoShootCommand.isFinished()) {

                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                    robot.drive.follower.followPath(shootMark1, true);

                    autoShootCommand = new AutoShootCommand(robot);
                    autoShootCommand.schedule();

                    setPathState(6);
                }
                break;
            case 6:
                if(autoShootCommand.isFinished()) {

                    robot.drive.follower.followPath(park, false);

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
                //.addParametricCallback(0.3, ()-> robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD))
                .build();

        shootMark1 = robot.drive.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                intakeMark1StartPose
                        )
                ).setTangentHeadingInterpolation()

                .addPath(
                        new BezierLine(
                                intakeMark1StartPose,
                                intakeMark1EndPose
                        )
                ).setTangentHeadingInterpolation()

                .addPath(
                        new BezierLine(
                                intakeMark1EndPose,
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(intakeMark1EndPose.getHeading(), shootPose.getHeading())
               // .addParametricCallback(0.8, ()-> robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD))

                .setBrakingStart(0.5)
                .build();

        shootMark2 = robot.drive.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                intakeMark2StartPose
                        )
                ).setLinearHeadingInterpolation(shootPose.getHeading(), intakeMark2StartPose.getHeading())

                .addPath(
                        new BezierLine(
                                intakeMark2StartPose,
                                intakeMark2EndPose
                        )
                ).setLinearHeadingInterpolation(intakeMark2StartPose.getHeading(), intakeMark2EndPose.getHeading())

                .addPath(
                        new BezierLine(
                                intakeMark2EndPose,
                                shootPose
                        )
                ).setLinearHeadingInterpolation(intakeMark2EndPose.getHeading(), shootPose.getHeading())
               // .addParametricCallback(0.8, ()-> robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD))

                .build();

        shootMark3 = robot.drive.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                intakeMark3StartPose
                        )
                ).setLinearHeadingInterpolation(shootPose.getHeading(), intakeMark3StartPose.getHeading())

                .addPath(
                        new BezierLine(
                                intakeMark3StartPose,
                                intakeMark3EndPose
                        )
                ).setLinearHeadingInterpolation(intakeMark3StartPose.getHeading(), intakeMark3EndPose.getHeading())

                .addPath(
                        new BezierLine(
                                intakeMark3EndPose,
                                shootPose
                        )
                ).setLinearHeadingInterpolation(intakeMark3EndPose.getHeading(), shootPose.getHeading())
               // .addParametricCallback(0.8, ()-> robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD))

                .build();

        gateContact = robot.drive.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                new Pose(38, 73)
                        )
                ).setTangentHeadingInterpolation()

                .addPath(
                        new BezierLine(
                                new Pose(38, 73),
                                gateContactPose
                        )
                )//.setLinearHeadingInterpolation(Math.toRadians(-149), gateContactPose.getHeading())
                .setConstantHeadingInterpolation(gateContactPose.getHeading())

                .build();

        gateShoot = robot.drive.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                gateContactPose,
                                new Pose(38, 73)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()

                .addPath(
                        new BezierLine(
                                new Pose(38, 73),
                                shootPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-149), shootPose.getHeading())

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
        startPose = new Pose(19.9, 120.6, Math.toRadians(141.7));
        // startPose = new Pose(56, 10, Math.toRadians(90));

        shootPose = new Pose(55.8, 83.5, Math.toRadians(134));
        // 20.3 75.1 90

        intakeMark1StartPose = new Pose(42, 84, Math.toRadians(180));

        intakeMark1EndPose = new Pose(28, 84, Math.toRadians(180));

        intakeMark2StartPose = new Pose(54, 60, Math.toRadians(180));

        intakeMark2EndPose = new Pose(20, 60, Math.toRadians(180));

        intakeMark3StartPose = new Pose(52, 36, Math.toRadians(180));

        intakeMark3EndPose = new Pose(20, 36, Math.toRadians(180));

        gateContactPose = new Pose(12, 59, Math.toRadians(151.26));

        parkPose = new Pose(42, 84, Math.toRadians(135));

        if (team == Robot.Team.RED) {
            startPose = startPose.mirror();

            shootPose = shootPose.mirror();

            intakeMark1StartPose = intakeMark1StartPose.mirror();
            intakeMark1EndPose = intakeMark1EndPose.mirror();

            intakeMark2StartPose = intakeMark2StartPose.mirror();
            intakeMark2EndPose = intakeMark2EndPose.mirror();

            intakeMark3StartPose = intakeMark3StartPose.mirror();
            intakeMark3EndPose = intakeMark3EndPose.mirror();

            gateContactPose = gateContactPose.mirror();

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
