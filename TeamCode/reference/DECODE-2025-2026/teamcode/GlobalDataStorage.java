package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toRadians;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.automations.odo.STATICLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class GlobalDataStorage {
    // EX: public static Pose2D robotPose = new Pose2D();

    // general static class for passing data between OpModes, like the decoded value of the obelisk, or robot ending positions after auto, etc.

    public static Motif motif = null;
    public static Robot.Team team;
    public static Pose goalPose;
    public static STATICLocalizer staticLocalizer = null;

    public static Robot.Team autoTeam = Robot.Team.BLUE;
   //public static Pose robotPose = new Pose(24, 24,0);
    public static Pose robotPose = new Pose();

    public static TransferSubsystem.TransferState transferState = null;
}
