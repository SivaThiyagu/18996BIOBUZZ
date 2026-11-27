package org.firstinspires.ftc.teamcode.automations.drive;

import static java.lang.Math.atan2;

import androidx.annotation.NonNull;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.automations.pedroPathing.Constants;

public class Drive {
    public final Follower follower;
    private DriveMode driveMode;
    private Pose holdPoint;

    private boolean headingLock = false;
    private boolean translationLock = false;
    private Pose translationLockPose;

    PIDFController headingController;
    PIDFController secondaryHeadingController;
    private double headingSwitch;

    PIDFController translationalController;
    PIDFController secondaryTranslationalController;
    private double translationalSwitch;

    public void setHoldPoint(Pose holdPoint) {
        this.holdPoint = holdPoint;

        follower.holdPoint(holdPoint, false);
    }

    public enum DriveMode {
        MANUAL,
        FINE,
        AUTO,
        HOLD_POINT;
    }

    public Drive(HardwareMap hardwareMap, Localizer localizer) {
        this(hardwareMap, localizer, DriveMode.MANUAL);
    }

    public Drive(HardwareMap hardwareMap, Localizer localizer, DriveMode driveMode) {
        follower = Constants.createFollower(hardwareMap, localizer);

        headingController = new PIDFController(follower.constants.coefficientsHeadingPIDF);
        secondaryHeadingController = new PIDFController(follower.constants.coefficientsSecondaryHeadingPIDF);
        headingSwitch = follower.constants.headingPIDFSwitch;

        translationalController = new PIDFController(follower.constants.coefficientsTranslationalPIDF);
        secondaryTranslationalController = new PIDFController(follower.constants.coefficientsSecondaryTranslationalPIDF);
        //translationalSwitch = follower.constants.translationalPIDFSwitch;
        translationalSwitch = 1; // override to 1 inch

        translationLockPose = new Pose(0, 0);

        setDriveMode(driveMode);

        holdPoint = follower.getPose();
    }

    public void update() {
        follower.update();

        translationLockPose = follower.getPose();
    }

    public void setTeleOpVectors(double gamepadX, double gamepadY, double gamepadH) {
        if (driveMode == DriveMode.MANUAL) {
            double headingError = getGoalHeadingError() + -gamepadH * 0.187; // 5 degree adjustment each way

            headingController.updateError(headingError);
            secondaryHeadingController.updateError(headingError);

            double x;
            double y;
            double h;

            if (headingLock) {

                if (Math.abs(getGoalHeadingError()) <= headingSwitch) {
                    h = secondaryHeadingController.run();
                } else {
                    h = headingController.run();
                }

                // pedro teleop follower adds vectors incorrectly,
                // limit x and y motor powers so the heading doesn't spin out
                gamepadY *= 0.3;
                gamepadX *= 0.3;

            } else h = -gamepadH;

            // todo make translational work
            x = -gamepadY;
            y = -gamepadX;

            follower.setTeleOpDrive(x, y, h);
        }

        else if (driveMode == DriveMode.FINE) {
            follower.setTeleOpDrive(-gamepadY * 0.3, -gamepadX * 0.3, -gamepadH * 0.3);
        }
    }

    public void setDriveMode(@NonNull DriveMode driveMode) {
        this.driveMode = driveMode;

        if (driveMode == DriveMode.MANUAL || driveMode == DriveMode.FINE) {
            follower.startTeleopDrive(true);
        } else {
            translationLock = false;
            headingLock = false;
        }
    }

    public double getGoalHeadingError() {
        double headingGoal = atan2(
                GlobalDataStorage.goalPose.minus(follower.getPose()).getY(),
                GlobalDataStorage.goalPose.minus(follower.getPose()).getX()
        );

        return MathFunctions.getTurnDirection(follower.getPose().getHeading(), headingGoal)
                * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), headingGoal);
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public boolean isHeadingLock() {
        return headingLock;
    }

    public void setHeadingLock(boolean headingLock) {
        this.headingLock = headingLock;
    }

    public boolean isTranslationLock() {
        return translationLock;
    }

    public void setTranslationLock(boolean translationLock) {
        this.translationLock = translationLock;
    }
}
