package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareMapNames;
import org.firstinspires.ftc.teamcode.automations.ShooterCalculations;

// TODO
// SHOOTER REDESIGN
// Active always tracks position and updates speed
// Bang only activated when firing
// Teleop toggles idle and such

@Configurable
public class ShooterSubsystem extends SubsystemBase {
    public MotorEx shooter1;
    public MotorEx shooter2;
    private ShooterMotorState shooterMotorState;
    private ShooterAngleState shooterAngleState;

    private double distanceToGoal = 0; // cant be zero idfk why

    public static double kS = 0.06;
    public static double kV = 0.00035;
    public static double kP = 0.001;
    public double idlePower = 0.4;

    private final ServoEx angleServo;

    private double localAngle = 35;
    private double localTargetTps = 1150;

    private double targetTps;

    public enum ShooterMotorState {
        ACTIVE,
        IDLE,
        MAX,
        UNPOWERED,
        BANG
    }

    public enum ShooterAngleState {
        AUTO,
        MANUAL
    }

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooter1 = new MotorEx(hardwareMap, HardwareMapNames.SHOOTER_MOTOR_1, Motor.GoBILDA.BARE);
        shooter2 = new MotorEx(hardwareMap, HardwareMapNames.SHOOTER_MOTOR_2, Motor.GoBILDA.BARE);

        shooter1.setRunMode(Motor.RunMode.RawPower);
        shooter1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter1.setInverted(true);

        shooter2.setRunMode(Motor.RunMode.RawPower);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        setShooterMotorState(ShooterMotorState.IDLE);
        setTargetTps(0);

        angleServo = new ServoEx(hardwareMap, HardwareMapNames.SHOOTER_SERVO, 365, AngleUnit.DEGREES); // THE SERVO IS NOW IN DEGREES 100%, set() TAKES DEGREES
        setAngle(5);
        setShooterAngleState(ShooterAngleState.AUTO);
    }

    @Override
    public void periodic() {
        if (shooterMotorState == ShooterMotorState.ACTIVE) {
            double output = 0;

            output += kS * Math.signum(targetTps);

            output += kV * targetTps;

            output += kP * getError();

            output = Range.clip(output, -1.0, 1.0);

            shooter1.set(output);
            shooter2.set(output);
        }
        else if (shooterMotorState == ShooterMotorState.IDLE) {
            shooter1.set(idlePower);
            shooter2.set(idlePower);
        }
        else if (shooterMotorState == ShooterMotorState.UNPOWERED) {
            shooter1.set(0);
            shooter2.set(0);
        }
        else if (shooterMotorState == ShooterMotorState.MAX) {
            shooter1.set(1.0);
            shooter2.set(1.0);
        }
        else if (shooterMotorState == ShooterMotorState.BANG) {
            if (getError() > 0) {
                shooter1.set(1.0);
                shooter2.set(1.0);
            } else if (getError() < 0) {
                shooter1.set(0);
                shooter2.set(0);
            }
        }

        if (shooterAngleState == ShooterAngleState.AUTO) {
            setAngle(ShooterCalculations.getAngle(distanceToGoal));
        }
    }

    public void updateDistanceToGoal(double distanceToGoal) {
        this.distanceToGoal = distanceToGoal;
    }

    public void setTargetTps(double targetTps) {
        this.targetTps = targetTps;
    }

    public void setAngle(double angle) {
        if (angle < 0) {
            angleServo.set(0);
        } else if (angle > 365) {
            angleServo.set(365);
        } else {
            angleServo.set(angle);
        }
    }

    public double getAngle() {
        return angleServo.get();
    }

    public double getTargetTps() {
        return targetTps;
    }

    public double getCurrentTps() {
        return shooter1.getCorrectedVelocity();
    }

    public ShooterMotorState getShooterMotorState() {
        return shooterMotorState;
    }

    public void setShooterMotorState(ShooterMotorState shooterMotorState) {
        this.shooterMotorState = shooterMotorState;
    }

    public ShooterAngleState getShooterAngleState() {
        return shooterAngleState;
    }

    public void setShooterAngleState(ShooterAngleState shooterAngleState) {
        this.shooterAngleState = shooterAngleState;
    }

    public double getError() {
        // uhhhhhhhh hmmmm
        if (shooterMotorState == ShooterMotorState.IDLE || shooterMotorState == ShooterMotorState.MAX || shooterMotorState == ShooterMotorState.UNPOWERED) return 0;
        else return targetTps - getCurrentTps();
    }

    @Deprecated
    public static void setkP(double kP) {
        ShooterSubsystem.kP = kP;
    }

    public void setLocal() {
        setShooterMotorState(ShooterMotorState.ACTIVE);
        setShooterAngleState(ShooterAngleState.MANUAL);
        targetTps = localTargetTps;
        angleServo.set(localAngle);
    }

    public void setLocalTargetTps(double localTargetTps) {
        this.localTargetTps = localTargetTps;
        setLocal();
    }

    public void setLocalAngle(double localAngle) {
        if (localAngle < 0) {
            this.localAngle = 0;
        } else if (localAngle > 365) {
            this.localAngle = 365;
        } else {
            this.localAngle = localAngle;
        }
        setLocal();
    }

    public double getLocalTargetTps() {
        return localTargetTps;
    }

    public double getLocalAngle() {
        return localAngle;
    }
}
