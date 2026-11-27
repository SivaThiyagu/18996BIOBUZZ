package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;

/**
 * Adapted from our DECODE reference IntakeSubsystem.
 * Motor type, direction, and power values below are from last year; verify on BIOBUZZ.
 * Creating this subsystem starts it stopped. An OpMode must call its methods to use it.
 */
public class IntakeSubsystem extends SubsystemBase {
    public final MotorEx intakeMotor;

    private IntakeState intakeState;

    public enum IntakeState {
        OUTTAKE(1.0),
        HOLD(0.2),
        INTAKE(1.0),
        REVERSE(-1.0),
        CUSTOM(0.0),
        DISABLED(0.0);

        private final double power;

        IntakeState(double power) {
            this.power = power;
        }

        public double getValue() {
            return power;
        }
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = new MotorEx(hardwareMap, HardwareMapNames.INTAKE_MOTOR, Motor.GoBILDA.RPM_1150);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setInverted(true);
        intakeMotor.set(0);

        setIntakeState(IntakeState.DISABLED);
    }

    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;

        intakeMotor.set(intakeState.getValue());
    }

    public void setCustomIntakeSpeed(double speed) {
        intakeState = IntakeState.CUSTOM;

        intakeMotor.set(speed);
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }
}
