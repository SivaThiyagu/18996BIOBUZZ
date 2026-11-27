package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareMapNames;

@Configurable
public class IntakeSubsystem extends SubsystemBase {
    public final MotorEx intakeMotor;

    private IntakeState intakeState;

    public enum IntakeState {
        OUTTAKE,
        HOLD,
        INTAKE,
        REVERSE,
        CUSTOM,
        DISABLED;

        public double getValue() {
            switch (this) {
                case OUTTAKE:
                    return 1.0;
                case HOLD:
                    return 0.2;
                case INTAKE:
                    return 1.0;
                case REVERSE:
                    return -1.0;
                case CUSTOM:
                case DISABLED:
                    return 0.0;
                default:
                    throw new IllegalArgumentException();
            }
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
