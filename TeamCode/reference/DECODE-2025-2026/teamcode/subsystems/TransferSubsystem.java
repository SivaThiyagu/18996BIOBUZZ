package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.HardwareMapNames;

@Configurable
public class TransferSubsystem extends SubsystemBase {
    private final ServoEx gateServo;

    public final MotorEx transferMotor;

    private TransferState transferState;
    private GatePosition gatePosition;

    public static double kP = 0.0025;
    public static double kD = 0.00000;
    public static double kF = 0.10;

    public static int INTAKE_POS = 0;
    public static int ONE_POS = 92;
    public static int TWO_POS = 303;
    public static int THREE_POS = 780;

    public static double maxProgressSpeed = 1.0;
    public static double maxReturnSpeed = 1.0;

    public static double constantSpeed = 1.0;

    private double targetPosition;
    private double previousPosition;

    Timer timer = new Timer();

    public enum GatePosition {
        OPEN,
        CLOSED;
        public double getValue() {
            switch (this) {
                case OPEN:
                    return 0.35;
                case CLOSED:
                    return 0.08;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum TransferState {
        INTAKE,
        ONE,
        TWO,
        THREE,
        ZEROING,
        CONSTANT;
        public double getValue() {
            switch (this) {
                case INTAKE:
                    return INTAKE_POS;
                case ONE:
                    return ONE_POS;
                case TWO:
                    return TWO_POS;
                case THREE:
                    return THREE_POS;
                case ZEROING:
                    return 0;
                case CONSTANT:
                    return THREE_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public TransferSubsystem(HardwareMap hardwareMap) {
        transferMotor = new MotorEx(hardwareMap, HardwareMapNames.TRANSFER_MOTOR, Motor.GoBILDA.RPM_435);

        transferMotor.setRunMode(Motor.RunMode.RawPower);
        transferMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        transferMotor.setInverted(true);

        previousPosition = getCurrentPosition();
        timer.resetTimer();

        gateServo = new ServoEx(hardwareMap, HardwareMapNames.GATE_SERVO);

        setGatePosition(GatePosition.OPEN);
        setGatePosition(GatePosition.CLOSED);

        if (GlobalDataStorage.transferState == null) resetEncoder();
        else setTransferState(GlobalDataStorage.transferState);
    }

    @Override
    public void periodic() {
        if (transferState != TransferState.ZEROING && transferState != TransferState.CONSTANT) {
            double output = 0;

            output += kF * Math.signum(getError());

            output += kP * getError();

            output += kD * (getCurrentPosition() - previousPosition) / timer.getElapsedTimeSeconds();

            if (Math.abs(getError()) > 0 && getError() > 0) {
                output = maxProgressSpeed;
            }

            if (transferState == TransferState.INTAKE && Math.abs(getError()) < 10) {
                output = -0.1;
            }

            output = Range.clip(output, -maxReturnSpeed, 0.4);

            transferMotor.set(output);

            previousPosition = getCurrentPosition();
            timer.resetTimer();
        } else if (transferState == TransferState.CONSTANT) {
            if (getError() > 0 && Math.abs(getError()) > 200) transferMotor.set(constantSpeed);
            else if (getError() > 0) transferMotor.set(constantSpeed/2);
            else transferMotor.set(0);
        }
        else transferMotor.set(-0.2); // Zeroing
    }

    public void setTransferState(TransferState transferState) {
        this.transferState = transferState;

        GlobalDataStorage.transferState = transferState;

        targetPosition = transferState.getValue();
    }

    public void resetEncoder() {
        transferMotor.resetEncoder();
        setTransferState(TransferState.INTAKE);
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getCurrentPosition() {
        return transferMotor.getCurrentPosition();
    }

    public double getError() {
        return targetPosition - getCurrentPosition();
    }

    public void setMaxProgressSpeed(double maxProgressSpeed) {
        TransferSubsystem.maxProgressSpeed = maxProgressSpeed;
    }

    public void setGatePosition(GatePosition gatePosition) {
        this.gatePosition = gatePosition;

        gateServo.set(gatePosition.getValue());
    }

    public TransferState getTransferState() {
        return transferState;
    }

    public GatePosition getGatePosition() {
        return gatePosition;
    }

    public boolean isNearlyFinished() {
        return getTransferState() == TransferSubsystem.TransferState.THREE && getError() < 50;
    }
}
