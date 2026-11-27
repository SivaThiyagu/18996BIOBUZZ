package org.firstinspires.ftc.teamcode.automations.commands;

import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class ZeroTransferCommand extends CommandBase {
    private TransferSubsystem transferSubsystem;
    private boolean immediateOverride;
    private Timer timer;
    private int time;

    public ZeroTransferCommand(TransferSubsystem transferSubsystem, boolean immediateOverride, int time) {
        this.transferSubsystem = transferSubsystem;
        this.immediateOverride = immediateOverride;
        timer = new Timer();
        this.time = time;
    }

    @Override
    public void initialize() {
        if (immediateOverride) transferSubsystem.setTransferState(TransferSubsystem.TransferState.ZEROING);
        else transferSubsystem.setTransferState(TransferSubsystem.TransferState.INTAKE);
        timer.resetTimer();
    }

    @Override
    public void execute() {
        if (transferSubsystem.getTransferState() == TransferSubsystem.TransferState.INTAKE
                && Math.abs(transferSubsystem.getError()) < 30)
        {
            transferSubsystem.setTransferState(TransferSubsystem.TransferState.ZEROING);
            timer.resetTimer();
        }
    }

    @Override
    public boolean isFinished() {
        return transferSubsystem.getTransferState() == TransferSubsystem.TransferState.ZEROING
                && timer.getElapsedTime() > time;
    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.resetEncoder();
        transferSubsystem.setTransferState(TransferSubsystem.TransferState.INTAKE);
    }
}
