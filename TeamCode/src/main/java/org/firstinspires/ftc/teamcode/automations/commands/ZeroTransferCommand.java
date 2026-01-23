package org.firstinspires.ftc.teamcode.automations.commands;

import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class ZeroTransferCommand extends CommandBase {
    private TransferSubsystem transferSubsystem;
    private Timer timer;
    private int time;

    public ZeroTransferCommand(TransferSubsystem transferSubsystem) {
        this(transferSubsystem, 250);
    }

    public ZeroTransferCommand(TransferSubsystem transferSubsystem, int time) {
        this.transferSubsystem = transferSubsystem;
        timer = new Timer();
        this.time = time;
    }

    @Override
    public void initialize() {
        transferSubsystem.setTransferState(TransferSubsystem.TransferState.ZEROING);
        timer.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return timer.getElapsedTime() > 250;
    }

    @Override
    public void end(boolean interrupted) {
        transferSubsystem.resetEncoder();
        transferSubsystem.setTransferState(TransferSubsystem.TransferState.INTAKE);
    }
}
