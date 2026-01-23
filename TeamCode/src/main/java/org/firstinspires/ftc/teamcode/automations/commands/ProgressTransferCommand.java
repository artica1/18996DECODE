package org.firstinspires.ftc.teamcode.automations.commands;

import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class ProgressTransferCommand extends CommandBase {
    private Robot robot;
    private boolean continuous;
    private boolean changedState;

    Timer openTimer = new Timer();
    Timer closeTimer = new Timer();
    Timer firstTimer = new Timer();

    public ProgressTransferCommand(Robot robot, boolean continuous, double maxProgressSpeed) {
        this.robot = robot;
        this.continuous = continuous;
        robot.transfer.setMaxProgressSpeed(0.5);
    }

    public ProgressTransferCommand(Robot robot, boolean continuous) {
        this(robot, continuous, 1.0);
    }

    public ProgressTransferCommand(Robot robot) {
        this(robot, false);
    }

    @Override
    public void initialize() {
        robot.transfer.setGatePosition(TransferSubsystem.GatePosition.OPEN);

        openTimer.resetTimer();
        changedState = false;
    }

    @Override
    public void execute() {
        if (openTimer.getElapsedTime() > 80) { // let gate servo open
            robot.intake.setIntakeState(IntakeSubsystem.IntakeState.OUTTAKE);
            // kick the first ball
            if (firstTimer.getElapsedTime() > 130 || robot.transfer.getTransferState() != TransferSubsystem.TransferState.INTAKE) {

                if (!changedState) {
                    if (continuous)
                        robot.transfer.setTransferState(TransferSubsystem.TransferState.THREE);
                    else {
                        switch (robot.transfer.getTransferState()) {
                            case INTAKE:
                                robot.transfer.setTransferState(TransferSubsystem.TransferState.ONE);
                                break;
                            case ONE:
                                robot.transfer.setTransferState(TransferSubsystem.TransferState.TWO);
                                break;
                            case TWO:
                                robot.transfer.setTransferState(TransferSubsystem.TransferState.THREE);
                                break;
                        }
                    }
                    changedState = true;
                    closeTimer.resetTimer();
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (robot.transfer.getTransferState() == TransferSubsystem.TransferState.THREE && robot.transfer.getError() < 0) return true;
        else return changedState && Math.abs(robot.transfer.getError()) < 5 && closeTimer.getElapsedTime() > 100;
    }

    @Override
    public void end(boolean interrupted) {
        robot.transfer.setGatePosition(TransferSubsystem.GatePosition.CLOSED);
        robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD);

        robot.transfer.setMaxProgressSpeed(1.0);

        if (robot.transfer.getTransferState() == TransferSubsystem.TransferState.THREE) {
            CommandScheduler.getInstance().schedule(new ZeroTransferCommand(robot.transfer));
        }
    }
}
