package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class ShootCommand extends SequentialCommandGroup {

    public ShootCommand(Robot robot) {
        addCommands(
                new InstantCommand(() -> robot.intake.setIntakeState(IntakeSubsystem.IntakeState.OUTTAKE)),
                new InstantCommand(() -> robot.transfer.setBeltState(TransferSubsystem.BeltState.OUTTAKE)),

                new WaitCommand(100),

                new InstantCommand(() -> robot.transfer.setGatePosition(TransferSubsystem.GatePosition.OPEN)),

                new WaitCommand(1000),

                new InstantCommand(() -> robot.transfer.setGatePosition(TransferSubsystem.GatePosition.CLOSED)),
                new InstantCommand(() -> robot.intake.setIntakeState(IntakeSubsystem.IntakeState.DISABLED)),
                new InstantCommand(() -> robot.transfer.setBeltState(TransferSubsystem.BeltState.DISABLED))
        );
    }
}
