package org.firstinspires.ftc.teamcode.automations.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.ShooterCalculations;

public class AutoShootCommand extends CommandBase {
    private final Robot robot;

    private int shootingState;

    CommandBase progressTransferCommand = new InstantCommand();

    public AutoShootCommand(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        setShootingState(0);
    }

    @Override
    public void execute() {
        if (shootingState == 0) {
            if (Math.abs(robot.shooter.getError()) < 30 && Math.abs(robot.drive.follower.getHeadingError()) < 0.03) {
                //progressTransferCommand = new ProgressTransferCommand(robot, true, ShooterCalculations.getTransferSpeed(robot.localizer.getDistanceToGoal()));
                progressTransferCommand = new ProgressTransferCommand(robot, true, 0.4);
                CommandScheduler.getInstance().schedule(progressTransferCommand);

                setShootingState(-1);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return (shootingState == -1 && progressTransferCommand.isFinished());
    }

    public void setShootingState(int shootingState) {
        this.shootingState = shootingState;
    }
}
