package org.firstinspires.ftc.teamcode.automations.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.ShooterCalculations;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class AdjustShooterCommand extends CommandBase {
    private Robot robot;

    public AdjustShooterCommand(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.ACTIVE);
    }

    @Override
    public void execute() {
        robot.shooter.setTargetTps(ShooterCalculations.getSpeed(robot.localizer.getDistanceToGoal()));
        robot.shooter.setAngle(ShooterCalculations.getAngle(robot.localizer.getDistanceToGoal()));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
