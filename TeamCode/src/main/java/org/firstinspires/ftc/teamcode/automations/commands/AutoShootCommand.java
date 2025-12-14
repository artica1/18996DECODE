package org.firstinspires.ftc.teamcode.automations.commands;

import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class AutoShootCommand extends CommandBase {
    private final Robot robot;

    private TransferSubsystem.BeltState previousBeltState;
    private IntakeSubsystem.IntakeState previousIntakeState;

    private Timer totalTime;
    private Timer noBallDetectedTime;
    private Timer shotTimer;
    private Timer chargeTimer;
    private int shootingState;
    private double previousError = 0;

    private final boolean continuous;

    public AutoShootCommand(Robot robot) {
        this(robot, false);
    }

    public AutoShootCommand(Robot robot, boolean continuous) {
        this.robot = robot;
        this.continuous = continuous;
    }

    @Override
    public void initialize() {
        totalTime = new Timer();
        noBallDetectedTime = new Timer();
        shotTimer = new Timer();
        chargeTimer = new Timer();

        previousBeltState = robot.transfer.getBeltState();

        previousIntakeState = robot.intake.getIntakeState();

        robot.transfer.setBeltState(TransferSubsystem.BeltState.INTAKE);

        setShootingState(0);
    }

    @Override
    public void execute() {
        if (robot.colorSensorManager.ballDetected()) noBallDetectedTime.resetTimer();

        switch(shootingState) {
            case 0:
                if (Math.abs(robot.shooter.getError()) < 30 && robot.drive.follower.getHeadingError() < 0.03) {
                    robot.transfer.setGatePosition(TransferSubsystem.GatePosition.OPEN);
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.OUTTAKE);

                    if (!continuous) {
                        setShootingState(1);
                    } else {
                        setShootingState(-1);
                    }
                }
                break;
            case 1:
                if (Math.abs(robot.shooter.getError()) > 100 && previousError < 100) {
                    shotTimer.resetTimer();
                    chargeTimer.resetTimer();

                    robot.transfer.setGatePosition(TransferSubsystem.GatePosition.CLOSED);

                    setShootingState(0);
                }
                break;
        }
        previousError = robot.shooter.getError();
    }

    @Override
    public void end(boolean interrupted) {
        robot.transfer.setGatePosition(TransferSubsystem.GatePosition.CLOSED);
        robot.intake.setIntakeState(IntakeSubsystem.IntakeState.DISABLED);
        robot.transfer.setBeltState(TransferSubsystem.BeltState.DISABLED);

        robot.intake.setIntakeState(previousIntakeState);
        robot.transfer.setBeltState(previousBeltState);
    }

    @Override
    public boolean isFinished() {
        return noBallDetectedTime.getElapsedTime() > 500 || shotTimer.getElapsedTime() > 5000 || totalTime.getElapsedTime() > 8000;
    }

    public void setShootingState(int shootingState) {
        this.shootingState = shootingState;
    }
}
