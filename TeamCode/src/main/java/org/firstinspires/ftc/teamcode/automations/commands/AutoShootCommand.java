package org.firstinspires.ftc.teamcode.automations.commands;

import static java.lang.Math.atan2;

import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.ShooterCalculations;
import org.firstinspires.ftc.teamcode.automations.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class AutoShootCommand extends CommandBase {
    private final Robot robot;
    private Drive.DriveMode previousDriveMode;

    private Timer shotTimer;
    private int shootingState;
    private final int numberOfArtifacts;
    private int totalArtifactsShot = 0;
    private int previousError = 0;

    public AutoShootCommand(Robot robot, int artifacts) {
        this.robot = robot;
        this.numberOfArtifacts = artifacts;
    }

    @Override
    public void initialize() {
        shotTimer = new Timer();

        previousDriveMode = robot.drive.getDriveMode();

        robot.shooter.setTargetTps(ShooterCalculations.getSpeed(robot.localizer.getDistanceToGoal()));
        robot.shooter.setAngle(ShooterCalculations.getAngle(robot.localizer.getDistanceToGoal()));
        robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.ACTIVE);

        robot.drive.setHoldPoint(
                robot.localizer.getPose().setHeading(
                        atan2(
                                GlobalDataStorage.goalPose.minus(robot.localizer.getPose()).getY(),
                                GlobalDataStorage.goalPose.minus(robot.localizer.getPose()).getX()
                        )
                )
        );
        robot.drive.setDriveMode(Drive.DriveMode.HOLD_POINT);

        robot.transfer.setBeltState(TransferSubsystem.BeltState.INTAKE);

        setShootingState(0);
    }

    @Override
    public void execute() {
        switch(shootingState) {
            case 0:
                if(Math.abs(robot.shooter.getError()) < 30) {
                    robot.transfer.setGatePosition(TransferSubsystem.GatePosition.OPEN);
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.OUTTAKE);
                    ShooterSubsystem.setkP(0.005);

                    setShootingState(1);
                }
                break;
            case 1:
                if(Math.abs(robot.shooter.getError()) > 100 && previousError < 100) {
                    shotTimer.resetTimer();
                    totalArtifactsShot++;
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

        ShooterSubsystem.setkP(0.001);

        robot.drive.setDriveMode(previousDriveMode);

        if (!interrupted && totalArtifactsShot == numberOfArtifacts) {
            robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.IDLE);
        }
    }

    @Override
    public boolean isFinished() {
        return (totalArtifactsShot == numberOfArtifacts || shotTimer.getElapsedTime() > 2000);
    }

    public void setShootingState(int shootingState) {
        this.shootingState = shootingState;
        shotTimer.resetTimer();
    }
}
