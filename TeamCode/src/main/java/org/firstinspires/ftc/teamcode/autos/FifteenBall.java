package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.commands.AdjustShooterSpeedCommand;
import org.firstinspires.ftc.teamcode.automations.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.automations.commands.HoldPointCommand;
import org.firstinspires.ftc.teamcode.automations.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

@Autonomous
public class FifteenBall extends OpMode {
    private Robot robot;

    private Timer pathTimer, autoTimer;
    private int pathState;

    private Pose startPose, farShootPose, intakeMark3StartPose, intakeMark3EndPose;

    public PathChain shootPreload, prepareMark3, intakeMark3, shootMark3;

    private Command autoShootCommand, adjustShooterSpeedCommand, holdPointCommand = new InstantCommand();

    @Override
    public void init() {
        pathTimer = new Timer();
        autoTimer = new Timer();

        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap, GlobalDataStorage.autoTeam);

        buildPoses(GlobalDataStorage.autoTeam);
        buildPaths();

        robot.drive.setDriveMode(Drive.DriveMode.AUTO);
        robot.drive.follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        robot.update();
        CommandScheduler.getInstance().run();

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", robot.localizer.getPose().getX());
        telemetry.addData("y", robot.localizer.getPose().getY());
        telemetry.addData("heading", robot.localizer.getPose().getHeading());
        telemetry.addData("busy", robot.drive.follower.isBusy());
        telemetry.addData("error", robot.shooter.getError());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.drive.follower.followPath(shootPreload);
                robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD);
                robot.transfer.setBeltState(TransferSubsystem.BeltState.HOLD);

                adjustShooterSpeedCommand = new AdjustShooterSpeedCommand(robot, robot.localizer.getPose());
                adjustShooterSpeedCommand.schedule();

                setPathState(1);
                break;
            case 1:
                if(!robot.drive.follower.isBusy()) {

                    holdPointCommand = new HoldPointCommand(robot);
                    autoShootCommand = new AutoShootCommand(robot);

                    holdPointCommand.schedule();
                    autoShootCommand.schedule();
                    setPathState(2);
                }
                break;
            case 2:
                if(autoShootCommand.isFinished()) {
                    robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.IDLE);

                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);
                    robot.transfer.setBeltState(TransferSubsystem.BeltState.INTAKE);

                    robot.drive.follower.followPath(prepareMark3);
                    setPathState(3);
                }
                break;
            case 3:
                if (!robot.drive.follower.isBusy()) {

                    robot.drive.follower.setMaxPower(0.3);
                    robot.drive.follower.followPath(intakeMark3);
                    setPathState(4);
                }
                break;
            case 4:
                if(!robot.drive.follower.isBusy()) {
                    robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.ACTIVE);

                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD);
                    robot.transfer.setBeltState(TransferSubsystem.BeltState.HOLD);

                    robot.drive.follower.setMaxPower(1.0);
                    robot.drive.follower.followPath(shootMark3);
                    setPathState(5);
                }
                break;
            case 5:
                if(!robot.drive.follower.isBusy() && autoTimer.getElapsedTimeSeconds() > 20) {
                    holdPointCommand = new HoldPointCommand(robot);
                    adjustShooterSpeedCommand = new AdjustShooterSpeedCommand(robot);
                    autoShootCommand = new AutoShootCommand(robot);

                    holdPointCommand.schedule();
                    adjustShooterSpeedCommand.schedule();
                    autoShootCommand.schedule();
                    setPathState(-1);
                }
                break;
        }
    }

    public void buildPaths() {
        shootPreload = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPose,
                                farShootPose
                        )
                ).setLinearHeadingInterpolation(startPose.getHeading(), farShootPose.getHeading())
                .build();

        prepareMark3 = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                farShootPose,
                                intakeMark3StartPose
                        )
                ).setLinearHeadingInterpolation(farShootPose.getHeading(), intakeMark3StartPose.getHeading())
                .build();

        intakeMark3 = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                intakeMark3StartPose,
                                intakeMark3EndPose
                        )
                ).setLinearHeadingInterpolation(intakeMark3StartPose.getHeading(), intakeMark3EndPose.getHeading())
                .build();

        shootMark3 = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                intakeMark3EndPose,
                                farShootPose
                        )
                ).setLinearHeadingInterpolation(intakeMark3EndPose.getHeading(), farShootPose.getHeading())
                .build();
    }

    public void buildPoses(Robot.Team team) {
        // BUILT FOR BLUE
        startPose = new Pose(48, 24, Math.toRadians(90));
        // startPose = new Pose(56, 10, Math.toRadians(90));

        farShootPose = new Pose(56, 16, Math.toRadians(112));

        intakeMark3StartPose = new Pose(42, 36, Math.toRadians(180));

        intakeMark3EndPose = new Pose(16, 36, Math.toRadians(180));

        if (team == Robot.Team.RED) {
            startPose.mirror();
            farShootPose.mirror();
            intakeMark3StartPose.mirror();
            intakeMark3EndPose.mirror();
        }

        // built for red mirrors
        // startPose.mirror();
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }
}
