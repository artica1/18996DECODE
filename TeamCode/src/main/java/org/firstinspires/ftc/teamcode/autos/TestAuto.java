package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.commands.AdjustShooterSpeedCommand;
import org.firstinspires.ftc.teamcode.automations.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.automations.commands.HoldPointCommand;
import org.firstinspires.ftc.teamcode.automations.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

@Autonomous
public class TestAuto extends OpMode {

    private Robot robot;

    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(119, 130, Math.toRadians(-45)).mirror();
    private final Pose shootPreloadPose = new Pose(48, 96, Math.toRadians(135));
    private final Pose shootStaged1Pose = new Pose(48, 96, Math.toRadians(135));
    private final Pose shootStaged2Pose = new Pose(48, 96, Math.toRadians(135));

    public PathChain shootPreload, intakeStaged1, shootStaged1, intakeStaged2, shootStaged2;

    private AutoShootCommand autoShootCommand;
    private AdjustShooterSpeedCommand adjustShooterCommand;
    private HoldPointCommand holdPointCommand;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap, Robot.Team.BLUE);
        buildPaths();
        robot.drive.setDriveMode(Drive.DriveMode.AUTO);
        robot.drive.follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.localizer.update();
        robot.drive.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", robot.localizer.getPose().getX());
        telemetry.addData("y", robot.localizer.getPose().getY());
        telemetry.addData("heading", robot.localizer.getPose().getHeading());
        telemetry.addData("busy", robot.drive.follower.isBusy());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.drive.follower.followPath(shootPreload);
                robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD);
                robot.transfer.setBeltState(TransferSubsystem.BeltState.HOLD);

                adjustShooterCommand = new AdjustShooterSpeedCommand(robot);
                CommandScheduler.getInstance().schedule(adjustShooterCommand.perpetually());

                setPathState(1);
                break;
            case 1:
                if(!robot.drive.follower.isBusy()) {
                    autoShootCommand = new AutoShootCommand(robot);
                    CommandScheduler.getInstance().schedule(autoShootCommand);
                    setPathState(2);
                }
                break;
            case 2:
                if(autoShootCommand.isFinished()) {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);
                    robot.transfer.setBeltState(TransferSubsystem.BeltState.INTAKE);
                    robot.transfer.setGatePosition(TransferSubsystem.GatePosition.CLOSED);

                    robot.drive.follower.followPath(intakeStaged1);
                    setPathState(3);
                }
                break;
            case 3:
                if (!robot.drive.follower.isBusy()) {
                    robot.transfer.setBeltState(TransferSubsystem.BeltState.HOLD);

                    robot.drive.follower.followPath(shootStaged1);
                    setPathState(4);
                }
                break;
            case 4:
                if(!robot.drive.follower.isBusy()) {
                    autoShootCommand = new AutoShootCommand(robot);
                    CommandScheduler.getInstance().schedule(autoShootCommand);
                    setPathState(5);
                }
                break;
            case 5:
                if(autoShootCommand.isFinished()) {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);
                    robot.transfer.setBeltState(TransferSubsystem.BeltState.INTAKE);
                    robot.transfer.setGatePosition(TransferSubsystem.GatePosition.CLOSED);

                    robot.drive.follower.followPath(intakeStaged2);
                    setPathState(6);
                }
                break;
            case 6:
                if (!robot.drive.follower.isBusy()) {
                    robot.transfer.setBeltState(TransferSubsystem.BeltState.HOLD);

                    robot.drive.follower.followPath(shootStaged2);
                    setPathState(7);
                }
                break;
            case 7:
                if(!robot.drive.follower.isBusy()) {
                    autoShootCommand = new AutoShootCommand(robot);
                    CommandScheduler.getInstance().schedule(autoShootCommand);
                    setPathState(-1);
                }
                break;
        }
    }

    public void buildPaths() {
        shootPreload = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPose,
                                //new Pose(45.640, 79.716),
                                //new Pose(48.583, 102.964),
                                shootPreloadPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        intakeStaged1 = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPreloadPose,
                                new Pose(55.962, 77.573),
                                new Pose(43.450, 82.123),
                                new Pose(18.882, 84.171)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        shootStaged1 = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18.882, 84.171),
                                //new Pose(76.311, 99.665),
                                //new Pose(40.995, 93.201),
                                shootStaged1Pose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        intakeStaged2 = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootStaged1Pose,
                                new Pose(59.374, 45.953),
                                new Pose(46.180, 60.967),
                                new Pose(18.882, 59.829)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        shootStaged2 = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(18.882, 59.829),
                                //new Pose(76.267, 101.715),
                                //new Pose(41.989, 88.821),
                                shootStaged2Pose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }
}
