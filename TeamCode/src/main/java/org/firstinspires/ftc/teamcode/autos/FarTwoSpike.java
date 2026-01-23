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

@Autonomous
public class FarTwoSpike extends OpMode {
    private Robot robot;

    private Timer pathTimer, autoTimer;
    private int pathState;

    private Pose startPose, shootPose, intakeMark3StartPose, intakeMark3EndPose, intakeMark2StartPose, intakeMark2EndPose, hpIntakeStartPose, hpIntakeEndPose, parkPose;

    public PathChain shootPreload, prepareMark3, intakeMark3, shootMark3, prepareMark2, intakeMark2, shootMark2, prepareHpIntake, hpIntake, shootHp, park;

    private Command autoShootCommand = new InstantCommand();
    private Command adjustShooterSpeedCommand = new InstantCommand();
    private Command holdPointCommand = new InstantCommand();

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

        telemetry.addData("heading", Math.toRadians(robot.localizer.getPose().getHeading()));
        telemetry.addData("busy", robot.drive.follower.isBusy());
        telemetry.addData("error", robot.shooter.getError());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.drive.follower.followPath(shootPreload);
                robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD);

                adjustShooterSpeedCommand = new AdjustShooterSpeedCommand(robot, shootPose);
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
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                    robot.drive.follower.followPath(prepareMark3);
                    setPathState(3);
                }
                break;
            case 3:
                if (!robot.drive.follower.isBusy()) {

                    robot.drive.follower.setMaxPower(0.5);
                    robot.drive.follower.followPath(intakeMark3);
                    setPathState(4);
                }
                break;
            case 4:
                if(!robot.drive.follower.isBusy()) {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD);

                    robot.drive.follower.setMaxPower(1.0);
                    robot.drive.follower.followPath(shootMark3);
                    setPathState(5);
                }
                break;
            case 5:
                if(!robot.drive.follower.isBusy()) {
                    holdPointCommand = new HoldPointCommand(robot);
                    autoShootCommand = new AutoShootCommand(robot);

                    holdPointCommand.schedule();
                    autoShootCommand.schedule();
                    setPathState(6);
                }
                break;
            case 6:
                if(autoShootCommand.isFinished()) {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                    robot.drive.follower.followPath(prepareMark2);
                    setPathState(7);
                }
                break;
            case 7:
                if (!robot.drive.follower.isBusy()) {

                    robot.drive.follower.setMaxPower(0.5);
                    robot.drive.follower.followPath(intakeMark2);
                    setPathState(8);
                }
                break;
            case 8:
                if(!robot.drive.follower.isBusy()) {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD);

                    robot.drive.follower.setMaxPower(1.0);
                    robot.drive.follower.followPath(shootMark2);
                    setPathState(9);
                }
                break;
            case 9:
                if(!robot.drive.follower.isBusy()) {
                    holdPointCommand = new HoldPointCommand(robot);
                    autoShootCommand = new AutoShootCommand(robot);

                    holdPointCommand.schedule();
                    autoShootCommand.schedule();
                    setPathState(10);
                }
                break;
            case 10:
                if(autoShootCommand.isFinished()) {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                    robot.drive.follower.setMaxPower(0.5);
                    robot.drive.follower.followPath(prepareHpIntake);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTime() > 2000) {

                    robot.drive.follower.followPath(hpIntake);
                    setPathState(12);
                }
                break;
            case 12:
                if(pathTimer.getElapsedTime() > 1000) {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD);

                    robot.drive.follower.setMaxPower(1.0);
                    robot.drive.follower.followPath(shootHp);
                    setPathState(9);
                }
                break;
            case 13:
                if(autoShootCommand.isFinished()) {
                    robot.drive.follower.setMaxPower(0.5);
                    robot.drive.follower.followPath(park);

                    robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.IDLE);

                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.DISABLED);
                    setPathState(-1);
                }
                break;
        }
    }

    public void buildPaths() {
        shootPreload = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPose,
                                shootPose
                        )
                ).setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        prepareMark2 = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPose,
                                intakeMark2StartPose
                        )
                )
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeMark2StartPose.getHeading())
                .build();

        prepareMark3 = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPose,
                                intakeMark3StartPose
                        )
                ).setLinearHeadingInterpolation(shootPose.getHeading(), intakeMark3StartPose.getHeading())
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
                                shootPose
                        )
                ).setLinearHeadingInterpolation(intakeMark3EndPose.getHeading(), shootPose.getHeading())
                .build();

        intakeMark2 = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                intakeMark2StartPose,
                                intakeMark2EndPose
                        )
                ).setLinearHeadingInterpolation(intakeMark2StartPose.getHeading(), intakeMark2EndPose.getHeading())
                .build();

        shootMark2 = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                intakeMark2EndPose,
                                shootPose
                        )
                ).setLinearHeadingInterpolation(intakeMark2EndPose.getHeading(), shootPose.getHeading())
                .build();

        prepareHpIntake = robot.drive.follower.pathBuilder().addPath(
                    new BezierCurve(
                            shootPose,
                            hpIntakeStartPose
                    )
                ).setTangentHeadingInterpolation()
                .build();

        hpIntake = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                hpIntakeStartPose,
                                hpIntakeEndPose
                        )
                ).setLinearHeadingInterpolation(hpIntakeStartPose.getHeading(), hpIntakeEndPose.getHeading())
                .build();

        shootHp = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                hpIntakeEndPose,
                                shootPose
                        )
                ).setLinearHeadingInterpolation(hpIntakeEndPose.getHeading(), shootPose.getHeading())
                .build();

        park = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                shootPose,
                                parkPose
                        )
                ).setConstantHeadingInterpolation(parkPose.getHeading())
                .build();
    }

    public void buildPoses(Robot.Team team) {
        // BUILT FOR BLUE
        startPose = new Pose(57, 8, Math.toRadians(90));

        shootPose = new Pose(59, 19, Math.toRadians(110));

        intakeMark3StartPose = new Pose(52, 36, Math.toRadians(180));

        intakeMark3EndPose = new Pose(12, 36, Math.toRadians(180));

        intakeMark2StartPose = new Pose(48, 60, Math.toRadians(180));

        intakeMark2EndPose = new Pose(12, 58, Math.toRadians(180));

        hpIntakeStartPose = new Pose(13, 14, Math.toRadians(190));

        hpIntakeEndPose = new Pose(11, 9, Math.toRadians(185));

        parkPose = new Pose(54, 24, Math.toRadians(135));

        if (team == Robot.Team.RED) {
            startPose = startPose.mirror();

            shootPose = shootPose.mirror();

            intakeMark3StartPose = intakeMark3StartPose.mirror();
            intakeMark3EndPose = intakeMark3EndPose.mirror();

            intakeMark2StartPose = intakeMark2StartPose.mirror();
            intakeMark2EndPose = intakeMark2EndPose.mirror();

            hpIntakeStartPose = hpIntakeStartPose.mirror();
            hpIntakeEndPose = hpIntakeEndPose.mirror();

            parkPose = parkPose.mirror();
        }

        // built for red mirrors
        // startPose.mirror();
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }
}
