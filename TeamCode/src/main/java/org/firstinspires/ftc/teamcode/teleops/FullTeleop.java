package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.HardwareMapNames;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

import java.util.List;

@Configurable
@TeleOp
public class FullTeleop extends CommandOpMode {
    private Robot robot;

    private Motor frontLeft;
    private Motor frontRight;
    private Motor backLeft;
    private Motor backRight;
    private MecanumDrive drive;

    private GamepadEx gamepad;

    @IgnoreConfigurable
    private static TelemetryManager panelsTelemetry;

    public void runOpMode() throws InterruptedException {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        waitForStart();
        reset();
        initialize();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            hubs.forEach(LynxModule::clearBulkCache);
            run();
        }
    }

    @Override
    public void initialize() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new Robot(hardwareMap, Robot.Team.RED);
        robot.localizer.setStartPose(new Pose(96, 72, Math.PI/2));

        frontLeft = new Motor(hardwareMap, HardwareMapNames.DRIVE_FRONT_LEFT);
        frontRight = new Motor(hardwareMap, HardwareMapNames.DRIVE_FRONT_RIGHT);
        backLeft = new Motor(hardwareMap, HardwareMapNames.DRIVE_BACK_LEFT);
        backRight = new Motor(hardwareMap, HardwareMapNames.DRIVE_BACK_RIGHT);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE)),
                                new InstantCommand(() -> robot.transfer.setBeltState(TransferSubsystem.BeltState.INTAKE))
                        )
                )
                .whenReleased(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.intake.setIntakeState(IntakeSubsystem.IntakeState.DISABLED)),
                                new InstantCommand(() -> robot.transfer.setBeltState(TransferSubsystem.BeltState.DISABLED))
                        )
                );

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.intake.setIntakeState(IntakeSubsystem.IntakeState.REVERSE)),
                                new InstantCommand(() -> robot.transfer.setBeltState(TransferSubsystem.BeltState.REVERSE))
                        )
                )
                .whenReleased(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.intake.setIntakeState(IntakeSubsystem.IntakeState.DISABLED)),
                                new InstantCommand(() -> robot.transfer.setBeltState(TransferSubsystem.BeltState.DISABLED))
                        )
                );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.transfer.setGatePosition(TransferSubsystem.GatePosition.OPEN))
                        )
                )
                .whenReleased(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.transfer.setGatePosition(TransferSubsystem.GatePosition.CLOSED))
                        )
                );

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .toggleWhenPressed(
                        new InstantCommand(() -> robot.shooter.setTargetTps(1500)),
                        new InstantCommand(() -> robot.shooter.setTargetTps(0))
                );

        gamepad.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(
                        new InstantCommand(() -> robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.IDLE))
                );

        gamepad.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(
                        new SequentialCommandGroup(
                            new InstantCommand(() -> robot.shooter.setTargetTps(1500)),
                            new InstantCommand(() -> robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.ACTIVE))
                        )
                );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.localizer.update();

        drive.driveRobotCentric(-gamepad.getLeftX(), -gamepad.getLeftY(), -gamepad.getRightX(), false);

        if(gamepad1.dpadUpWasPressed()) {
            robot.shooter.setAngle(robot.shooter.getAngle() + 5);
        }

        else if(gamepad1.dpadDownWasPressed()) {
            robot.shooter.setAngle(robot.shooter.getAngle() - 5);
        }

        if(gamepad1.dpadRightWasPressed()) {
            robot.shooter.setTargetTps(robot.shooter.getTargetTps() + 100);
        }

        else if(gamepad1.dpadLeftWasPressed()) {
            robot.shooter.setTargetTps(robot.shooter.getCurrentTps() - 100);
        }

        panelsTelemetry.addData("Current Tps", robot.shooter.getCurrentTps());
        panelsTelemetry.addData("Target Tps", robot.shooter.getTargetTps());
        panelsTelemetry.addData("Shooter Motor State", robot.shooter.getShooterMotorState());
        panelsTelemetry.addData("Error", robot.shooter.getError());

        panelsTelemetry.addData("Angle", robot.shooter.getAngle());

        panelsTelemetry.addData("Distance To Goal", robot.localizer.getDistanceToGoal());

        panelsTelemetry.addData("Pose", robot.localizer.getPose());

        panelsTelemetry.update(telemetry);

        telemetry.update();
    }
}
