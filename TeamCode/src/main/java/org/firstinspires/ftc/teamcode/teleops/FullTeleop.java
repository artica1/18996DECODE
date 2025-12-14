package org.firstinspires.ftc.teamcode.teleops;

import static java.lang.Math.PI;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.automations.commands.AdjustShooterSpeedCommand;
import org.firstinspires.ftc.teamcode.automations.commands.HoldPointCommand;
import org.firstinspires.ftc.teamcode.automations.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

import java.util.List;

@Configurable
@TeleOp
public class FullTeleop extends CommandOpMode {
    private Robot robot;

    private GamepadEx gamepad;

    private double previousVelocityError = 0;

    // Commands are single instanced for teleop
    private Command autoShootCommand = new InstantCommand();
    private Command adjustShooterSpeedCommand = new InstantCommand();
    private Command holdPointCommand = new InstantCommand();

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

        robot = new Robot(hardwareMap, GlobalDataStorage.autoTeam);
        robot.localizer.setStartPose(GlobalDataStorage.robotPose.copy());

        telemetry.addData("Global Pose", GlobalDataStorage.robotPose);
        telemetry.update();

        gamepad = new GamepadEx(gamepad1);

        // AUTO SHOOT SYSTEM
        gamepad.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(() -> {
                    robot.shooter.setTargetTps(1400);
                    robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.ACTIVE);
                })
                .whenReleased(() -> {
                    autoShootCommand = new AutoShootCommand(robot);
                    schedule(autoShootCommand);
                });

        // HOLD AIM (Right Back Paddle)
        gamepad.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(() -> {
                    holdPointCommand = new HoldPointCommand(robot);
                    adjustShooterSpeedCommand = new AdjustShooterSpeedCommand(robot, robot.localizer.getPose());
                    schedule(holdPointCommand);
                    schedule(adjustShooterSpeedCommand);
                }).whenReleased(() -> holdPointCommand.end(false));

        // GATE CONTROL
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> {
                    robot.transfer.setGatePosition(TransferSubsystem.GatePosition.OPEN);
                })
                .whenReleased(() -> {
                    robot.transfer.setGatePosition(TransferSubsystem.GatePosition.CLOSED);
                });

        // INTAKE/BELTS OUTTAKE MODE
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> {
                    robot.transfer.setBeltState(TransferSubsystem.BeltState.INTAKE);
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.OUTTAKE);
                })
                .whenReleased(() -> {
                    robot.transfer.setBeltState(TransferSubsystem.BeltState.CUSTOM);
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.CUSTOM);
                });

        // IDLE/RESET
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(() -> {
                    autoShootCommand.cancel();
                    holdPointCommand.cancel();
                    adjustShooterSpeedCommand.cancel();

                    robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.IDLE);
                    robot.drive.setDriveMode(Drive.DriveMode.MANUAL);
                    robot.shooter.setShooterAngleState(ShooterSubsystem.ShooterAngleState.AUTO);
                });

        // SET SPEED AND ANGLE, CLOSE LAUNCH ZONE (Left Back Paddle)
        gamepad.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(() -> {
                    robot.shooter.setTargetTps(1200);
                    robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.ACTIVE);
                });

        // SET SPEED AND ANGLE, FAR LAUNCH ZONE
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(() -> {
                    robot.shooter.setTargetTps(1450);
                    robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.ACTIVE);
                });

        // FINE DRIVE ENABLE
        gamepad.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(() -> robot.drive.setDriveMode(Drive.DriveMode.FINE));

        /*
        // SET INSTANCED
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(() -> robot.shooter.setLocal());

         */

        // EDIT INSTANCED
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> robot.shooter.setLocalAngle(robot.shooter.getLocalAngle() + 5));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> robot.shooter.setLocalAngle(robot.shooter.getLocalAngle() - 5));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> robot.shooter.setLocalTargetTps(robot.shooter.getLocalTargetTps() + 100));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> robot.shooter.setLocalTargetTps(robot.shooter.getLocalTargetTps() - 100));
    }

    @Override
    public void run() {
        robot.update();
        CommandScheduler.getInstance().run();

        robot.drive.setTeleOpVectors(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (!gamepad1.left_bumper && autoShootCommand.isFinished() ) {
            double triggerValue = gamepad1.right_trigger - gamepad1.left_trigger;

            if (triggerValue != 0) {
                robot.transfer.setCustomBeltSpeed(triggerValue);
                robot.intake.setCustomIntakeSpeed(triggerValue);
            } else {
                robot.transfer.setCustomBeltSpeed(0.0);
                robot.intake.setCustomIntakeSpeed(0.2);
            }
        }

        if (Math.abs(robot.shooter.getError()) < 30 && previousVelocityError > 30) {
            gamepad1.rumble(100);
        }

        panelsTelemetry.addData("Current Tps", robot.shooter.getCurrentTps());
        panelsTelemetry.addData("Target Tps", robot.shooter.getTargetTps());
        panelsTelemetry.addData("Shooter Motor State", robot.shooter.getShooterMotorState());
        panelsTelemetry.addData("Error", robot.shooter.getError());

        panelsTelemetry.addData("Angle", robot.shooter.getAngle());

        panelsTelemetry.addData("Distance To Goal", robot.localizer.getDistanceToGoal());

        panelsTelemetry.addData("Pose", robot.localizer.getPose());

        panelsTelemetry.addData("Goal Pose", GlobalDataStorage.goalPose);

        panelsTelemetry.addData("Ball detected", robot.colorSensorManager.ballDetected());

        panelsTelemetry.update(telemetry);

        telemetry.update();
    }
}
