package org.firstinspires.ftc.teamcode.teleops;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.ShooterCalculations;
import org.firstinspires.ftc.teamcode.automations.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.automations.commands.AdjustShooterCommand;
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

    CommandBase autoShootCommand;
    CommandBase liveAdjustShooterCommand;

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

        robot = new Robot(hardwareMap, Robot.Team.BLUE);
        robot.drive.follower.setStartingPose(GlobalDataStorage.robotPose);
        //robot.localizer.setStartPose(new Pose(48, 72, PI/2));

        gamepad = new GamepadEx(gamepad1);

        autoShootCommand = new InstantCommand();
        liveAdjustShooterCommand = new InstantCommand();

        // AUTO SHOOT SYSTEM (Left Back Paddle)
        gamepad.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(() -> {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD);
                    robot.transfer.setBeltState(TransferSubsystem.BeltState.HOLD);

                    liveAdjustShooterCommand = new AdjustShooterCommand(robot);
                    schedule(liveAdjustShooterCommand.perpetually());
                })
                .whenReleased(() -> {
                    liveAdjustShooterCommand.end(true);
                    autoShootCommand = new AutoShootCommand(robot, 3);
                    schedule(autoShootCommand);
                });

        // HOLD AIM (Right Back Paddle)
        gamepad.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(() -> {
                    robot.drive.setHoldPoint(
                            robot.localizer.getPose().setHeading(
                                    atan2(
                                            GlobalDataStorage.goalPose.minus(robot.localizer.getPose()).getY(),
                                            GlobalDataStorage.goalPose.minus(robot.localizer.getPose()).getX()
                                    )
                            )
                    );
                    robot.drive.setDriveMode(Drive.DriveMode.HOLD_POINT);
                }
                )
                .whenReleased(() -> robot.drive.setDriveMode(Drive.DriveMode.MANUAL));

        // GATE CONTROL
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> {
                    robot.transfer.setGatePosition(TransferSubsystem.GatePosition.OPEN);
                    ShooterSubsystem.setkP(0.005);
                })
                .whenReleased(() -> {
                    robot.transfer.setGatePosition(TransferSubsystem.GatePosition.CLOSED);
                    ShooterSubsystem.setkP(0.001);
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
        gamepad.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(() -> {
                    robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.IDLE);
                    robot.drive.setDriveMode(Drive.DriveMode.MANUAL);
                    autoShootCommand.end(true);
                });

        // AUTO SET SPEED AND ANGLE
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new AdjustShooterCommand(robot));

        // SET INSTANCED
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(() -> robot.shooter.setLocal());

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
        CommandScheduler.getInstance().run();
        robot.localizer.update();
        robot.drive.update();

        robot.drive.setTeleOpVectors(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (!gamepad1.left_bumper && autoShootCommand.isFinished()) {
            double triggerValue = Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 0.2);
            robot.transfer.setCustomBeltSpeed(triggerValue);
            robot.intake.setCustomIntakeSpeed(triggerValue);
        }

        panelsTelemetry.addData("Current Tps", robot.shooter.getCurrentTps());
        panelsTelemetry.addData("Target Tps", robot.shooter.getTargetTps());
        panelsTelemetry.addData("Shooter Motor State", robot.shooter.getShooterMotorState());
        panelsTelemetry.addData("Error", robot.shooter.getError());

        panelsTelemetry.addData("Angle", robot.shooter.getAngle());

        panelsTelemetry.addData("Distance To Goal", robot.localizer.getDistanceToGoal());

        panelsTelemetry.addData("Pose", robot.localizer.getPose());

        panelsTelemetry.addData("GLOBAL POSE", GlobalDataStorage.goalPose);

        panelsTelemetry.update(telemetry);

        telemetry.update();
    }
}
