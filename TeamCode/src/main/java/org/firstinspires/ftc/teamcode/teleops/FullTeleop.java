package org.firstinspires.ftc.teamcode.teleops;

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
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

@TeleOp
public class FullTeleop extends CommandOpMode {
    private Robot robot;

    private Motor frontLeft;
    private Motor frontRight;
    private Motor backLeft;
    private Motor backRight;
    private MecanumDrive drive;

    private GamepadEx gamepad;

    public void runOpMode() throws InterruptedException {
        waitForStart();
        reset();
        initialize();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
    }

    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, Robot.Team.RED);

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
                        new InstantCommand(() -> robot.shooter.setRPM(0.7)),
                        new InstantCommand(() -> robot.shooter.setRPM(0))
                );
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        drive.driveRobotCentric(-gamepad.getLeftX(), -gamepad.getLeftY(), -gamepad.getRightX(), false);

        if(gamepad1.dpadUpWasPressed()) {
            robot.shooter.setAngle(robot.shooter.getAngle() + 5);
        }

        else if(gamepad1.dpadDownWasPressed()) {
            robot.shooter.setAngle(robot.shooter.getAngle() - 5);
        }

        if(gamepad1.dpadRightWasPressed()) {
            robot.shooter.setRPM(robot.shooter.getRpm() + 0.05);
        }

        else if(gamepad1.dpadLeftWasPressed()) {
            robot.shooter.setRPM(robot.shooter.getRpm() - 0.05);
        }

        telemetry.addData("RPM", robot.shooter.flywheelMotor.getCorrectedVelocity());
        telemetry.addData("SETPOINT", robot.shooter.getRpm());
        telemetry.addData("Angle", robot.shooter.getAngle());

        telemetry.update();
    }
}
