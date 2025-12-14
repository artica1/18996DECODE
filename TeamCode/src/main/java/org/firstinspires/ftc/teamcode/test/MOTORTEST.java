package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.teamcode.HardwareMapNames;
import org.firstinspires.ftc.teamcode.automations.ColorSensorManager;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.Arrays;

@TeleOp
@Configurable
public class MOTORTEST extends LinearOpMode {

    public static double kS = 0.15;
    public static double kV = 0.00044;
    public static double kP = 0.005;

    @IgnoreConfigurable
    private static TelemetryManager panelsTelemetry;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        MotorEx shooter1 = new MotorEx(hardwareMap, HardwareMapNames.SHOOTER_MOTOR_1, Motor.GoBILDA.BARE);
        MotorEx shooter2 = new MotorEx(hardwareMap, HardwareMapNames.SHOOTER_MOTOR_2, Motor.GoBILDA.BARE);

        shooter1.setRunMode(Motor.RunMode.RawPower);
        shooter1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        shooter2.setRunMode(Motor.RunMode.RawPower);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2.setInverted(true);

        waitForStart();

        while (opModeIsActive()) {
            double output = 0;

            output += kS * Math.signum(1500);

            output += kV * 1500;

            output += kP * (1500 - shooter1.getCorrectedVelocity());

            output = Range.clip(output, -1.0, 1.0);

            shooter1.set(output);
            shooter2.set(output);

            panelsTelemetry.addData("Error", 1500 - shooter1.getCorrectedVelocity());
            panelsTelemetry.addData("Velocity", shooter1.getCorrectedVelocity());

            panelsTelemetry.update(telemetry);
        }
    }
}
