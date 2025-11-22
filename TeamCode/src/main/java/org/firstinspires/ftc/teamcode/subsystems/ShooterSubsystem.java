package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareMapNames;

@Configurable
public class ShooterSubsystem extends SubsystemBase {
    private final MotorEx flywheelMotor;
    private ShooterMotorState shooterMotorState;
    public static double kS = 0.1;
    public static double kV = 0.00041;
    public static double kP = 0.001;
    private double idlePower = 0.4;

    private final ServoEx angleServo;
    private double angle;

    private int targetTps;

    public enum ShooterMotorState {
        ACTIVE,
        IDLE,
        UNPOWERED
    }

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheelMotor = new MotorEx(hardwareMap, HardwareMapNames.SHOOTER_MOTOR, Motor.GoBILDA.BARE);
        flywheelMotor.setRunMode(Motor.RunMode.RawPower);
        flywheelMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setInverted(true);
        setShooterMotorState(ShooterMotorState.IDLE);
        setTargetTps(0);

        angleServo = new ServoEx(hardwareMap, HardwareMapNames.SHOOTER_SERVO, 365, AngleUnit.DEGREES); // THE SERVO IS NOW IN DEGREES 100%, set() TAKES DEGREES
        setAngle(5);
    }

    @Override
    public void periodic() {
        if (shooterMotorState == ShooterMotorState.ACTIVE) {
            double output = 0;

            output += kS * Math.signum(targetTps);

            output += kV * targetTps;

            output += kP * getError();

            output = Range.clip(output, -1.0, 1.0);

            flywheelMotor.set(output);
        }
        else if (shooterMotorState == ShooterMotorState.IDLE) {
            flywheelMotor.set(idlePower);
        }
        else if (shooterMotorState == ShooterMotorState.UNPOWERED) {
            flywheelMotor.set(0);
        }
    }

    public void setTargetTps(int targetTps) {
        this.targetTps = targetTps;
    }

    public void setAngle(double angle) {
        if (angle < 5) {
            angleServo.set(5);
        } else if (angle > 45) {
            angleServo.set(45);
        } else {
            this.angle = angle;
            angleServo.set(angle);
        }
    }

    public double getAngle() {
        return angle;
    }

    public int getTargetTps() {
        return targetTps;
    }

    public int getCurrentTps() {
        return (int)flywheelMotor.getCorrectedVelocity();
    }

    public ShooterMotorState getShooterMotorState() {
        return shooterMotorState;
    }

    public void setShooterMotorState(ShooterMotorState shooterMotorState) {
        this.shooterMotorState = shooterMotorState;
    }

    public int getError() {
        return targetTps - getCurrentTps();
    }
}
