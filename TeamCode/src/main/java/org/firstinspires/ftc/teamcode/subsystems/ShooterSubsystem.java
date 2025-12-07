package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareMapNames;
import org.firstinspires.ftc.teamcode.automations.ShooterCalculations;

@Configurable
public class ShooterSubsystem extends SubsystemBase {
    private final MotorEx flywheelMotor;
    private ShooterMotorState shooterMotorState;
    private ShooterAngleState shooterAngleState;

    private double distanceToGoal = 0; // cant be zero idfk why

    public static double kS = 0.1;
    public static double kV = 0.00039;
    public static double kP = 0.003;
    private final double idlePower = 0.3;

    private final ServoEx angleServo;

    private double localAngle = 40;
    private int localTargetTps = 1350;

    private int targetTps;

    public enum ShooterMotorState {
        ACTIVE,
        IDLE,
        UNPOWERED
    }

    public enum ShooterAngleState {
        AUTO,
        MANUAL
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
        setShooterAngleState(ShooterAngleState.AUTO);
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

        if (shooterAngleState == ShooterAngleState.AUTO) {
            setAngle(ShooterCalculations.getAngle(distanceToGoal));
        }
    }

    public void updateDistanceToGoal(double distanceToGoal) {
        this.distanceToGoal = distanceToGoal;
    }

    public void setTargetTps(double targetTps) {
        this.targetTps = (int)targetTps;
    }

    public void setAngle(double angle) {
        if (angle < 5) {
            angleServo.set(5);
        } else if (angle > 45) {
            angleServo.set(45);
        } else {
            angleServo.set(angle);
        }
    }

    public double getAngle() {
        return angleServo.get();
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

    public ShooterAngleState getShooterAngleState() {
        return shooterAngleState;
    }

    public void setShooterAngleState(ShooterAngleState shooterAngleState) {
        this.shooterAngleState = shooterAngleState;
    }

    public int getError() {
        return targetTps - getCurrentTps();
    }

    public static void setkP(double kP) {
        ShooterSubsystem.kP = kP;
    }

    public void setLocal() {
        setShooterMotorState(ShooterMotorState.ACTIVE);
        setShooterAngleState(ShooterAngleState.MANUAL);
        targetTps = localTargetTps;
        angleServo.set(localAngle);
    }

    public void setLocalTargetTps(int localTargetTps) {
        this.localTargetTps = localTargetTps;
        setLocal();
    }

    public void setLocalAngle(double localAngle) {
        if (localAngle < 5) {
            this.localAngle = 5;
        } else if (localAngle > 45) {
            this.localAngle = 45;
        } else {
            this.localAngle = localAngle;
        }
        setLocal();
    }

    public int getLocalTargetTps() {
        return localTargetTps;
    }

    public double getLocalAngle() {
        return localAngle;
    }
}
