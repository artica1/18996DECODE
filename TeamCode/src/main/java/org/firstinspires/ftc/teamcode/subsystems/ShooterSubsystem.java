package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.PI;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareMapNames;

@Configurable
public class ShooterSubsystem extends SubsystemBase {
    public final MotorEx flywheelMotor;
    private final ServoEx angleServo;
    private double rpm;
    private double angle;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheelMotor = new MotorEx(hardwareMap, HardwareMapNames.SHOOTER_MOTOR, Motor.GoBILDA.BARE);
        flywheelMotor.setRunMode(Motor.RunMode.RawPower);
        flywheelMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setVeloCoefficients(1, 0, 0);
        flywheelMotor.setFeedforwardCoefficients(0, 1, 0);
        //flywheelMotor.set(0);

        angleServo = new ServoEx(hardwareMap, HardwareMapNames.SHOOTER_SERVO, 365, AngleUnit.DEGREES); // THE SERVO IS NOW IN DEGREES 100%, set() TAKES DEGREES
        setAngle(20);
    }

    @Override
    public void periodic() {
        //flywheelMotor.setVelocity(rpm * (PI / 30), AngleUnit.RADIANS);
    }

    public void setRPM(int rpm) {
        this.rpm = rpm;
    }

    public void setAngle(double angle) {
        if (angle < 20) {
            angleServo.set(angle);
        } else if (angle > 60) {
            angleServo.set(60);
        } else {
            this.angle = angle;
            angleServo.set(angle);
        }
    }

    public double getAngle() {
        return angle;
    }

    public double getRpm() {
        return rpm;
    }
}
