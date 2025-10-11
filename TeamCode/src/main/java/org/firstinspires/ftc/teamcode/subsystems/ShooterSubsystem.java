package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.HardwareMapNames;

public class ShooterSubsystem extends SubsystemBase {

    private final MotorEx motor;
    private int rpm;
    private final ServoEx servo;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        motor = new MotorEx(hardwareMap, HardwareMapNames.SHOOTER_MOTOR, Motor.GoBILDA.BARE);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setVeloCoefficients(1, 0, 0);
        motor.setFeedforwardCoefficients(0, 1, 0);

        servo = new ServoEx(hardwareMap, HardwareMapNames.SHOOTER_SERVO);
        servo.set(0);
    }

    @Override
    public void periodic() {
        motor.set(rpm/6000.0);
    }

    public void setRPM(int rpm) {
        this.rpm = rpm;
    }

    public void setAngle(double angle) {
        servo.set(angle); // TODO: CONVERSION FACTOR
    }
}
