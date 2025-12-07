package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import org.firstinspires.ftc.teamcode.HardwareMapNames;

@Configurable
public class TransferSubsystem extends SubsystemBase {
    private final CRServoEx servoRight;
    private final CRServoEx servoLeft;

    private final ServoEx gateServo;

    private BeltState beltState;
    private GatePosition gatePosition;

    public enum GatePosition {
        OPEN,
        CLOSED;
        public double getValue() {
            switch (this) {
                case OPEN:
                    return 0.35;
                case CLOSED:
                    return 0.08;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum BeltState {
        INTAKE,
        REVERSE,
        DISABLED,
        HOLD,
        CUSTOM;
        public double getValue() {
            switch (this) {
                case INTAKE:
                    return 1.0;
                case REVERSE:
                    return -1.0;
                case HOLD:
                    return 0.5;
                case CUSTOM:
                case DISABLED:
                    return 0;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public TransferSubsystem(HardwareMap hardwareMap) {
        servoRight = new CRServoEx(hardwareMap, HardwareMapNames.TRANSFER_SERVO_RIGHT);
        servoRight.set(0);

        servoLeft = new CRServoEx(hardwareMap, HardwareMapNames.TRANSFER_SERVO_LEFT);
        servoLeft.set(0);
        servoLeft.setInverted(true);

        gateServo = new ServoEx(hardwareMap, HardwareMapNames.GATE_SERVO);

        setGatePosition(GatePosition.OPEN);
        setGatePosition(GatePosition.CLOSED);
        setBeltState(BeltState.DISABLED);
    }

    public void setBeltState(BeltState beltState) {
        this.beltState = beltState;

        servoRight.set(beltState.getValue());
        servoLeft.set(beltState.getValue());
    }

    public void setCustomBeltSpeed(double speed) {
        beltState = BeltState.CUSTOM;

        servoRight.set(speed);
        servoLeft.set(speed);
    }

    public void setGatePosition(GatePosition gatePosition) {
        this.gatePosition = gatePosition;

        gateServo.set(gatePosition.getValue());
    }

    public BeltState getBeltState() {
        return beltState;
    }

    public GatePosition getGatePosition() {
        return gatePosition;
    }
}
