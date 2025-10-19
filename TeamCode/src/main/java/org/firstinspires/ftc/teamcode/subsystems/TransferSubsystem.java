package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import org.firstinspires.ftc.teamcode.HardwareMapNames;

@Configurable
public class TransferSubsystem extends SubsystemBase {
    private final CRServoEx servo1;
    private final CRServoEx servo2;
    private final ServoEx gate;
    public static double OPEN_POS = 0.15;
    public static double CLOSED_POS = 0.35;

    public enum GatePosition {
        OPEN,
        CLOSED;

        public double getValue() {
            switch (this) {
                case OPEN:
                    return OPEN_POS;
                case CLOSED:
                    return CLOSED_POS;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public TransferSubsystem(HardwareMap hardwareMap) {
        servo1 = new CRServoEx(hardwareMap, HardwareMapNames.INTAKE_SERVO_FRONT_RIGHT);
        servo1.set(0);

        servo2 = new CRServoEx(hardwareMap, HardwareMapNames.INTAKE_SERVO_FRONT_LEFT);
        servo2.set(0);
        servo2.setInverted(true);


        gate = new ServoEx(hardwareMap, HardwareMapNames.INTAKE_GATE_SERVO);
        this.setGatePosition(GatePosition.OPEN);
    }

    public void setRow1Speed(double speed) {
        servo1.set(speed);
        servo2.set(speed);
    }



    public void stopAllServos() {
        setRow1Speed(0);
    }

    public void setGatePosition(GatePosition gatePosition) {
        gate.set(gatePosition.getValue());
    }
}
