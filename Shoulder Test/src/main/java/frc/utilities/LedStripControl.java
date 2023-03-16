package frc.utilities;

import edu.wpi.first.wpilibj.DigitalOutput;

import frc.robot.Robot;

public class LedStripControl {

    public DigitalOutput ledStrip_0;
    public DigitalOutput ledStrip_1;
    public DigitalOutput ledStrip_2;

    public enum LED_MODE {
        LED_OFF,
        LED_AUTO,
        LED_TELEOP_NORMAL,
        LED_TELOP_BALANCING,
        LED_TELEOP_HOLD_POSITION,
        LED_PURPLE_FOR_CUBE,
        LED_YELLOW_FOR_CONE,
        UNUSED_7
    };

    public LedStripControl() {
        ledStrip_0 = new DigitalOutput(RoboRioPorts.DIO_LEDSTRIP_0);
        ledStrip_1 = new DigitalOutput(RoboRioPorts.DIO_LEDSTRIP_1);
        ledStrip_2 = new DigitalOutput(RoboRioPorts.DIO_LEDSTRIP_2);
    }

    public void setMode(LED_MODE mode) {
        int myMode = mode.ordinal();
        boolean b0 = (0 != (myMode & 1));
        boolean b1 = (0 != (myMode & 2));
        boolean b2 = (0 != (myMode & 4));
        ledStrip_0.set(b0);
        ledStrip_1.set(b1);
        ledStrip_2.set(b2);
    }

    public void updatePeriodic(LED_MODE mode) {
        switch (mode) {
            case LED_TELEOP_NORMAL:
            case LED_OFF:
            case LED_AUTO:
            case LED_TELOP_BALANCING:
            case LED_TELEOP_HOLD_POSITION:
            case LED_PURPLE_FOR_CUBE:
            case LED_YELLOW_FOR_CONE:
            default:
                break;
        }

        if (Robot.xboxController.getRawButton(Xbox.X)
                || Robot.rightJoystick.getRawButton(3)) {
            mode = LedStripControl.LED_MODE.LED_TELEOP_HOLD_POSITION;

        } else if (Robot.xboxController.getRawButton(Xbox.B)) {
            mode = LedStripControl.LED_MODE.LED_TELOP_BALANCING;

        } else if (Robot.xboxController.getRawButton(Xbox.LB)) {
            mode = LedStripControl.LED_MODE.LED_PURPLE_FOR_CUBE;

        } else if (Robot.xboxController.getRawButton(Xbox.RB)) {
            mode = LedStripControl.LED_MODE.LED_YELLOW_FOR_CONE;

        }
        setMode(mode);
    }

}
