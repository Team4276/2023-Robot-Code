package frc.systems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.utilities.Xbox;

public class Elbow {
    private static CANSparkMax driveElbow;


    private static RelativeEncoder driveElbowEncoder;

    private static SparkMaxLimitSwitch driveElbowReverseLimitSwitch;
    private static SparkMaxLimitSwitch driveElbowForwardLimitSwitch;

    private static double deadband = 0.2;


    public Elbow(int port) {
        driveElbow = new CANSparkMax(port, MotorType.kBrushless);

        driveElbowReverseLimitSwitch = driveElbow.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        driveElbowReverseLimitSwitch.enableLimitSwitch(true);
        driveElbowForwardLimitSwitch = driveElbow.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        driveElbowForwardLimitSwitch.enableLimitSwitch(true);

        driveElbowEncoder = driveElbow.getEncoder();

        
    }

    private static void elbowManual(){
        double power = Robot.xboxController.getLeftY() /3.75;

        driveElbow.set(power);

    }

    public static void ElbowUpdate() {
        double leftY = Robot.xboxController.getLeftY();
        if (leftY > deadband) {
            if (driveElbowForwardLimitSwitch.isPressed()) {
                // Joystick indicates move farther forward, but the forward limit switch is
                // pressed
                driveElbow.set(0);
            } else {
                elbowManual();
            }

        } else if (leftY < deadband) {
            if (driveElbowReverseLimitSwitch.isPressed()) {
                // Joystick indicates move farther reverse, but the reverse limit switch is
                // pressed
                driveElbow.set(0);
            } else {
                elbowManual();
            }

        }

        

        SmartDashboard.putNumber("Raw Elbow Encoder:  ", driveElbowEncoder.getPosition());
        SmartDashboard.putNumber("Elbow Power", driveElbow.getAppliedOutput());
    }

}
