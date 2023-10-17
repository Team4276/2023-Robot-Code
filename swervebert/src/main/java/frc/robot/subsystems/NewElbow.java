package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElbowConstants;

public class NewElbow extends SubsystemBase {
    private final CANSparkMax motor;

    private final SparkMaxLimitSwitch forwardLimitSwitch;
    private final SparkMaxLimitSwitch reverseLimitSwitch;

    private final SparkMaxAbsoluteEncoder encoder;

    private final PIDController pidController;

    private double setPoint;

    private double zero;

    private NewElbow(){
        motor = new CANSparkMax(ElbowConstants.ElbowID, MotorType.kBrushless);

        forwardLimitSwitch = motor.getForwardLimitSwitch(Type.kNormallyOpen);
        reverseLimitSwitch = motor.getReverseLimitSwitch(Type.kNormallyOpen);

        forwardLimitSwitch.enableLimitSwitch(true);
        reverseLimitSwitch.enableLimitSwitch(true);

        encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        zero = ElbowConstants.elbowZeroDefault;

        pidController = new PIDController(
            ElbowConstants.kP, 
            ElbowConstants.kI, 
            ElbowConstants.kD);

        setPoint = encoder.getPosition();
    }

    private static NewElbow mInstance = null; 
    public static NewElbow getInstance(){
        if (mInstance == null){
            mInstance = new NewElbow();
        }

        return mInstance;
    }

    public void setZero(){
        zero = encoder.getPosition();
    }

    // Forward on Controller Returns Negative
    // Left on Controller Returns Negative
    public void manual(double speed){
        speed = limitSwitchCheck(speed);
        
        motor.set(speed * ElbowConstants.maxPower * ElbowConstants.manualCoefficient * ElbowConstants.motorCoefficient);
    }

    public void update() {
        double speed;

        speed = MathUtil.clamp(
            pidController.calculate(encoder.getPosition(), setPoint + zero),
            -ElbowConstants.maxPower, ElbowConstants.maxPower)
            * ElbowConstants.motorCoefficient;

        speed = limitSwitchCheck(speed);

        motor.set(speed);
    }

    private double limitSwitchCheck(double speed){
        if (forwardLimitSwitch.isPressed()){
            if (speed > 0){
                speed = 0;
            }
        } 
        
        if (reverseLimitSwitch.isPressed()){
            if (speed < 0){
                speed = 0;
            }
        }

        return speed;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elbow Encoder: ", encoder.getPosition() - zero);
        SmartDashboard.putNumber("Elbow Setpoint: ", setPoint);
        SmartDashboard.putNumber("Elbow Zero: ", zero);

        SmartDashboard.putBoolean("Forward Limit: ", forwardLimitSwitch.isPressed());
        SmartDashboard.putBoolean("Reverse Limit: ", reverseLimitSwitch.isPressed());

        SmartDashboard.putNumber("Elbow Power: ", motor.getAppliedOutput());
    }

    public Command Stow(){
        return new InstantCommand(() -> {setPoint = ElbowConstants.setPointStow;});

    }

    public Command Intake(){
        return new InstantCommand(() -> {setPoint = ElbowConstants.setPointIntake;});
    }

    public Command ScoreHigh(){
        return new InstantCommand(() -> {setPoint = ElbowConstants.setPointHigh;});
    }

    public Command ScoreMid(){
        return new InstantCommand(() -> {setPoint = ElbowConstants.setPointMid;});
    }

    public Command ScoreLow(){
        return new InstantCommand(() -> {setPoint = ElbowConstants.setPointLow;});
    }

}
