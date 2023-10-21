package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
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

    private final ArmFeedforward armFeedforward;

    private double setPoint;

    private NewElbow(){
        motor = new CANSparkMax(ElbowConstants.ElbowID, MotorType.kBrushless);

        forwardLimitSwitch = motor.getForwardLimitSwitch(Type.kNormallyOpen);
        reverseLimitSwitch = motor.getReverseLimitSwitch(Type.kNormallyOpen);

        forwardLimitSwitch.enableLimitSwitch(true);
        reverseLimitSwitch.enableLimitSwitch(true);

        encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        pidController = new PIDController(
            ElbowConstants.kP, 
            ElbowConstants.kI, 
            ElbowConstants.kD);

        pidController.enableContinuousInput(0, 1);
        
        pidController.setIntegratorRange(-ElbowConstants.kIRange, ElbowConstants.kIRange);
        
        pidController.setTolerance(0.01, 0.05);

        armFeedforward = new ArmFeedforward(
            ElbowConstants.kS, 
            ElbowConstants.kG, 
            ElbowConstants.kV, 
            ElbowConstants.kA);

        setPoint = encoder.getPosition();
    }

    private static NewElbow mInstance = null; 
    public static NewElbow getInstance(){
        if (mInstance == null){
            mInstance = new NewElbow();
        }

        return mInstance;
    }

    // Forward on Controller Returns Negative
    // Left on Controller Returns Negative
    public void manual(double speed){
        speed = limitSwitchCheck(speed);
        
        motor.set(speed * ElbowConstants.maxPower * ElbowConstants.manualCoefficient);

        setPoint = encoder.getPosition();
    }

    public void update() {
        double speed;

        speed = pidController.calculate(encoder.getPosition(), setPoint);

        speed = armFF(speed);

        speed = limitSwitchCheck(speed);

        speed = MathUtil.clamp(speed, -ElbowConstants.maxPower, ElbowConstants.maxPower);

        motor.set(speed);
    }

    private double armFF(double speed){
        double position = setPoint - ElbowConstants.elbowGroundOffset;
        if (position > 1){
            position -= 1;
        }

        if (position < 0){
            position = 1 + position;
        }

        SmartDashboard.putNumber("FF Position ", position);

        speed += armFeedforward.calculate(
            2 * Math.PI * position,
            speed * ElbowConstants.powerToRadsConversion);

        return speed;
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

    public boolean isStable(){
        return pidController.atSetpoint();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elbow Encoder: ", encoder.getPosition());
        SmartDashboard.putNumber("Elbow Setpoint: ", setPoint);

        SmartDashboard.putBoolean("Forward Limit: ", forwardLimitSwitch.isPressed());
        SmartDashboard.putBoolean("Reverse Limit: ", reverseLimitSwitch.isPressed());

        SmartDashboard.putNumber("Elbow Power: ", motor.getAppliedOutput());

        SmartDashboard.putBoolean("Elbow at Setpoint: ", pidController.atSetpoint());
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
