package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        forwardLimitSwitch = motor.getForwardLimitSwitch(Type.kNormallyClosed);
        reverseLimitSwitch = motor.getReverseLimitSwitch(Type.kNormallyClosed);

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

    private static NewElbow m_NewElbow = null; 

    public static NewElbow getInstance(){
        if (m_NewElbow == null){
            m_NewElbow = new NewElbow();
        }

        return m_NewElbow;
    }

    public void setZero(){
        zero = encoder.getPosition();
    }

    public void manual(double speed){
        if (limitSwitchCheck()){
            update();
        } else {
            SmartDashboard.putNumber("Elbow Power:", speed);

            setPoint = encoder.getPosition();
        }
    }

    public void update() {
        limitSwitchCheck();

        double speed;

        speed = pidController.calculate(encoder.getPosition() / 360, setPoint / 360);

        if (speed > 0.5){
            speed = 0.5;
        }

        if (speed < -0.5){
            speed = -0.5;
        }

        SmartDashboard.putNumber("Elbow Power: ", speed);
    }

    private boolean limitSwitchCheck(){
        if (forwardLimitSwitch.isPressed()){
            //TODO: check which is forwardlimit which is reverselimit
            //TODO: if stow limit switch is forward, swap the setPoint statements below
            setPoint = ElbowConstants.setPointStow + zero;
            return true;
        } else if (reverseLimitSwitch.isPressed()){
            setPoint = ElbowConstants.setPointIntake + zero;
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elbow Encoder: ", encoder.getPosition() - zero);
    }

    public void Stow(){
        setPoint = ElbowConstants.setPointStow + zero;
    }

    public void Intake(){
        setPoint = ElbowConstants.setPointIntake + zero;
    }

    public void ScoreHigh(){
        setPoint = ElbowConstants.setPointHigh + zero;
    }

    public void ScoreMid(){
        setPoint = ElbowConstants.setPointMid + zero;
    }

    public void ScoreLow(){
        setPoint = ElbowConstants.setPointLow + zero;
    }

}
