package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmSubsystemConstants;

public class ArmSubsystem extends SubsystemBase {
    public final CANSparkMax elbowMotor;

    private static SparkMaxPIDController elbowPidController;

    private static SparkMaxAbsoluteEncoder elbowEncoder;
    
    private static SparkMaxLimitSwitch elbowReverseLimitSwitch;
    private static SparkMaxLimitSwitch elbowForwardLimitSwitch;

     // PID coefficients
     private static double kP = 2e-2;
     private static double kI = 0;
     private static double kD = 0;
     private static double kIz = 0;
     private static double kFF = 0.000156;
 
     private static double kMaxOutput = 1;
     private static double kMinOutput = -1;
 
     // Smart Motion Coefficients
     private static double maxVel = 80; // rpm
     private static double maxAcc = 10;
     private static double minVel = 0;
 
     private static double allowedErr = 0;

    private static double elbowZero = 0;

    public ArmSubsystem(){
        elbowMotor = new CANSparkMax(ArmSubsystemConstants.elbowMotorCanId, MotorType.kBrushless);

        elbowReverseLimitSwitch = elbowMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        elbowForwardLimitSwitch = elbowMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        elbowReverseLimitSwitch.enableLimitSwitch(true);
        elbowForwardLimitSwitch.enableLimitSwitch(true);

        elbowEncoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);

        elbowEncoder.setPositionConversionFactor(1);

        elbowPidController = elbowMotor.getPIDController();
        elbowPidController.setFeedbackDevice(elbowEncoder);

        int smartMotionSlot = 0;

        CANSparkMax[] motorArray = { elbowMotor };
        for (CANSparkMax motor : motorArray) {
            SparkMaxPIDController pidController = motor.getPIDController();

            pidController.setP(kP);
            pidController.setI(kI);
            pidController.setD(kD);
            pidController.setIZone(kIz);
            pidController.setFF(kFF);
            pidController.setOutputRange(kMinOutput, kMaxOutput);
            pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
            pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
            pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
            pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
        }

        this.register();

    }

    private boolean checkForwardLimitSwitch(){
        return elbowForwardLimitSwitch.isPressed();
    }

    private boolean checkReverseLimitSwitch(){
        return elbowReverseLimitSwitch.isPressed();
    }

    private void setForwardZero() {
        elbowZero = elbowEncoder.getPosition();
    }

    private void setReverseZero() {
        elbowZero = elbowEncoder.getPosition();
    }

    private double getCorrectedPos() {
        return elbowEncoder.getPosition() - elbowZero;
    }

    private void setElbowPos(double pos){
        elbowPidController.setReference(pos + elbowZero, ControlType.kSmartMotion);

    }

    private void setElbowVel(double vel){
        elbowPidController.setReference(vel, ControlType.kSmartVelocity);
    }

    public Command setElbowVelCmd(double vel){
        Command command;

        if(vel > 0){
            if(checkForwardLimitSwitch()){
                command = new PrintCommand("Don't give Ethan PTSD");
            } else {
                command = new InstantCommand(() -> {setElbowVel(vel);});
            }

        } else {
            if(checkReverseLimitSwitch()){
                command = new PrintCommand("Don't give Ethan PTSD");
            } else {
                command = new InstantCommand(() -> {setElbowVel(vel);});
            }

        }

        return command;

    }

    public Command elbowTemp(){
        return new InstantCommand(() -> {setElbowPos(ArmSubsystemConstants.tempPos); });

    }

    public Command stow(){
        return new InstantCommand(() -> {setElbowPos(ArmSubsystemConstants.stow); });
    }

    

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    // Resets the angle based on the limit switch
    if(checkForwardLimitSwitch() && checkReverseLimitSwitch()){
        // Physical Obstruction

    } else if(checkForwardLimitSwitch()){
        setForwardZero();


    } else if(checkReverseLimitSwitch()){
        setReverseZero();

    }

    SmartDashboard.putNumber("Corrected Elbow Pos: ", getCorrectedPos());


  }


    
}
