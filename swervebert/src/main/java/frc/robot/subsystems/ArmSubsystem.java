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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmSubsystemConstants;
import frc.robot.Constants.OIConstants;

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

    private double holdPos = 0;

    private double currSetpoint = 0;

    private final double DEADZONE = 0.05;

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
        currSetpoint = pos + elbowZero;

        elbowPidController.setReference(currSetpoint, ControlType.kSmartMotion);

    }

    private void setElbowVel(double vel){
        elbowPidController.setReference(vel, ControlType.kSmartVelocity);
    }

    /** Janky cmd for manual arm movement do not use it as an example */
    public Command setElbowVelCmd(double vel){
        Command command;

        if(vel > OIConstants.kJoystickDeadband){
            if(checkForwardLimitSwitch()){
                command = new InstantCommand(() -> {setElbowVel(0);});

            } else {
                command = new InstantCommand(() -> {setElbowVel(vel);});
            }

        } else if(vel < -OIConstants.kJoystickDeadband) {
            if(checkReverseLimitSwitch()){
                command = new InstantCommand(() -> {setElbowVel(0);});

            } else {
                command = new InstantCommand(() -> {setElbowVel(vel);});

            }

        } else {
            command = new InstantCommand(() -> {setElbowPos(holdPos);});
            
        }

        // Dont want to rewrite stuff
        if(Math.abs(vel) < OIConstants.kJoystickDeadband){
            holdPos = getCorrectedPos();
        }

        return command;

    }

    public Command stow(){
        return new InstantCommand(() -> {setElbowPos(ArmSubsystemConstants.stow); });
    }

    public Command ScoreConeHigh(){
        return new InstantCommand(() -> {setElbowPos(ArmSubsystemConstants.scoreConeHigh); });
    }

    public Command ScoreConeMid(){
        return new InstantCommand(() -> {setElbowPos(ArmSubsystemConstants.scoreConeMid); });
    }

    public Command ScoreConeLow(){
        return new InstantCommand(() -> {setElbowPos(ArmSubsystemConstants.scoreConeLow); });
    }

    public Command ScoreCubeHigh(){
        return new InstantCommand(() -> {setElbowPos(ArmSubsystemConstants.scoreCubeHigh); });
    }

    public Command ScoreCubeMid(){
        return new InstantCommand(() -> {setElbowPos(ArmSubsystemConstants.scoreCubeMid); });
    }

    public Command ScoreCubeLow(){
        return new InstantCommand(() -> {setElbowPos(ArmSubsystemConstants.scoreCubeLow); });
    }

    public Command IntakeConeGround(){
        return new InstantCommand(() -> {setElbowPos(ArmSubsystemConstants.intakeConeGround); });
    }

    public Command IntakeConeFeeder(){
        return new InstantCommand(() -> {setElbowPos(ArmSubsystemConstants.intakeConeFeed); });
    }

    public Command IntakeConeDouble(){
        return new InstantCommand(() -> {setElbowPos(ArmSubsystemConstants.intakeConeDouble); });
    }

    public Command IntakeCubeGround(){
        return new InstantCommand(() -> {setElbowPos(ArmSubsystemConstants.intakeConeGround); });
    }

    public Command IntakeCubeFeeder(){
        return new InstantCommand(() -> {setElbowPos(ArmSubsystemConstants.intakeConeFeed); });
    }

    public Command IntakeCubeDouble(){
        return new InstantCommand(() -> {setElbowPos(ArmSubsystemConstants.intakeConeDouble); });
    }

    /** Checks whether it is close to the setpoint */
    public boolean isStable(){
        if(Math.abs(getCorrectedPos() - currSetpoint) < DEADZONE){
            return true;
        } else {
            return false;
        }
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
