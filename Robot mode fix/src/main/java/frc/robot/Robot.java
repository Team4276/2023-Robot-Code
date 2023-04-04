// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auto.AutoScoringFunctions;
import frc.auto.BabyAuto;
import frc.auto.MainAutoFunctions;
import frc.auto.BabyAuto.AUTO_MOBILITY_MODE;
import frc.auto.MainAutoFunctions.AUTOS;
import frc.systems.Balance;
import frc.systems.FeederFinder;
import frc.systems.Intake;
import frc.systems.PIDDrivetrain;
import frc.systems.PIDElbow;
import frc.systems.TeleopDrivetrain;
import frc.utilities.Gyroscope;
import frc.utilities.LedStripControl;
import frc.utilities.Location4276;
import frc.utilities.LogFile;
import frc.utilities.LogJoystick;
import frc.utilities.Pathing;
import frc.utilities.RoboRioPorts;
import frc.utilities.RobotMode;
import frc.utilities.Xbox;
import frc.utilities.RobotMode.ROBOT_MODE;

public class Robot extends TimedRobot {

  public static Joystick leftJoystick;
  public static Joystick rightJoystick;
  public static XboxController xboxController;

  Notifier driveRateGroup;
  public static TeleopDrivetrain mTeleopDrivetrain;
  public static PIDDrivetrain mPIDDrivetrain;

  Notifier armRateGroup;
  public static PIDElbow mElbow;
  public static Intake mIntake;

  public static FeederFinder mFeederFinder;

  public static NetworkTable ntLimelight;

  public static LedStripControl myLedStrip;

  public static LogFile myLogFile;

  public static Timer systemTimer;

  public static double initialPitch = 0;

  public static Location4276 myLocation;

  public static double pov;

  public static double deadband = 0.05;

  private static DigitalInput Switch1;
  private static DigitalInput Switch2;
  private static DigitalInput Switch3;

  private static int autoselector = 0;

  public static void timedDrive() {
    if ((RobotMode.get() != ROBOT_MODE.AUTO_DRIVING) && (RobotMode.get() != ROBOT_MODE.AUTO_BALANCING)){ // Dont Reset Mode in Auto
      RobotMode.set(ROBOT_MODE.IDLING);

    }

    // ************************************************ \\
    // Command Inputs
    boolean goDrive = false;
    if (TeleopDrivetrain.currentMode == TeleopDrivetrain.DriveMode.ARCADE) {
      if ((Math.abs(Robot.leftJoystick.getY()) > deadband)
          || (Math.abs(Robot.leftJoystick.getZ()) > deadband)) {
        goDrive = true;
      }
    } else { // TANK drive
      if ((Math.abs(Robot.rightJoystick.getY()) > deadband)
          || (Math.abs(Robot.leftJoystick.getY()) > deadband)) {
        goDrive = true;
      }
    }

    if (goDrive) {
      RobotMode.set(ROBOT_MODE.TELEOP_DRIVING);

    } else if (Robot.xboxController.getRawButton(Xbox.X)
        || Robot.rightJoystick.getRawButton(LogJoystick.B1)) {
          RobotMode.set(ROBOT_MODE.HOLD_POSITION);

    } else if (Robot.xboxController.getRawButton(Xbox.B)
        || (Robot.leftJoystick.getRawButton(LogJoystick.B1))) {
          RobotMode.set(ROBOT_MODE.BALANCING);

    } else if (Robot.rightJoystick.getRawButton(LogJoystick.B7)) {
      FeederFinder.updatePeriodic();

    }

    // ************************************************ \\
    // Executing Commands
    if (RobotMode.get() == ROBOT_MODE.TELEOP_DRIVING){
      mTeleopDrivetrain.operatorDrive();
    }
    
    if (RobotMode.get() == ROBOT_MODE.AUTO_DRIVING){
      if (BabyAuto.get() == AUTO_MOBILITY_MODE.NOPOWER) {
        TeleopDrivetrain.assignMotorPower(0, 0);
      } else if (BabyAuto.get() == AUTO_MOBILITY_MODE.FORWARD) {
        TeleopDrivetrain.assignMotorPower(-1 * BabyAuto.MOTORPOWER, BabyAuto.MOTORPOWER);
      } else if (BabyAuto.get() == AUTO_MOBILITY_MODE.BACKWARD) {
        TeleopDrivetrain.assignMotorPower(BabyAuto.MOTORPOWER, -1 * BabyAuto.MOTORPOWER);

      }
      
    }

    if (RobotMode.get() == ROBOT_MODE.IDLING){
      TeleopDrivetrain.assignMotorPower( 0, 0);
    }

    if (RobotMode.get() == ROBOT_MODE.HOLD_POSITION){
      PIDDrivetrain.PIDDrivetrainUpdate();
        
    } else {
      PIDDrivetrain.newPositiontohold = true;

  
    }
  
    if ((RobotMode.get() != ROBOT_MODE.BALANCING) && (RobotMode.get() != ROBOT_MODE.AUTO_BALANCING)){
      Balance.pause = false;
    } else {
      Balance.balance(Gyroscope.GetCorrectPitch(Gyroscope.GetPitch()));
      if (!Balance.pause) {
        PIDDrivetrain.PIDDrivetrainUpdate();
      }
    }
    

  }

  

  public static void timedArm() {
    mIntake.updatePeriodic();
    PIDElbow.PIDElbowUpdate();
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    myLogFile = new LogFile();

    MainAutoFunctions.MainAutoFunctionsInit();
    
    RobotMode.RobotModeInit();
    BabyAuto.BabyAutoInit();

    Pathing.IntiateServer();

    CameraServer.startAutomaticCapture();

    Switch1 = new DigitalInput(5);
    Switch2 = new DigitalInput(6);
    Switch3 = new DigitalInput(7);

    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
    xboxController = new XboxController(2);

    mTeleopDrivetrain = new TeleopDrivetrain(RoboRioPorts.CAN_DRIVE_L1, RoboRioPorts.CAN_DRIVE_L2,
        RoboRioPorts.CAN_DRIVE_R1, RoboRioPorts.CAN_DRIVE_R2);

    mPIDDrivetrain = new PIDDrivetrain(RoboRioPorts.CAN_DRIVE_L1, RoboRioPorts.CAN_DRIVE_L2,
        RoboRioPorts.CAN_DRIVE_R1, RoboRioPorts.CAN_DRIVE_R2);

    PIDDrivetrain.PIDDrivetrainInit();

    // Drive train motor control is done on its own timer driven thread regardless
    // of disabled/teleop/auto mode selection
    driveRateGroup = new Notifier(Robot::timedDrive);
    driveRateGroup.startPeriodic(0.05);

    mElbow = new PIDElbow(RoboRioPorts.CAN_ELBOW);
    mIntake = new Intake(RoboRioPorts.CAN_INTAKE);
    
    armRateGroup = new Notifier(Robot::timedArm);
    armRateGroup.startPeriodic(0.05);

    mFeederFinder = new FeederFinder();

    ntLimelight = NetworkTableInstance.getDefault().getTable("limelight");

    myLocation = new Location4276();

    myLedStrip = new LedStripControl();

    Balance.balanceinit();

    myLedStrip.setMode(frc.utilities.LedStripControl.LED_MODE.LED_OFF);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    myLocation.updatePosition();
    myLocation.updateTelemetry();

    pov = xboxController.getPOV();

    SmartDashboard.putNumber("Pitch", Gyroscope.GetCorrectPitch(Gyroscope.GetPitch()));

    autoselector = 0;

    if (!Switch1.get())
      autoselector += 1;
    if (!Switch2.get())
      autoselector += 2;
    if (!Switch3.get())
      autoselector += 4;
    SmartDashboard.putString("Auto: ", MainAutoFunctions.getString());
    SmartDashboard.putString("Robot Mode: ", RobotMode.getString());
    SmartDashboard.putString("Auto Mobility Mode: ", BabyAuto.getString());

    Pathing.SetSimOrinitation();
  }

  @Override
  public void autonomousInit() {
    autoselector = 0;

    if (!Switch1.get())
      autoselector += 1;
    if (!Switch2.get())
      autoselector += 2;
    if (!Switch3.get())
      autoselector += 4;

    if (autoselector == 0)
      MainAutoFunctions.set(AUTOS.NO_BANANA);
    if (autoselector == 1)
      MainAutoFunctions.set(AUTOS.SHOOT_BACKUP);
    if (autoselector == 2)
      MainAutoFunctions.set(AUTOS.SHOOT);
    if (autoselector == 4)
      MainAutoFunctions.set(AUTOS.BALANCE);
    if (autoselector == 7)
      MainAutoFunctions.set(AUTOS.SHOOT_BALANCE);

    AutoScoringFunctions.AutoScoringFunctionsInit();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {


    if (MainAutoFunctions.get() == AUTOS.NO_BANANA) {
      // do nothing
    } else if (MainAutoFunctions.get() == AUTOS.SHOOT_BACKUP) {
      MainAutoFunctions.auto_shoot_backup();
    } else if (MainAutoFunctions.get() == AUTOS.SHOOT) {
      MainAutoFunctions.auto_shoot();
    } else if (MainAutoFunctions.get() == AUTOS.BALANCE) {
      MainAutoFunctions.auto_balance();
    } else if (MainAutoFunctions.get() == AUTOS.SHOOT_BALANCE) {
      MainAutoFunctions.auto_shoot_balance();
    }

    myLedStrip.updatePeriodic(LedStripControl.LED_MODE.LED_AUTO);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    initialPitch = Gyroscope.GetPitch();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    myLedStrip.updatePeriodic(LedStripControl.LED_MODE.LED_TELEOP_NORMAL);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    myLedStrip.updatePeriodic(LedStripControl.LED_MODE.LED_OFF);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    myLedStrip.updatePeriodic(LedStripControl.LED_MODE.LED_OFF);
  }
}
