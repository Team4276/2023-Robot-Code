// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.auto.AutoPicker;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NewElbow;
import frc.robot.subsystems.PIDElbow;
import frc.utils.BetterXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = DriveSubsystem.getInstance();

    private final NewElbow newElbow = NewElbow.getInstance();

    private final Intake m_intake = Intake.getInstance();

    //private static PIDElbow pidElbow;

    // The driver's controller
    private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    private final XboxController m_opController = new XboxController(OIConstants.kopControllerPort);
    private final BetterXboxController m_BetterXboxController = new BetterXboxController(m_opController);

    private final AutoPicker chooser = new AutoPicker(m_robotDrive, newElbow, m_intake);

    //Notifier armRateGroup;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        //pidElbow = new PIDElbow();

        //armRateGroup = new Notifier(RobotContainer::timedArm);
        //armRateGroup.startPeriodic(0.05);

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                true, false),
                        m_robotDrive));

        m_intake.setDefaultCommand(
                new RunCommand(() -> m_intake.idle(),m_intake)
        );

        newElbow.setDefaultCommand(
                new RunCommand(() -> newElbow.update(), newElbow)
        );

        
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(m_driverController, Button.kR1.value)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setX(),
                        m_robotDrive));

        new JoystickButton(m_opController, Button.kCross.value)
                .onTrue(new InstantCommand(() -> {PIDElbow.setZero();}));

        new Trigger(m_BetterXboxController::getLT)
                .whileTrue(new RunCommand(() -> m_intake.intake(), m_intake));

        new Trigger(m_BetterXboxController::getRT)
                .whileTrue(new RunCommand(() -> m_intake.outtake(), m_intake));

        new Trigger(m_BetterXboxController::isPOVUPPressed)
                .whileTrue(new InstantCommand(() -> newElbow.Stow()));
        
        new Trigger(m_BetterXboxController::isPOVRIGHTPressed)
                .whileTrue(new InstantCommand(() -> newElbow.ScoreHigh()));

        new Trigger(m_BetterXboxController::isPOVDOWNPressed)
                .whileTrue(new InstantCommand(() -> newElbow.Intake()));

        new Trigger(m_BetterXboxController::isPOVLEFTPressed)
                .whileTrue(new InstantCommand(() -> newElbow.ScoreMid()));

        new Trigger(m_BetterXboxController::leftYIsPushed).whileTrue(
                new RunCommand(() -> newElbow.manual(m_opController.getLeftY()), newElbow)
        );

        new Trigger(m_opController::getAButton).whileTrue(
                new InstantCommand(() -> newElbow.setZero())
        );

        }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return chooser.getAutoCommand();

        }

        //public static void timedArm(){
        //        pidElbow.PIDElbowUpdate(m_opController.getLeftY(), m_BetterXboxController.getPOV());
        //}
}
