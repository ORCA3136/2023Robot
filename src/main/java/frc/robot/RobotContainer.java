// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
 // private final Joystick joystick = new Joystick(2);
  private final XboxController controller = new XboxController(1);
  private final Drivetrain m_drivetrain = new Drivetrain(); 
  //private final Intake m_innerIntake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureButtonBindings();

  }

  public void configureButtonBindings(){

    //test forward and back
    new JoystickButton(controller, XboxController.Button.kA.value)
    .onTrue(new InstantCommand(()-> m_drivetrain.driveForward(Constants.kTestSpeed), m_drivetrain))
      .onFalse(new InstantCommand(m_drivetrain::driveStop,m_drivetrain));

    new JoystickButton(controller, XboxController.Button.kB.value)
    .onTrue(new InstantCommand(()-> m_drivetrain.driveReverse(Constants.kTestSpeed), m_drivetrain))
      .onFalse(new InstantCommand(m_drivetrain::driveStop,m_drivetrain));
    

    //solenoid
    /*new JoystickButton(controller, XboxController.Button.kX.value)
    .onTrue(new InstantCommand(()-> m_innerIntake.deployIntake(), m_innerIntake))
      .onFalse(new InstantCommand(m_innerIntake::off,m_innerIntake));

    new JoystickButton(controller, XboxController.Button.kY.value)
    .onTrue(new InstantCommand(()-> m_innerIntake.retractIntake(), m_innerIntake))
      .onFalse(new InstantCommand(m_innerIntake::off,m_innerIntake));
*/
    

    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
 
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
