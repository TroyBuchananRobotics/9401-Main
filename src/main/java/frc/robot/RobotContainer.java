// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Dock;
import frc.robot.commands.DriveByController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.JoystickLeftTrigger;
import frc.robot.utilities.JoystickRightTrigger;

import java.io.File;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
  private final Drivetrain m_drive = new Drivetrain();
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OperatorConstants.kOperatorControllerPort);
  private final Elevator m_elevator = new Elevator();
  private final Wrist m_wrist = new Wrist();
  private final Claw m_claw = new Claw();

  private final DriveByController m_driveCommand = new DriveByController(m_drive, m_driverController);
  private final Dock m_dock = new Dock(m_drive);

  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  private File[] m_autoPathFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/").listFiles();
  private final HashMap<String, Command> events = new HashMap<>();

  private final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(m_drive::getPose, m_drive::resetOdometry, new PIDConstants(5.0, 0, 0), new PIDConstants(5.0,0.0,0), m_drive::setModuleStates, events,true, m_drive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drive.setDefaultCommand(m_driveCommand);
    // Configure the trigger bindings
    configureBindings();
    configureAutoEvents();
    configureAutoChooser();
  }

  private void configureAutoEvents() {
    events.put("GoToHigh", new InstantCommand(()->m_elevator.setPosition(81.0))
      .alongWith(new InstantCommand(()->m_wrist.setPosition(45.0))));
  
    events.put("GoToMid", new InstantCommand(()->m_elevator.setPosition(52.0))
      .alongWith(new InstantCommand(()->m_wrist.setPosition(45.0))));

    events.put("GoToHome", new InstantCommand(()->m_elevator.setPosition(4.0))
      .alongWith(new InstantCommand(()->m_wrist.setPosition(4.0))));

    events.put("GoToLow", new InstantCommand(()->m_elevator.setPosition(15.0))
      .alongWith(new InstantCommand(()->m_wrist.setPosition(48.2))));

    events.put("ConeIntake", new InstantCommand(()->m_elevator.setPosition(4.2))
      .alongWith(new InstantCommand(()->m_wrist.setPosition(48.2)))
      .alongWith(new InstantCommand(()->m_claw.setVelocity(-4000.0))));

    events.put("StopConeIntake", new InstantCommand(()->m_elevator.setPosition(4))
      .alongWith(new InstantCommand(()->m_wrist.setPosition(4)))
      .alongWith(new InstantCommand(()->m_claw.setVelocity(-500.0))));

    events.put("CubeIntake", new InstantCommand(()->m_elevator.setPosition(4.2))
      .alongWith(new InstantCommand(()->m_wrist.setPosition(48.2)))
      .alongWith(new InstantCommand(()->m_claw.setVelocity(2500.0))));

    events.put("StopCubeIntake", new InstantCommand(()->m_elevator.setPosition(4))
      .alongWith(new InstantCommand(()->m_wrist.setPosition(4)))
      .alongWith(new InstantCommand(()->m_claw.setVelocity(100.0))));

    events.put("ScoreCone", new InstantCommand(()->m_claw.setVelocity(1500))
      .andThen(new WaitCommand(0.25))
      .andThen(()->m_claw.setVelocity(0.0)));

    events.put("ScoreCube", new InstantCommand(()->m_claw.setVelocity(-2000))
      .andThen(new WaitCommand(0.25))
      .andThen(()->m_claw.setVelocity(0.0)));

    events.put("WaitForMove", new WaitCommand(1.0));

    events.put("StopDrive", new InstantCommand(()->m_drive.stop()));

    events.put("Dock", new Dock(m_drive));
  }

  private void configureAutoChooser() {
    m_chooser.setDefaultOption("Do Nothin", new WaitCommand(20.0));

    for (File auto : m_autoPathFiles) {
      
        m_chooser.addOption(
          auto.getName(), 
          autoBuilder.fullAuto(PathPlanner.loadPathGroup(auto.getName().replace(".path", ""), 2.0, 2.0)));
    }
    SmartDashboard.putData(m_chooser);
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
    new POVButton(m_driverController, 0)
    .onTrue(new InstantCommand(() -> m_drive.resetOdometry(new Pose2d())));
  new POVButton(m_driverController, 180)
    .onTrue(new InstantCommand(() -> m_drive.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)))));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(m_operatorController, Button.kA.value)
    .onTrue(new InstantCommand(()->m_elevator.setPosition(15.0))
      .alongWith(new InstantCommand(()->m_wrist.setPosition(48.2)))
      .alongWith(new InstantCommand(()->m_drive.changeSlewRate(7.0, 11.0)))
      .alongWith(new InstantCommand(()->m_drive.setSpeedScale(1.0))));

    new JoystickButton(m_operatorController, Button.kB.value)
      .onTrue(new InstantCommand(()->m_elevator.setPosition(4.0))
        .alongWith(new InstantCommand(()->m_wrist.setPosition(4.0)))
        .alongWith(new InstantCommand(()->m_drive.changeSlewRate(7.0, 11.0)))
        .alongWith(new InstantCommand(()->m_drive.setSpeedScale(1.0))));

    new JoystickButton(m_driverController, Button.kB.value)
      .onTrue(new InstantCommand(()->m_elevator.setPosition(4.0))
        .alongWith(new InstantCommand(()->m_wrist.setPosition(4.0)))
        .alongWith(new InstantCommand(()->m_drive.changeSlewRate(7.0, 11.0)))
        .alongWith(new InstantCommand(()->m_drive.setSpeedScale(1.0))));

    new JoystickButton(m_operatorController, Button.kY.value)
      .onTrue(new InstantCommand(()->m_elevator.setPosition(52.0))
        .alongWith(new InstantCommand(()->m_wrist.setPosition(45.0)))
        .alongWith(new InstantCommand(()->m_drive.changeSlewRate(6.0, 10.0)))
        .alongWith(new InstantCommand(()->m_drive.setSpeedScale(0.75))));

    new JoystickRightTrigger(m_operatorController)
      .onTrue(new InstantCommand(()->m_elevator.setPosition(81.0))
        .alongWith(new InstantCommand(()->m_wrist.setPosition(45.0)))
        .alongWith(new InstantCommand(()->m_drive.changeSlewRate(4.0, 8.0)))
        .alongWith(new InstantCommand(()->m_drive.setSpeedScale(0.5))));

    new JoystickLeftTrigger(m_driverController).onTrue(new InstantCommand(()->m_claw.setVelocity(1500))).onFalse(new InstantCommand(()->m_claw.setVelocity(100.0)));
    new JoystickRightTrigger(m_driverController).onTrue(new InstantCommand(()->m_claw.setVelocity(-4000))).onFalse(new InstantCommand(()->m_claw.setVelocity(-500.0)));
    
    new JoystickButton(m_driverController, Button.kX.value)
          .onTrue(new InstantCommand(()->m_elevator.setPosition(81.0))
            .alongWith(new InstantCommand(()->m_wrist.setPosition(50.0)))
            .alongWith(new InstantCommand(()->m_drive.changeSlewRate(4.0, 8.0)))
            .alongWith(new InstantCommand(()->m_drive.setSpeedScale(0.5))));
    
    new JoystickButton(m_driverController, Button.kRightBumper.value)
      .onTrue(new InstantCommand(()->m_elevator.setPosition(4.2))
        .alongWith(new InstantCommand(()->m_wrist.setPosition(48.2)))
        .alongWith(new InstantCommand(()->m_claw.setVelocity(-4000.0)))
        .alongWith(new InstantCommand(()->m_drive.changeSlewRate(7.0, 11.0)))
        .alongWith(new InstantCommand(()->m_drive.setSpeedScale(1.0))))
      .onFalse(new InstantCommand(()->m_elevator.setPosition(4))
        .alongWith(new InstantCommand(()->m_wrist.setPosition(4)))
        .alongWith(new InstantCommand(()->m_claw.setVelocity(-500.0)))
        .alongWith(new InstantCommand(()->m_drive.changeSlewRate(7.0, 11.0)))
        .alongWith(new InstantCommand(()->m_drive.setSpeedScale(1.0))));
    
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
      .onTrue(new InstantCommand(()->m_elevator.setPosition(4.2))
          .alongWith(new InstantCommand(()->m_wrist.setPosition(48.2)))
          .alongWith(new InstantCommand(()->m_claw.setVelocity(2500.0)))
          .alongWith(new InstantCommand(()->m_drive.changeSlewRate(7.0, 11.0)))
          .alongWith(new InstantCommand(()->m_drive.setSpeedScale(1.0))))
      .onFalse(new InstantCommand(()->m_elevator.setPosition(4))
          .alongWith(new InstantCommand(()->m_wrist.setPosition(4)))
          .alongWith(new InstantCommand(()->m_claw.setVelocity(100.0)))
          .alongWith(new InstantCommand(()->m_drive.changeSlewRate(7.0, 11.0)))
          .alongWith(new InstantCommand(()->m_drive.setSpeedScale(1.0))));

    new JoystickButton(m_driverController, Button.kA.value).whileTrue(m_dock);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
