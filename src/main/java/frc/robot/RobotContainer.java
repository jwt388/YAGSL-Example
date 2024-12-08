// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  private SendableChooser<String> driveChooser = new SendableChooser<>();
  private SendableChooser<Command> autoChooser;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private final PowerDistribution pdp = new PowerDistribution();

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                  () -> Constants.SPEED_SCALING_3*MathUtil.applyDeadband(-driverXbox.getLeftY(),
                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                  () -> Constants.SPEED_SCALING_3*MathUtil.applyDeadband(-driverXbox.getLeftX(),
                                                                OperatorConstants.LEFT_X_DEADBAND),
                                  () -> Constants.SPEED_SCALING_3*MathUtil.applyDeadband(-driverXbox.getRightX(),
                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                  ()-> (driverXbox.getHID().getPOV() == 0),
                                  ()-> (driverXbox.getHID().getPOV() == 180),
                                  ()-> (driverXbox.getHID().getPOV() == 90),
                                  ()-> (driverXbox.getHID().getPOV() == 270));

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> Constants.SPEED_SCALING * MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> Constants.SPEED_SCALING * MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX() * -1,
      () -> driverXbox.getRightY() * -1);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAngularVelocity = drivebase.driveCommand(
      () -> Constants.SPEED_SCALING * MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> Constants.SPEED_SCALING * MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> Constants.SPEED_SCALING_3 * driverXbox.getRightX() * -1,
      true);

  Command driveRobotOrientedAngularVelocity= drivebase.driveCommand(
    () -> Constants.SPEED_SCALING * MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
    () -> Constants.SPEED_SCALING * MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
    () -> Constants.SPEED_SCALING_3 * driverXbox.getRightX() * -1,
    false);

  Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -driverXbox.getRightX());

  Command driveSetpointGenSim = drivebase.simDriveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRawAxis(2));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
        // Setup an addressable LED strip on a PWM channel
    setupLEDs();
    
    // Setup Auto commands and chooser
    NamedCommands.registerCommand(
        "SayHello",
        new InstantCommand(()->DataLogManager.log("Hello ...")));

    NamedCommands.registerCommand(
        "SayGoodbye",
        new InstantCommand(()->DataLogManager.log("... Goodbye")));

    NamedCommands.registerCommand(
        "Set LED Red",
        new InstantCommand(()->solidRGB(128,0,0)));

    NamedCommands.registerCommand(
        "Set LED Green",
        new InstantCommand(()->solidRGB(0,128,0)));

    NamedCommands.registerCommand(
        "Set LED Blue",
        new InstantCommand(()->solidRGB(0,0,128)));


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);

    // Publish PDP data for dashboard and logging
    SmartDashboard.putData(pdp);

    // Configure the trigger bindings
    configureBindings();

        // Setup chooser for selecting drive mode
        driveChooser.setDefaultOption("Drive Mode - AngularVelocity", "angular");
        driveChooser.addOption("Drive Mode - Direct Angle", "direct");
        driveChooser.addOption("Drive Mode - Advanced", "advanced");
        driveChooser.addOption("Drive Mode - Robot Oriented", "robot");
        SmartDashboard.putData(driveChooser);
    
        setDriveMode();
        solidRGB(128,128,0);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    if (DriverStation.isTest())
    {
      driverXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());

    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    switch (driveChooser.getSelected()) {

      case "direct":
        drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
        return;

      case "advanced":
        drivebase.setDefaultCommand(closedAbsoluteDriveAdv);
        return;

      case "robot":
        drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
        return;

      case "angular":
      default:
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);        


    }
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  // Setup an interface for an addressable LED strip
  private void setupLEDs(){
    m_led = new AddressableLED(0);

    // Create the buffer. Start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(30);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  // Set all LEDs in the strip to a solid color
  private void solidRGB(int red, int green, int blue) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, red, green, blue);
    }
    m_led.setData(m_ledBuffer);
  }
}
