/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168;

import java.io.IOException;
import java.nio.file.Path;

import org.team2168.commands.DriveWithConstant;
import org.team2168.commands.DriveWithJoystick;
import org.team2168.subsystems.Drivetrain;
import org.team2168.utils.F310;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * TODO pick paths on smart dashboard
 * TODO fix the dozens of errors about stale sensors
 * TODO validate why that weird path didn't work (below)
 *     An example trajectory to follow.  All units in meters.
    TODO replace with a path weaver thing
    exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(0.5, 0.5)
        ),
        // End 8 meters straight ahead of where we started, facing forward
        new Pose2d(1, 1, new Rotation2d(270)),
        // Pass config
        getConfig()
    );
 * TODO add the other subsystems
 * TODO use a fancy dashboard
 */

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static RobotContainer instance = null;

  public static final Boolean USING_XBOX = false; // I don't have an f310 lol

  // The robot's subsystems and commands are defined here:
  private final Drivetrain dt;
  private static F310 driverJoystick;

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private static Trajectory exampleTrajectory;

  // Create a voltage constraint to ensure we don't accelerate too fast
  private static DifferentialDriveVoltageConstraint autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(Constants.kDriveS,
                                      Constants.kDriveV,
                                      Constants.kDriveA),
          Constants.kDriveKinematics,
          10);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    dt = Drivetrain.getInstance();
    driverJoystick = new F310(RobotMap.DRIVER_JOYSTICK);

    // Configure the button bindings
    configureButtonBindings();

    initialize_trajectories();
    // dt.setDefaultCommand(new DriveWithJoystick());
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverJoystick.ButtonA().whenHeld(new DriveWithConstant(1.0, 1.0));
    driverJoystick.ButtonA().whenReleased(new DriveWithConstant(0.0, 0.0)); //dunno if I need this but just to be safe

  }

  public double getLeftStick() {
    return driverJoystick.getLeftStickRaw_Y();
  }

  public double getRightStick() {
    return (USING_XBOX ? -driverJoystick.getRawAxis(4) : driverJoystick.getRightStickRaw_Y());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Trajectory trajectory) {
    // An ExampleCommand will run in autonomous

    // Paste this variable in
    RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
              double angularVelocityRefRadiansPerSecond) {
          return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
      }
    };

    PIDController leftController = new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);
    PIDController rightController = new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);
    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        dt::getPose,
        // new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        disabledRamsete,
        new SimpleMotorFeedforward(Constants.kDriveS,
                                   Constants.kDriveV,
                                   Constants.kDriveA),
        Constants.kDriveKinematics,
        dt::getWheelSpeeds,
        leftController,
        rightController,
        // new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD),
        // new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD),
        // RamseteCommand passes volts to the callback
        // dt::setVolts,
        (leftVolts, rightVolts) -> {
          dt.setVolts(leftVolts, rightVolts);
  
          SmartDashboard.putNumber("left_value", dt.getWheelSpeeds().leftMetersPerSecond);
          SmartDashboard.putNumber("left reference", leftController.getSetpoint());
  
          SmartDashboard.putNumber("right value", dt.getWheelSpeeds().rightMetersPerSecond);
          SmartDashboard.putNumber("right reference", rightController.getSetpoint());
      },
        dt
    );

    // Reset odometry to the starting pose of the trajectory.
    dt.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> dt.setVolts(0, 0));
  }

  private static TrajectoryConfig getConfig() {
    return new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                                Constants.kMaxAccelerationMetersPerSecondSquared)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(Constants.kDriveKinematics)
                                // Apply the voltage constraint
                                .addConstraint(autoVoltageConstraint);
  }

  private void initialize_trajectories() {
    String trajectoryJson = "paths/Straightline.wpilib.json";
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson);
        exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        System.out.println("Unable to open trajectory: " + trajectoryJson);
        System.out.println(ex.getStackTrace());
      }
  }

  public static RobotContainer getInstance() {
    if (instance == null)
      instance = new RobotContainer();
    return instance;
  }
}
