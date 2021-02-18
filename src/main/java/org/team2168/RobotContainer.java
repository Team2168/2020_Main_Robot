/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

import java.util.List;

import org.team2168.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here:
  private final Drivetrain dt;

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private Trajectory exampleTrajectory;

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
  public RobotContainer() {
    dt = Drivetrain.getInstance();

    // Configure the button bindings
    configureButtonBindings();

    initialize_trajectories();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        dt::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.kDriveS,
                                   Constants.kDriveV,
                                   Constants.kDriveA),
        Constants.kDriveKinematics,
        dt::getWheelSpeeds,
        new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD),
        new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD),
        // RamseteCommand passes volts to the callback
        dt::tankDriveVolts,
        dt
    );

    // Reset odometry to the starting pose of the trajectory.
    dt.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> dt.tankDriveVolts(0, 0));
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
    // An example trajectory to follow.  All units in meters.
    // TODO replace with a path weaver thing
    exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(2, 0.1),
            new Translation2d(4, -0.1)
        ),
        // End 8 meters straight ahead of where we started, facing forward
        new Pose2d(8, 0, new Rotation2d(0)),
        // Pass config
        getConfig()
    );
    System.out.println("Trajectory generation complete.");
  }
}
