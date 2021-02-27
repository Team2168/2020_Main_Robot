/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168;

import java.io.IOException;
import java.nio.file.Path;

import org.team2168.commands.DriveWithConstant;
import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static SendableChooser<String> m_chooser;
  private Drivetrain dt;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    dt = Drivetrain.getInstance();
    m_robotContainer = RobotContainer.getInstance();
    m_chooser = new SendableChooser<String>();
    m_chooser.setDefaultOption("Drive straight", "paths/Straightline.wpilib.json");
    m_chooser.addOption("Curvy", "paths/SomeCurve.wpilib.json");
    m_chooser.addOption("Short Stright", "paths/ShortStraight.wpilib.json");
    // m_chooser.addOption("Option 3", "null");
    // m_chooser.addOption("Do nothing", "null");
    SmartDashboard.putData("Auto Chooser", m_chooser);
    dt.setAllMotorsBrake();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("gyro heading", dt.getHeading());
    SmartDashboard.putNumber("average encoder distance", dt.getAverageEncoderDistance());
    SmartDashboard.putNumberArray("dtvoltages", dt.getVoltages());
    SmartDashboard.putNumber("left encoder", dt.getRightEncoderDistance());
    SmartDashboard.putNumber("right encoder", dt.getLeftEncoderDistance());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putData("Auto Chooser", m_chooser);
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    dt.setAllMotorsBrake();  // I don't trust the autos not to coast into something 
    String trajectoryJson = m_chooser.getSelected();
    System.out.println("Driving path: " + m_chooser.getSelected());
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson);
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      m_autonomousCommand = m_robotContainer.getAutonomousCommand(trajectory);
    } catch (IOException ex) {
      System.out.println("Unable to open trajectory: " + trajectoryJson);
      System.out.println(ex.getStackTrace().toString());
    }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    dt.setAllMotorsCoast();  // set it to test when I need to push around the robot
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().schedule(new DriveWithConstant(0.0, 0.0));
  }
}
