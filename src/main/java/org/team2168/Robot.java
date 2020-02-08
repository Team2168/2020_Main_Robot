/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The MotionMagic_TalonFX_AuxStraightPigeon example demonstrates the Talon auxiliary and 
 * remote features to peform complex closed loops. This example has the robot performing 
 * Motion Magic with an auxiliary closed loop on Pigeon Yaw to keep the robot straight.
 * 
 * This example uses:
 * - 2x Talon FX's (one per side).  
 *     Talon FX calculates the distance by taking the sum of both integrated sensors and dividing it by 2.
 * - Pigeon IMU wired on CAN Bus for Auxiliary Closed Loop on Yaw
 * 
 * This example has two modes of operation, which can be switched between with Button 2.
 * 1.) Arcade Drive
 * 2.) Motion Magic with Talon FX's Encoders and Drive Straight With Pigeon yaw
 * 
 * Controls:
 * Button 1: When pressed, zero sensors. Set integrated encoders' positions + Pigeon yaw to 0.
 * Button 2: When pressed, toggle between Arcade Drive and Motion Magic
 * 	When toggling into Motion Magic, the current heading is saved and used as the 
 * 	auxiliary closed loop target. Can be changed by toggling out and in again.
 * Y Button (button 4): when in motion magic mode, sets the heading to 0.0 degrees
 * Button 5(Left shoulder): When pushed, will decrement the smoothing of the motion magic down to 0
 * Button 6(Right shoulder): When pushed, will increment the smoothing of the motion magic up to 8
 * Select button (button 7): when in motion magic mode, sets the heading to -90 degrees
 * Start button (button 8): when in motion magic mode, sets the heading to +90.0 degrees
 * Left Joystick Y-Axis: 
 * 	+ Arcade Drive: Drive robot forward and reverse
 * 	+ Motion Magic: Servo robot forward and reverse [-6, 6] rotations
 * Right Joystick X-Axis: 
 *  + Arcade Drive: Turn robot in left and right direction
 *  + Motion Magic: Not used
 * 
 * Gains for Motion Magic and Auxiliary may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * - Talon FX: 20.2.3.0
 * - Pigeon IMU: 20.0
 */

package org.team2168;

import org.team2168.subsystems.Balancer;
import org.team2168.subsystems.ColorWheel;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Hopper;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeMotor;
import org.team2168.subsystems.IntakePivot;
//import org.team2168.utils.Debouncer;
import org.team2168.utils.PowerDistribution;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {	
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public static IntakeMotor intakeMotor;
  public static IntakePivot intakePivot;
  public static Indexer indexer;

  private static Drivetrain drivetrain;
  private static PowerDistribution pdp;

  static boolean autoMode;
  // private static boolean matchStarted = false;
  private static int gyroReinits;
  // private double lastAngle;
  // private Debouncer gyroDriftDetector = new Debouncer(1.0);
  private static boolean gyroCalibrating = false;

  // Subsystems
  private static Balancer balancer;
  private static Hopper hopper;
  public static ColorWheel colorWheel;

  private static OI oi;


  // private boolean lastGyroCalibrating = false;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // colorWheel = ColorWheel.getInstance();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
   // intakeMotor = IntakeMotor.getInstance();
   // intakePivot = IntakePivot.getInstance();
    //  indexer = Indexer.GetInstance();

   // hopper = Hopper.getInstance();
    
    drivetrain = Drivetrain.getInstance();
    oi = OI.getInstance();
    // pdp = new PowerDistribution(RobotMap.PDPThreadPeriod);
	// pdp.startThread();
	ConsolePrinter.init();
    ConsolePrinter.putNumber("DT Position", ()->{return drivetrain.getPosition();}, true, false);
    ConsolePrinter.putNumber("Heading", ()->{return drivetrain.getHeading();}, true, false);
    ConsolePrinter.putNumber("Position Error", ()->{return drivetrain.getErrorPosition();}, true, false);
	ConsolePrinter.putNumber("Heading Error", ()->{return drivetrain.getErrorHeading();}, true, false);
	ConsolePrinter.putNumber("Velocity", ()->{return drivetrain.getVelocity();}, true, false);
	ConsolePrinter.putNumber("Velocity Error", ()->{return drivetrain.getErrorVelocity();}, true, false);


    ConsolePrinter.startThread();
  }

  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;

    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    //getControlStyleInt();
    //controlStyle = (int) controlStyleChooser.getSelected();
    Scheduler.getInstance().run();
  }
}
