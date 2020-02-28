/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.new_subsystems;

import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private DoubleSolenoid intakePivot;

  private static IntakePivot _instance = null;

  /**
   * Default constructors for Intake
   */
  private IntakePivot() {
    intakePivot = new DoubleSolenoid(RobotMap.INTAKE_ENGAGE_PCM, RobotMap.INTAKE_DISENGAGE_PCM);
    SmartDashboard.putBoolean("Intake Extended", isIntakeExtended());
    SmartDashboard.putBoolean("Intake Retracted", isIntakeRetracted());
  }

  /**
   * @return an instance of the Intake Subsystem
   */
  public static IntakePivot getInstance() {
    if (_instance == null)
    {
      _instance = new IntakePivot();
    }
    return _instance;
  }

    /**
   * extends intake pneumatic
   */
  public void extendIntake() {
    intakePivot.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * retracts intake pneumatic
  */
  public void retractIntake() {
    intakePivot.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * checks if pneumatic is extended
   * @return a boolean, whether or not the pneumatic is extended
   */
  public boolean isIntakeExtended() {
    return intakePivot.get() == DoubleSolenoid.Value.kForward;
  }
  
  /**
   * checks if pneumatic is retracted
   * @return a boolean, whether or not the pneumatic is retracted
  */
  public boolean isIntakeRetracted() {
    return intakePivot.get() == DoubleSolenoid.Value.kReverse;
  }

  // @Override
  // public void initDefaultCommand() {
    // Set the default command for a subsystem here.
  }