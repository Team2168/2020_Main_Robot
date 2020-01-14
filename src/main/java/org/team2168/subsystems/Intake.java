/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team2168.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import org.team2168.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private CANSparkMax intakeMotor;
  private DoubleSolenoid intakePivot;

  private static Intake _instance = null;

  private Intake() {
    intakeMotor = new CANSparkMax(RobotMap.INTAKE_MOTOR_PDP, MotorType.kBrushless);
    intakePivot = new DoubleSolenoid(RobotMap.INTAKE_ENGAGE_PCM, RobotMap.INTAKE_DISENGAGE_PCM);

    //speed limit 60
    intakeMotor.setSmartCurrentLimit(60);

    //control frame every 20ms
    intakeMotor.setControlFramePeriodMs(20);

    //status frame every half sec
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
  }

  public static Intake getInstance() {
    if (_instance == null)
    {
      _instance = new Intake();
    }
    return _instance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  //can set negative speed if a wire is reversed
  private void driveMotor(double speed)
  {
    if (RobotMap.INTAKE_MOTOR_REVERSE) {
      speed = -speed;
    }
    intakeMotor.set(speed);
  }

  public void setSpeed(double speed) {
    driveMotor(speed);
  }

  public void extendIntake() {
    intakePivot.set(DoubleSolenoid.Value.kForward);
  }

  public void retractIntake() {
    intakePivot.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean isIntakeExtended() {
    return intakePivot.get() == DoubleSolenoid.Value.kForward;
  }
}