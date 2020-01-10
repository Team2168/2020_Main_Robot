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

import org.team2168.Robot;
import org.team2168.RobotMap;
import org.team2168.PID.controllers.PIDPosition;
import org.team2168.PID.controllers.PIDSpeed;
import org.team2168.PID.sensors.ADXRS453Gyro;
import org.team2168.PID.sensors.AverageEncoder;
import org.team2168.PID.sensors.CanAnalogInput;
import org.team2168.PID.sensors.IMU;
import org.team2168.PID.sensors.Limelight;
import org.team2168.utils.TCPSocketSender;
import org.team2168.utils.consoleprinter.ConsolePrinter;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drivetrain extends Subsystem 
{  
  private static CANSparkMax _leftMotor1;
  private static CANSparkMax _leftMotor2;
  private static CANSparkMax _leftMotor3;
  private static CANSparkMax _rightMotor1;
  private static CANSparkMax _rightMotor2;
  private static CANSparkMax _rightMotor3;

  @Override
  public void initDefaultCommand() 
  {
    System.out.println("CAN Comp Bot Drivetrain Enabled - 6 Motors");
    _leftMotor1 = new CANSparkMax(RobotMap.DRIVETRAIN_LEFT_MOTOR_1_PDP, MotorType.kBrushless);
    _leftMotor2 = new CANSparkMax(RobotMap.DRIVETRAIN_LEFT_MOTOR_2_PDP, MotorType.kBrushless);
    _leftMotor3 = new CANSparkMax(RobotMap.DRIVETRAIN_LEFT_MOTOR_3_PDP, MotorType.kBrushless);
    _rightMotor1 = new CANSparkMax(RobotMap.DRIVETRAIN_RIGHT_MOTOR_1_PDP, MotorType.kBrushless);
    _rightMotor2 = new CANSparkMax(RobotMap.DRIVETRAIN_RIGHT_MOTOR_2_PDP, MotorType.kBrushless);
    _rightMotor3 = new CANSparkMax(RobotMap.DRIVETRAIN_RIGHT_MOTOR_3_PDP, MotorType.kBrushless);

    //speed limit is set to 60 ma homies
    _leftMotor1.setSmartCurrentLimit(60);
    _leftMotor2.setSmartCurrentLimit(60);
    _leftMotor3.setSmartCurrentLimit(60);
    _rightMotor1.setSmartCurrentLimit(60);
    _rightMotor2.setSmartCurrentLimit(60);
    _rightMotor3.setSmartCurrentLimit(60); 

     //control the frame every 20 ms
     _leftMotor1.setControlFramePeriodMs(20);
     _leftMotor2.setControlFramePeriodMs(20);
     _leftMotor3.setControlFramePeriodMs(20);
     _rightMotor1.setControlFramePeriodMs(20);
     _rightMotor2.setControlFramePeriodMs(20);
     _rightMotor3.setControlFramePeriodMs(20);
     //status of the frame every 500 ms
     _leftMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);
     _leftMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);
     _leftMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);
     _rightMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);
     _rightMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);
     _rightMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus0,500);

     //status of the frame every 500 ms
     _leftMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);
     _leftMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);
     _leftMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);
     _rightMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);
     _rightMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);
     _rightMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus1,500);

     //status of the frame every 500 ms
     _leftMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);
     _leftMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);
     _leftMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);
     _rightMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);
     _rightMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);
     _rightMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus2,500);
  }
}
