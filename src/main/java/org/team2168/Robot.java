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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Robot extends TimedRobot {
	TalonFX _leftMaster = new TalonFX(0);
	TalonFX _left2 = new TalonFX(1);
	TalonFX _left3 = new TalonFX(2);
	TalonFX _rightMaster = new TalonFX(15);
	TalonFX _right2 = new TalonFX(14);
	TalonFX _right3 = new TalonFX(13);
	PigeonIMU _pidgey = new PigeonIMU(17);
	Joystick _gamepad = new Joystick(5);

	/** Invert Directions for Left and Right */
	TalonFXInvertType _leftInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
	TalonFXInvertType _rightInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"

	/** Config Objects for motor controllers */
	TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
	TalonFXConfiguration _rightConfig = new TalonFXConfiguration();
	
	/** Latched values to detect on-press events for buttons */
	boolean[] _previous_currentBtns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] _currentBtns = new boolean[Constants.kNumButtonsPlusOne];
	
	/** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	double _targetAngle = 0;

	/** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing;

	private static final double TICKS_PER_REV = 2048.0; //one event per edge on each quadrature channel
	private static final double TICKS_PER_100MS = TICKS_PER_REV / 10.0;
	private static final double GEAR_RATIO = (50.0/10.0) * (40.0/22.0);
	private static final double WHEEL_DIAMETER = 6.0; //inches
	private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; //inches
	private static final double PIGEON_UNITS_PER_ROTATION = 8192.0;;
	private static final double DEGREES_PER_REV = 360.0;
	private static final double PIGEON_UNITS_PER_DEGREE = PIGEON_UNITS_PER_ROTATION/360;
	private static final double WHEEL_BASE = 24.0; //distance between wheels (width) in inches


	private SupplyCurrentLimitConfiguration talonCurrentLimit;
	private final boolean ENABLE_CURRENT_LIMIT = true;
	private final double CONTINUOUS_CURRENT_LIMIT = 40; //amps
	private final double TRIGGER_THRESHOLD_LIMIT = 60; //amp
	private final double TRIGGER_THRESHOLD_TIME = 200; //ms

	@Override
	public void robotInit() {

		_rightMaster.configFactoryDefault();
		_right2.configFactoryDefault();
		_right3.configFactoryDefault();
		_leftMaster.configFactoryDefault();
		_left2.configFactoryDefault();
		_left3.configFactoryDefault();

		talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
		CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);
	
		_rightMaster.configSupplyCurrentLimit(talonCurrentLimit);
		_right2.configSupplyCurrentLimit(talonCurrentLimit);
		_right3.configSupplyCurrentLimit(talonCurrentLimit);
		_leftMaster.configSupplyCurrentLimit(talonCurrentLimit);
		_left2.configSupplyCurrentLimit(talonCurrentLimit);
		_left3.configSupplyCurrentLimit(talonCurrentLimit);

		/* Set Neutral Mode */
		_leftMaster.setNeutralMode(NeutralMode.Brake);
		_left2.setNeutralMode(NeutralMode.Coast);
		_left3.setNeutralMode(NeutralMode.Coast);
		_rightMaster.setNeutralMode(NeutralMode.Brake);
		_right2.setNeutralMode(NeutralMode.Coast);
		_right3.setNeutralMode(NeutralMode.Coast);

		/* Configure output and sensor direction */
		_leftMaster.setInverted(_leftInvert);
		_left2.setInverted(_leftInvert);
		_left3.setInverted(_leftInvert);
		_rightMaster.setInverted(_rightInvert);
		_right2.setInverted(_rightInvert);
		_right3.setInverted(_rightInvert);

		/* Reset Pigeon Configs */
		_pidgey.configFactoryDefault();


		/** Feedback Sensor Configuration */

		/** Distance Configs */

		/* Configure the left Talon's selected sensor as integrated sensor */
		_leftConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; //Local Feedback Source

		/* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
		_rightConfig.remoteFilter0.remoteSensorDeviceID = _leftMaster.getDeviceID(); //Device ID of Remote Source
		_rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type
		
		/* Now that the Left sensor can be used by the master Talon,
		 * set up the Left (Aux) and Right (Master) distance into a single
		 * Robot distance as the Master's Selected Sensor 0. */
		setRobotDistanceConfigs(_rightInvert, _rightConfig);

		/* FPID for Distance */
		_rightConfig.slot0.kF = Constants.kGains_Distance.kF;
		_rightConfig.slot0.kP = Constants.kGains_Distance.kP;
		_rightConfig.slot0.kI = Constants.kGains_Distance.kI;
		_rightConfig.slot0.kD = Constants.kGains_Distance.kD;
		_rightConfig.slot0.integralZone = Constants.kGains_Distance.kIzone;
		_rightConfig.slot0.closedLoopPeakOutput = Constants.kGains_Distance.kPeakOutput;



		/** Heading Configs */
		_rightConfig.remoteFilter1.remoteSensorDeviceID = _pidgey.getDeviceID();    //Pigeon Device ID
		_rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw; //This is for a Pigeon over CAN
		_rightConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor1; //Set as the Aux Sensor
		_rightConfig.auxiliaryPID.selectedFeedbackCoefficient = 3600.0 / Constants.kPigeonUnitsPerRotation; //Convert Yaw to tenths of a degree

		/* false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 *   This is typical when the master is the right Talon FX and using Pigeon
		 * 
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 *   This is typical when the master is the left Talon FX and using Pigeon
		 */
		_rightConfig.auxPIDPolarity = false;

		/* FPID for Heading */
		_rightConfig.slot1.kF = Constants.kGains_Turning.kF;
		_rightConfig.slot1.kP = Constants.kGains_Turning.kP;
		_rightConfig.slot1.kI = Constants.kGains_Turning.kI;
		_rightConfig.slot1.kD = Constants.kGains_Turning.kD;
		_rightConfig.slot1.integralZone = Constants.kGains_Turning.kIzone;
		_rightConfig.slot1.closedLoopPeakOutput = Constants.kGains_Turning.kPeakOutput;


		/* Config the neutral deadband. */
		_leftConfig.neutralDeadband = Constants.kNeutralDeadband;
		_rightConfig.neutralDeadband = Constants.kNeutralDeadband;


		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		_rightMaster.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
		_rightMaster.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

		/* Motion Magic Configs */
		_rightConfig.motionAcceleration = 7500; //(distance units per 100 ms) per second
		_rightConfig.motionCruiseVelocity = 10000; //distance units per 100 ms



		/* APPLY the config settings */
		_leftMaster.configAllSettings(_leftConfig);
		_rightMaster.configAllSettings(_rightConfig);


		/* Set status frame periods to ensure we don't have stale data */
		/* These aren't configs (they're not persistant) so we can set these after the configs.  */
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, Constants.kTimeoutMs);
		_leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);
		_pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 5, Constants.kTimeoutMs);
	}

	@Override
	public void teleopInit(){
		/* Disable all motor controllers */
		_rightMaster.set(ControlMode.PercentOutput, 0);
		_leftMaster.set(ControlMode.PercentOutput, 0);
		
		/* Initialize */
		_firstCall = true;
		_state = false;
		zeroSensors();
	}
	
	@Override
	public void teleopPeriodic() {
		/* Gamepad processing */
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getTwist();
		forward = Deadband(forward);
		turn = Deadband(turn);

	
		/* Button processing for state toggle and sensor zeroing */
		getButtons(_currentBtns, _gamepad);
		if(_currentBtns[2] && !_previous_currentBtns[2]){
			_state = !_state; 		// Toggle state
			_firstCall = true;		// State change, do first call operation
			_targetAngle = _rightMaster.getSelectedSensorPosition(1);
		}else if (_currentBtns[1] && !_previous_currentBtns[1]) {
			zeroSensors();			// Zero Sensors
		}
		if(_currentBtns[5] && !_previous_currentBtns[5]) {
			_smoothing--; // Decrement smoothing
			if(_smoothing < 0) _smoothing = 0; // Cap smoothing
			_rightMaster.configMotionSCurveStrength(_smoothing);

			System.out.println("Smoothing value is: " + _smoothing);
		}
		if(_currentBtns[6] && !_previous_currentBtns[6]) {
			_smoothing++; // Increment smoothing
			if(_smoothing > 8) _smoothing = 8; // Cap smoothing
			_rightMaster.configMotionSCurveStrength(_smoothing);
			
			System.out.println("Smoothing value is: " + _smoothing);
		}

		if(_currentBtns[7] && !_previous_currentBtns[7]) {	//Back (select) button
			_targetAngle = degrees_to_ticks(-90.0);

			System.out.println("setting heading to -90.0");
		}
		if(_currentBtns[8] && !_previous_currentBtns[8]) {	//Start button
			_targetAngle = degrees_to_ticks(90.0);

			System.out.println("setting heading to +90.0");
		}
		if(_currentBtns[4] && !_previous_currentBtns[4]) {	//Y button
			_targetAngle = 0.0;
			System.out.println("setting heading to 0.0");
		}

		System.arraycopy(_currentBtns, 0, _previous_currentBtns, 0, Constants.kNumButtonsPlusOne);
		
		if(!_state){
			if (_firstCall)
				System.out.println("This is Arcade Drive.\n");
			
			_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
			_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		}else{
			if (_firstCall) {
				System.out.println("This is Motion Magic with the Auxiliary PID using the Pigeon yaw.");
				System.out.println("Servo [-6,6] rotations while also maintaining a straight heading.\n");
				zeroDistance();
				
				/* Determine which slot affects which PID */
				_rightMaster.selectProfileSlot(Constants.kSlot_Distanc, Constants.PID_PRIMARY);
				_rightMaster.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
			}
			
			/* Calculate targets from gamepad inputs */
			double target_sensorUnits = forward * inches_to_ticks(12.0 * 10.0);
			double target_turn = _targetAngle;
			
			/* Configured for MotionMagic on Quad Encoders' Sum and Auxiliary PID on Pigeon */
			_rightMaster.set(ControlMode.MotionMagic, target_sensorUnits, DemandType.AuxPID, target_turn);
			_leftMaster.follow(_rightMaster, FollowerType.AuxOutput1);
		}
		_firstCall = false;
	}
	
	/** Zero all sensors, both Talons and Pigeon */
	void zeroSensors() {
		_leftMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		_pidgey.setYaw(0, Constants.kTimeoutMs);
		_pidgey.setAccumZAngle(0, Constants.kTimeoutMs);
		System.out.println("[Quadrature Encoders + Pigeon] All sensors are zeroed.\n");
	}
	
	/** Zero QuadEncoders, used to reset position when initializing Motion Magic */
	void zeroDistance(){
		_leftMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		System.out.println("[Quadrature Encoders] All encoders are zeroed.\n");
	}
	
	/** Deadband 5 percent, used on the gamepad */
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */
		return 0;
	}
	
	/** Gets all buttons from gamepad */
	void getButtons(boolean[] _currentBtns, Joystick gamepad) {
		for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
			_currentBtns[i] = gamepad.getRawButton(i);
		}
	}

	/** 
	 * Determines if SensorSum or SensorDiff should be used 
	 * for combining left/right sensors into Robot Distance.  
	 * 
	 * Assumes Aux Position is set as Remote Sensor 0.  
	 * 
	 * configAllSettings must still be called on the master config
	 * after this function modifies the config values. 
	 * 
	 * @param masterInvertType Invert of the Master Talon
	 * @param masterConfig Configuration object to fill
	 */
	 void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
		/**
		 * Determine if we need a Sum or Difference.
		 * 
		 * The auxiliary Talon FX will always be positive
		 * in the forward direction because it's a selected sensor
		 * over the CAN bus.
		 * 
		 * The master's native integrated sensor may not always be positive when forward because
		 * sensor phase is only applied to *Selected Sensors*, not native
		 * sensor sources.  And we need the native to be combined with the 
		 * aux (other side's) distance into a single robot distance.
		 */

		/* THIS FUNCTION should not need to be modified. 
		   This setup will work regardless of whether the master
		   is on the Right or Left side since it only deals with
		   distance magnitude.  */

		/* Check if we're inverted */
		if (masterInvertType == TalonFXInvertType.Clockwise){
			/* 
				If master is inverted, that means the integrated sensor
				will be negative in the forward direction.
				If master is inverted, the final sum/diff result will also be inverted.
				This is how Talon FX corrects the sensor phase when inverting 
				the motor direction.  This inversion applies to the *Selected Sensor*,
				not the native value.
				Will a sensor sum or difference give us a positive total magnitude?
				Remember the Master is one side of your drivetrain distance and 
				Auxiliary is the other side's distance.
					Phase | Term 0   |   Term 1  | Result
				Sum:  -1 *((-)Master + (+)Aux   )| NOT OK, will cancel each other out
				Diff: -1 *((-)Master - (+)Aux   )| OK - This is what we want, magnitude will be correct and positive.
				Diff: -1 *((+)Aux    - (-)Master)| NOT OK, magnitude will be correct but negative
			*/

			masterConfig.diff0Term = FeedbackDevice.IntegratedSensor; //Local Integrated Sensor
			masterConfig.diff1Term = FeedbackDevice.RemoteSensor0;   //Aux Selected Sensor
			masterConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.SensorDifference; //Diff0 - Diff1
		} else {
			/* Master is not inverted, both sides are positive so we can sum them. */
			masterConfig.sum0Term = FeedbackDevice.RemoteSensor0;    //Aux Selected Sensor
			masterConfig.sum1Term = FeedbackDevice.IntegratedSensor; //Local IntegratedSensor
			masterConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.SensorSum; //Sum0 + Sum1
		}

		/* Since the Distance is the sum of the two sides, divide by 2 so the total isn't double
		   the real-world value */
		masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
	 }


 	/**
	 * Converts a setpoint in degrees to IMU 'encoder ticks'
	 * @param setpoint
	 * @return
	 */
	private double degrees_to_ticks(double setpoint) {
		return (setpoint / DEGREES_PER_REV) * PIGEON_UNITS_PER_ROTATION / 2.0;
	}

	private double ticks_to_degrees(double setpoint) {
		return (setpoint / PIGEON_UNITS_PER_ROTATION) * DEGREES_PER_REV;
	}

	private double degrees_per_sec_to_ticks_per_100ms(double setpoint) {
		return (degrees_to_ticks(setpoint) / 10.0);
	}

	private double ticks_per_100ms_to_degrees_per_sec(double setpoint) {
		return (ticks_to_degrees(setpoint) * 10.0);
	}

	private double inches_to_ticks(double setpoint) {
		return (setpoint * TICKS_PER_REV * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;
	}

	private double ticks_to_inches(double setpoint) {
		return (setpoint * WHEEL_CIRCUMFERENCE) / (TICKS_PER_REV * GEAR_RATIO);
	}

	private double inches_per_sec_to_ticks_per_100ms(double setpoint) {
		return inches_to_ticks(setpoint) / 10.0;
	}

	private double ticks_per_100ms_to_inches_per_sec(double setpoint) {
		return ticks_to_inches(setpoint) * 10.0;
	}

	private double revs_to_ticks(double revs) {
		return revs * (TICKS_PER_REV * GEAR_RATIO);
	}

	private double degrees_to_wheel_revs (double degrees) {
		return (degrees / 360.0) * (WHEEL_BASE / WHEEL_DIAMETER);
	}
}
