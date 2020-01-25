# Getting Started with VS Code for 2020 Season
Installation guide for VS Code with WPILib 2020 can be found here:  
https://docs.wpilib.org/en/latest/docs/getting-started/getting-started-frc-control-system/wpilib-setup.html

Feel free to uninstall existing VS Code installations.

Brief Synopsis for windows:

  1. Download the FRC VS Code installer for your operating system from here: https://github.com/wpilibsuite/allwpilib/releases
  2. Extract the above installer and run it for all users
  3. Use the installer to download and install VS code (top right button)
  4. Make sure all checkboxes are checked then run the execute install (bottom center button)
  5. Install git command line on your computer https://git-scm.com/downloads

## Clone this Repo
  1. In VS Code press `ctrl + shift + p`
  2. In command dropdown, type `git clone` and hit enter.
  3. Enter repo URL https://github.com/Team2168/2020_Main_Robot.git and hit enter.
  4. Select a Folder to store the files in (Make a new folder) for all robotics like:  
  C:\Users\2168\Documents\RoboticsDevelopment
  5. A window will pop up to login, enter your github login credentials
  6. The code will download into the folder above
  7. A pop up will ask if you would like to open the project, select yes
  8. If any pop-ups show up to update WPI Lib, select yes

# Common tasks in VS Code
## Videos
Videos for the above can be found at the following playlist  
https://www.youtube.com/playlist?list=PLUTJdMwEWueIyWRVWkQE8N3XxPGucEx0Q

## Setup git credentials
1. Open git bash
2. Set global name to your name type `git config --global user.name "Kevin Harrilal"
3. Set global email to your email type `git config --global user.email "Kevin@team2168.org"

## To pull the latest code
1. In VS Code press `ctrl + shift + p`
2. Type `git fetch`

## To Commit
1. Use the source control pane
2. Add files to the commit using the + symbol
3. Add a commit message and hit the check symbol
4. Push using the menu

## To build the code
1. In VS Code press `ctrl + shift + p`
2. Type `Wpilib build`

## To deploy the code
1. In VS code press `ctrl + shift + p`
2. Type `WPILib deploy`

or

1. `Shift` + `F5`

# 2020_Main_Robot
Code for the [FIRST Infinite Recharge](https://www.youtube.com/watch?v=gmiYWTmFRVE) game. This readme provide all of the information required to get started and programming for the 2020 season. 

## Requirements for Robot
1. RoboRio must be flashed to latest 2020 image using USB (This only needs to be done once for the season): (https://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/144984-imaging-your-roborio)
2. Radio must be programed (https://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/144986-programming-your-radio)
3. Roborio IP on ethernet must be set to static using web dashboard to : 10.21.68.2
4. Any IP Camera must be set to 10.21.68.90
5. Vision Processor: If Tegra must be set to 10.21.68.71 (for "TK1"), if Beablebone must be set to 10.21.68.33 (for "BB"), if android must be set to 10.21.68.46 (for "AD")

## Requirements for Students

https://docs.wpilib.org is the official documentation from FRC on the control system: Please give it a read:  
https://docs.wpilib.org/en/latest/docs/getting-started/getting-started-frc-control-system/control-system-hardware.html

1. If you would like to have the driverstation on your computer as well then install NI Update suite, but this is not a requirement to develop or deploy programs, only to flash robot images (https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html#333285)
2. For returning students, and new students interested: understand what has changed in WPI Library since 2019 season (https://docs.wpilib.org/en/latest/docs/software/wpilib-overview/new-for-2020.html)
3. Understand how the robot is wired as it affects your code. (https://docs.wpilib.org/en/latest/docs/getting-started/getting-started-frc-control-system/how-to-wire-a-robot.html)

## Cool things to know

### Everything you need to know about the control system is here:
  - WPILib readthedocs: https://docs.wpilib.org/en/latest/
  - CTRE Pheonix API
    - Readthedocs: https://phoenix-documentation.readthedocs.io/
    - Javadocs: https://www.ctr-electronics.com/downloads/api/java/html/index.html
    - Software downloads and manuals: https://www.ctr-electronics.com/talon-srx.html#product_tabs_technical_resources
  - Spark MAX
    - Javadocs: http://www.revrobotics.com/content/sw/max/sw-docs/java/index.html
    - Software downloads & manuals: http://www.revrobotics.com/sparkmax-software/
  - More information on the control system can be found at our controls website at http://controls.team2168.org

### Radio
1. You can access radio web page by logging into http://10.21.68.1 root/admin
2. Roborio should always be plugged into the port on the radio labeled "18-24 vPOE" only! Or connect to this port via a switch

### Roborio
1. You can access roborio diagnostics webpage by http://roboRIO-2168-FRC.local (using IE web browser) or http://10.21.68.2
2. You can program roborio over ethernet, usb, or wifi (if USB, NI Update suite needs to be installed to get usb drivers)
3. More information on the control system can be found at our controls website at http://controls.team2168.org
4. Files will be logged to /home/lvuser/Logs
5. You can ftp files to/from the roborio using filezilla, winscp, web browser, or your local file explorer at ftp://10.21.68.2:21
6. You can ssh into roborio using putty or console application at ssh 10.21.68.2:22 username:lvuser password: blank

### Dashboard (on driver station)
1. Java dashboard will open if Java is selected from the driverstation menu
2. Python dash (if it installed) will open if "default" dashboard is selected from drivestation menu
3. If smartdashboard doesn't update, but you have robot comms, in smart dash preferences toggle "use mDNS" until it does

# Repository Guidelines
## Branches
Our repository and workflow loosely follows the gitflow workflow. This workflow is simple and is one of the most popular workflows when using git with a large number of developers. More info: https://www.atlassian.com/git/tutorials/comparing-workflows#gitflow-workflow
- The master branch contains code that is known-working, has been tested, and can be deployed to a competition ready robot.
- The develop branch is our sandbox for integrating and testing new features and fixing problems. This isnt the latests and greatest code, but it may have problems and needs to be checked out on the robot before being pushed into master. 
- Everything else is lumped under feature/bugfix branches. When we need to add new capabilities, start by branching the latest code  in the develop branch.  

## Checklist for committing/pushing code
- Commit often and create detailed log messages that describe what and why you're making a change. Be specific.
- Review the changes you make before pushsing them. You should look through all the files being added/modified/removed before you commit.
- Always verify your code compiles before pushing to the repo. There shouldn't be any red-underlined text in your commits. Use the build button (Green triangle) at the top of eclipse to verify a build completes without error.
- Push your changes into a branch with a name that identifies what feature it is adding or problem it is addressing in the code.
- Never push to the master branch 
- After pushing your changes to the repo, verify you can see your changes in GitHub from a web browser.

# Robot Design (To be continued)
- All students are assigned one or many subsystems on the robot. Your task is the following:
1. Pull master and create a new branch (naming it after the subsystem your working on, followed by your initials is a good idea)
2. Write the subsystem with all the hardware called out below.
3. RobotMap ports may be pre-determined and assigned to your subsystem, but you need to add the code to RobotMap java
4. Write all the commands for your subsystem, and place them in the approporate command subfolders/packages for your subsystem
5. Add the subsystem to robot.java
6. Push your code to your branch for review by Kevin or James (we will write issues for you to fix)
7. Once tested on a robot, it will be merged into Master.

# Assignments
Robot code for the 2020 FRC season

NOTE:  
When using NEO motors:
 - Use the CANSparkMax class. http://www.revrobotics.com/content/sw/max/sw-docs/java/com/revrobotics/CANSparkMax.html
 - When initializing, for the type put `kBrushless`.
 - In the construtor of your subsystem,
   - *instance*.setSmartCurrentLimit(60)
   - *instance*.setControlFramePeriodMs(20)
   - *instance*.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500)
   - *instance*.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500)
   - *instance*.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500)

When using Falcon motors:
 - Use the TalonFX class. http://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_talon_f_x.html
 - In the contructor of your subsystem,
   - The constructor of the motor is the same as PWMSpeedController.
   - Create a new SupplyCurrentLimitConfiguration. Use that to configure the current.
   - An example can be found in the Drivetrain subsystem, on the `Drivetrain_WZ` branch.

### Style and Architecture Guidelines
When creating member variables:
  - Set the accessibility to `private` and use a `_` before the name.
    ```java
    private <MotorController> _motor
    private DoubleSolenoid _piston
    private <Subsystem> _instance
    ```
  - Constants for subsystems should be added to their respective subsystems.
    ```java
    private final boolean IS_MOTOR_REVERSED
    private final double LIFT_HOLDING_VOLTAGE
    public final PIDPosition PID_POSITION_1
    ```
  - Constants for subsystems that can conflict with each other such as CAN IDs should be added to `RobotMap.java`.
    ```java
    public static final int <Subsystem1>_MOTOR_PDP = 0
    public static final int <Subsystem2>_MOTOR_PDP = 1
    ```

When creating a subsystem:
  - Every `Subsystem` should use singleton design pattern (private constructor, public getInstance() method).

When creating methods in the subsystem:
  - Motors:
    - Create a method to set the speed of the motor.
    - The method name should be `drive`.
    - Add a comment for the polarity of the motor (What direction does postive go to)
      - Positive for a shooter wheel should be out.
      - Positive for a lift should be up.
    - EX:
      ```java
      /**
       * Sets the speed of the XYZ motor.
       *
       * @param speed : a value of 1.0 to -1.0 (positive is into the robot, negative is out of the robot)
       */
      public void drive(double speed) {
        Whatever this method does...
      }
      ```
  - Pneumatics
    - The class commonly used is DoubleSolenoid.
    - Create a method to extend the pneumatic.
      - The method name should be `extend`.
      - Unless there are multiple pneumatics, then extend the specific pneumatic.
    - Create a method to retract the pneumatic.
      - The method name should be `retract`.
      - Unless there are multiple pneumatics, then extend the specific pneumatic.
    - EX:
      ```java
      /**
       * Extends the XYZ pneumatic.
       *
       */
      public void extend() {
        Whatever this method does...
      }

      /**
       * Retract the XYZ pneumatic.
       *
       */
      public void retract() {
        Whatever this method does...
      }
      ```
When adding an instance of a subsystem to the `Robot` class:
  - The variable should be of `private` access type.
  - Any access to the subsystem elsewhere in the code should use the static getInstance method for the respective subsystem. (e.g. `<SubsystemName>.getInstance()`).

When creating commands:
  - Motors:
     - Create a command to drive the motor with a constant.
     - Create a command to drive the motor with a joystick.
  - Pneumatics:
     - Create a command to extend the pneumatic.
     - Create a command to retract the pneumatic.

## Drivetrain
### Liam
  6x Motors (Falcon 500s - TalonFX) (3 per side)
  2x Rev Encoder
  1x Pigeon IMU

## Shooter
### Kaleb
  2x Motors (Falcon 500s - TalonFX)
  1x Rev Encoder

## Hood_Adjust
### Kaleb
  2x Pneumatics
   - One pushes the hood
   - One activates a hard stop
   - This creates 4 positions to shoot from.

## Intake
```
This needs to be two seperate subsystems. e.g.
  - Raise/lower
  - motors in/out
```
### Ian
  1x Motor (TalonSRX) to spin wheels  
  1x Pneumatic to pivot intake

## Hopper
### Cierra
  1x Motor (TalonSRX) to spin

## Indexer
### Nathan K.
  1x Motor (Spark MAX)
  2x Line Break (IR) Sensors

## Climber
### Conor
  2x Motors (775, use TalonSRX) to control pulley system
  TBD sensors

## Color Wheel raise/lower
```
This needs to be split up into multiple subsystems. E.g.
  - Raise/lower
  - rotate
  - color sense
  - camera
```
### Derek
  1x Motor (Neo) to spin wheel
  1x Pneumatic to adjust vertical position (rasise/lower) 
  1x color sensor to read the color for position control (Sensor comes from vendor)    
  1x Encoder to read the number of rotations for rotation control
  1x Camera
  1x Hall Effect 
  
## Balancer
### Caeden
  1x Motor (Neo) to move along the bar  
  1x Pneumatic to brake

## Buddy_Climb
### TBD

## LEDs
### Greyson
  
## Vision
  TBD
