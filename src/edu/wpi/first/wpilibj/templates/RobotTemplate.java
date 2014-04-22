/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


//Here you always declare what you add.

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.DigitalOutput;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {
    
//Here you add objects, integers, etc. after you declare them in the class above.   
    
//    Talon tMotor1Left = new Talon(1);
//    Talon tMotor2Left = new Talon(2);
//    Talon tMotor1Right = new Talon(3);
//    Talon tMotor2Right = new Talon(4);
//    Talon tConveyor = new Talon(5);
//    Talon tJellyfish = new Talon(6);

    DriverStationEnhancedIO dsio;
    
    Solenoid sGearHigh = new Solenoid(3);
    Solenoid sGearLow = new Solenoid(1);
    Solenoid sTrapDoor1 = new Solenoid(5);
    Solenoid sTrapDoor2 = new Solenoid(6);
    Solenoid sElevator = new Solenoid(4);
//    Solenoid sClaw = new Solenoid(2);
    
    Joystick joystickDriver = new Joystick(2);
    Joystick joystickClimber = new Joystick(1);
    
//    DigitalInput dioElevatorMax = new DigitalInput(1);
//    DigitalInput dioClaw = new DigitalInput(2); 
//    DigitalInput dioElevatorMin = new DigitalInput(3);
    
    Encoder leftEncoder = new Encoder(1, 2);
    Encoder rightEncoder = new Encoder(3, 4);

    Compressor compressor = new Compressor(1,1);
            //Digital out, Digital in

    Jaguar shooter = new Jaguar(5);
    
    RobotDrive m_robotDrive=new RobotDrive(1,2);
    
    DriverStation ds ;
    DriverStationLCD m_dsLCD;
    
    Timer climbTimer = new Timer();
    
    Relay relay2 = new Relay(2);
    Relay relay3 = new Relay(3);
    DigitalOutput do2 = new DigitalOutput(7);
    
    int intClimbingState;
    int intStopClimbingState;
    
    final int climbNormalMode = 0;
    final int climbExtendPastRung1=1;
    final int climbLiftToRung1 = 2;
    final int climbFistOfGoodnessRung1 = 3;
    final int climbExtendPastRung2=4;
    final int climbLiftToRung2 = 5;
    final int climbRemainingLiftToRung2 = 6;
    final int climbFistOfGoodnessRung2 = 7;
    final int climbExtendPastRung3 = 8;
    final int climbLiftToRung3 = 9;
    final int climbRemainingLiftToRung3 = 10;
    final int climbFistOfGoodnessRung3 = 11;
    final int climbDumpJellyfishNet = 12;
    
    int jsButtonClimbProceed = 1;
    int jsButtonClimbStop = 2;
    int jsButtonTrapdoor = 5;
    int jsButtonTurbo = 6;
    
    double leftDistance;
    double rightDistance;
    double rotationDistance;

    // encoder scales
    double inchesPerTick = 0.072;
    double lastLeft = 0;
    double lastRight = 0;
    double lastRotation = 0;
    double lastTime = 0;
    double lastDistanceTime = 0; // distance and speed m,easurement time base
    double leftSpeed = 0;
    double rightSpeed = 0;
    double motionSpeed;
    double rotationSpeed;
    double motionThrottle = 0;
    double rotationThrottle = 0;
    double taskTime = 0;

    boolean automatic = false;
    boolean highGear = false;
    double averageSpeed = 0;
    double upShiftSpeed = 45.0;
    
    double downShiftSpeed = 44.0;

    double dblLastTime = 0.0;
    int intAutoState = 0;

    boolean bClimbButtonPressed = false;
    boolean bTrapdoorLast = false;
    boolean bTrapdoorCurr = false;
    int intTrapDoor = 0;
    
    public RobotTemplate() {
        
        //Here we set the robotDrive motors and the driver station.
        
        m_robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, false);
        m_robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        m_robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, false);
        
        m_robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);

        ds = DriverStation.getInstance();
        dsio = ds.getEnhancedIO();
        m_dsLCD = DriverStationLCD.getInstance();
        
    }
    
    public void robotInit() {
       
        //This is where we tell the encoders and compressor to start, and 
        //  we also set the pnuematics and solenoids to their positions. 
       
        // init encoders
        leftEncoder.start();
        rightEncoder.start();
//        leftEncoder.setDistancePerPulse(inchesPerTick);
//        rightEncoder.setDistancePerPulse(inchesPerTick);

        // init transmission
        highGear = false;
        automatic = false;
        compressor.start();
        
        intClimbingState = climbNormalMode;
        sElevator.set(false);
        sTrapDoor1.set(false);
        
//        relay.set(Relay.Value.kForward); 
    }

    public void autonomousInit() {
        climbTimer.reset();
        climbTimer.start();
        
    }
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        double dblNewTime;
        dblNewTime = climbTimer.get();
        
        switch (intAutoState)
        {
            case 0:
                dblLastTime = dblNewTime;
                intAutoState = 1;
                break;
            case 1:
                m_robotDrive.arcadeDrive(.05,-.4,false);
                if (Math.abs(dblNewTime-dblLastTime)>4)
                    intAutoState = 2;
                break;
            case 2:
                m_robotDrive.arcadeDrive(0,0,false);
                sTrapDoor1.set(false);
                sTrapDoor2.set(true);
                if (Math.abs(dblNewTime-dblLastTime)>9)
                    intAutoState = 3;
                break;
            case 3:
                m_robotDrive.arcadeDrive(0,0,false);
                sTrapDoor1.set(true);
                sTrapDoor2.set(false);
//                if (Math.abs(dblNewTime-dblLastTime)>9)
//                    intAutoState = 4;
                break;        
        }           
    }

    public void teleopInit()
    {
        m_dsLCD.println(DriverStationLCD.Line.kUser1, 1, "START");
        m_dsLCD.updateLCD();
        intClimbingState = climbNormalMode;
        sElevator.set(false);
        sGearLow.set(false);
        sGearHigh.set(true);
        sTrapDoor1.set(false); 

        bClimbButtonPressed=false;
    }
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        
//        
//        // transmission control
        leftDistance = leftEncoder.getRaw(); //Distance();
        rightDistance = rightEncoder.getRaw(); //Distance();
//
//        // adjust speed readings only if at least some time has expired
//        if(taskTime > (lastDistanceTime + 0.1)){
//            // set PID speed inputs
//            leftSpeed = ((leftDistance - lastLeft) / (taskTime - lastDistanceTime));
//            rightSpeed = ((rightDistance - lastRight) / (taskTime - lastDistanceTime));
//            motionSpeed = (rightSpeed + leftSpeed) / 2;
//            averageSpeed = (Math.abs(rightSpeed) + Math.abs(leftSpeed)) / 2;
//
//            rotationDistance = leftDistance - rightDistance;
//            rotationSpeed = (rotationDistance - lastRotation) / (taskTime - lastDistanceTime);
//
//            lastLeft = leftDistance;
//            lastRight = rightDistance;
//            lastRotation = rotationDistance;
//            lastDistanceTime = taskTime;
//        }
//        // handle transmission
//        //if(joystick1.getRawButton(7)) automatic = true;
//        //if(robot.driverStation.leftDriveStick.getRawButton(6)) automatic = false;
        automatic = false;
        do2.set(false);
        relay2.set(Relay.Value.kForward);
        relay3.set(Relay.Value.kReverse);
        
        if(highGear) {
            if((automatic && averageSpeed < downShiftSpeed) ||
                    (!automatic && joystickDriver.getRawButton(jsButtonTurbo))){
                sGearHigh.set(true);
                sGearLow.set(false);
                highGear = false;
            }
        }
        else {
            if((automatic && averageSpeed > upShiftSpeed) ||
                    (!automatic && !joystickDriver.getRawButton(jsButtonTurbo))){
                sGearHigh.set(false);
                sGearLow.set(true);
                highGear = true;
            }
        }
        if (highGear){
            sGearHigh.set(true);
            sGearLow.set(false);

        }
        else{
            sGearHigh.set(false);
            sGearLow.set(true);

        }
        
        // Drive train
        double xx;
        double yy;
          
        // Tank drive
        if (false)
        {
            if (joystickDriver.getRawButton(6)==false)
            {
                yy = joystickDriver.getRawAxis(5);//*joystickDriver.getRawAxis(4);
                xx = joystickDriver.getRawAxis(2);//*joystickDriver.getRawAxis(2);
            }
            else
            {
                yy = joystickDriver.getRawAxis(5);//*joystickDriver.getRawAxis(4);
                xx = joystickDriver.getRawAxis(2);//*joystickDriver.getRawAxis(2);
            }
            /*System.out.println(yy + " " + xx + " " + leftDistance + " " + rightDistance + " " +highGear );
            m_dsLCD.println(DriverStationLCD.Line.kUser2, 1, "Joystick YY" + yy);
            m_dsLCD.println(DriverStationLCD.Line.kUser3, 1, "Joystick XX" + xx);
            m_dsLCD.println(DriverStationLCD.Line.kUser4, 1, "Left Encoder" + leftDistance);
            m_dsLCD.println(DriverStationLCD.Line.kUser5, 1, "Right Encoder" + rightDistance);

            m_dsLCD.updateLCD();
*/
            m_robotDrive.tankDrive(yy, xx);
        }
        else  // dual arcade drive?
        {
            if (joystickDriver.getRawButton(6)==false)
            {
                if (highGear) {yy = joystickDriver.getRawAxis(4) / -4;} //*joystickDriver.getRawAxis(4)
                
                else
                {
                yy = joystickDriver.getRawAxis(4) * -.8;//*joystickDriver.getRawAxis(4);
                }
                
                xx = joystickDriver.getRawAxis(2);//*joystickDriver.getRawAxis(2);
            }
            else
            {
                if (highGear) {yy = joystickDriver.getRawAxis(4) / -4;} //*joystickDriver.getRawAxis(4)

                else
                {
                yy = joystickDriver.getRawAxis(4) * -.8;//*joystickDriver.getRawAxis(4);
                }
                
                xx = joystickDriver.getRawAxis(2);//*joystickDriver.getRawAxis(2);
            }
            //System.out.println(yy + " " + xx + " " + leftDistance + " " + rightDistance + " " +highGear );
            m_dsLCD.println(DriverStationLCD.Line.kUser2, 1, "Joystick YY" + yy);
            m_dsLCD.println(DriverStationLCD.Line.kUser3, 1, "Joystick XX" + xx);
            m_dsLCD.println(DriverStationLCD.Line.kUser4, 1, "Left Encoder" + leftDistance);
            m_dsLCD.println(DriverStationLCD.Line.kUser5, 1, "Right Encoder" + rightDistance);
            m_dsLCD.println(DriverStationLCD.Line.kUser6, 1, "moving?");
            m_dsLCD.updateLCD();

            xx = .5;
            yy = .5;
            m_robotDrive.arcadeDrive(yy, xx, false);
        }
        
        bTrapdoorCurr = joystickDriver.getRawButton(jsButtonTrapdoor);
        if (bTrapdoorCurr != bTrapdoorLast)
        {
            if (bTrapdoorCurr)
            {
                intTrapDoor = 1 - intTrapDoor;
            }
        }
        
        bTrapdoorLast=bTrapdoorCurr;
        if (intTrapDoor != 0)
        {
            sTrapDoor1.set(false);
            sTrapDoor2.set(true);
        } 
        else 
        {
            sTrapDoor1.set(true);
            sTrapDoor2.set(false);
        }
        double pow;
        double axis;
        axis = joystickClimber.getRawAxis(3);
        pow = (-1*axis+1)/2;
        shooter.set(pow);
        m_dsLCD.println(DriverStationLCD.Line.kUser6, 1, "Power " + 100*pow + "%     ");
        m_dsLCD.println(DriverStationLCD.Line.kUser1, 1, "Duck and Cover!");
        m_dsLCD.updateLCD(); 
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        //Testing code for first prototype shooter;
       
    }
    
}

////        // Climbing code
//        if (joystickClimber.getRawButton(jsButtonClimbProceed)) // if pressed
//        {
//            bClimbButtonPressed = true;
//            if (intClimbingState==climbNormalMode)
//            {
//                intClimbingState=climbExtendPastRung1;
//            }
//        } else { // if released
//            bClimbButtonPressed = false;
//        }
//        
//        if (bClimbButtonPressed)
//        {
//            intStopClimbingState = intClimbingState;
//        }
//       
//        if (intStopClimbingState!=intClimbingState)
//        {
//            m_robotDrive.tankDrive(0,0);
//        }
//        else
//        {
//            // Deal with climbing process
//            switch (intClimbingState)
//            {
//                case climbNormalMode:
//                    break;
//                case climbExtendPastRung1:
//                    // Here we go from drive mode to lift mode
//                    sElevator.set(true);
//                    sClaw.set(false);
//
//                    // assume no time to switch to elevator lift, add state to handle delay if necessary
//                    m_robotDrive.tankDrive(1, 1);
//                    if (dioElevatorMax.get())
//                    {
//                            m_robotDrive.tankDrive(0,0);
//                            intClimbingState=climbLiftToRung1;
//                    }
//                    break;
//                case climbLiftToRung1:
//                    m_robotDrive.tankDrive(-1, -1);
//                    if (dioElevatorMin.get())
//                    {
//                        intClimbingState=climbLiftToRung1;
//                        climbTimer.reset();
//                        dblLastTime = climbTimer.get();        
//                        intClimbingState = climbFistOfGoodnessRung1;
//                    }
//                    break;
//                case climbFistOfGoodnessRung1:
//                    sClaw.set(true);
//                    if (climbTimer.get()>(dblLastTime+.1))
//                    {
//                        intClimbingState=climbExtendPastRung2;
//                    }
//                    break;
//                case climbExtendPastRung2:
//                    m_robotDrive.tankDrive(1, 1);
//                    if (dioElevatorMax.get())
//                    {
//                            m_robotDrive.tankDrive(0,0);
//                            intClimbingState=climbLiftToRung2;
//                    }
//                    break;
//                case climbLiftToRung2:
//                    m_robotDrive.tankDrive(-1, -1);
//                    if (dioClaw.get())
//                    {
//                        sClaw.set(false);
//                        intClimbingState = climbRemainingLiftToRung2;
//                    }
//                    break;
//                case climbRemainingLiftToRung2:
//                    m_robotDrive.tankDrive(-1,-1);
//                    if (dioElevatorMin.get())
//                    {
//                        m_robotDrive.tankDrive(0,0);
//                        climbTimer.reset();
//                        dblLastTime = climbTimer.get();        
//                        intClimbingState = climbFistOfGoodnessRung2;
//                    }                
//                    break;
//                case climbFistOfGoodnessRung2:
//                    sClaw.set(true);
//                    if (climbTimer.get()>(dblLastTime+.1))
//                    {
//                        intClimbingState=climbExtendPastRung3;
//                    }
//                    break;
//                case climbExtendPastRung3:
//                    m_robotDrive.tankDrive(1, 1);
//                    if (dioElevatorMax.get())
//                    {
//                            m_robotDrive.tankDrive(0,0);
//                            intClimbingState=climbLiftToRung3;
//                    }
//                    break;
//                case climbLiftToRung3:
//                    m_robotDrive.tankDrive(-1, -1);
//                    if (dioClaw.get())
//                    {
//                        sClaw.set(false);
//                        intClimbingState = climbRemainingLiftToRung3;
//                    }
//                    break;
//                case climbRemainingLiftToRung3:
//                    m_robotDrive.tankDrive(-1,-1);
//                    if (dioElevatorMin.get())
//                    {
//                        m_robotDrive.tankDrive(0,0);
//                        climbTimer.reset();
//                        dblLastTime = climbTimer.get();        
//                        intClimbingState = climbFistOfGoodnessRung3;
//                    }
//                    break;
//
//                case climbFistOfGoodnessRung3:
//                    sClaw.set(true);
//                    if (climbTimer.get()>(dblLastTime+.1))
//                    {
//                        intClimbingState=climbExtendPastRung3;
//                    }
//                    break;                    
//            }
//        }


/**
 *   //solenoid switch
 * if (joystick1.getRawButton(3)==true)
 * {
 *      sGearLeft.set(true);
 *      sGearRight.set(true);

 * 
 *  motors spin same way
 * 
 * button 2 = new button up;
 *    if (joystick1.getRawButton(5) ==true)
 *   
 * 
 * 
 * 
 * //elevator for last year   
 * boolean elevatorUp=false;
    boolean debounceElevator=false;
 *  if((joystick1.getRawButton(3)==true)||(joystick1.getRawButton(3)==true))
        {
            if(debounceElevator==false)
            {
                elevatorUp=!elevatorUp;
                debounceElevator=true;
            }
        }
        else
        {
            debounceElevator=false;
        }
        if(elevatorUp==true)
        {
            pickup.setDirection(Relay.Direction.kForward);
            pickup.set(Relay.Value.kOn);
            LED(11,true);
            //System.out.println("Ele-on");
        }
        else
        {
            pickup.setDirection(Relay.Direction.kForward);
            pickup.set(Relay.Value.kOff);
            LED(11,false);
            //System.out.println("Ele-off");
 */
