import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.camera.AxisCamera;

public class FRC2014Java extends SimpleRobot {
    
    //Constants
    //Drive motor controller ports
    public static final int TALON_PORT_L = 9;
    public static final int TALON_PORT_R = 10;
    
    //Intake and catapult motor controller ports
    public static final int TALON_PORT_I = 1;
    public static final int TALON_PORT_C = 2;
    
    //Intake speeds
    public static final double INTAKE_IN = 1;
    public static final double INTAKE_OUT = -0.55;
    
    //Winch speed
    public static final double WINCH_SPEED = 1;
    
    //Deadzone
    public static final double DEADZONE_HIGH = 0.2;
    public static final double DEADZONE_LOW = -0.2;
    
    //Number of Buttons
    public static final int BUTTONS = 16;
    
    //Vars
    //Motor controllers
    Talon leftDrive;
    Talon rightDrive;
    
    //Winch
    Talon winch;
    
    //Intake
    Talon intake;
    
    //Cam
    Talon cam;
    
    //Xbox Controller
    Joystick xbox;
    
    //Catapult Limit
    DigitalInput catapultLimit;
    
    //Cam Limit
    DigitalInput camLimit;
    
    //Intake Limit
    DigitalInput intakeLimit;
    
    //Front Camera
    AxisCamera cameraFront;
    
    //Back Camera
    AxisCamera cameraBack;

    //Watchdog
    Watchdog dog;
    
    //Constructor
    FRC2014Java(){
        
        //Motor Controllers
        leftDrive = new Talon(TALON_PORT_L);
        rightDrive = new Talon(TALON_PORT_R);
        
        //Joystick
        xbox = new Joystick(1);
        
        //Winch
        winch = new Talon(2);
        
        //Intake
        intake = new Talon(8);
        
        //Cam
        cam = new Talon(3);
        
        //Catapult Limit Switch
        catapultLimit = new DigitalInput(1);
        
        //Cam Limit Switch
        camLimit = new DigitalInput(2);
        
        //Intake Limit Switch
        intakeLimit = new DigitalInput(3);
        
        //Cameras
        cameraFront = AxisCamera.getInstance("10.10.2.11");
        cameraBack = AxisCamera.getInstance("10.10.2.12");
        
        //Watchdog
        dog = Watchdog.getInstance();
    }
    
    /**
     * This method is called once each tome the robot starts
     */
    public void robotInit() {
        
        //Set watchdog refresh rate
        dog.setExpiration(0.5);
        
    }

    public void autonomous() {
        
    }

    /**
     * This method is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        
        //Variable indicates which direction Y-axis is
        double coefficientOfDriveDirection = 1;
        
        //Loops
        while (isOperatorControl()) {
			
            //Watchdog
            dog.feed();
            
            /**
             * DRIVE
             */
            //Controller Mapping
            double lStick_X = monsieurKevin(xbox.getRawAxis(1), 2);
            double lStick_Y = monsieurKevin(xbox.getRawAxis(2), 2) * -1;
            
            double rStick_X = xbox.getRawAxis(4);
            double rStick_Y = xbox.getRawAxis(5) * -1;
            
            double leftTrigger = xbox.getRawAxis(3);
            double rightTrigger = xbox.getRawAxis(3) * -1;
            
            double dPad = xbox.getRawAxis(6);
            
            boolean buttonA = xbox.getRawButton(1);
            boolean buttonB = xbox.getRawButton(2);
            boolean buttonX = xbox.getRawButton(3);
            boolean buttonY = xbox.getRawButton(4);
            
            boolean leftBumper = xbox.getRawButton(5);
            boolean rightBumper = xbox.getRawButton(6);
            
            boolean back = xbox.getRawButton(7);
            boolean start = xbox.getRawButton(8);
            
            //Default motor controller speed is 40% of the input
            double driveLevel = 0.4;
            
            //Holding Y sets motor controller speed to 100% of the input
            if(buttonY){
                driveLevel = 1;
            }
            
            //Switches the direction of Y-axis based on the left and right keys
            //on the d-pad
            if(dPad == 1){
                coefficientOfDriveDirection = 1;
            }else if(dPad == -1){
                coefficientOfDriveDirection = -1;
            }
            
            //Set drive variables
            double leftSpeed = driveLevel * (coefficientOfDriveDirection *
                    deadzone(lStick_Y) + deadzone(lStick_X));
            double rightSpeed = driveLevel * (coefficientOfDriveDirection *
                    deadzone(-lStick_Y) + deadzone(lStick_X));
            
            //Set motor controller speeds to the above drive variables
            leftDrive.set(leftSpeed);
            rightDrive.set(rightSpeed);
            
            /**
             * INTAKE
             */
            //Gets the state of the the limit switch positioned below the intake
            boolean isIntakeDown = !intakeLimit.get();
            
            //X pulls the intake in; A sends the intake out until the intake is
            //pressed in which case the intake is set to stop
            if(buttonX){
                intake.set(INTAKE_IN);
            }else if(buttonA && !isIntakeDown){
                intake.set(INTAKE_OUT);
            }else{
                intake.set(0);
            }
            
            /**
             * WINCH
             */
            //Gets the state of the the limit switch positioned below the
            //catapult
            boolean isCatapultTensioned = catapultLimit.get();
            
            //Check
            if(isCatapultTensioned && leftBumper){
                winch.set(-1);
            }else{
                winch.set(0);
            }
            
            /**
             * CAM (Release)
             */
            //Gets the state of the the limit switch positioned at the
            //starting point of the cam
            boolean isCamEngaged = camLimit.get();
            
            //Checks if the cam is at the starting position. If so, it checks if
            //the right bumper is being pressed. If so, start the cam turning
            //while the cam is still in the starting position. Then keep the cam
            //turning until it returns to the starting position. Then, it will
            //stop because the code will loop back to the beginning and the
            //right bumper will not be pressed and the cam will be set to stop.
            //The top else is a safety check in case the cam is not engaged.
            if(!isCamEngaged){
                if(rightBumper){
                    do{
                        cam.set(-1);
                        isCamEngaged = camLimit.get();
                    }while(!isCamEngaged);
                    
                    do{
                        cam.set(-1);
                        isCamEngaged = camLimit.get();
                    }while(isCamEngaged);
                }else{
                    cam.set(0);
                }
            }else{
                cam.set(-1);
            }
        }
    }
    
    //Deadzone method
    /**
     * If you are unaware what a deadzone method does, it sets a value if an
     * input is in a close range. For example, this method sets the value to 0
     * when the joystick is in the range (-0.2, 0.2). This is prevent the robot
     * from drifting when the joystick is slightly off.
     * @param d is the input
     * @return the updated value (unchanged if not in the range)
     */
    public double deadzone(double d){
        
        //Check if value is inside deadzone
        if(d < DEADZONE_HIGH && d > DEADZONE_LOW){
            d = 0;
        }
        
        return d;
    }
    
    /**
     * This method does exponent math without changing the original sign
     * (+ or -). For example, Math.pow(-2, 4) evaluates to 16 while
     * monsieurKevin(-2, 4) evaluates to -16. (The method name is an inside
     * joke).
     * @param base is the base
     * @param exp is the exponent
     * @return @param base^@param exp while keeping the sign of @param base
     */
    public double monsieurKevin(double base, int exp){
        double multiplier = 1;
        
        if(base < 0){
            multiplier *= -1;
        }
        
        double ans = MathUtils.pow(base, exp);
        ans *= multiplier;
        
        return ans;
    }
}