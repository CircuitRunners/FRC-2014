import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.*;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Sample program to use NIVision to find rectangles in the scene that are illuminated
 * by a LED ring light (similar to the model from FIRSTChoice). The camera sensitivity
 * is set very low so as to only show light sources and remove any distracting parts
 * of the image.
 * 
 * The CriteriaCollection is the set of criteria that is used to filter the set of
 * rectangles that are detected. In this example we're looking for rectangles with
 * a minimum width of 30 pixels and maximum of 400 pixels.
 * 
 * The algorithm first does a color threshold operation that only takes objects in the
 * scene that have a bright green color component. Then a small object filter
 * removes small particles that might be caused by green reflection scattered from other 
 * parts of the scene. Finally all particles are scored on rectangularity, and aspect ratio,
 * to determine if they are a target.
 *
 * Look in the VisionImages directory inside the project that is created for the sample
 * images.
 */

public class FRC2014Java extends SimpleRobot {

    //Camera constants used for distance calculation
    final int Y_IMAGE_RES = 480;		//X Image resolution in pixels, should be 120, 240 or 480
    final double VIEW_ANGLE = 49;		//Axis M1013
    //final double VIEW_ANGLE = 41.7;		//Axis 206 camera
    //final double VIEW_ANGLE = 37.4;  //Axis M1011 camera
    final double PI = 3.141592653;

    //Score limits used for target identification
    final int  RECTANGULARITY_LIMIT = 40;
    final int ASPECT_RATIO_LIMIT = 55;

    //Score limits used for hot target determination
    final int TAPE_WIDTH_LIMIT = 50;
    final int  VERTICAL_SCORE_LIMIT = 50;
    final int LR_SCORE_LIMIT = 50;

    //Minimum area of particles to be considered
    final int AREA_MINIMUM = 150;

    
    //Maximum number of particles to process
    final int MAX_PARTICLES = 8;
    
        //Constants
    //Motor Controller Ports
    public static final int TALON_PORT_L = 9;
    public static final int TALON_PORT_R = 10;
    
    public static final int TALON_PORT_I = 1;
    public static final int TALON_PORT_C = 2;
    
    public static final double INTAKE_FORWARD = 1;
    public static final double INTAKE_REVERSE = -0.55;
    
    public static final double WINCH_SPEED = 1;
    
    //Deadband
    public static final double DEADBAND_HIGH = 0.2;
    public static final double DEADBAND_LOW = -0.2;
    
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

    //Watchdog
    Watchdog dog;

    AxisCamera camera;          // the axis camera object (connected to the switch)
    CriteriaCollection cc;      // the criteria for doing the particle filter operation
    
    public class Scores {
        double rectangularity;
        double aspectRatioVertical;
        double aspectRatioHorizontal;
    }
    
    public class TargetReport {
		int verticalIndex;
		int horizontalIndex;
		boolean Hot;
		double totalScore;
		double leftScore;
		double rightScore;
		double tapeWidthScore;
		double verticalScore;
    };
    
    FRC2014Java(){
        
        //Construct Motor Controllers
        leftDrive = new Talon(TALON_PORT_L);
        rightDrive = new Talon(TALON_PORT_R);
        
        //Construct Joysticks
        xbox = new Joystick(1);
        
        //Construct Winch
        winch = new Talon(2);
        
        //Construct Spike
        intake = new Talon(8);
        
        //Construct Cam
        cam = new Talon(3);
        
        //Catapult Limit
        catapultLimit = new DigitalInput(1);
        
        //Cam Limit
        camLimit = new DigitalInput(2);
        
        //Intake Limit
        intakeLimit = new DigitalInput(3);
        
        //Construct Watchdog
        dog = Watchdog.getInstance();
    }
    
    public void robotInit() {
        dog.setExpiration(0.5);
        camera = AxisCamera.getInstance("10.10.2.12");  // get an instance of the camera
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        cc.addCriteria(MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, true);
    }

    public void autonomous() {
        TargetReport target = new TargetReport();
		int verticalTargets[] = new int[MAX_PARTICLES];
		int horizontalTargets[] = new int[MAX_PARTICLES];
		int verticalTargetCount, horizontalTargetCount;
        
		boolean hasFired = false;
		
        while (isAutonomous() && isEnabled() && !hasFired) {
            try {
                /**
                 * Do the image capture with the camera and apply the algorithm described above. This
                 * sample will either get images from the camera or from an image file stored in the top
                 * level directory in the flash memory on the cRIO. The file name in this case is "testImage.jpg"
                 * 
                 */
                ColorImage image = camera.getImage();     // comment if using stored images
                image.write("/image.jpg");
                //ColorImage image;                           // next 2 lines read image from flash on cRIO
                //image = new RGBImage("/testImage.jpg");		// get the sample image from the cRIO flash
                BinaryImage thresholdImage = image.thresholdHSV(85, 255, 100, 255, 100, 255);   // keep only green objects
                thresholdImage.write("/threshold.bmp");
                BinaryImage filteredImage = thresholdImage.particleFilter(cc);           // filter out small particles
                filteredImage.write("/filteredImage.bmp");
                
                //iterate through each particle and score to see if it is a target
                Scores scores[] = new Scores[filteredImage.getNumberParticles()];
                horizontalTargetCount = verticalTargetCount = 0;
                System.out.println("Caleb");
                if(filteredImage.getNumberParticles() > 0)
                {
                    System.out.println("Caleb2");
			for (int i = 0; i < MAX_PARTICLES && i < filteredImage.getNumberParticles(); i++) {
			ParticleAnalysisReport report = filteredImage.getParticleAnalysisReport(i);
                        scores[i] = new Scores();
					
			//Score each particle on rectangularity and aspect ratio
			scores[i].rectangularity = scoreRectangularity(report);
			scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, i, true);
			scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, i, false);			
					
			//Check if the particle is a horizontal target, if not, check if it's a vertical target
			if(scoreCompare(scores[i], false))
			{
                            System.out.println("particle: " + i + "is a Horizontal Target centerX: " + report.center_mass_x + "centerY: " + report.center_mass_y);
                            horizontalTargets[horizontalTargetCount++] = i; //Add particle to target array and increment count
			} else if (scoreCompare(scores[i], true)) {
                            System.out.println("particle: " + i + "is a Vertical Target centerX: " + report.center_mass_x + "centerY: " + report.center_mass_y);
                            verticalTargets[verticalTargetCount++] = i;  //Add particle to target array and increment count
			} else {
                            System.out.println("particle: " + i + "is not a Target centerX: " + report.center_mass_x + "centerY: " + report.center_mass_y);
			}
                            System.out.println("rect: " + scores[i].rectangularity + "ARHoriz: " + scores[i].aspectRatioHorizontal);
                            System.out.println("ARVert: " + scores[i].aspectRatioVertical);	
			}

			//Zero out scores and set verticalIndex to first target in case there are no horizontal targets
			target.totalScore = target.leftScore = target.rightScore = target.tapeWidthScore = target.verticalScore = 0;
			target.verticalIndex = verticalTargets[0];
			for (int i = 0; i < verticalTargetCount; i++)
			{
				ParticleAnalysisReport verticalReport = filteredImage.getParticleAnalysisReport(verticalTargets[i]);
				for (int j = 0; j < horizontalTargetCount; j++)
				{
                                    ParticleAnalysisReport horizontalReport = filteredImage.getParticleAnalysisReport(horizontalTargets[j]);
                                    double horizWidth, horizHeight, vertWidth, leftScore, rightScore, tapeWidthScore, verticalScore, total;
	
                                    //Measure equivalent rectangle sides for use in score calculation
                                    horizWidth = NIVision.MeasureParticle(filteredImage.image, horizontalTargets[j], false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
                                    vertWidth = NIVision.MeasureParticle(filteredImage.image, verticalTargets[i], false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
                                    horizHeight = NIVision.MeasureParticle(filteredImage.image, horizontalTargets[j], false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
						
                                    //Determine if the horizontal target is in the expected location to the left of the vertical target
                                    leftScore = ratioToScore(1.2*(verticalReport.boundingRectLeft - horizontalReport.center_mass_x)/horizWidth);
                                    //Determine if the horizontal target is in the expected location to the right of the  vertical target
                                    rightScore = ratioToScore(1.2*(horizontalReport.center_mass_x - verticalReport.boundingRectLeft - verticalReport.boundingRectWidth)/horizWidth);
                                    //Determine if the width of the tape on the two targets appears to be the same
                                    tapeWidthScore = ratioToScore(vertWidth/horizHeight);
                                    //Determine if the vertical location of the horizontal target appears to be correct
                                    verticalScore = ratioToScore(1-(verticalReport.boundingRectTop - horizontalReport.center_mass_y)/(4*horizHeight));
                                    total = leftScore > rightScore ? leftScore:rightScore;
                                    total += tapeWidthScore + verticalScore;

                                    //If the target is the best detected so far store the information about it
                                    if(total > target.totalScore)
                                    {
                                            target.horizontalIndex = horizontalTargets[j];
                                            target.verticalIndex = verticalTargets[i];
                                            target.totalScore = total;
                                            target.leftScore = leftScore;
                                            target.rightScore = rightScore;
                                            target.tapeWidthScore = tapeWidthScore;
                                            target.verticalScore = verticalScore;
                                    }
                                }
                                //Determine if the best target is a Hot target
                                target.Hot = hotOrNot(target);
                            }

                            if(verticalTargetCount > 0)
                            {
                                    //Information about the target is contained in the "target" structure
                                    //To get measurement information such as sizes or locations use the
                                    //horizontal or vertical index to get the particle report as shown below
                                    ParticleAnalysisReport distanceReport = filteredImage.getParticleAnalysisReport(target.verticalIndex);
                                    double distance = computeDistance(filteredImage, distanceReport, target.verticalIndex);
                                    if(target.Hot)
                                    {
                                            System.out.println("Hot target located");
                                            System.out.println("Distance: " + distance);
                                            cam.set(-1);
											hasFired = true;
                                    } else {
                                            System.out.println("No hot target present");
                                            System.out.println("Distance: " + distance);
                                            cam.set(0);
                                    }
                            }
                }

                /**
                 * all images in Java must be freed after they are used since they are allocated out
                 * of C data structures. Not calling free() will cause the memory to accumulate over
                 * each pass of this loop.
                 */
                filteredImage.free();
                thresholdImage.free();
                image.free();
                
            } catch (AxisCameraException ex) {        // this is needed if the camera.getImage() is called
                ex.printStackTrace();
            } catch (NIVisionException ex) {
                ex.printStackTrace();
            }
        }
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        
        double coefficientOfDriveDirection = 1;
        
        while (isOperatorControl() && isEnabled()) {
			
            //Watchdog
            dog.feed();
            
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
            
            //Drive Level
            double driveLevel = 0.4;
            
            //Boost
            if(buttonY){
                driveLevel = 1;
            }
            
            //Drive Controls
            if(dPad == 1){
                coefficientOfDriveDirection = 1;
            }else if(dPad == -1){
                coefficientOfDriveDirection = -1;
            }
            
            //Set Drive Variables
            double leftSpeed = driveLevel * (coefficientOfDriveDirection * deadband(lStick_Y) + deadband(lStick_X));
            double rightSpeed = driveLevel * (coefficientOfDriveDirection * deadband(-lStick_Y) + deadband(lStick_X));
            
            /*
            //SwagDrive
            double swagA=
            
            if(Math.abs(lstick_Y)
            double leftSpeed = driveLevel * (coefficientOfDriveDirection * (deadband(lStick_Y) + deadband(lStick_X)));
            double rightSpeed = driveLevel * (coefficientOfDriveDirection * (deadband(-lStick_Y) + deadband(lStick_X)));

            */
            
            //Set Motor Controllers
            leftDrive.set(leftSpeed);
            rightDrive.set(rightSpeed);
            
            //Intake
            boolean isIntakeDown = intakeLimit.get();
            
            if(buttonX){
                intake.set(INTAKE_FORWARD);
            }else if(buttonA && isIntakeDown){
                intake.set(INTAKE_REVERSE);
            }else{
                intake.set(0);
            }
            
            //Winch
            //Catapult Limit
            boolean isCatapultTensioned = catapultLimit.get();
            
            //Check
            if(isCatapultTensioned && leftBumper){
                winch.set(-1);
            }else{
                winch.set(0);
            }
            
            //Cam
            //Limit
            boolean isCamEngaged = camLimit.get();
            //Check
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
    
    /**
     * Computes the estimated distance to a target using the height of the particle in the image. For more information and graphics
     * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
     * 
     * @param image The image to use for measuring the particle estimated rectangle
     * @param report The Particle Analysis Report for the particle
     * @param outer True if the particle should be treated as an outer target, false to treat it as a center target
     * @return The estimated distance to the target in Inches.
     */
    double computeDistance (BinaryImage image, ParticleAnalysisReport report, int particleNumber) throws NIVisionException {
            double rectLong, height;
            int targetHeight;

            rectLong = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
            //using the smaller of the estimated rectangle long side and the bounding rectangle height results in better performance
            //on skewed rectangles
            height = Math.min(report.boundingRectHeight, rectLong);
            targetHeight = 32;

            return Y_IMAGE_RES * targetHeight / (height * 12 * 2 * Math.tan(VIEW_ANGLE*Math.PI/(180*2)));
    }
    
    /**
     * Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target. This method uses
     * the equivalent rectangle sides to determine aspect ratio as it performs better as the target gets skewed by moving
     * to the left or right. The equivalent rectangle is the rectangle with sides x and y where particle area= x*y
     * and particle perimeter= 2x+2y
     * 
     * @param image The image containing the particle to score, needed to perform additional measurements
     * @param report The Particle Analysis Report for the particle, used for the width, height, and particle number
     * @param outer	Indicates whether the particle aspect ratio should be compared to the ratio for the inner target or the outer
     * @return The aspect ratio score (0-100)
     */
    public double scoreAspectRatio(BinaryImage image, ParticleAnalysisReport report, int particleNumber, boolean vertical) throws NIVisionException
    {
        double rectLong, rectShort, aspectRatio, idealAspectRatio;

        rectLong = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
        rectShort = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
        idealAspectRatio = vertical ? (4.0/32) : (23.5/4);	//Vertical reflector 4" wide x 32" tall, horizontal 23.5" wide x 4" tall
	
        //Divide width by height to measure aspect ratio
        if(report.boundingRectWidth > report.boundingRectHeight){
            //particle is wider than it is tall, divide long by short
            aspectRatio = ratioToScore((rectLong/rectShort)/idealAspectRatio);
        } else {
            //particle is taller than it is wide, divide short by long
            aspectRatio = ratioToScore((rectShort/rectLong)/idealAspectRatio);
        }
	return aspectRatio;
    }
    
    /**
     * Compares scores to defined limits and returns true if the particle appears to be a target
     * 
     * @param scores The structure containing the scores to compare
     * @param outer True if the particle should be treated as an outer target, false to treat it as a center target
     * 
     * @return True if the particle meets all limits, false otherwise
     */
    boolean scoreCompare(Scores scores, boolean vertical){
	boolean isTarget = true;

	isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
	if(vertical){
            isTarget &= scores.aspectRatioVertical > ASPECT_RATIO_LIMIT;
	} else {
            isTarget &= scores.aspectRatioHorizontal > ASPECT_RATIO_LIMIT;
	}

	return isTarget;
    }
    
    /**
     * Computes a score (0-100) estimating how rectangular the particle is by comparing the area of the particle
     * to the area of the bounding box surrounding it. A perfect rectangle would cover the entire bounding box.
     * 
     * @param report The Particle Analysis Report for the particle to score
     * @return The rectangularity score (0-100)
     */
    double scoreRectangularity(ParticleAnalysisReport report){
            if(report.boundingRectWidth*report.boundingRectHeight !=0){
                    return 100*report.particleArea/(report.boundingRectWidth*report.boundingRectHeight);
            } else {
                    return 0;
            }	
    }
    
    	/**
	 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
	 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
	 */
	double ratioToScore(double ratio)
	{
		return (Math.max(0, Math.min(100*(1-Math.abs(1-ratio)), 100)));
	}
	
	/**
	 * Takes in a report on a target and compares the scores to the defined score limits to evaluate
	 * if the target is a hot target or not.
	 * 
	 * Returns True if the target is hot. False if it is not.
	 */
	boolean hotOrNot(TargetReport target)
	{
		boolean isHot = true;
		
		isHot &= target.tapeWidthScore >= TAPE_WIDTH_LIMIT;
		isHot &= target.verticalScore >= VERTICAL_SCORE_LIMIT;
		isHot &= (target.leftScore > LR_SCORE_LIMIT) | (target.rightScore > LR_SCORE_LIMIT);
		
		return isHot;
	}
     //Deadband method
    public double deadband(double d){
        
        //Check if value is inside deadzone
        if(d < DEADBAND_HIGH && d > DEADBAND_LOW){
            d = 0;
        }
        
        return d;
    }
    
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