
package org.usfirst.frc.team6544.robot;

import edu.wpi.first.wpilibj.SampleRobot;        // imports parts of the first library
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

//import java.io.File; 
//import javax.swing.BoxLayout; 
//import javax.swing.JButton; 
//import javax.swing.JFrame;
//import org.opencv.core.Core; 
//import org.opencv.core.Mat; 

/*
 * XBOX BUTTON MAPPING FOR DRIVER STATION AS FOLLOWS
 * Button 1 = A
 * Button 2 = B
 * Button 3 = X OR small left wheel button up
 * Button 4 = Y
 * Button 5 = Left Bumper
 * Button 6 = Right Bumper OR small right wheel down
 * Button 7 = Select / Menu Button
 * Button 8 = Start / Enter Button
 * Button 9 = Click in Left Analog Stick OR small left wheel down
 * Button 10 = Click in Right Analog Stick
 * 
 * Axis 0 = Left stick left + right
 * Axis 1 = Left stick up + down
 * Axis 2 = Left trigger
 * Axis 3 = Right trigger
 * Axis 4 = Right stick left + right
 * Axis 5 = Right stick up + down
 * 
 * POV = D-Pad
 * POV LEFT = small right wheel up
 */

//Object: an object is a name the we use to name something that we will use later in our program, in front of the object name is a constructor class which tells the name we gave it what it is 

public class Robot extends SampleRobot {
	RobotDrive chassis; //Declares robot drive object as chassis this can be later used to make the robot drive; uses joystick values to control the speed of the motors 
	Talon frontLeft, frontRight, rearLeft, rearRight; //Declares a object all four Talons for the drive motors
	Joystick xboxController; //Declares the controller
	Solenoid solArm; //Declares arm solenoid
	Solenoid solGrip; //Declares gripper solenoid
	DoubleSolenoid solBall; //Declares DoubleSolenoid...Info Here: http://wpilib.screenstepslive.com/s/3120/m/7912/l/132407-operating-pneumatic-cylinders-solenoids
	ADXRS450_Gyro aGyro; //Declares gyro object
	double driveAngle; //Declares a double for the driving angle
	double scalingConstant; //Declares a double for the scaling constant
	Talon climbmotor; //Declares the talon object for the climbing motor
	CameraProcessor Cam1;
	CameraProcessor Cam2;
	Joystick winchController;
	Command autoCommand;
	SendableChooser<Integer> speedControl;
	SendableChooser<Integer> autoChooser;
	String GripperOpen;
	String GripperClose;
	String ArmUp;
	String ArmDown; 
	int kSpeedGear;
	//boolean speedSate;
	
	
public void robotInit(){
	//SmartDashboard.putString("DB/String 1", "Arm Up");
	//SmartDashboard.putString("DB/String 0", "Gripper Closed");
	 frontLeft = new Talon(1); //Assigns port 1 to the front left talon
	 frontRight = new Talon(3); //Assigns port 3 to the front right talon
	 rearLeft = new Talon(2); //Assigns port 2 to the rear left talon
	 rearRight = new Talon(4); //Assigns port 4 to the rear right talon
	 climbmotor = new Talon(5); //Assigns port 5 to the climbing motor talon
	 climbmotor.setInverted(true); //Sets the climbing motor to be inverted - YOU NEED THIS ALWAYS!!!!
	 chassis = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight); // Assigns robot drive using the four talons
	winchController = new Joystick(2);									
	 xboxController = new Joystick(1); // Sets the drive station USB position to the controller object
	solArm = new Solenoid(0); //Assigns solenoid port 0 to the Arm solenoid
	solGrip = new Solenoid(1); //Assigns solenoid port 1 to the Gripper solenoid
	Cam1 = new CameraProcessor("GripperCam",0);
	Cam2 = new CameraProcessor("DriverCam",1);
	kSpeedGear = 1;
	Cam1.start();//Cam1.SendToDashboard("cam0");
	Cam2.start();//Cam2.SendToDashboard("cam1");
	autoChooser = new SendableChooser<Integer>();
	autoChooser.addObject("Drive past line",2);
	autoChooser.addDefault("Go for center pin", 1);
	autoChooser.addObject("Go for left Side Gear",3);
	autoChooser.addObject("Go for Right Side Gear",4);
	autoChooser.addObject("Dump balls in boiler", 5);
	SmartDashboard.putData("Auto mode chooser", autoChooser);
//new below
	speedControl = new SendableChooser<Integer>();
	speedControl.addDefault("Full Speed",1);
	speedControl.addObject("Half Speed",2);
	SmartDashboard.putData("Speed Control", speedControl);
//new stops here
	solBall = new DoubleSolenoid(4,5); //INPUT 4 ON PCM(pneumatic Control Module) is FORWARD, INPUT 5 ON PCM is BACKWARDS
	aGyro = new ADXRS450_Gyro(); //Instantiates the Gyro object
	driveAngle = 0; //Default the drive angle to 0 starts the angle at 0*
	scalingConstant = 0.03; //Basic scaling constant for the Gyro to avoid oscillation, so the robot does not over correct
}
	 

	 public Robot(){ //Constructor for the class
	 
	 }
	 
	 
  
	 
	 public void autonomous() { //Start of autonomous 
		int mode; //creates an integer number variable.
		mode = (int) autoChooser.getSelected();
		if(mode == 1) {
			 aGyro.reset(); //Gyro reset prior to movement for re-calibration.
	    	 chassis.setSafetyEnabled(true); //Sets the safety mode to enabled. It's just a saftey mechanisim that firt has provided in there library. 
	    	 
	    	 solGrip.set(false);
	    	 
	    	 int loopCount = 0; // Declares and assigns a zero
	      	 while (isAutonomous() && isEnabled() && loopCount < 243){    //Checks to see if autonomous is enabled
	      		driveAngle = aGyro.getAngle(); 
	      		chassis.drive(0.35,-driveAngle * scalingConstant);
	      	//chassis.tankDrive(.8,.8); //Sets the speed of the robot for each wheel and makes them turn
	      	 Timer.delay(0.01);                      //creates a delay
	     	 loopCount = loopCount + 1; //Increments the loop counter by 1
	      }
		
	      	 
	      	Timer.delay(1);
	      	 
	      	 solGrip.set(true);
	      	
	      	Timer.delay(1);
	      	 
	        int loopCount1 = 0;
	        while (loopCount1 < 65){    
	      		chassis.drive(-0.35,0); 
	      	//chassis.tankDrive(.8,.8); //Sets the speed of the robot for each wheel and makes them turn 
	      	 Timer.delay(0.01);                      //creates a delay
	     	 loopCount1 = loopCount1 + 1; //Increments the loop counter by 1
	      }	
	        
		} 
		
		else if(mode == 2){
			 aGyro.reset(); //Gyro reset prior to movement for re-calibration
	    	 chassis.setSafetyEnabled(true); //Sets the safety mode to enabled
	    	 solGrip.set(false);
	     	int loopCount = 0; // Declares and assigns a zero
	      	while (isAutonomous() && isEnabled() && loopCount < 400){    //Checks to see if autonomous is enabled
	      		driveAngle = aGyro.getAngle(); //Gets the angle from the Gyro
	      		chassis.drive(0.35,-driveAngle * scalingConstant); //Tells the robot to move in the direction opposite of it's heading and factored with it's scaling constant
	      	//chassis.tankDrive(.8,.8); //Sets the speed of the robot for each wheel and makes them turn
	      	Timer.delay(0.01);                      //creates a delay
	      	
	     	loopCount = loopCount + 1; //Increments the loop counter by 1
	     	
	      	 }
		}
            
		
		else if(mode == 3) {
			 aGyro.reset(); //Gyro reset prior to movement for re-calibration
	    	 chassis.setSafetyEnabled(true); //Sets the safety mode to enabled
	    	 SmartDashboard.putNumber("Gyro Auto", aGyro.getAngle());
	    	 solGrip.set(false);
	    	 
	    	 int loopCount = 0; // Declares and assigns a zero
	      	 while (isAutonomous() && isEnabled() && loopCount < 199){    //Checks to see if autonomous is enabled
	      		driveAngle = aGyro.getAngle(); 
	      		chassis.drive(0.35,-driveAngle * scalingConstant);
	      	//chassis.tankDrive(.8,.8); //Sets the speed of the robot for each wheel and makes them turn
	      	 Timer.delay(0.01);                      //creates a delay
	     	 loopCount = loopCount + 1; //Increments the loop counter by 1
	      	 }
	      	Timer.delay(.5);
	      	 
	        aGyro.reset();
	        while (driveAngle < 32){ 
	        	driveAngle = aGyro.getAngle();
	      		chassis.drive(0.25, .35); 
	      	//chassis.tankDrive(.8,.8); //Sets the speed of the robot for each wheel and makes them turn 
	      	 Timer.delay(0.01);                      //creates a delay
	     	 //Increments the loop counter by 1
	      }
		
	        Timer.delay(.9);
	        
	      int loopCount1 = 0;  
	      while (loopCount1 < 42) {
	    	  chassis.drive(0.3, 0);
	    	  Timer.delay(.01);
	      loopCount1 = loopCount1 + 1;
		}
	      
	      Timer.delay(1);
	      
	      solGrip.set(true);
	      
	      Timer.delay(1);
	      
	     int loopCount2 = 0;
	     while (loopCount2 < 50) {
	    	 chassis.drive(-.3, 0);
	    	 loopCount2 = loopCount2 +1;
	    	 Timer.delay(.01);
	     }
	      
		}
		
            
		else if(mode == 4) {
			 aGyro.reset(); //Gyro reset prior to movement for re-calibration
	    	 chassis.setSafetyEnabled(true); //Sets the safety mode to enabled
	    	 
	    	 solGrip.set(false);
	    	 
	    	 int loopCount = 0; // Declares and assigns a zero
	      	 while (isAutonomous() && isEnabled() && loopCount < 280){    //Checks to see if autonomous is enabled
	      		driveAngle = aGyro.getAngle(); 
	      		chassis.drive(0.35,-driveAngle * scalingConstant);
	      	//chassis.tankDrive(.8,.8); //Sets the speed of the robot for each wheel and makes them turn
	      	 Timer.delay(0.01);                      //creates a delay
	     	 loopCount = loopCount + 1; //Increments the loop counter by 1
	      }
	      	 Timer.delay(.1); 
	      	 
	      aGyro.reset();
	        while (driveAngle > -32 ){
	        	driveAngle = aGyro.getAngle();
	      		chassis.arcadeDrive(-0.00, .5);
	      	//chassis.tankDrive(.8,.8); //Sets the speed of the robot for each wheel and makes them turn 
	      	 Timer.delay(0.01);                      //creates a delay
	     	 
	      }
		
            
	        int loopCount1 = 0;  
		      while (loopCount1 < 35) {
		    	  chassis.drive(0.3, 0);
		    	  Timer.delay(.01);
		      loopCount1 = loopCount1 + 1;
			}
		      
		      Timer.delay(1);
		      
		      solGrip.set(true);
		      
		      Timer.delay(1);
		      
		     int loopCount2 = 0;
		     while (loopCount2 < 50) {
		    	 chassis.drive(-.3, 0);
		    	 loopCount2 = loopCount2 +1;
		    	 Timer.delay(.01);
		     }
		}

		
		else if(mode == 5){
        aGyro.reset(); //Gyro reset prior to movement for re-calibration
	    chassis.setSafetyEnabled(true); //Sets the safety mode to enabled
	    
	    int loopCount1 = 0; // Declares and assigns a zero
     	 while (isAutonomous() && isEnabled() && loopCount1 < 205){    //Checks to see if autonomous is enabled
     		driveAngle = aGyro.getAngle(); 
     		chassis.drive(0.35,-driveAngle * scalingConstant);
     	//chassis.tankDrive(.8,.8); //Sets the speed of the robot for each wheel and makes them turn
     	 Timer.delay(0.01);                      //creates a delay
    	 loopCount1 = loopCount1 + 1; //Increments the loop counter by 1
     }
     	aGyro.reset();
        while (driveAngle > -32 ){
        	driveAngle = aGyro.getAngle();
      		chassis.arcadeDrive(-0.00, .5);
      	//chassis.tankDrive(.8,.8); //Sets the speed of the robot for each wheel and makes them turn 
      	 Timer.delay(0.01);                      //creates a delay
      }
     	 Timer.delay(1);
     	 
     	 int loopCount2 = 0;
     	while (loopCount2 < 200){
     		chassis.arcadeDrive(-.5, -.01);
     		Timer.delay(.01);
        loopCount2 = loopCount2 + 1;
			}
         Timer.delay(.5);
     
     	}
	}

		
 
	 
	 
	/**
	 * Runs the motors with tank steering.
	 */
	
	 @Override
	public void operatorControl() {     // starts teleop period
		 aGyro.reset();
		 chassis.setSafetyEnabled(true);         //enables motor prevention safety device 
		 int speed;//new
		driveAngle = aGyro.getAngle();
		while (isOperatorControl() && isEnabled()){       // while teleop is enabled do the code below
			SmartDashboard.putNumber("Gyro", aGyro.getAngle());
			if (xboxController.getRawButton(6)){        //Sets controller axis button to RT making the Arm move Down
				 //SmartDashboard.putString("DB/String 1", "Arm Up");
				 SmartDashboard.putBoolean(ArmUp, false);
					//SmartDashboard.putString(ArmUp, "solArm");
				solArm.set(false);
			 }
			  
			if (xboxController.getRawAxis(3) > .35){             //Sets controller button to RB making the Arm move up
				//SmartDashboard.putString("DB/String 1", "Arm Down");
				SmartDashboard.putBoolean(ArmDown, true);
				//SmartDashboard.putString(ArmDown, "solArm");
				solArm.set(true);
			}
			
			if (xboxController.getRawAxis(2) > .35 ){       // Sets controller button axis to LT making the gripper open
				//SmartDashboard.putString("DB/String 0", "Gripper Close"); 
				solGrip.set(false);
				SmartDashboard.putBoolean("GripperClose", false);
				//SmartDashboard.putString("GripperClose", "solGrip");
				
			 }
			
			if (xboxController.getRawButton(5)){             //Sets controller button LB making the gripper close
				//SmartDashboard.putString("DB/String 0", "Gripper Open");
				solGrip.set(true);
				SmartDashboard.putBoolean(GripperOpen, true);
				//SmartDashboard.putString(GripperOpen, "solGrip");
				
				}
			
			if (xboxController.getRawButton(4)){ //set "forward" to controller button "a"
			solBall.set(DoubleSolenoid.Value.kForward);
			}
			
			if (xboxController.getRawButton(1)){ //set "reverse" to controller button "y"
				solBall.set(DoubleSolenoid.Value.kReverse);
	}
			
			if (winchController.getRawButton(2)) {
				int loopCount1 = 0;  
			      while (loopCount1 < 42) {
			    	  chassis.drive(0.3, 0);
			    	  Timer.delay(.01);
			      loopCount1 = loopCount1 + 1;
				}
			      Timer.delay(.5);
			      solGrip.set(true);
			      Timer.delay(.5);
			      solArm.set(true);
			}
			
		//CHECK CHECK CHECK CHECK to see if the comments match the code make sure else climbmotor set to 0
		  if(winchController.getRawButton(1)) { // gets the controller button to back upnleft thing
    	     // set to true 
			climbmotor.set(1); //set to one 
			  } 
       else{ climbmotor.set(0); //Must be set to zero or else!!!!!!!!!!!!!!!!!!!!crunch break boom off the team!!!!!!!!
       }
		//Orgininal, chassis.arcadeDrive(-xboxController.getRawAxis(1), -xboxController.getRawAxis(4),false);{}//sets the arcade drive forward backward to joystick one and left right to joystic 2
		
		speed = (int) speedControl.getSelected();
		if (speed == 1){
		chassis.arcadeDrive(-xboxController.getRawAxis(1), -xboxController.getRawAxis(4),false);{ //sets the arcade drive forward backward to joystick one and left right to joystic 2
		}
		}
		else if(speed ==2){
			chassis.arcadeDrive(-xboxController.getRawAxis(1)/2, -xboxController.getRawAxis(4)/2,false);{ //sets the arcade drive forward backward to joystick one and left right to joystic 2 to change the speed change the division operator 
		}
		}
		Timer.delay(0.005); // wait for a motor update time	
		}
	 }
}

