/**
 * Change middle code to run from either the left or right side of boxes so we can get to baseline
 **/

package org.usfirst.frc.team447.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.wpilibj.PIDController;
import org.usfirst.frc.team447.customPID.*;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.analog.adis16448.frc.ADIS16448_IMU;

import org.usfirst.frc.team447.errorChecking.*;

@SuppressWarnings("unused")
public class Robot extends IterativeRobot {

	Spark leftMotor = new Spark(1);
	Spark rightMotor = new Spark(0);
	Spark liftMotor = new Spark(2);
	Spark grabberMotor = new Spark(3);

	DigitalInput grabStop = new DigitalInput(6);

	DifferentialDrive myRobot = new DifferentialDrive(leftMotor, rightMotor); // class that handles basic drive\

	Joystick leftStick = new Joystick(0); // set to ID 2 in DriverStation
	Joystick rightStick = new Joystick(1); // set to ID 1 in DriverStation
	Joystick operator = new Joystick(2);
	Joystick autoSelect = new Joystick(3);

	Timer timer = new Timer();

	DoubleSolenoid transmission = new DoubleSolenoid(0, 1);
	DoubleSolenoid grabberArmExt = new DoubleSolenoid(2, 3);
	DoubleSolenoid grabberArmGrab = new DoubleSolenoid(4, 5);
	DoubleSolenoid climbLock = new DoubleSolenoid(6, 7);
	Thread visionThread;

	Encoder rightDriveEncode = new Encoder(0, 1);
	Encoder leftDriveEncode = new Encoder(2, 3);
	Encoder liftEncode = new Encoder(4, 5);

	double rDrvEncSpd;
	double lDrvEncSpd;
	double decreaseFactor;

	String gameData = "---";
	String selectedAuto = null;

	DriverStation driveStation;

	// ADXRS450_Gyro gyroBoard = new ADXRS450_Gyro();
	ADIS16448_IMU IMU = new ADIS16448_IMU();

	double gyroAngle;
	
	double crossMoveDist;
	double firstMoveGyro;
	double firstMoveEncode;

	static Boolean leftStepDone = false;
	static Boolean rightStepDone = false;
	static Boolean liftStepDone = false;
	static Boolean gyroSetpointDone = false;
	static boolean practiceRobot = true; // Change Before Competition

	static Integer autoStepNum;
	static Integer liftLevel;
	
	static double gyroCount = 0;
	static double carpetCorrection = 634;
	static double leftTargetDist = 3900 /*4693 - carpetCorrection*/; //Left Position Start
	static double rightTargetDist = 3650 /*4693 - carpetCorrection*/; //Right Position Start

	// For movement between robots.
	static double liftSpeed = 1;
	static double driveMultiplier = 1;
	static double grabberInputSpeed = 0.5;
	static double grabberOutputSpeed = 0.35;
	static double liftSpeedOffset = 0;

	PowerDistributionPanel PDP = new PowerDistributionPanel();

	// averagedGyroPIDSource PIDGyroInput = new averagedGyroPIDSource(gyroBoard,
	// IMU);
	// averagedGyroPIDOutput PIDGyroOutput = new averagedGyroPIDOutput();

	// TODO Fix PID controller 
	// PIDController driveToEncodePID = new PIDController(0.1, 0.001, 0.0,
	// PIDGyroInput, PIDGyroOutput);
	// PIDController turnToGyroPID = new PIDController(01, 0.001, 0.01, gyroBoard,
	// PIDGyroOutput);

	@Override
	public void robotInit() {
		gameData = "---";
		climbLock.set(DoubleSolenoid.Value.kForward);
		// gyroBoard.calibrate();
		IMU.calibrate();
		// turnToGyroPID.setAbsoluteTolerance(1);

		visionThread = new Thread(() -> {
			// Get the UsbCamera from CameraServer
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// Set the resolution
			camera.setResolution(160, 120);
			camera.setBrightness(10);
			camera.setFPS(10);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Video", 640, 480);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat. If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}

				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});

		visionThread.setDaemon(true);
		visionThread.start();
	}

	public void autonomousInit() {
		climbLock.set(DoubleSolenoid.Value.kForward);
		leftStepDone = false;
		rightStepDone = false;
		gyroSetpointDone = false;
		autoStepNum = 0;
		decreaseFactor = 0;
		// gyroBoard.reset();
		IMU.reset();
		leftDriveEncode.reset();
		rightDriveEncode.reset();
		timer.reset();
		timer.start();
	}

	@Override
	public void teleopInit() {
		climbLock.set(DoubleSolenoid.Value.kForward);
		leftDriveEncode.reset();
		rightDriveEncode.reset();
	}

	@Override
	public void disabledInit() {
		System.out.println("Someday, we'll know what we're doing.");
		climbLock.set(DoubleSolenoid.Value.kForward);
	}

	@Override
	public void robotPeriodic() {
		// gyroAngle = (gyroBoard.getAngle()+IMU.getAngleZ())/2;
		// gyroAngle = gyroBoard.getAngle();
		gyroAngle = IMU.getAngleZ();
		//SmartDashboard.putNumber("IMU", IMU.getAccelY());
		//System.out.println("IMU: "+ IMU.getAccelY());
		SmartDashboard.putNumber("Gyro", gyroAngle);
		//System.out.println("Gyro: " + gyroAngle);
		SmartDashboard.putData("Drive", myRobot);
		SmartDashboard.putData("PDP", PDP);
		//System.out.println(selectedAuto);		
		SmartDashboard.putString("Game Data", gameData);
		SmartDashboard.putData("Left Encode", leftDriveEncode);
		SmartDashboard.putData("Right Encode", rightDriveEncode);
		
		try {
			if (autoSelect.getRawButton(3)) {
				if (autoSelect.getRawButton(4)) {

					if (gameData.charAt(1) == 'R') {

						if (autoSelect.getRawButton(2)) {

							selectedAuto = "rightScaleRight";

						} else if (!autoSelect.getRawButton(2)&&!autoSelect.getRawButton(1)) {

							selectedAuto = "midScaleRight";

						} else if (autoSelect.getRawButton(1)) {

							selectedAuto = "baseline";

						}
					} else if (gameData.charAt(1) == 'L') {

						if (autoSelect.getRawButton(2)) {

							selectedAuto = "baseline";

						} else if (!autoSelect.getRawButton(2)&&!autoSelect.getRawButton(1)) {

							selectedAuto = "midScaleLeft";

						} else if (autoSelect.getRawButton(1)) {

							selectedAuto = "leftScaleLeft";

						}
					}
				} else if (autoSelect.getRawButton(5)) {

					if (gameData.charAt(0) == 'R') {

						if (autoSelect.getRawButton(2)) {

							selectedAuto = "rightSwitchRight";

						} else if (!autoSelect.getRawButton(2)&&!autoSelect.getRawButton(1)) {

							selectedAuto = "midSwitchRight";

						} else if (autoSelect.getRawButton(1)) {

							selectedAuto = "baseline";

						}

					} else if (gameData.charAt(0) == 'L') {

						if (autoSelect.getRawButton(2)) {

							selectedAuto = "baseline";

						} else if (!autoSelect.getRawButton(2)&&!autoSelect.getRawButton(1)) {

							selectedAuto = "midSwitchLeft";

						} else if (autoSelect.getRawButton(1)) {

							selectedAuto = "leftSwitchLeft";

						}
					}
				} else {
					selectedAuto = null;
				}	
			} else {
				if (autoSelect.getRawButton(4)) {

					if (gameData.charAt(1) == 'R') {

						if (autoSelect.getRawButton(2)) {

							selectedAuto = "rightScaleRight";

						} else if (!autoSelect.getRawButton(2)&&!autoSelect.getRawButton(1)) {

							selectedAuto = "midScaleRight";

						} else if (autoSelect.getRawButton(1)) {

							selectedAuto = "leftScaleRight";

						}
					} else if (gameData.charAt(1) == 'L') {

						if (autoSelect.getRawButton(2)) {

							selectedAuto = "rightScaleLeft";

						} else if (!autoSelect.getRawButton(2)&&!autoSelect.getRawButton(1)) {

							selectedAuto = "midScaleLeft";

						} else if (autoSelect.getRawButton(1)) {

							selectedAuto = "leftScaleLeft";

						}
					}
				} else if (autoSelect.getRawButton(5)) {

					if (gameData.charAt(0) == 'R') {

						if (autoSelect.getRawButton(2)) {

							selectedAuto = "rightSwitchRight";

						} else if (!autoSelect.getRawButton(2)&&!autoSelect.getRawButton(1)) {

							selectedAuto = "midSwitchRight";

						} else if (autoSelect.getRawButton(1)) {

							selectedAuto = "leftSwitchRight";

						}

					} else if (gameData.charAt(0) == 'L') {

						if (autoSelect.getRawButton(2)) {

							selectedAuto = "rightSwitchLeft";

						} else if (!autoSelect.getRawButton(2)&&!autoSelect.getRawButton(1)) {

							selectedAuto = "midSwitchLeft";

						} else if (autoSelect.getRawButton(1)) {

							selectedAuto = "leftSwitchLeft";

						}
					}
				} else {
					selectedAuto = null;
				}	
			}

		} catch (Exception e) {
			selectedAuto = "baseline";
		}
		
		// SmartDashboard.putString("Selected Auto", selectedAuto);

	}

	@Override
	public void disabledPeriodic() {
		SmartDashboard.putBoolean("Enabled", false);
		try {
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		} catch (Exception e) {
			gameData = "---";
		}
	}

	@Override
	public void teleopPeriodic() {
		SmartDashboard.putBoolean("Enabled", true);
		myRobot.setSafetyEnabled(true);
		
		System.out.println("left encode count: "+leftDriveEncode.get() + "\nright encode count: " + rightDriveEncode.get() + "\naveraged count: " + (leftDriveEncode.get()+rightDriveEncode.get())/2);

		// lDrvEncSpd = leftDriveEncode.getRate();
		// rDrvEncSpd = rightDriveEncode.getRate();

		
		if(leftStick.getRawButton(1)&&leftStick.getRawButton(2)) {
			myRobot.tankDrive(-(rightStick.getY() * driveMultiplier), -(leftStick.getY() * driveMultiplier));
		} else {
			myRobot.tankDrive(leftStick.getY() * driveMultiplier, rightStick.getY() * driveMultiplier);
		}
		
		// transmission
		if (rightStick.getRawButton(1)) {
			transmission.set(DoubleSolenoid.Value.kForward);
		} else if (rightStick.getRawButton(2)) {
			transmission.set(DoubleSolenoid.Value.kReverse);
		}

		// lift
		if (true) {
			// for testing
			if (operator.getPOV() == 0) {
				liftToEncode(2900000);
			} else if (operator.getPOV() == 180) {
				lowerToEncode(-2000000);
			} else if (operator.getRawButton(10)) {
				liftToEncode(1214);
			} else if (operator.getRawButton(9)) {
				lowerToEncode(1214);
			} else {
				liftMotor.set(0);
			}

		} else {
			switch (liftLevel) {
			case 0:
				if (liftEncode.get() > 10) {
					liftMotor.set(-liftSpeed);
				} else if (liftEncode.get() < 10) {
					liftMotor.set(liftSpeed);
				}
				break;
			case 1:
				if (liftEncode.get() > 200) {
					liftMotor.set(-liftSpeed);
				} else if (liftEncode.get() < 200) {
					liftMotor.set(liftSpeed);
				}
				break;
			}
		}
		// Grabber

		// System.out.println("Grab Stop: "+grabStop.get());

		if (operator.getRawButton(7) && grabStop.get()) {
			grabberMotor.set(-grabberInputSpeed);

		} else if (operator.getRawButton(8)) {
			grabberMotor.set(grabberOutputSpeed);
		} else {
			grabberMotor.set(0);
		}

		if (operator.getRawButton(5)) {
			grabberArmExt.set(DoubleSolenoid.Value.kReverse);
		} else if (operator.getRawButton(6)) {
			grabberArmExt.set(DoubleSolenoid.Value.kForward);
		}

		if (operator.getRawButton(1)) {
			grabberArmGrab.set(DoubleSolenoid.Value.kReverse);
		} else if (operator.getRawButton(4)) {
			grabberArmGrab.set(DoubleSolenoid.Value.kForward);
		}

		if (operator.getRawButton(11)) {
			climbLock.set(DoubleSolenoid.Value.kForward);
		} else if (operator.getRawButton(12)) {
			climbLock.set(DoubleSolenoid.Value.kReverse);
		}

		// System.out.println("Lift Encode: "+liftEncode.get());

		/**
		 * //Automatic transmission
		 * 
		 * if((lDrvEncSpd>1000||lDrvEncSpd<-1000)&&(rDrvEncSpd>1000||rDrvEncSpd<-1000))
		 * { transmission.set(DoubleSolenoid.Value.kForward); } else if
		 * ((lDrvEncSpd<400&&lDrvEncSpd>-400)||(rDrvEncSpd<400&&rDrvEncSpd>-400)) {
		 * transmission.set(DoubleSolenoid.Value.kReverse); }
		 **/

	}

	@Override
	public void autonomousPeriodic() {
		try {
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		} catch (Exception e) {
			gameData = "---";
		}

		if (!autoSelect.getRawButton(4) && !autoSelect.getRawButton(5)) {
			if (!autoSelect.getRawButton(2)&&!autoSelect.getRawButton(1)) {
				// right

				switch (gameData) {
				case "RRR":
					rightSwitchRight();
					break;

				case "RLR":
					rightSwitchRight();
					break;

				case "LLL":
					baseline();
					break;

				case "LRL":
					rightScaleRight();
					break;
				}

			} else if (autoSelect.getRawButton(2)) {
				// mid

				switch (gameData) {
				case "RRR":
					middleSwitchRight();
					break;

				case "RLR":
					middleSwitchRight();
					break;

				case "LLL":
					middleSwitchLeft();
					break;

				case "LRL":
					middleSwitchLeft();
					break;

				default:
					baseline();
					break;
				}

			} else if (autoSelect.getRawButton(1)) {
				// left

				switch (gameData) {
				case "RRR":
					baseline();
					break;

				case "RLR":
					leftScaleLeft();
					break;

				case "LLL":
					leftSwitchLeft();
					break;

				case "LRL":
					leftSwitchLeft();
					break;

				default:
					baseline();
					break;
				}

			} else {
				baseline();
			}
		} else {
			// scale
			// System.out.println(selectedAuto);
			switch (selectedAuto) {
			case "rightScaleRight":
				rightScaleRight();
				break;

			case "rightScaleLeft":
				rightScaleLeft();
				break;

			case "midScaleRight":
				middleScaleRight();
				break;

			case "midScaleLeft":
				middleScaleLeft();
				break;

			case "leftScaleRight":
				leftScaleRight();
				break;

			case "leftScaleLeft":
				leftScaleLeft();
				break;

			case "rightSwitchRight":
				rightSwitchRight();
				break;

			case "rightSwitchLeft":
				rightSwitchLeft();
				break;

			case "midSwitchRight":
				middleSwitchRight();
				break;

			case "midSwitchLeft":
				middleSwitchLeft();
				break;

			case "leftSwitchRight":
				leftSwitchRight();
				break;

			case "leftSwitchLeft":
				leftSwitchLeft();
				break;

			default:
				baseline();
				break;
			}
		}

		// SmartDashboard.putString("Selected Auto", selectedAuto);
		// SmartDashboard.putNumber("Step Num", autoStepNum);

		if (leftStepDone && rightStepDone && liftStepDone) {

			System.out.println(autoStepNum + "\n" + leftStepDone + "\n" + rightStepDone);

			System.out.println("Done, resetting");

			leftStepDone = false;
			rightStepDone = false;
			liftStepDone = false;
			// turnToGyroEnd();
			System.out.println("Done, incrementing");
			gyroCount = gyroAngle;
			autoStepNum += 1;
			leftDriveEncode.reset();
			rightDriveEncode.reset();
			timer.reset();
			timer.start();
		}
	}

	private void baseline() {
		transmission.set(DoubleSolenoid.Value.kForward);

		// Using if/else as opposed to a bunch of if statements without else, that way
		// the roboRIO doesn't have to process as much code if it is on the early steps

		if (autoStepNum == 0) {
			grabberArmExt.set(DoubleSolenoid.Value.kReverse);
			grabberArmGrab.set(DoubleSolenoid.Value.kReverse);
			grabberMotor.set(0);
			rightStepDone = leftStepDone = liftStepDone = true;
		} else if (autoStepNum == 1) {
			driveToEncode(1800, 1800, 0.6);
		}

		/*
		 * //Testing if (autoStepNum == 0) {
		 * 
		 * driveToEncode(280, -280); }
		 */

	}

	private void rightSwitchRight() {
		transmission.set(DoubleSolenoid.Value.kForward);

		// Using if/else as opposed to a bunch of if statements without else, that way
		// the roboRIO doesn't have to process as much code if it is on the early steps
		if (autoStepNum == 0) {
			grabberArmExt.set(DoubleSolenoid.Value.kReverse);
			grabberArmGrab.set(DoubleSolenoid.Value.kReverse);
			grabberMotor.set(0);
			rightStepDone = leftStepDone = liftStepDone = true;
		} else if (autoStepNum == 1) {
			liftToEncode(4300);
			driveToEncode(2300 - carpetCorrection, 2300 - carpetCorrection, 0.60); //adjusting for carpet
		} else if (autoStepNum == 2) {
			liftStepDone = true;
			turnToGyro(gyroCount-35, "Left");
		} else if(autoStepNum == 3) {
			if(correctTurn(-90, "Left")) liftStepDone = rightStepDone = leftStepDone = true;
		} else if (autoStepNum == 4) {
			driveToEncode(400, 400, 0.60);
			if(leftDriveEncode.getRate()==0&&rightDriveEncode.getRate()==0) {
				leftStepDone = rightStepDone = true;
			}
			liftStepDone = true;
		} else if (autoStepNum == 5) {
			grabberArmGrab.set(DoubleSolenoid.Value.kForward);
			grabberMotor.set(0.6);
		}

	}

	private void rightSwitchLeft() {
		if (autoStepNum == 0) {
			grabberArmExt.set(DoubleSolenoid.Value.kReverse);
			grabberArmGrab.set(DoubleSolenoid.Value.kReverse);
			transmission.set(DoubleSolenoid.Value.kForward);
			grabberMotor.set(0);
			rightStepDone = leftStepDone = liftStepDone = true;
		} else if (autoStepNum == 1) {

			driveToEncode(3650 - carpetCorrection, 3650 - carpetCorrection, 0.6); // accounting for carpet length
			liftStepDone = true;

		} else if (autoStepNum == 2) {
			transmission.set(DoubleSolenoid.Value.kReverse);
			driveToEncode(-1, -1, 0.2);
			rightStepDone = leftStepDone = liftStepDone = true;

		} else if (autoStepNum == 3) {
			turnToGyro(-42, "Left");
			// driveToEncode(-280, 280);
			liftStepDone = true;

		} else if (autoStepNum == 4) {
			transmission.set(DoubleSolenoid.Value.kForward);
			driveToEncode(3535, 3535, 0.60);
			liftStepDone = true;

		} else if (autoStepNum == 5) {
			transmission.set(DoubleSolenoid.Value.kReverse);
			driveToEncode(-1, -1, 0.2);
			rightStepDone = leftStepDone = liftStepDone = true;

		} else if (autoStepNum == 6) {

			turnToGyro(gyroCount-50, "Left");
			liftStepDone = true;

		} else if (autoStepNum == 7) {
			transmission.set(DoubleSolenoid.Value.kForward);
			driveToEncode(800, 800, 0.60);

			liftStepDone = true;

		} else if (autoStepNum == 8) {
			transmission.set(DoubleSolenoid.Value.kForward);

			turnToGyro(gyroCount-53, "Left");
			liftToEncode(1500);

		} else if (autoStepNum == 9) {
			driveToEncode(400, 400, 0.6);
			if(leftDriveEncode.getRate()==0&&rightDriveEncode.getRate()==0) {
				leftStepDone = rightStepDone = true;
			}
			liftStepDone = true;
		} else if (autoStepNum == 10) {

			transmission.set(DoubleSolenoid.Value.kForward);
			grabberArmGrab.set(DoubleSolenoid.Value.kReverse);
			grabberMotor.set(.60);

		}
	}

	private void rightScaleRight() {
		transmission.set(DoubleSolenoid.Value.kForward);

		System.out.println(gyroAngle);
		// System.out.println(leftDriveEncode.get()+"\n"+rightDriveEncode.get());

		if (autoStepNum == 0) {
			grabberArmExt.set(DoubleSolenoid.Value.kReverse);
			grabberArmGrab.set(DoubleSolenoid.Value.kReverse);
			grabberMotor.set(0);
			rightStepDone = leftStepDone = liftStepDone = true;
		} else if (autoStepNum == 1) {

			driveToEncode(4740 - carpetCorrection, 4740 - carpetCorrection, 0.60);
			System.out.println(liftEncode.get());
			if (leftDriveEncode.get() > 4500) {
				liftToEncode(18000);
			}
			if (leftDriveEncode.get() % 200 == 0) {
				decreaseFactor += 0.02;
				System.out.println(-0.5 + decreaseFactor);
			}

		} else if (autoStepNum == 2) {
			turnToGyro(-52, "Left");
			liftStepDone = true;
		}  else if(autoStepNum == 3) {
			driveToEncode(100, 100, 0.5);
			liftStepDone = true;
		} else if (autoStepNum == 4) {
			grabberMotor.set(0.7);
		}

	}


	private void rightScaleLeft() {
		
		double targetAngle = -90;
		
		if (autoStepNum == 0) { //reset values
			grabberArmExt.set(DoubleSolenoid.Value.kReverse);
			grabberArmGrab.set(DoubleSolenoid.Value.kReverse);
			transmission.set(DoubleSolenoid.Value.kForward);
			grabberMotor.set(0);
			liftMotor.set(0);
			firstMoveEncode = 0;
			crossMoveDist = 0;
			rightStepDone = leftStepDone = liftStepDone = true;
		} else if (autoStepNum == 1) { //move 1

			driveToEncode(3600 - carpetCorrection, 3600 - carpetCorrection, 0.6); // accounting for carpet length
			firstMoveEncode = (leftDriveEncode.get() + rightDriveEncode.get())/2;
			if(firstMoveEncode > 3300) transmission.set(DoubleSolenoid.Value.kReverse);
			liftStepDone = true;

		} else if (autoStepNum == 2) { //stop
			transmission.set(DoubleSolenoid.Value.kReverse);
			driveToEncode(-1, -1, 0.2);
			rightStepDone = leftStepDone = liftStepDone = true;

		} else if (autoStepNum == 3) { //turn 1

			turnToGyro(-52, "Left");
			firstMoveGyro = gyroAngle;
			liftStepDone = true;

		} else if (autoStepNum == 4) { //move 2
			transmission.set(DoubleSolenoid.Value.kForward);
			
			System.out.println("Current Angle: " + gyroAngle + "\nLogged Angle: " + gyroCount);
			if(ErrorChecking.safeGyroCheck(firstMoveGyro, targetAngle, 5)) { //make sure we don't run into boxes
				driveToEncode(2976, 2976, 0.60);
				crossMoveDist = (leftDriveEncode.get() + rightDriveEncode.get())/2;
				liftStepDone = true;
			} else {
				
				if(Math.abs(gyroAngle) < Math.abs(targetAngle)) {
					turnToGyro(gyroAngle - 1, "Left", 0.3);
				} else if(Math.abs(gyroAngle) > Math.abs(targetAngle)) {
					turnToGyro(gyroAngle + 1, "Right", 0.3);
				}
				
				firstMoveGyro = gyroAngle;
				
			}

		} else if (autoStepNum == 5) { //stop
			transmission.set(DoubleSolenoid.Value.kReverse);
			driveToEncode(-1, -1, 0.2);
			rightStepDone = leftStepDone = liftStepDone = true;

		} else if (autoStepNum == 6) { //turn 2

			turnToGyro(gyroCount+54, "Right");
			liftStepDone = true;

		} else if (autoStepNum == 7) { // lift
			
			transmission.set(DoubleSolenoid.Value.kReverse);
			System.out.println(gyroAngle);
			if(ErrorChecking.safeGyroCheck(gyroCount, 0, 10)) { 
				liftToEncode(18000);
				leftStepDone = rightStepDone = true;
			}
			

		} else if(autoStepNum == 8) { //move 3
			
			double finalMove = ErrorChecking.getFinalMove(crossMoveDist, rightTargetDist, firstMoveEncode, firstMoveGyro, targetAngle);
			
			transmission.set(DoubleSolenoid.Value.kForward);
			
			System.out.println("Final Move: "+finalMove);
			driveToEncode(finalMove, finalMove /*300, 300*/, 0.60);

			liftStepDone = true;
		}  else if (autoStepNum == 9) { //stop
			transmission.set(DoubleSolenoid.Value.kReverse);
			driveToEncode(-1, -1, 0.2);
			rightStepDone = leftStepDone = liftStepDone = true;

		}else if(autoStepNum == 10) { // correct if next to ramp
			turnToGyro(gyroCount + 2, "Right");
			grabberArmExt.set(DoubleSolenoid.Value.kForward);
			liftStepDone = true;
		} else if (autoStepNum == 11) { // shoot

			transmission.set(DoubleSolenoid.Value.kForward);
			grabberMotor.set(.50);
			
			liftStepDone = leftStepDone = rightStepDone = true;

		} else if (autoStepNum == 12) { //move back
			
			driveToEncode(-400, -400, 0.5);
			liftStepDone = true;
		}

		
	}

	private void middleSwitchRight() {
		transmission.set(DoubleSolenoid.Value.kForward);
		double firstTurnGyro;

		if (autoStepNum == 0) {
			grabberArmExt.set(DoubleSolenoid.Value.kReverse);
			grabberArmGrab.set(DoubleSolenoid.Value.kReverse);
			grabberMotor.set(0);
			liftMotor.set(0);
			rightStepDone = leftStepDone = liftStepDone = true;
		} else if (autoStepNum == 1) {
			driveToEncode(600, 600, 0.50);
			liftStepDone = true;
		} else if (autoStepNum == 2) {
			turnToGyro(gyroCount+23, "Right");
			liftStepDone = true;
		} else if (autoStepNum == 3) {
			
			if(correctTurn(90, "Right")) liftStepDone = rightStepDone = leftStepDone = true;
				
		} else if (autoStepNum == 4) {
			driveToEncode(590, 590, 0.50);
			liftStepDone = true;
		} else if (autoStepNum == 5) {
			turnToGyro(gyroCount-23, "Left");
			liftStepDone = true;
		} else if(autoStepNum == 6)	{
			if(correctTurn(0, "Left")) liftStepDone = rightStepDone = leftStepDone = true;
		} else if (autoStepNum == 7) {
			driveToEncode(850, 850, 0.50);
			if(leftDriveEncode.getRate()==0&&rightDriveEncode.getRate()==0) {
				leftStepDone = rightStepDone = true;
			}
			liftToEncode(3700); // halve for competition bot
		} else if (autoStepNum == 8) {
			grabberArmGrab.set(DoubleSolenoid.Value.kForward);
			grabberArmExt.set(DoubleSolenoid.Value.kForward);
			grabberMotor.set(0.6);
		}
	}

	private void middleSwitchLeft() {
		transmission.set(DoubleSolenoid.Value.kForward);
		double firstTurnGyro;

		if (autoStepNum == 0) {
			grabberArmExt.set(DoubleSolenoid.Value.kReverse);
			grabberArmGrab.set(DoubleSolenoid.Value.kReverse);
			grabberMotor.set(0);
			liftMotor.set(0);
			rightStepDone = leftStepDone = liftStepDone = true;
		} else if (autoStepNum == 1) {
			driveToEncode(600, 600, 0.50);
			liftStepDone = true;
		} else if (autoStepNum == 2) {
			turnToGyro(gyroCount-23, "Left");
			liftStepDone = true;
		} else if (autoStepNum == 3) {
			
			if(correctTurn(-90, "Left")) liftStepDone = rightStepDone = leftStepDone = true;
				
		} else if (autoStepNum == 4) {
			driveToEncode(600, 600, 0.50);
			liftStepDone = true;
		} else if (autoStepNum == 5) {
			turnToGyro(gyroCount+23, "Right");
			liftStepDone = true;
		} else if(autoStepNum == 6)	{
			if(correctTurn(0, "Right")) liftStepDone = rightStepDone = leftStepDone = true;
		} else if (autoStepNum == 7) {
			driveToEncode(850, 850, 0.50);
			if(leftDriveEncode.getRate()==0&&rightDriveEncode.getRate()==0) {
				leftStepDone = rightStepDone = true;
			}
			liftToEncode(3700); // halve for competition bot
		} else if (autoStepNum == 8) {
			grabberArmGrab.set(DoubleSolenoid.Value.kForward);
			grabberArmExt.set(DoubleSolenoid.Value.kForward);
			grabberMotor.set(0.6);
		}
	}

	private void middleScaleRight() {
		transmission.set(DoubleSolenoid.Value.kForward);

		if (autoStepNum == 0) {
			grabberArmExt.set(DoubleSolenoid.Value.kReverse);
			grabberArmGrab.set(DoubleSolenoid.Value.kReverse);
			grabberMotor.set(0);
			rightStepDone = leftStepDone = liftStepDone = true;
		} else if (autoStepNum == 1) {
			driveToEncode(400, 400, 0.60);
			liftStepDone = true;
		} else if (autoStepNum == 2) {
			turnToGyro(gyroCount + 20, "Right"); //change to 45
			liftStepDone = true;
		} else if (autoStepNum == 3) {
			driveToEncode(1000, 1000, 0.60);
			liftStepDone = true;
		} else if (autoStepNum == 4) {
			turnToGyro(gyroCount - 17, "Left"); //change to 55
			System.out.println("Current Gyro: "+gyroCount+"\nTarget Gyro: "+(gyroCount-17));
			liftStepDone = true;
		} else if (autoStepNum == 5) {
			driveToEncode(2300, 2300, 0.60);
			liftStepDone = true;
		}

	}

	private void middleScaleLeft() {
		transmission.set(DoubleSolenoid.Value.kForward);

		if (autoStepNum == 0) {
			grabberArmExt.set(DoubleSolenoid.Value.kReverse);
			grabberArmGrab.set(DoubleSolenoid.Value.kReverse);
			grabberMotor.set(0);
			rightStepDone = leftStepDone = liftStepDone = true;
		} else if (autoStepNum == 1) {
			driveToEncode(400, 400, 0.60);
			liftStepDone = true;
		} else if (autoStepNum == 2) {
			turnToGyro(-75, "Left");
			liftStepDone = true;
		} else if (autoStepNum == 3) {
			driveToEncode(1000, 1000, 0.60);
			liftStepDone = true;
		} else if (autoStepNum == 4) {
			turnToGyro(-20, "Right");
			liftStepDone = true;
		} else if (autoStepNum == 5) {
			driveToEncode(2300, 2300, 0.60);
			liftStepDone = true;
		}
	}

	private void leftScaleLeft() {
		transmission.set(DoubleSolenoid.Value.kForward);

		System.out.println(gyroAngle);
		// System.out.println(leftDriveEncode.get()+"\n"+rightDriveEncode.get());

		if (autoStepNum == 0) {
			grabberArmExt.set(DoubleSolenoid.Value.kReverse);
			grabberArmGrab.set(DoubleSolenoid.Value.kReverse);
			grabberMotor.set(0);
			rightStepDone = leftStepDone = liftStepDone = true;
		} else if (autoStepNum == 1) {

			driveToEncode(4720 - carpetCorrection, 4720 - carpetCorrection, 0.60);
			System.out.println(liftEncode.get());
			if (leftDriveEncode.get() > 4500) {
				liftToEncode(18500);
			}
			if (leftDriveEncode.get() % 200 == 0) {
				decreaseFactor += 0.02;
				System.out.println(-0.5 + decreaseFactor);
			}

		} else if (autoStepNum == 2) {
			turnToGyro(52, "Right");
			liftStepDone = true;
		}  else if(autoStepNum == 3) {
			driveToEncode(100, 100, 0.3);
			liftStepDone = true;
		} else if (autoStepNum == 4) {
			grabberMotor.set(0.75);
		}

	}

	private void leftScaleRight() {
		
		double targetAngle = 90;
		
		if (autoStepNum == 0) { //reset values
			grabberArmExt.set(DoubleSolenoid.Value.kReverse);
			grabberArmGrab.set(DoubleSolenoid.Value.kReverse);
			transmission.set(DoubleSolenoid.Value.kForward);
			grabberMotor.set(0);
			liftMotor.set(0);
			rightStepDone = leftStepDone = liftStepDone = true;
		} else if (autoStepNum == 1) { //move 1

			driveToEncode(3600 - carpetCorrection, 3600 - carpetCorrection, 0.6); // accounting for carpet length
			firstMoveEncode = (leftDriveEncode.get() + rightDriveEncode.get())/2;
			if(firstMoveEncode > 3300) transmission.set(DoubleSolenoid.Value.kReverse);
			liftStepDone = true;

		} else if (autoStepNum == 2) { //stop
			transmission.set(DoubleSolenoid.Value.kReverse);
			driveToEncode(-1, -1, 0.2);
			rightStepDone = leftStepDone = liftStepDone = true;

		} else if (autoStepNum == 3) { //turn 1

			turnToGyro(53, "Right");
			firstMoveGyro = gyroAngle;
			liftStepDone = true;

		} else if (autoStepNum == 4) { //move 2
			transmission.set(DoubleSolenoid.Value.kForward);
			
			System.out.println(gyroAngle);
			if(ErrorChecking.safeGyroCheck(firstMoveGyro, targetAngle, 2)) { //make sure we don't run into boxes
				driveToEncode(2926, 2926, 0.60);
				crossMoveDist = (leftDriveEncode.get() + rightDriveEncode.get())/2;
				liftStepDone = true;
			} else {
				
				if(Math.abs(gyroAngle) < Math.abs(targetAngle)) {
					turnToGyro(gyroAngle + 1, "Right", 0.3);
				} else if(Math.abs(gyroAngle) > Math.abs(targetAngle)) {
					turnToGyro(gyroAngle - 1, "Left", 0.3);
				}
				
				firstMoveGyro = gyroAngle;
				
			}

		} else if (autoStepNum == 5) { //stop
			transmission.set(DoubleSolenoid.Value.kReverse);
			driveToEncode(-1, -1, 0.2);
			rightStepDone = leftStepDone = liftStepDone = true;

		} else if (autoStepNum == 6) { //turn 2

			turnToGyro(gyroCount-55, "Left");
			liftStepDone = true;

		} else if (autoStepNum == 7) { // lift
			
			transmission.set(DoubleSolenoid.Value.kReverse);
			System.out.println(gyroAngle);
			if(ErrorChecking.safeGyroCheck(gyroCount, 4, 10)) { 
				liftToEncode(18000);
				leftStepDone = rightStepDone = true;
			}
			

		} else if(autoStepNum == 8) { //move 3
			
			double finalMove = ErrorChecking.getFinalMove(crossMoveDist, leftTargetDist, firstMoveEncode, firstMoveGyro, targetAngle);
			
			transmission.set(DoubleSolenoid.Value.kForward);
			
			System.out.println("Final Move: "+finalMove);
			driveToEncode(finalMove, finalMove /*300, 300*/, 0.60);

			liftStepDone = true;
		}  else if (autoStepNum == 9) { //stop
			transmission.set(DoubleSolenoid.Value.kReverse);
			driveToEncode(-1, -1, 0.2);
			rightStepDone = leftStepDone = liftStepDone = true;

		}else if(autoStepNum == 10) { // correct if next to ramp
			turnToGyro(gyroCount - 2, "Left");
			grabberArmExt.set(DoubleSolenoid.Value.kForward);
			liftStepDone = true;
		} else if (autoStepNum == 11) { // shoot

			transmission.set(DoubleSolenoid.Value.kForward);
			grabberMotor.set(.50);
			
			liftStepDone = leftStepDone = rightStepDone = true;

		} else if (autoStepNum == 12) { //move back
			
			driveToEncode(-400, -400, 0.5);
			liftStepDone = true;
		}

		
	}

	private void leftSwitchLeft() {
		if (autoStepNum == 0) { //reset everything
			grabberArmExt.set(DoubleSolenoid.Value.kReverse);
			grabberArmGrab.set(DoubleSolenoid.Value.kReverse);
			transmission.set(DoubleSolenoid.Value.kForward);
			liftMotor.set(0);
			grabberMotor.set(0);
			rightStepDone = leftStepDone = liftStepDone = true;
		} else if (autoStepNum == 1) { //move 1
			liftToEncode(4300);
			driveToEncode(2300 - carpetCorrection, 2300 - carpetCorrection, 0.60); //adjusting for carpet
		} else if (autoStepNum == 2) { //turn 1
			liftStepDone = true;
			turnToGyro(gyroCount+65, "Right");
		} else if (autoStepNum == 3) { //move 2
			driveToEncode(300, 300, 0.60);
			if(leftDriveEncode.getRate()==0&&rightDriveEncode.getRate()==0) {
				leftStepDone = rightStepDone = true;
			}
			liftStepDone = true;
		} else if (autoStepNum == 4) { //shoot
			grabberArmGrab.set(DoubleSolenoid.Value.kForward);
			grabberArmExt.set(DoubleSolenoid.Value.kForward);
			grabberMotor.set(0.6);
		}

	}

	private void leftSwitchRight() { // reset everything
		if (autoStepNum == 0) { //reset values
			grabberArmExt.set(DoubleSolenoid.Value.kReverse);
			grabberArmGrab.set(DoubleSolenoid.Value.kReverse);
			transmission.set(DoubleSolenoid.Value.kForward);
			grabberMotor.set(0);
			liftMotor.set(0);
			rightStepDone = leftStepDone = liftStepDone = true;
		} else if (autoStepNum == 1) { //move 1

			driveToEncode(3700 - carpetCorrection, 3700 - carpetCorrection, 0.6); // accounting for carpet length
			liftStepDone = true;

		} else if (autoStepNum == 2) { //stop
			transmission.set(DoubleSolenoid.Value.kReverse);
			driveToEncode(-1, -1, 0.2);
			rightStepDone = leftStepDone = liftStepDone = true;

		} else if (autoStepNum == 3) { //turn 1

			turnToGyro(48, "Right");
			// driveToEncode(-280, 280);
			liftStepDone = true;

		} else if (autoStepNum == 4) { //move 2
			transmission.set(DoubleSolenoid.Value.kForward);
			driveToEncode(3905, 3905, 0.60);
			liftStepDone = true;

		} else if (autoStepNum == 5) { //stop
			transmission.set(DoubleSolenoid.Value.kReverse);
			driveToEncode(-1, -1, 0.2);
			rightStepDone = leftStepDone = liftStepDone = true;

		} else if (autoStepNum == 6) { //turn 2

			turnToGyro(gyroCount+48, "Right");
			liftStepDone = true;

		} else if (autoStepNum == 7) { //move 3
			transmission.set(DoubleSolenoid.Value.kForward);
			driveToEncode(800, 800, 0.60);

			liftStepDone = true;

		} else if (autoStepNum == 8) { //turn 3
			transmission.set(DoubleSolenoid.Value.kForward);

			turnToGyro(gyroCount+53, "Right");
			liftToEncode(1500);

		} else if (autoStepNum == 9) { //move 4
			driveToEncode(400, 400, 0.6);
			if(leftDriveEncode.getRate()==0&&rightDriveEncode.getRate()==0) {
				leftStepDone = rightStepDone = true;
			}
			liftStepDone = true;
		} else if (autoStepNum == 10) { //shoot

			transmission.set(DoubleSolenoid.Value.kForward);
			grabberArmGrab.set(DoubleSolenoid.Value.kForward);
			grabberMotor.set(0.6);

		}
	}

	private void driveToEncode(double encoderCountL, double encoderCountR, double speed) {

		if (encoderCountL > 0) {
			if (leftDriveEncode.get() < encoderCountL) {
				// drive forwards half speed
				leftMotor.set(-speed);
				leftStepDone = false;
			} else if (leftDriveEncode.get() > encoderCountL) {
				leftMotor.set(0); // stop robot
				leftStepDone = true;
			}
		} else if (encoderCountL < 0) {
			if (leftDriveEncode.get() > encoderCountL) {
				// drive forwards half speed
				leftMotor.set(speed);
				leftStepDone = false;
			} else if (leftDriveEncode.get() < encoderCountL) {
				leftMotor.set(0); // stop robot
				leftStepDone = true;
			}
		} else if (encoderCountL == 0) {
			leftMotor.set(0); // stop robot
			leftStepDone = true;
		}

		if (encoderCountR > 0) {
			if (rightDriveEncode.get() < encoderCountR) {
				// drive forwards half speed
				rightMotor.set(0.60);
				rightStepDone = false;
			} else if (rightDriveEncode.get() > encoderCountR) {
				rightMotor.set(0); // stop robot
				rightStepDone = true;
			}
		} else if (encoderCountR < 0) {
			if (rightDriveEncode.get() > encoderCountR) {
				// drive forwards half speed
				rightMotor.set(-0.60);
				rightStepDone = false;
			} else if (rightDriveEncode.get() < encoderCountR) {
				rightMotor.set(0); // stop robot
				rightStepDone = true;
			}
		} else if (encoderCountR == 0) {
			rightMotor.set(0); // stop robot
			rightStepDone = true;
		}

	}

	private void liftToEncode(double encoderCount) {
		System.out.println("Lift: "+liftEncode.get());
		if (liftEncode.get() < encoderCount) {
			// drive forwards half speed
			liftMotor.set(1);
			liftStepDone = false;
		} else if (liftEncode.get() > encoderCount) {
			liftMotor.set(0); // stop robot
			liftStepDone = true;
		}

	}

	private void lowerToEncode(double encoderCount) {
		if (liftEncode.get() > encoderCount) {
			// drive forwards half speed
			liftMotor.set(-0.8);
			liftStepDone = false;
		} else if (liftEncode.get() < encoderCount) {
			liftMotor.set(0); // stop robot
			liftStepDone = true;
		}

	}

	private void turnToGyro(double Angle, String Direction, double turnSpeed) {
		// System.out.println("Left Encode: \n"+leftDriveEncode.get()+"\nRight Encode:
		// \n"+rightDriveEncode.get());

		switch (Direction) {
		case "Right":
			if (gyroAngle < Angle) {
				// drive forwards half speed
				rightMotor.set(-turnSpeed);
				leftMotor.set(-turnSpeed);
				leftStepDone = false;
				rightStepDone = false;
			} else if (gyroAngle > Angle) {
				rightMotor.set(0); // stop robot
				leftMotor.set(0);
				if(leftDriveEncode.getRate() != 0&&rightDriveEncode.getRate() != 0) break;
				leftStepDone = true;
				rightStepDone = true;
			}
			break;
		case "Left":
			if (gyroAngle > Angle) {
				// drive forwards half speed
				rightMotor.set(turnSpeed);
				leftMotor.set(turnSpeed);
				leftStepDone = false;
				rightStepDone = false;
			} else if (gyroAngle < Angle) {
				rightMotor.set(0); // stop robot
				leftMotor.set(0);
				if(leftDriveEncode.getRate() != 0&&rightDriveEncode.getRate() != 0) break;
				leftStepDone = true;
				rightStepDone = true;
			}
			break;
		default:
			System.out.println("Please Specify turn direction");
		}
	}
	
	private void turnToGyro(double Angle, String Direction) {
		// System.out.println("Left Encode: \n"+leftDriveEncode.get()+"\nRight Encode:
		// \n"+rightDriveEncode.get());
		double turnSpeed = 0;
		
		/*if(transmission.get() == DoubleSolenoid.Value.kReverse) {
			turnSpeed = 0.7;
		} else {
			turnSpeed = 0.5;
		}*/
		
		turnSpeed = 1;

		switch (Direction) {
		case "Right":
			if (gyroAngle < Angle) {
				// drive forwards half speed
				rightMotor.set(-turnSpeed);
				leftMotor.set(-turnSpeed);
				leftStepDone = false;
				rightStepDone = false;
			} else if (gyroAngle > Angle) {
				rightMotor.set(0); // stop robot
				leftMotor.set(0);
				if(leftDriveEncode.getRate() != 0&&rightDriveEncode.getRate() != 0) break;
				leftStepDone = true;
				rightStepDone = true;
			}
			break;
		case "Left":
			if (gyroAngle > Angle) {
				// drive forwards half speed
				rightMotor.set(turnSpeed);
				leftMotor.set(turnSpeed);
				leftStepDone = false;
				rightStepDone = false;
			} else if (gyroAngle < Angle) {
				rightMotor.set(0); // stop robot
				leftMotor.set(0);
				if(leftDriveEncode.getRate() != 0&&rightDriveEncode.getRate() != 0) break;
				leftStepDone = true;
				rightStepDone = true;
			}
			break;
		default:
			System.out.println("Please Specify turn direction");
		}
	}
	
	private boolean correctTurn(double targetAngle, String direction) {
		
		double tolMin = targetAngle - 3;
		double tolMax = targetAngle + 3;
		
		if(targetAngle == 0) direction = "Center";
		
		System.out.println("Target Angle: "+targetAngle+"\nCurrent Angle: "+gyroAngle);
		switch(direction) {
			case "Right": 
				if(Math.abs(gyroAngle) < Math.abs(targetAngle)) {
					turnToGyro(gyroAngle + 1, "Right", 0.3);
				} else if(Math.abs(gyroAngle) > Math.abs(targetAngle)) {
					turnToGyro(gyroAngle - 1, "Left", 0.3);
				}
				break;
			
			case "Left": 
				if(Math.abs(gyroAngle) < Math.abs(targetAngle)) {
					turnToGyro(gyroAngle - 1, "Left", 0.3);
				} else if(Math.abs(gyroAngle) > Math.abs(targetAngle)) {
					turnToGyro(gyroAngle + 1, "Right", 0.3);
				}
				break;
				
			case "Center":
				if(gyroAngle > 0) {
					turnToGyro(gyroAngle - 1, "Left", 0.3);
				} else if(gyroAngle < 0) {
					turnToGyro(gyroAngle + 1, "Right", 0.3);
				}
				break;
				
			default: System.out.println("Please input a turn direction to correct");
		}
		if(gyroAngle > tolMax||gyroAngle < tolMin) {
			return false;
		} else {
			return true;
		}
	}
	
private boolean correctTurn(double targetAngle, String direction, double tol) {
		
		double tolMin = targetAngle - tol;
		double tolMax = targetAngle + tol;
		
		if(targetAngle == 0) direction = "Center";
			
		switch(direction) {
			case "Right": 
				if(Math.abs(gyroAngle) < Math.abs(targetAngle)) {
					turnToGyro(gyroAngle + 1, "Right", 0.3);
				} else if(Math.abs(gyroAngle) > Math.abs(targetAngle)) {
					turnToGyro(gyroAngle - 1, "Left", 0.3);
				}
				break;
			
			case "Left": 
				if(Math.abs(gyroAngle) < Math.abs(targetAngle)) {
					turnToGyro(gyroAngle - 1, "Left", 0.3);
				} else if(Math.abs(gyroAngle) > Math.abs(targetAngle)) {
					turnToGyro(gyroAngle + 1, "Right", 0.3);
				}
				break;
				
			case "Center":
				if(gyroAngle > 0) {
					turnToGyro(gyroAngle - 1, "Left", 0.3);
				} else if(gyroAngle < 0) {
					turnToGyro(gyroAngle + 1, "Right", 0.3);
				}
				break;
				
			default: System.out.println("Please input a turn direction to correct");
		}
		if(gyroAngle > tolMax||gyroAngle < tolMin) {
			return false;
		} else {
			return true;
		}
	}


	/**
	 * private void turnToGyro(double Angle, String Direction) {
	 * turnToGyroPID.setSetpoint(Angle); turnToGyroPID.enable();
	 * 
	 * if (!turnToGyroPID.onTarget()) { // drive forwards half speed
	 * rightMotor.set(-PIDGyroOutput.value);
	 * leftMotor.set(-PIDGyroOutput.value);
	 * leftStepDone = false; rightStepDone = false; 
	 * } else if
	 * (turnToGyroPID.onTarget()) { 
	 * rightMotor.set(0); // stop robot
	 * leftMotor.set(0); 
	 * leftStepDone = true; rightStepDone = true; } } private void
	 * turnToGyroEnd() {
	 * turnToGyroPID.disable();
	 * }
	 **/

}

/*
 * There are two kinds of people: Type 1 (the right ones):
 * 
 * if() {
 * 
 * }
 * 
 * Type 2:
 * 
 * if() {
 * 
 * }
 */
