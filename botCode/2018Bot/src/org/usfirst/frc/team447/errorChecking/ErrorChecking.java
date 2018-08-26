package org.usfirst.frc.team447.errorChecking;

public class ErrorChecking {
	
	public static double getFinalMove(double setDist, double targetDist, double move1, double gyroAngle, double targetAngle) { //Input set dist, target dist, first move, and end gyro angle.
		
		if(gyroAngle < 0) gyroAngle = -gyroAngle; //if angle is negative, make it positive to avoid errors.
		
		if(targetAngle < 0) targetAngle = -targetAngle; //if target angle is negative, make it positive to avoid errors.
		
		System.out.println("Move 1: " + move1 + "\nMove 2: " + setDist + "\nTarget Dist: " + targetDist + "\nGyro Input: " + gyroAngle + "\nGyro Error: "+(targetAngle-gyroAngle));
		
		double err; //declare error num
		double finalMove; //declare final move num
		
		err = Math.sin(
					Math.toRadians(targetAngle-gyroAngle) //change angle from Degrees to Radians to avoid errors
				) * setDist; //calculate the error
		
		System.out.println("Calculated Error: "+err);
		
		finalMove = (targetDist - move1) - err; //calculate the final move
		
		return finalMove; //return the final move
		
	}
	
	public static boolean safeGyroCheck(double gyroCurr, double gyroTarget, double tol) { //input current gyro, target gyro, and tolerance
		
		double tolMin = gyroTarget - tol; //calculate minimum value
		double tolMax = gyroTarget + tol; //calculate maximum value
		
		if(gyroCurr > tolMax||gyroCurr < tolMin) { //return false if the value is outside of the tolerance
			return false;
		} else {
			return true;
		}
		
	}
}
