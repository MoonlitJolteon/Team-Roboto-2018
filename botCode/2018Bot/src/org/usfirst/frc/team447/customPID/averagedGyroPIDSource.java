package org.usfirst.frc.team447.customPID;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.analog.adis16448.frc.ADIS16448_IMU;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

//@SuppressWarnings("unused")
public class averagedGyroPIDSource implements PIDSource {
	
	private ADXRS450_Gyro gyro1;
	private ADIS16448_IMU gyro2;
	private PIDSourceType sourceType;
	
	public averagedGyroPIDSource(ADXRS450_Gyro gyroInput1, ADIS16448_IMU gyroInput2) {
		this.gyro1 = gyroInput1;
		this.gyro2 = gyroInput2;
	}
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		this.sourceType = pidSource;
	}
	@Override
	public PIDSourceType getPIDSourceType() {
		return this.sourceType;
	}
	@Override
	public double pidGet() {
		return ((this.gyro1.getAngle()+this.gyro2.getAngleX())/2);
	}
	
	
}
