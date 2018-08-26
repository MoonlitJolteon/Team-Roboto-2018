package org.usfirst.frc.team447.customPID;

import edu.wpi.first.wpilibj.PIDOutput;

public class averagedGyroPIDOutput implements PIDOutput {
	
	public double value;

	@Override
	public void pidWrite(double output) {
		this.value = output;
	}

}
