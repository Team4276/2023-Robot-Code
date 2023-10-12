package frc.utils;

import edu.wpi.first.wpilibj.Timer;

public class SoftwareTimer {

	private double expirationTime = 0;

	public void setTimer(double timerValue) {
		expirationTime = Timer.getFPGATimestamp() + timerValue;
	}

	public boolean isExpired() {
		return (Timer.getFPGATimestamp() > expirationTime);
		// if robotTime exceeds expirationTime, then this returns true
	}

}
