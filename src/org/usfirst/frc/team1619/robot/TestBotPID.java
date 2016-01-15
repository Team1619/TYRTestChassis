package org.usfirst.frc.team1619.robot;

public class TestBotPID {
	private double kP;
	private double kI;
	private double kD;
	
	private double setPoint;
	private double measuredValue;
	private double prevError;
	private double integral;
	
	public TestBotPID() {
		this(0.0, 0.0, 0.0);
	}
	
	public TestBotPID(double pValue, double iValue, double dValue) {
		this.kP = pValue;
		this.kI = iValue;
		this.kD = dValue;
		
		setPoint = 0;
		measuredValue = 0;
		prevError = 0;
		integral = 0;
	}
	
	public void setValues(double pValue, double iValue, double dValue) {
		this.kP = pValue;
		this.kI = iValue;
		this.kD = dValue;
	}
	
	public void setTarget(double target) {
		this.setPoint = target;
	}
	
	public double get(double currentValue) {
		return calcPID(currentValue);
	}
	
	private double calcPID(double error) {
		double currentError = error;
		double pCalc;
		double iCalc;
		double dCalc;
		double output;
		
		//P term
		pCalc = currentError * this.kP;
		
		//I term
		integral += currentError;
		iCalc = integral * this.kI;
		
		//D term
		dCalc = (currentError - prevError) * this.kD;
		
		//output
		output = pCalc + iCalc + dCalc;
		this.prevError = currentError;
		
		return output;
	}
}
