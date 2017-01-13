package subsystems;

public class Drive {
	// Drive parameters
	double maxSpeed = 12; // ft/s
	double maxAccel = 20; // ft/s^2
	
	double robotWidth = 3; 	//ft
	double robotLength = 3; //ft
	
	// Storage of drive dynamics
	double mLeftMotorSpeed = 0.0;
	double mRightMotorSpeed = 0.0;

	double mLeftDistance = 0.0;
	double mRightDistance = 0.0;
	
	double mYaw = 0.0;		//deg
	double mPitch = 0.0;	//deg
	double mRoll = 0.0;		//deg
	
	double leftInverted = 1.0;
	double rightInverted = 1.0;
	
	// Other
	private static Drive instance = null;
	
	public static Drive getInstance()
    {
        if( instance == null )
        {
            instance = new Drive();
        }
        return instance;
    }
	
	public void reinit(){
		mLeftMotorSpeed = 0.0;
		mRightMotorSpeed = 0.0;
		mYaw = 0.0;
		mPitch = 0.0;
		mRoll = 0.0;
		leftInverted = 1.0;
		rightInverted = 1.0;
		
		resetEncoders();
	}
	
	public void invertLeftMotor(){
		leftInverted = -1.0;
	}
	
	public void invertRightMotor(){
		rightInverted = -1.0;
	}
	
	private void set(double leftMotorSpeed, double rightMotorSpeed){
		mLeftMotorSpeed = leftMotorSpeed;
		mRightMotorSpeed = rightMotorSpeed;
	}

	public void setSpeed(double left, double right) {
		set(left, right);
	}
	
	public void setSpeedTurn(double speed, double turn){
		double leftMotorSpeed = speed + turn;
		double rightMotorSpeed = speed - turn;
		set(leftMotorSpeed, rightMotorSpeed);
	}
	
	public double getAverageDistance(){
		return (getLeftEncoderDistance() + getRightEncoderDistance())/2;
	}
	
	public double getLeftEncoderDistance(){
		return mLeftDistance;		
	}
	
	public double getRightEncoderDistance(){
		return mRightDistance;		
	}
	
	public void step(double dt){
		double leftSpeed = leftInverted*dt*maxSpeed*mLeftMotorSpeed;
		double rightSpeed = rightInverted*dt*maxSpeed*mRightMotorSpeed;
		//double forwardDelta = leftSpeed - rightSpeed;
		//double dYaw = Math.atan2(forwardDelta, robotWidth);
		
		mLeftDistance += Math.signum(leftSpeed)*Math.min(leftSpeed, maxAccel);
		mRightDistance += Math.signum(rightSpeed)*Math.min(rightSpeed, maxAccel);
		
		/*
		mYaw += dYaw;
		if (mYaw >= Math.PI) {
			mYaw -= 2.0*Math.PI;
		} else if (mYaw <= -Math.PI){
			mYaw += 2.0*Math.PI;
		}
		*/
	}

	public double getGyroAngleInRadians() {
		return mYaw;
	}
	
	public double getGyroAngleInDegrees(){
		return mYaw * 180.0 / Math.PI;
	}

	public void resetEncoders() {
		mLeftDistance = 0.0;
		mRightDistance = 0.0;
		
	}	
	

}
