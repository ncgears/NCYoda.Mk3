
package frc.team1918.robot;

// import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
//Spark MAX
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//Talon SRX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//WPILib
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class SwerveModule {
    private WPI_TalonSRX turn;
    private CANSparkMax drive;
    private final double FULL_ROTATION = Constants.DriveTrain.DT_TURN_ENCODER_FULL_ROTATION;
    private final double FULL_ROT_RADS = (2 * Math.PI);
    private final double TURN_P, TURN_I, TURN_D;
    private final int TURN_IZONE;
    private final int TURN_ALLOWED_ERROR;
    private boolean isDrivePowerInverted = false;
    private String moduleName;
    private double wheelOffsetMM = 0;
    private int homePos = 0;
    private boolean absEncoderEnabled = false;
    private int debug_ticks1, debug_ticks2;
    private CANPIDController m_drive_pidController;

//SparkMAX Java API Doc: https://www.revrobotics.com/content/sw/max/sw-docs/java/index.html

 	/**
	 * 1918 Swerve Module - Uses Spark Max for drive (Neo) and Talon SRX for turn (bag with gearbox)
	 * @param driveMC_ID This is the CAN ID of the drive motor controller
	 * @param turnMC_ID This is the CAN ID of the turn motor controller
	 * @param tP The P constant (double) for the turning PID
	 * @param tI The I constant (double) for the turning PID
	 * @param tD The D constant (double) for the turning PID
	 * @param tIZone The IZone value (int) for the turning PID
	 * @param name The name of this module instance
	 * @param wheelOffsetMM Adjustment to size of the wheel to account for wear
	 * @param homePos The home position of this swerve module
	 */
    public SwerveModule(String name, int driveMC_ID, int turnMC_ID, double tP, double tI, double tD, int tIZone, int tAllowedError, double wheelOffsetMM, boolean sensorPhase, boolean inverted, int homePos){
        drive = new CANSparkMax(driveMC_ID, MotorType.kBrushless);
        turn = new WPI_TalonSRX(turnMC_ID);
        moduleName = name;
        isDrivePowerInverted = false;
        TURN_P = tP;
        TURN_I = tI;
        TURN_D = tD;
        TURN_IZONE = tIZone;
        TURN_ALLOWED_ERROR = tAllowedError;

        turn.configFactoryDefault(); //Reset controller to factory defaults to avoid wierd stuff
        turn.set(ControlMode.PercentOutput, 0); //Set controller to disabled
        turn.setNeutralMode(NeutralMode.Brake); //Set controller to brake mode
        turn.configSelectedFeedbackSensor(  FeedbackDevice.CTRE_MagEncoder_Absolute, // Local Feedback Source
                                            Constants.Global.PID_PRIMARY,				// PID Slot for Source [0, 1]
                                            Constants.Global.kTimeoutMs);				// Configuration Timeout
        turn.configFeedbackNotContinuous(true, 0); //Disable continuous feedback tracking (so 0 and 4096 are effectively one and the same)
        turn.setSensorPhase(sensorPhase);
        turn.setInverted(inverted);
        turn.config_kP(0, TURN_P);
        turn.config_kI(0, TURN_I);
        turn.config_kD(0, TURN_D);
        turn.config_IntegralZone(0, TURN_IZONE);
        turn.configAllowableClosedloopError(0, TURN_ALLOWED_ERROR); 

        drive.restoreFactoryDefaults();
        drive.setIdleMode(IdleMode.kBrake);
        m_drive_pidController = drive.getPIDController();
        m_drive_pidController.setP(0.0003); //PID P
        m_drive_pidController.setI(0.0); //PID I
        m_drive_pidController.setD(0.0); //PID D
        // m_drive_pidController.setIZone(0); //IZone
        m_drive_pidController.setFF(1/4740); //Feed forward
        m_drive_pidController.setOutputRange(-1, 1);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        double rawRpm = drive.getEncoder().getVelocity();
        double wheelRpm = Helpers.General.gearCalcDouble(rawRpm,Constants.DriveTrain.DT_DRIVE_FIRST_GEARONE,
            Constants.DriveTrain.DT_DRIVE_FIRST_GEARTWO,
            Constants.DriveTrain.DT_DRIVE_SECOND_GEARONE,
            Constants.DriveTrain.DT_DRIVE_SECOND_GEARTWO);
        double wheelDiam = Constants.DriveTrain.DT_WHEEL_DIAM_MM - this.wheelOffsetMM;
        // double angle = Helpers.General.ticksToRadians(getTurnAbsPos() - homePos); //subtract homePos so it is 0 based
        // return new SwerveModuleState(Helpers.General.rpmToMetersPerSecond(wheelRpm, wheelDiam), new Rotation2d(angle));
        return new SwerveModuleState(Helpers.General.rpmToMetersPerSecond(wheelRpm, wheelDiam), getTurnAbsPosAsRotation2d());
    }

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins.  Since we are using a Mag encoder with a 0-4096 value, we have
     * to set the FeedbackSensor
     * @param desiredState The desired state.
     */
    public SwerveModuleState optimize(SwerveModuleState desiredState) {  //New optimize test 2021-03-17 JRB
        Rotation2d currentAngle = getTurnAbsPosAsRotation2d();
        Rotation2d delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0 && Math.abs(delta.getDegrees()) < 270) { //new requested turn is between 90 and 270 degrees, invert drive speed and rotate 180 degrees from desired
            return new SwerveModuleState(-desiredState.speedMetersPerSecond,desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else { //no optimization necessary
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }
    public SwerveModuleState optimizeBad(SwerveModuleState desiredState) { //TODO: Remove optimizeBad() once optimize() is working properly
        double wheelSpeed = desiredState.speedMetersPerSecond;
        double waRads = desiredState.angle.getRadians();
        double currentAngleRads = Helpers.General.ticksToRadians(getTurnAbsPos());
        double targetAngleRads = waRads;
        int currentNumRotations = (int) (currentAngleRads / FULL_ROT_RADS ); //figure out how many rotations current position is
        targetAngleRads += (currentNumRotations >= 0) ? currentNumRotations * FULL_ROT_RADS : (currentNumRotations + 1) * FULL_ROT_RADS; //add current rotations to target

        if ((targetAngleRads > currentAngleRads + FULL_ROT_RADS * 0.25) || (targetAngleRads < currentAngleRads - FULL_ROT_RADS * 0.25)) { //if target is more than 25% of a rotation either way
            if (currentAngleRads < targetAngleRads) { //left strafe
                if (targetAngleRads - currentAngleRads > FULL_ROT_RADS * 0.75) { //if target would require moving less than 75% of a rotation, just go there
                    targetAngleRads -= FULL_ROT_RADS;
                } else { //otherwise, turn half a rotation from the target and reverse the drive power
                    targetAngleRads -= FULL_ROT_RADS * 0.5;
                    wheelSpeed *= -1;
                }
            } else { //right strafe
                if ( currentAngleRads - targetAngleRads > FULL_ROT_RADS * 0.75) { //if target would require moving less than 75% of a rotation, just go there
                    targetAngleRads += FULL_ROT_RADS;
                } else { //otherwise, turn half a rotation from the target and reverse the drive power
                    targetAngleRads += FULL_ROT_RADS * 0.5;
                    wheelSpeed *= -1;
                }
            }
        }
        return new SwerveModuleState(wheelSpeed, new Rotation2d(targetAngleRads));
    }

    /**
     * This function takes a desiredState and instructs the motor controllers to move based on the desired state
     * @param desiredState Desired state with speed and angle.
     * FL = 5570, FR = 5200, RL = 5200, RR = 4740
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        double wheelDiam = Constants.DriveTrain.DT_WHEEL_DIAM_MM - this.wheelOffsetMM;
        SwerveModuleState state = (Constants.Swerve.USE_OPTIMIZATION) ? optimize(desiredState) : desiredState;
        int turn_ticks = (Constants.Global.SWERVE_SENSOR_NONCONTINUOUS) 
            ? (40960 + Helpers.General.radiansToTicks(state.angle.getRadians(),this.homePos)) & 0xFFF
            : Helpers.General.radiansToTicks(state.angle.getRadians(),this.homePos);
        if (Constants.Swerve.USE_DRIVE_PID) {
            double motorRpm = (Helpers.General.metersPerSecondToRPM(state.speedMetersPerSecond, wheelDiam) / 0.194444);
            Helpers.Debug.debug(moduleName+" desired mps: "+state.speedMetersPerSecond+" motorRpm: "+motorRpm);
            m_drive_pidController.setReference(motorRpm, ControlType.kVelocity);
        } else {
            drive.set(state.speedMetersPerSecond);
        }
        turn.set(ControlMode.Position, turn_ticks);
        if(Helpers.Debug.debugThrottleMet(debug_ticks1)) {
            Helpers.Debug.debug(moduleName+" Speed="+Helpers.General.roundDouble(state.speedMetersPerSecond,3)+" Turn="+(Helpers.General.radiansToTicks(state.angle.getRadians(),this.homePos)));
        }
        debug_ticks1++;
    }

    /**
     * This function sets the conversion factor on the SparkMAX from the constants DT_DRIVE_CONVERSION_FACTOR
     */
    public void setDriveConversionFactor() {
        // drive.getEncoder().setVelocityConversionFactor(Constants.DriveTrain.DT_DRIVE_CONVERSION_FACTOR);
    }

    /**
	 * @param p turn power from -1 to 1
    */
    public void setTurnPower(double p){
        this.turn.set(ControlMode.PercentOutput, p);
    }

    /**
	 * @param p drive motor power from -1 to 1
    */
    public void setDrivePower(double p){
        if (this.isDrivePowerInverted) {
            this.drive.set(-p);
        } else {
            this.drive.set(p);
        }
    }

    /**
     * Gets the position of the relative encoder in encoder ticks
     * @return Integer of relative encoder ticks
     */
    public int getTurnRelPos(){
        return turn.getSensorCollection().getQuadraturePosition();
        //https://github.com/CrossTheRoadElec/Phoenix-Documentation/blob/master/Legacy/Migration%20Guide.md
    }

    /**
     * Gets the position of the absolute encoder in encoder ticks
     * @return Integer of absolute encoder ticks
     */
    public int getTurnAbsPos(){
        return (turn.getSensorCollection().getPulseWidthPosition() & 0xFFF); //This gets only the most significant bits (0-4095)
        //Explanation: & is a bitwise "AND" operator, and 0xFFF is 4095 in Hex, consider "0101010101 AND 1111 = 0101"
    }

    /**
     * Gets the position of the absolute encoder in encoder ticks, subtracts the homePos, and returns a rotation2d object
     * @return Rotation2d object of the current position
     */
    public Rotation2d getTurnAbsPosAsRotation2d(){
        return new Rotation2d(Helpers.General.ticksToRadians(getTurnAbsPos() - homePos));
    }
    /**
     * Stores the home position for this module
     * @param homePos Absolute encoder value of the home position
     */
    public void setHomePos(int pos) {
		this.homePos = pos;
	}

    /**
     * Returns a boolean indicating if the module is at home position within the margin of error defined in constants by DriveTrain.DT_HOME_MARGIN_OF_ERROR
     * @param pos Absolute encoder value of the target home position.
     * @return Boolean value indicating if this swerve module is at the home position.
     */
    public boolean isTurnAtHome(int pos) {
        int currentPos = getTurnAbsPos();
        int marginErr = Constants.DriveTrain.DT_HOME_MARGIN_OF_ERROR;
        int offset = 0; //(pos < marginErr || pos > 4095 - marginErr) ? 1024 : 0;

        int lowHome = pos + offset - marginErr; //could use this value % 4096
        int highHome = pos + offset + marginErr;

        lowHome -= (lowHome > 4095) ? 4096 : 0;
        highHome -= (highHome > 4095) ? 4096 : 0;
        currentPos -= (currentPos + offset > 4095) ? 4096 : 0;

        if (currentPos + offset <= highHome && currentPos + offset >= lowHome) {
            debug_ticks2 = Helpers.Debug.debug(moduleName + " isTurnAtHome=true; current="+currentPos+"; target="+pos+";",debug_ticks2);
            return true;
        } else {
            debug_ticks2 = Helpers.Debug.debug(moduleName + " isTurnAtHome=false; current="+currentPos+"; target="+pos+";",debug_ticks2);
            return false;
        }
    }

    /**
     * Resets the relative encoder to 0.
     */
    public void resetTurnEnc() {
        Helpers.Debug.debug(moduleName + " resetTurnEnc");
		turn.getSensorCollection().setQuadraturePosition(0,10);
    }

    /**
     * Sets the relative encoder to a specific value
     * @param value Integer from 0 to 4095 indicating the relative encoder position to set
     */
    public void setEncPos(int value) {
        turn.getSensorCollection().setQuadraturePosition(value,10);
    }

    /**
     * Checks if the turn encoder is connected and valid
     * @return true if the encoder is connected, false otherwise
     */
    public boolean isTurnEncConnected() {
        /**The isSensorPresent() routine had only supported pulse width sensors as these allow for simple
         * detection of the sensor signal. The getPulseWidthRiseToRiseUs() routine can be used to accomplish
         * the same task. The getPulseWidthRiseToRiseUs() routine returns zero if the pulse width signal is
         * no longer present (120ms timeout).
         */
        return (turn.getSensorCollection().getPulseWidthRiseToRiseUs() > 0) ? true : false;
        //isSensorPresent(FeedbackDevice.CTRE_MagEncoder_Relative) == FeedbackDeviceStatus.FeedbackDeviceStatusPresent;
    }

    /**
     * Gets the number of rotations that the relative encoder has detected
     * @return Integer indicating the number of rotations of the relative encoder
     */
    public int getTurnRotations() {
        return (int) (turn.getSensorCollection().getQuadraturePosition() / FULL_ROTATION);
    }

    /**
     * Gets the relative encoder position within the current rotation
     * @return Integer indicating the current location within the current rotation
     */
    public double getTurnLocation() {
        return (turn.getSensorCollection().getQuadraturePosition() % FULL_ROTATION) / FULL_ROTATION;
    }

    /**
	 * Set turn to pos from 0 to 1 using PID using shortest turn to get the wheels aimed the right way
	 * @param wa wheel angle location to set to in radians
	 */
	public void setTurnLocation(double waRads) {
        double currentAngleRads = Helpers.General.ticksToRadians(getTurnAbsPos());
        double targetAngleRads = waRads;
        int currentNumRotations = (int) (currentAngleRads / FULL_ROTATION);
        targetAngleRads += (currentNumRotations >= 0) ? currentNumRotations * FULL_ROTATION : (currentNumRotations + 1) * FULL_ROTATION;

        if ((targetAngleRads > currentAngleRads + FULL_ROTATION * 0.25) || (targetAngleRads < currentAngleRads - FULL_ROTATION * 0.25)) { //if target is more than 25% of a rotation either way
            if (currentAngleRads < targetAngleRads) { //left strafe
                if (targetAngleRads - currentAngleRads > FULL_ROTATION * 0.75) { //if target would require moving less than 75% of a rotation, just go there
                    targetAngleRads -= FULL_ROTATION;
                } else { //otherwise, turn half a rotation from the target and reverse the drive power
                    targetAngleRads -= FULL_ROTATION * 0.5;
                    this.isDrivePowerInverted = true;
                }
            } else { //right strafe
                if ( currentAngleRads - targetAngleRads > FULL_ROTATION * 0.75) { //if target would require moving less than 75% of a rotation, just go there
                    targetAngleRads += FULL_ROTATION;
                } else { //otherwise, turn half a rotation from the target and reverse the drive power
                    targetAngleRads += FULL_ROTATION * 0.5;
                    this.isDrivePowerInverted = true;
                }
            }
        }
        turn.set(ControlMode.Position,targetAngleRads);
        // System.out.println(moduleName + " setTurnLocation="+targetAngle+"; isDrivePowerInverted="+this.isDrivePowerInverted);
    }

    /**
     * Gets the closed-loop error. The units depend on which control mode is in use. If closed-loop is seeking a target sensor position, closed-loop error is the difference between target and current sensor value (in sensor units. Example 4096 units per rotation for CTRE Mag Encoder). If closed-loop is seeking a target sensor velocity, closed-loop error is the difference between target and current sensor value (in sensor units per 100ms). If using motion profiling or Motion Magic, closed loop error is calculated against the current target, and not the "final" target at the end of the profile/movement. See Phoenix-Documentation information on units.
     * @return Double precision units of error
     */
    public double getError() {
        return turn.getClosedLoopError();
    }

    /**
     * Stops both turning and driving by setting their respective motor power to 0.
     */
    public void stopBoth() {
        setDrivePower(0);
        setTurnPower(0);
    }

    /**
     * Stops the drive by setting the motor power to 0.
     */
    public void stopDrive() {
        setDrivePower(0);
    }

    /**
     * Sets the brake mode for the motor controller
     * @param device String of either "turn" or "drive" indicating which device to set
     * @param brake Boolean indicating if the brake mode should be set to brake (true) or coast (false)
     */
    public void setBrakeMode(String device, boolean brake) {
        switch (device) {
            case "turn": //turn is a TalonSRX
                if (brake) {
                    turn.setNeutralMode(NeutralMode.Brake);
                } else {
                    turn.setNeutralMode(NeutralMode.Coast);
                }
                break;
            case "drive": //drive is a SparkMAX
                if (brake) {
                    drive.setIdleMode(CANSparkMax.IdleMode.kBrake);
                } else {
                    drive.setIdleMode(CANSparkMax.IdleMode.kCoast);
                }
                break;
        }
    }

    /**
     * Sets the turn power to a specific PercentOutput
     * @param p Double from -1 to 1 indicating the turn power, where 0.0 is stopped
     */
    public void setTurnPowerPercent(double p) {
           turn.set(ControlMode.PercentOutput, p);
    }

    /**
     * Switches the turn encoder to either Absolute or Relative.
     * @param useAbsolute Boolean indicating whether to enable the absolute encoder (true) or the relative encoder (false)
     */
    public void setTurnEncoderAbsolute(boolean useAbsolute) {
        if (useAbsolute) {
            turn.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.Global.PID_PRIMARY, Constants.Global.kTimeoutMs);
        } else {
            turn.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.Global.PID_PRIMARY, Constants.Global.kTimeoutMs);
            if (this.absEncoderEnabled != useAbsolute) {
                //if we just switched to relative, change the setpoint to 0
                setTurnLocationInEncoderTicks(0.0);
            }
        }
        this.absEncoderEnabled = useAbsolute;
    }

    /**
     * Sets the turn position to a specific setpoint using the current encoder (absolute or relative)
     * @param et Encoder Ticks to turn module to.  This depends on which encoder is active.
     */
    public void setTurnLocationInEncoderTicks(double et) {
        // System.out.print(moduleName + " setTurnLocationInEncoderTicks = "+et+"\n");
        turn.set(ControlMode.Position, et);
    }
}