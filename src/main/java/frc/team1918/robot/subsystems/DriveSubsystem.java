package frc.team1918.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Dashboard;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.SwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//file read/write operations
import java.io.BufferedWriter;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileWriter;
import java.io.FileReader;
import java.io.IOException;
//kinematics and odometry
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.geometry.Translation2d;

public class DriveSubsystem extends SubsystemBase {

	private static DriveSubsystem instance;
	private static int flHome = Constants.DriveTrain.DT_FL_MECHZERO;
	private static int frHome = Constants.DriveTrain.DT_FR_MECHZERO;
	private static int rlHome = Constants.DriveTrain.DT_RL_MECHZERO;
	private static int rrHome = Constants.DriveTrain.DT_RR_MECHZERO;
	private File f;
	private BufferedWriter bw;
	private FileWriter fw;
	private BufferedReader br;
	private FileReader fr;
	private static double l = Constants.Global.ROBOT_LENGTH, w = Constants.Global.ROBOT_WIDTH, r = Math.sqrt((l * l) + (w * w));
	private static boolean driveControlsLocked = false; //true while homing operation
	private int debug_ticks, dash_gyro_ticks, dash_dt_ticks;
	private static double desiredAngle; //Used for driveStraight function
	private static boolean angleLocked = false;

	//initialize 4 swerve modules
	private static SwerveModule m_dtFL = new SwerveModule("dtFL",
		Constants.Swerve.FL.DRIVE_MC_ID, Constants.Swerve.FL.TURN_MC_ID,
		Constants.Swerve.FL.TURN_kP, Constants.Swerve.FL.TURN_kI, Constants.Swerve.FL.TURN_kD, Constants.Swerve.FL.TURN_kIZone,
		Constants.Swerve.FL.TURN_ALLOWED_ERROR,
		Constants.Swerve.FL.DRIVE_wheelDiamOffsetMM,
		Constants.Swerve.FL.TURN_sensorPhase, Constants.Swerve.FL.TURN_isInverted,
		flHome); // Front Left
	private static SwerveModule m_dtFR = new SwerveModule("dtFR",
		Constants.Swerve.FR.DRIVE_MC_ID, Constants.Swerve.FR.TURN_MC_ID,
		Constants.Swerve.FR.TURN_kP, Constants.Swerve.FR.TURN_kI, Constants.Swerve.FR.TURN_kD, Constants.Swerve.FR.TURN_kIZone,
		Constants.Swerve.FR.TURN_ALLOWED_ERROR,
		Constants.Swerve.FR.DRIVE_wheelDiamOffsetMM,
		Constants.Swerve.FR.TURN_sensorPhase, Constants.Swerve.FR.TURN_isInverted,
		frHome); // Front Right
	private static SwerveModule m_dtRL = new SwerveModule("dtRL",
		Constants.Swerve.RL.DRIVE_MC_ID, Constants.Swerve.RL.TURN_MC_ID,
		Constants.Swerve.RL.TURN_kP, Constants.Swerve.RL.TURN_kI, Constants.Swerve.RL.TURN_kD, Constants.Swerve.RL.TURN_kIZone,
		Constants.Swerve.RL.TURN_ALLOWED_ERROR,
		Constants.Swerve.RL.DRIVE_wheelDiamOffsetMM,
		Constants.Swerve.RL.TURN_sensorPhase, Constants.Swerve.RL.TURN_isInverted,
		rlHome); // Rear Left
	private static SwerveModule m_dtRR = new SwerveModule("dtRR",
		Constants.Swerve.RR.DRIVE_MC_ID, Constants.Swerve.RR.TURN_MC_ID,
		Constants.Swerve.RR.TURN_kP, Constants.Swerve.RR.TURN_kI, Constants.Swerve.RR.TURN_kD, Constants.Swerve.RR.TURN_kIZone,
		Constants.Swerve.RR.TURN_ALLOWED_ERROR,
		Constants.Swerve.RR.DRIVE_wheelDiamOffsetMM,
		Constants.Swerve.RR.TURN_sensorPhase, Constants.Swerve.RR.TURN_isInverted,
		rrHome); // Rear Right
	//initialize gyro object
	private static AHRS m_gyro = new AHRS(SPI.Port.kMXP);
	//intialize odometry class for tracking robot pose
	SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Constants.Swerve.kDriveKinematics, m_gyro.getRotation2d());

	public static DriveSubsystem getInstance() {
		if (instance == null)
			instance = new DriveSubsystem();
		return instance;
	}

	public DriveSubsystem() { //initialize the class
		setAllConversionFactor();
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_odometry.update(
			new Rotation2d(getHeading()),
			m_dtFL.getState(),
			m_dtFR.getState(),
			m_dtRL.getState(),
			m_dtRR.getState()
		);
		if(dash_gyro_ticks % 5 == 0) {
			Dashboard.Gyro.setGyroAngle(Helpers.General.roundDouble(m_gyro.getAngle(),3)); 
		} 
		dash_gyro_ticks++;
		if(dash_dt_ticks % 5 == 0) {
			Dashboard.DriveTrain.setDTAngle("FL", m_dtFL.getTurnAbsPos());
			Dashboard.DriveTrain.setDTAngle("FR", m_dtFR.getTurnAbsPos());
			Dashboard.DriveTrain.setDTAngle("RL", m_dtRL.getTurnAbsPos());
			Dashboard.DriveTrain.setDTAngle("RR", m_dtRR.getTurnAbsPos());
		}
		dash_dt_ticks++;
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 * @return The pose.
	 */
	public Pose2d getPose() {
	  return m_odometry.getPoseMeters();
	}

	/**
     * Returns the heading of the robot.
     * @return the robot's heading in degrees, from -180 to 180
     */
	public double getHeading() {
		return m_gyro.getRotation2d().getDegrees() * (Constants.Swerve.kGyroReversed ? -1.0 : 1.0);
	}

	/**
	 * Returns the turn rate of the robot.
	 * @return The turn rate of the robot, in degrees per second
	 */
	public double getTurnRate() {
		return m_gyro.getRate() * (Constants.Swerve.kGyroReversed ? -1.0 : 1.0);
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
	  m_odometry.resetPosition(pose, m_gyro.getRotation2d());
	}

	public void lockAngle() {
		desiredAngle = Helpers.General.roundDouble(m_gyro.getAngle(), 3);
		angleLocked = true;
		Helpers.Debug.debug("Angle Locked to "+desiredAngle);
	}

	public void unlockAngle() {
		if(angleLocked) Helpers.Debug.debug("Angle Unlocked");
		angleLocked = false;
	}

	/**
	 * Method to drive the robot using joystick info.
	 * @param fwd Speed of the robot in the x direction (forward).
	 * @param str Speed of the robot in the y direction (sideways).
	 * @param rot Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the field.
	 */
	@SuppressWarnings("ParameterName")
	public void drive(double fwd, double str, double rot, boolean fieldRelative) {

		if(Constants.DriveTrain.DT_USE_DRIVESTRAIGHT) {
			if(rot == 0) { //We are not applying rotation
				if(!angleLocked) { //We havent locked an angle, so lets save the desiredAngle and lock it
					lockAngle();
				} else {
					if (Math.abs(fwd) > 0 || Math.abs(str) > 0) { //Only do angle correction while moving, for safety reasons
						rot += calcAngleStraight(desiredAngle,m_gyro.getAngle(),Constants.DriveTrain.DT_DRIVESTRAIGHT_P); //Add some correction to the rotation to account for angle drive
					}
				}
			}
		}
		double fwdMPS = fwd * Constants.DriveTrain.DT_kMaxMetersPerSecond;
		double strMPS = str * Constants.DriveTrain.DT_kMaxMetersPerSecond;
		var swerveModuleStates =
		Constants.Swerve.kDriveKinematics.toSwerveModuleStates(fieldRelative
					? ChassisSpeeds.fromFieldRelativeSpeeds(fwdMPS, strMPS, rot, getRot2d())
					: new ChassisSpeeds(fwdMPS, strMPS, rot));
		if (Helpers.Debug.debugThrottleMet(debug_ticks)) {
			Helpers.Debug.debug((fieldRelative)
			? ChassisSpeeds.fromFieldRelativeSpeeds(fwdMPS, strMPS, rot, getRot2d()).toString()+" fieldCentric"
			: new ChassisSpeeds(fwdMPS, strMPS, rot).toString()+" robotCentric");
		}
		debug_ticks++;
		SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
		if(!Constants.Swerve.DISABLE_FL) m_dtFL.setDesiredState(swerveModuleStates[0]);
		if(!Constants.Swerve.DISABLE_FR) m_dtFR.setDesiredState(swerveModuleStates[1]);
		if(!Constants.Swerve.DISABLE_RL) m_dtRL.setDesiredState(swerveModuleStates[2]);
		if(!Constants.Swerve.DISABLE_RR) m_dtRR.setDesiredState(swerveModuleStates[3]);
	}

	public double calcAngleStraight(double targetAngle, double currentAngle, double kP) {
		double errorAngle = (targetAngle - currentAngle) % 360.0;
		double correction = errorAngle * kP;
		return correction;
	}

	/**
	 * Sets the swerve ModuleStates.
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
		m_dtFL.setDesiredState(desiredStates[0]);
		m_dtFR.setDesiredState(desiredStates[1]);
		m_dtRL.setDesiredState(desiredStates[2]);
		m_dtRR.setDesiredState(desiredStates[3]);
	}

	public static AHRS getm_gyro() {
        return m_gyro;
	}

	public static void setDrivePower(double flPower, double frPower, double rlPower, double rrPower) {
		if (!Constants.DriveTrain.DT_DRIVE_DISABLED) {
			m_dtFL.setDrivePower(flPower);
			m_dtFR.setDrivePower(frPower);
			m_dtRL.setDrivePower(rlPower);
			m_dtRR.setDrivePower(rrPower);
		}
	}

	public static void setTurnPower(double flPower, double frPower, double rlPower, double rrPower) {
	    m_dtFL.setTurnPower(flPower);
		m_dtFR.setTurnPower(frPower);
		m_dtRL.setTurnPower(rlPower);
		m_dtRR.setTurnPower(rrPower);
	}

	public static void setLocation(double flLoc, double frLoc, double rlLoc, double rrLoc) {
	    m_dtFL.setTurnLocation(flLoc);
		m_dtFR.setTurnLocation(frLoc);
		m_dtRL.setTurnLocation(rlLoc);
		m_dtRR.setTurnLocation(rrLoc);
	}

	public void stopAllDrive() {
	    m_dtFL.stopDrive();
		m_dtFR.stopDrive();
		m_dtRL.stopDrive();
		m_dtRR.stopDrive();
	}

	/*
	 * Drive methods
	 */
	public static void swerveDrive(double fwd, double str, double rot) {
		double a = str - (rot * (l / r));
		double b = str + (rot * (l / r));
		double c = fwd - (rot * (w / r));
		double d = fwd + (rot * (w / r));

		//Wheel Speed
		double ws1 = Math.sqrt((b * b) + (c * c)); //FR
		double ws2 = Math.sqrt((a * a) + (c * c)); //RR
		double ws3 = Math.sqrt((a * a) + (d * d)); //RL
		double ws4 = Math.sqrt((b * b) + (d * d)); //FL

		//Wheel Angle
		double wa1 = Math.atan2(b, c); //FR
		double wa2 = Math.atan2(a, c); //RR
		double wa3 = Math.atan2(a, d); //RL
		double wa4 = Math.atan2(b, d); //FL

		double max = ws1;
		max = Math.max(max, ws2);
		max = Math.max(max, ws3);
		max = Math.max(max, ws4);
		if (max > 1) {
			ws1 /= max;
			ws2 /= max;
			ws3 /= max;
			ws4 /= max;
		}
		SmartDashboard.putNumber("ws1", ws1);
		SmartDashboard.putNumber("ws2", ws2);
		SmartDashboard.putNumber("ws3", ws3);
		SmartDashboard.putNumber("ws4", ws4);
		SmartDashboard.putNumber("wa1", wa1);
		SmartDashboard.putNumber("wa2", wa2);
		SmartDashboard.putNumber("wa3", wa3);
		SmartDashboard.putNumber("wa4", wa4);

		DriveSubsystem.setDrivePower(ws4, ws1, ws3, ws2);
		DriveSubsystem.setLocation(wa4, wa1, wa3, wa2);
	}
	//#region GYRO STUFF
	public void resetGyro() {
		Helpers.Debug.debug("Gyro Reset");
		m_gyro.reset();
		resetOdometry(getPose());
	}

	public static Rotation2d getRot2d() {
		return Rotation2d.fromDegrees(getGyroAngle());
	}

	public static double getGyroAngle() {
		return m_gyro.getAngle();
	}

	public static double getGyroAngleInRad() {
		return m_gyro.getAngle() * (Math.PI / 180d);
	}
	//#endregion GYRO STUFF

	//#region ENCODER STUFF
	public void getAllAbsPos() {
		flHome = m_dtFL.getTurnAbsPos();
		frHome = m_dtFR.getTurnAbsPos();
		rlHome = m_dtRL.getTurnAbsPos();
		rrHome = m_dtRR.getTurnAbsPos();

		String outString = "flHome:"+flHome;
		outString += " frHome:"+frHome;
		outString += " rlHome:"+rlHome;
		outString += " rrHome:"+rrHome;
		System.out.println("getAllAbsPos: " + outString);
	}
	//#endregion ENCODER STUFF

	//#region MOTOR CONTROLLER STUFF
	public static void setAllConversionFactor() {
		m_dtFL.setDriveConversionFactor();
		m_dtFR.setDriveConversionFactor();
		m_dtRL.setDriveConversionFactor();
		m_dtRR.setDriveConversionFactor();
	}

	public static void setAllDriveBrakeMode(boolean b) {
		m_dtFL.setBrakeMode("drive", b);
		m_dtFR.setBrakeMode("drive", b);
		m_dtRL.setBrakeMode("drive", b);
		m_dtRR.setBrakeMode("drive", b);
	}

	public static void setAllTurnBrakeMode(boolean b) {
		m_dtFL.setBrakeMode("turn", b);
		m_dtFR.setBrakeMode("turn", b);
		m_dtRL.setBrakeMode("turn", b);
		m_dtRR.setBrakeMode("turn", b);
	}
	//#endregion MOTOR CONTROLLER STUFF

	//#region USER CONTROLS
	public boolean isDriveControlsLocked() {
		return driveControlsLocked;
	}

	public void lockDriveControls(boolean lock) {
		driveControlsLocked = lock;
		System.out.println("drive controls lock state: " + lock);
	}
	//#endregion USER CONTROLS

	//#region HOMING AND CALIBRATION
	public void saveAllHomes() {
		try {
			f = new File(Constants.DriveTrain.DT_HOMES_FILE);
			if(!f.exists()){
				f.createNewFile();
			}
			fw = new FileWriter(f);
		} catch (IOException e) {
			e.printStackTrace();
		}
		bw = new BufferedWriter(fw);
		String outString = "flHome:"+flHome+"\n";
		outString += "frHome:"+frHome+"\n";
		outString += "rlHome:"+rlHome+"\n";
		outString += "rrHome:"+rrHome+"\n";
		Helpers.Debug.debug("saveAllHomes: " + outString);

		// m_dtFL.setHomePos(flHome);
		// m_dtFR.setHomePos(frHome);
		// m_dtRL.setHomePos(rlHome);
		// m_dtRR.setHomePos(rrHome);

		try {
			bw.write(outString+"\n");
			bw.close();
			fw.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void readAllHomes() {
		//Read values from file and store in private variables
		f = new File(Constants.DriveTrain.DT_HOMES_FILE);
		if(!f.exists()){
			saveAllHomes();
		}
		try {
			fr = new FileReader(f);
			br = new BufferedReader(fr);
			String line = br.readLine(); //read all the lines from the file and beg for bread
			while (line != null) {
				// System.out.print("readAllHomes line="+line+"\n");
				String part[] = line.split(":",2);
				// System.out.print("readAllHomes part0="+part[0]+"\n");
				switch (part[0]) {
					case "flHome":
						flHome = Integer.parseInt(part[1]);
						Helpers.Debug.debug("readAllHomes: flHome="+flHome);
						break;
					case "frHome":
						frHome = Integer.parseInt(part[1]);
						Helpers.Debug.debug("readAllHomes: frHome="+frHome);
						break;
					case "rlHome":
						rlHome = Integer.parseInt(part[1]);
						Helpers.Debug.debug("readAllHomes: rlHome="+rlHome);
						break;
					case "rrHome":
						rrHome = Integer.parseInt(part[1]);
						Helpers.Debug.debug("readAllHomes: rrHome="+rrHome);
						break;
				}
				line = br.readLine(); //beg for more bread
			}
			// m_dtFL.setHomePos(0); //flHome);
			// m_dtFR.setHomePos(0); //frHome);
			// m_dtRL.setHomePos(0); //rlHome);
			// m_dtRR.setHomePos(0); //rrHome);

			br.close();
			fr.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void startCalibrationMode() {
		Helpers.Debug.debug("startCalibrationMode");
		lockDriveControls(true);
		setAllTurnBrakeMode(false);
		m_dtFL.setTurnPowerPercent(0);
		m_dtFR.setTurnPowerPercent(0);
        m_dtRL.setTurnPowerPercent(0);
        m_dtRR.setTurnPowerPercent(0);
	}

	public void stopCalibrationMode() {
		Helpers.Debug.debug("stopCalibrationMode");
		//resetAllAbsEnc(); //reset rotation counter
		//getAllAbsPos(); //get absolute positions
		//saveAllHomes();
		// readAllHomes();
		setAllTurnBrakeMode(true);
		lockDriveControls(false);
	}

	public void moveAllToHomes() {
		Helpers.Debug.debug("moveAllToHomes");
		readAllHomes();
		m_dtFL.setTurnLocationInEncoderTicks(flHome);
		m_dtFR.setTurnLocationInEncoderTicks(frHome);
		m_dtRL.setTurnLocationInEncoderTicks(rlHome);
		m_dtRR.setTurnLocationInEncoderTicks(rrHome);
	}

	public void resetAllAbsEnc() {
		m_dtFL.resetTurnAbsEnc();
		m_dtFR.resetTurnAbsEnc();
		m_dtRL.resetTurnAbsEnc();
		m_dtRR.resetTurnAbsEnc();
	}

	public boolean isAllTurnAtHome() {
		if (
			m_dtFL.isTurnAtHome() &&
			m_dtFR.isTurnAtHome() &&
			m_dtRL.isTurnAtHome() &&
			m_dtRR.isTurnAtHome()
			) {
				return true;
			}
		return false;
	}
	//#endregion HOMING AND CALIBRATION
}