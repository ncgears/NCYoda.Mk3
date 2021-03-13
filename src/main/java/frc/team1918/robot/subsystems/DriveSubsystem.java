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

	//initialize 4 swerve modules
	private static SwerveModule m_dtFL = new SwerveModule(Constants.DriveTrain.DT_FL_DRIVE_MC_ID,
		Constants.DriveTrain.DT_FL_TURN_MC_ID,
		Constants.DriveTrain.DT_TURN_P,
		Constants.DriveTrain.DT_TURN_I,
		Constants.DriveTrain.DT_TURN_D,
		Constants.DriveTrain.DT_TURN_IZONE, "dtFL",
		Constants.DriveTrain.DT_FL_WHEEL_DIAM_OFFSET_MM,
		flHome); // Front Left
	private static SwerveModule m_dtFR = new SwerveModule(Constants.DriveTrain.DT_FR_DRIVE_MC_ID,
		Constants.DriveTrain.DT_FR_TURN_MC_ID,
		Constants.DriveTrain.DT_TURN_P,
		Constants.DriveTrain.DT_TURN_I,
		Constants.DriveTrain.DT_TURN_D,
		Constants.DriveTrain.DT_TURN_IZONE, "dtFR",
		Constants.DriveTrain.DT_FR_WHEEL_DIAM_OFFSET_MM,
		frHome); // Front Right
	private static SwerveModule m_dtRL = new SwerveModule(Constants.DriveTrain.DT_RL_DRIVE_MC_ID,
		Constants.DriveTrain.DT_RL_TURN_MC_ID,
		Constants.DriveTrain.DT_TURN_P,
		Constants.DriveTrain.DT_TURN_I,
		Constants.DriveTrain.DT_TURN_D,
		Constants.DriveTrain.DT_TURN_IZONE, "dtRL",
		Constants.DriveTrain.DT_RL_WHEEL_DIAM_OFFSET_MM,
		rlHome); // Rear Left
	private static SwerveModule m_dtRR = new SwerveModule(Constants.DriveTrain.DT_RR_DRIVE_MC_ID,
		Constants.DriveTrain.DT_RR_TURN_MC_ID,
		Constants.DriveTrain.DT_TURN_P,
		Constants.DriveTrain.DT_TURN_I,
		Constants.DriveTrain.DT_TURN_D,
		Constants.DriveTrain.DT_TURN_IZONE, "dtRR",
		Constants.DriveTrain.DT_RR_WHEEL_DIAM_OFFSET_MM,
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
		Dashboard.Gyro.setGyroAngle(m_gyro.getAngle());
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
	  return m_odometry.getPoseMeters();
	}

	  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }
  /**
   * Returns the turn rate of the robot.
   *
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

	/**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double fwd, double str, double rot, boolean fieldRelative) {
    var swerveModuleStates =
	Constants.Swerve.kDriveKinematics.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(fwd, str, rot, m_gyro.getRotation2d())
				: new ChassisSpeeds(fwd, str, rot));
	Helpers.Debug.debug(fieldRelative
		? ChassisSpeeds.fromFieldRelativeSpeeds(fwd, str, rot, m_gyro.getRotation2d()).toString()
		: new ChassisSpeeds(fwd, str, rot).toString());
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
    m_dtFL.setDesiredState(swerveModuleStates[0]);
    m_dtFR.setDesiredState(swerveModuleStates[1]);
    m_dtRL.setDesiredState(swerveModuleStates[2]);
    m_dtRR.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
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

	public static void setAllTurnPower(double power) {
		setTurnPower(power, power, power, power);
	}

	public static void setAllDrivePower(double power) {
		setDrivePower(power, power, power, power);
	}

	public static void setAllLocation(double loc) {
		setLocation(loc, loc, loc, loc);
	}

	public static boolean isdtLFTurnEncConnected() {
		return m_dtFL.isTurnEncConnected();
	}

	public static boolean isdtFRTurnEncConnected() {
		return m_dtFR.isTurnEncConnected();
	}

	public static boolean isdtRLTurnEncConnected() {
		return m_dtRL.isTurnEncConnected();
	}

	public static boolean isdtRRTurnEncConnected() {
		return m_dtRR.isTurnEncConnected();
	}

	public void stopAllDrive() {
	    m_dtFL.stopDrive();
		m_dtFR.stopDrive();
		m_dtRL.stopDrive();
		m_dtRR.stopDrive();
	}


	public static double getAverageError() {
		return (Math.abs(m_dtFL.getError()) + Math.abs(m_dtFR.getError())
				+ Math.abs(m_dtRL.getError()) + Math.abs(m_dtRR
				.getError())) / 4d;
	}

	/*
	 * Drive methods
	 */
	public static void swerveDrive(double fwd, double str, double rot) {
		//TODO: If fwd, str, and rot is all 0.0, shortcut this math and do a static thing to save cpu
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
	//#region m_gyro STUFF
	public void resetGyro() {
		Helpers.Debug.debug("Gyro Reset");
		m_gyro.reset();
		resetOdometry(getPose());
	}

	public static double getm_gyroAngle() {
		return m_gyro.getAngle();
	}

	public static double getm_gyroAngleInRad() {
		return m_gyro.getAngle() * (Math.PI / 180d);
	}
	//#endregion m_gyro STUFF

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
		System.out.print("saveAllHomes: " + outString);

		m_dtFL.setHomePos(flHome);
		m_dtFR.setHomePos(frHome);
		m_dtRL.setHomePos(rlHome);
		m_dtRR.setHomePos(rrHome);

		try {
			bw.write(outString);
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
						// System.out.print("readAllHomes: flHome="+flHome+"\n");
						break;
					case "frHome":
						frHome = Integer.parseInt(part[1]);
						// System.out.print("readAllHomes: frHome="+frHome+"\n");
						break;
					case "rlHome":
						rlHome = Integer.parseInt(part[1]);
						// System.out.print("readAllHomes: rlHome="+rlHome+"\n");
						break;
					case "rrHome":
						rrHome = Integer.parseInt(part[1]);
						// System.out.print("readAllHomes: rrHome="+rrHome+"\n");
						break;
				}
				line = br.readLine(); //beg for more bread
			}
			m_dtFL.setHomePos(flHome);
			m_dtFR.setHomePos(frHome);
			m_dtRL.setHomePos(rlHome);
			m_dtRR.setHomePos(rrHome);

			br.close();
			fr.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void startCalibrationMode() {
		System.out.println("startCalibrationMode");
		lockDriveControls(true);
		setAllTurnBrakeMode(false);

		m_dtFL.setTurnPowerPercent(0);
		m_dtFR.setTurnPowerPercent(0);
        m_dtRL.setTurnPowerPercent(0);
        m_dtRR.setTurnPowerPercent(0);
	}

	public void stopCalibrationMode() {
		System.out.println("stopCalibrationMode");
		getAllAbsPos();
		saveAllHomes();
		// readAllHomes();
		setAllTurnBrakeMode(true);
		lockDriveControls(false);
	}

	public void moveAllToHomes() {
		System.out.println("moveAllToHomes");
		readAllHomes();
		m_dtFL.setTurnLocationInEncoderTicks(flHome);
		m_dtFR.setTurnLocationInEncoderTicks(frHome);
		m_dtRL.setTurnLocationInEncoderTicks(rlHome);
		m_dtRR.setTurnLocationInEncoderTicks(rrHome);
	}

	public boolean isAllTurnAtHome() {
		if (
			m_dtFL.isTurnAtHome(flHome) &&
			m_dtFR.isTurnAtHome(frHome) &&
			m_dtRL.isTurnAtHome(rlHome) &&
			m_dtRR.isTurnAtHome(rrHome)
			) {
				return true;
			}
		return false;
	}

	public void moveAllToMechZero() {
		System.out.println("moveAllToMechZero");
		m_dtFL.setTurnLocationInEncoderTicks(Constants.DriveTrain.DT_FL_MECHZERO);
		m_dtFR.setTurnLocationInEncoderTicks(Constants.DriveTrain.DT_FR_MECHZERO);
		m_dtRL.setTurnLocationInEncoderTicks(Constants.DriveTrain.DT_RL_MECHZERO);
		m_dtRR.setTurnLocationInEncoderTicks(Constants.DriveTrain.DT_RR_MECHZERO);
	}

	public boolean isAllTurnAtMechZero() {
		if (
			m_dtFL.isTurnAtHome(Constants.DriveTrain.DT_FL_MECHZERO) &&
			m_dtFR.isTurnAtHome(Constants.DriveTrain.DT_FR_MECHZERO) &&
			m_dtRL.isTurnAtHome(Constants.DriveTrain.DT_RL_MECHZERO) &&
			m_dtRR.isTurnAtHome(Constants.DriveTrain.DT_RR_MECHZERO)
			) {
				return true;
			}
		return false;
	}
	//#endregion HOMING AND CALIBRATION
}