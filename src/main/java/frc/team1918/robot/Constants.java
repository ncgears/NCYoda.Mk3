
package frc.team1918.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

//Sometimes it is useful to comment out the following to see what variables or what controller buttons are not assigned yet
@SuppressWarnings("unused") //We silence the "unused variables" warnings in VSCode
/**
 * Constants let us quickly define the characteristics of our robot without having to search through code
 */
public class Constants {
    /**
     * Constants that are Global for the robot
     */
    public static final class Global {
        //Global Constants
        public final static boolean CAMERA_ENABLED = false; //set to false if UsbCamera is removed
        public final static boolean SWERVE_SENSOR_NONCONTINUOUS = false;
        public final static int kTimeoutMs = 30; //Timeout for reporting in DS if action fails, set to 0 to skip confirmation
        public final static int PID_PRIMARY = 0;  //Talon PID slot for primary loop
        public final static int PID_TURN = 1; //Talon PID slot for auxillary loop
        public final static int ROBOT_WIDTH = 23; //Width of the robot frame
        public final static int ROBOT_LENGTH = 26; //Length of the robot frame
        public final static boolean DEBUG_ENABLED_DEFAULT = true; //Default starting state of debug mode
        public final static int DEBUG_RECURRING_TICKS = 100; //Periodic cycles for recurring debug messages
        public final static int DASH_RECURRING_TICKS = 50; //Periodic cycles for dashboard updates
        public final static boolean HOME_ON_TELEOP = true; //Enable home sequence at beginning of teleop
        public final static boolean HOME_ON_AUTON = !HOME_ON_TELEOP; //Enable home sequence at beginning of auton, always opposit teleop
        public final static boolean ALLOW_CAL_IN_TELEOP = true; //Allow calibration mode during teleop
    }

    /**
     * Constants for the Autonomous subsystem
     */
    public static final class Auton {
        public final static double kMaxSpeedMetersPerSecond = 0.0;
        public final static double kMaxAccelerationMetersPerSecondSquared = 0.0;
    }

    /**
     * Constants for the Pneumantics system
     * This is not a subsystem, the pneumatics are controlled directly in their respective subsystems
     */
    public static final class Air {
        public final static boolean AIR_DISABLED = false; //Set to true to disable compressor
        public final static int AIR_COLLECTOR1_ID = 0; //ID of solonoid for collector stage 1
        public final static int AIR_COLLECTOR2_ID = 1; //ID of solonoid for collector stage 2
        public final static int AIR_HOOD_ID = 2; //ID of solonoid for hood control
        public final static int AIR_ANTIGRAV_ID = 3; //ID of solonoid for climber anti-backdrive
        public final static int AIR_SPARE_ID = 4; //ID of solonoid for spare 
        public final static boolean AIR_COLL1_DOWN = true; //State of the solenoid when COLL1 is down
        public final static boolean AIR_COLL2_DOWN = true; //State of the solenoid when COLL2 is down
        public final static boolean AIR_HOOD_UP = true; //State of the solenoid when HOOD is up
    }
    
    /**
     * Constants for the Shooter subsystem
     */
    public static final class Shooter {
        public final static boolean FEED_isDisabled = false; //Disable the feed for testing

        public final static int SHOOT_WALL_RPM = 2400; //Wall
        public final static boolean SHOOT_WALL_HOOD = !Air.AIR_HOOD_UP;
        public final static int SHOOT_SHORT_RPM = 3400; //Short
        public final static boolean SHOOT_SHORT_HOOD = Air.AIR_HOOD_UP;
        public final static int SHOOT_LINE_RPM = 3000; //Line
        public final static boolean SHOOT_LINE_HOOD = !Air.AIR_HOOD_UP;
        public final static int SHOOT_TRENCH_RPM = 3150; //Trench
        public final static boolean SHOOT_TRENCH_HOOD = Air.AIR_HOOD_UP;

        public final static int SHOOT_MC_ID = 12; //ID of the Shooter SparkMAX
        public final static int FEED_1_MC_ID = 14; //ID of the Shooter stage 1 TalonSRX
        public final static int FEED_2_MC_ID = 6; //ID of the Shooter stage 2 TalonSRX
        public final static boolean FEED_1_isInverted = true; //Should the Feed1 talon be inverted
        public final static double FEED_1_SPEED = 1.0; //Speed for the Feed1 talon
        public final static boolean FEED_2_isInverted = false; //Should the Feed2 talon be inverted
        public final static double FEED_2_SPEED = 1.0; //Speed for the Feed2 talon
        public final static double SHOOT_MAX_RPM = 5600; //Maximum RPMs for setting shooter
        public final static double SHOOT_MIN_RPM = 2200; //Minimum RPMs for setting shooter
        public final static double SHOOT_speedIncrement = 25; //Incremental amounts to adjust shooter throttle
        public final static boolean SHOOT_isInverted = true;
        public final static double SHOOT_PID_P = 0.001; //9e-6;
        public final static double SHOOT_PID_I = 0; //4e-7; 
        public final static double SHOOT_PID_D = 0; 
        public final static double SHOOT_PID_IZONE = 0;
        public final static double SHOOT_PID_FF = 1/SHOOT_MAX_RPM; //Feed forward - This should be 1/MAX_RPM
        //Feed Forward Explanation: Reference output times desired result.. IE, if the MAX is 5400 RPM at a reference output of 1.0, then 1/5400 FF is appropriate.  
        //This supplies the controller with a known output value to get the desired target result, then the PID can handle deviations from the known behavior.
        //We should get the MAX RPM from trial and error by running with a reference power of 100% and measuring the actual result from the encoder
    }

    /**
     * Constants for the Mixer subsystem
     */
    public static final class Mixer {
        public final static int MIXER_MC_ID = 13; //ID of the Mixer talon
        public final static double MIXER_SPEED = 0.5; //Speed for the Mixer talon
    }

    /**
     * Constants for the Collector subsystem
     */
    public static final class Collector {
        public final static int COLLECTOR_MC_ID = 15; //ID of the Collector talon
        public final static double COLLECTOR_SPEED = 1.0; //Speed for the Collector talon
    }

    /**
     * Constants for the Swerve Modules
     */
    public static final class Swerve {
        public static final boolean USE_OPTIMIZATION = false; //false to disable shortest path optimization
        public static final boolean USE_DRIVE_PID = false; //true to enable PID based drive control
        public static final boolean DISABLE_FL = false; //Disable FL Module
        public static final boolean DISABLE_FR = true; //Disable FR Module
        public static final boolean DISABLE_RL = true; //Disable RL Module
        public static final boolean DISABLE_RR = true; //Disable RR Module
        // swerve control definitions
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
        public static final double kMaxSpeedMetersPerSecond = 3.677; //12.0fps calculated; 13.7fps per Mike
        public static final boolean kGyroReversed = false;
        //Forward Positive, Left Positive, Up Positive (NWU Convention)
        public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(Global.ROBOT_LENGTH / 2), Units.inchesToMeters(-Global.ROBOT_WIDTH / 2)),
            new Translation2d(Units.inchesToMeters(Global.ROBOT_LENGTH / 2), Units.inchesToMeters(Global.ROBOT_WIDTH / 2)),
            new Translation2d(Units.inchesToMeters(-Global.ROBOT_LENGTH / 2), Units.inchesToMeters(-Global.ROBOT_WIDTH / 2)),
            new Translation2d(Units.inchesToMeters(-Global.ROBOT_LENGTH / 2), Units.inchesToMeters(Global.ROBOT_WIDTH / 2))
        );
        /**
         * Constants for Front Left Swerve Module
         */
        public static final class FL {  //TODO: PID Tuning
            public static final int DRIVE_MC_ID = 33; //Falcon500 Motor Controller ID
            public static final int TURN_MC_ID = 7; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is correct, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = 5.5; //PID P //6.0
            public static final double TURN_kI = 0.005; //PID I //0
            public static final double TURN_kD = 0.9; //PID D //0.9
            public static final int TURN_kIZone = 0; //PID IZONE
            public static final int TURN_ALLOWED_ERROR = 2;
            public static final double DRIVE_wheelDiamOffsetMM = 0.0; //offset to wheel diam to account for wear, in mm from nominal (negative for worn wheels)
        }
        /**
         * Constants for Front Right Swerve Module
         */
        public static final class FR {
            public static final int DRIVE_MC_ID = 31; //Falcon500 Motor Controller ID
            public static final int TURN_MC_ID = 4; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is correct, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = 5.5; //PID P
            public static final double TURN_kI = 0.005; //PID I
            public static final double TURN_kD = 0.9; //PID D
            public static final int TURN_kIZone = 0; //PID IZONE
            public static final int TURN_ALLOWED_ERROR = 2;
            public static final double DRIVE_wheelDiamOffsetMM = 0.0; //offset to wheel diam to account for wear, in mm from nominal (negative for worn wheels)
        }
        /**
         * Constants for Rear Left Swerve Module
         */
        public static final class RL {
            public static final int DRIVE_MC_ID = 34; //Falcon500 Motor Controller ID
            public static final int TURN_MC_ID = 8; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is correct, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = 5.5; //PID P
            public static final double TURN_kI = 0.005; //PID I
            public static final double TURN_kD = 0.9; //PID D
            public static final int TURN_kIZone = 0; //PID IZONE
            public static final int TURN_ALLOWED_ERROR = 2;
            public static final double DRIVE_wheelDiamOffsetMM = 0.0; //offset to wheel diam to account for wear, in mm from nominal (negative for worn wheels)
        }
        /**
         * Constants for Rear Right Swerve Module
         */
        public static final class RR { //Rear Right
            public static final int DRIVE_MC_ID = 32; //Falcon500 Motor Controller ID
            public static final int TURN_MC_ID = 11; //TalonSRX Motor Controller ID
            public static final boolean TURN_sensorPhase = false; //When forward/reverse of controller doesn't match forward/reverse of sensor, set to true
            public static final boolean TURN_isInverted = true; //Once sensor phase is correct, we can invert these so fwd always is green, reverse is always is red
            public static final double TURN_kP = 5.5; //PID P
            public static final double TURN_kI = 0.005; //PID I
            public static final double TURN_kD = 0.9; //PID D
            public static final int TURN_kIZone = 0; //PID IZONE
            public static final int TURN_ALLOWED_ERROR = 2;
            public static final double DRIVE_wheelDiamOffsetMM = 0.0; //offset to wheel diam to account for wear, in mm from nominal (negative for worn wheels)
        }
    }

    /**
     * Constants for the DriveTrain subsystem
     */
    public static final class DriveTrain {
        ////Global Tuning
        public final static boolean DT_USE_DRIVESTRAIGHT = true; //enable driveStraight functionality in drive() method
        public final static double DT_DRIVESTRAIGHT_P = 0.065; //kP for driveStraight correction
        public static final double DT_kMaxMetersPerSecond = 3.677; //limit full stick speed meters to 12.0fps
        ////Home Tuning
        public final static String DT_HOMES_FILE = "/home/lvuser/swerveHomes2021.txt"; //The file where we save the homes data to persist reboots
        public final static double DT_HOME_DELAY = 0.75; //Seconds to wait for homing before reset encoders
        public final static int DT_HOME_MARGIN_OF_ERROR = 20; //Encoder ticks margin to consider home (plus or minus this amount)
        ////Turn Tuning
        public final static double DT_TURN_MULT_STATIONARY = 0.5; //Turn speed multiplier while not moving
        public final static double DT_TURN_MULT_MOVING = 0.9; //Turn speed multiplier while moving
        public final static boolean DT_TURN_MULT_BEFORE_DB = true; //Apply turn multiplier before deadband
        public final static int DT_TURN_ENCODER_FULL_ROTATION = 1024;
        public final static boolean DT_USE_FIELD_CENTRIC = true; //Set to true to use field-centric drive
        ////Drive Tuning
        public final static double DT_FWD_MULT = 1.0; //Fwd throttle multiplier
        public final static double DT_STR_MULT = 1.0; //Str throttle multiplier
        public final static boolean DT_DRIVE_DISABLED = false; //Set to true to disable the drive motors (for lab)
        public final static double DT_WHEEL_DIAM_MM = 77.1; //diameter of drive wheels in millimeters
        public final static int DT_DRIVE_FIRST_GEARONE = 21; //swerve drive first gear set input teeth
        public final static int DT_DRIVE_FIRST_GEARTWO = 36; //swerve drive first gear set output teeth
        public final static int DT_DRIVE_SECOND_GEARONE = 15; //swerve drive second gear set input teeth
        public final static int DT_DRIVE_SECOND_GEARTWO = 45; //swerve drive second gear set output teeth
        public final static double DT_DRIVE_CONVERSION_FACTOR = 0.194444444; //first_gearone / first_geartwo * second_gearone / second_geartwo
        // public final static double DT_DRIVE_CONVERSION_FACTOR = (DT_DRIVE_FIRST_GEARONE / DT_DRIVE_FIRST_GEARTWO) * (DT_DRIVE_SECOND_GEARONE / DT_DRIVE_SECOND_GEARTWO); //Conversion factor to correct RPM from SparkMax getVelocity()
        ////Defaults if calibration file fails to read -- this shouldnt happen
        public final static int DT_FL_MECHZERO = 0; //Front Left encoder value at mechanical zero, only change if mechanics broke things
        public final static int DT_FR_MECHZERO = 0; //Front Right encoder value at mechanical zero, only change if mechanics broke things
        public final static int DT_RL_MECHZERO = 0; //Rear Left encoder value at mechanical zero, only change if mechanics broke things
        public final static int DT_RR_MECHZERO = 0; //Rear Right encoder value at mechanical zero, only change if mechanics broke things
    }
    
    /**
     * Constants for the Operator Interface
     * The OI is based on 2 Logitech Controllers, a driver and an operator, setup for swerve drive.
     * The driver left stick controls the forward rate (up/down), and strafe rate (left/right).
     * The driver right stick controls the rotation rate (left/right).
     */
    public static final class OI {
        public final static int OI_JOY_DRIVER = 0; //ID of Driver Joystick
        public final static int OI_JOY_OPER = 1; //ID of Operator Joystick
        public final static double OI_JOY_DEADBAND = 0.1; //Deadband for analog joystick axis

        /**
         * Constants for the Driver controller
         */
        public static final class Driver {
            public final static int AXIS_STRAFE = Logitech.AXIS_LH; //Axis that moves the robot side to side on the field
            public final static int AXIS_FWD = Logitech.AXIS_LV; //Axis that moves the robot up and down the field
            public final static int AXIS_TURN = Logitech.AXIS_RH; //Axis that controls the rotation of the robot
            public final static int BTN_LOCKANGLE = Logitech.BTN_B; //Move collector to Up position
            public final static int BTN_UNLOCKANGLE = Logitech.BTN_A; //Engage anti-backdrive for climber
            public final static int BTN_MIXER_FEED = Logitech.BTN_X; //Run the mixer in the forward direction
            public final static int BTN_MIXER_FEEDSTUCK = Logitech.BTN_LB; //Reverse the mixer direction to unstick power cells
            public final static int BTN_HOMESWERVE = Logitech.BTN_Y; //Home the swerve modules
            public final static int BTN_MECHZERO = Logitech.BTN_BACK; //DRIVER MECHZERO and OPER MECHZERO are required for this
            public final static int BTN_TOG_DEBUG = Logitech.BTN_START; //Toggle the debugging console messages
            public final static int DPAD_GYRO_RESET = Logitech.DPAD_LEFT;
            //Driver controller DPAD used as range selector for shooter speed (top half and bottom half)
            public final static int DPAD_THROTUP_UL = Logitech.DPAD_UPLEFT;
            public final static int DPAD_THROTUP_UP = Logitech.DPAD_UP;
            public final static int DPAD_THROTUP_UR = Logitech.DPAD_UPRIGHT;
            public final static int DPAD_THROTDN_DL = Logitech.DPAD_DNLEFT;
            public final static int DPAD_THROTDN_DN = Logitech.DPAD_DN;
            public final static int DPAD_THROTDN_DR = Logitech.DPAD_DNRIGHT;
        }
        /**
         * Constants for the Operator controller
         */
        public static final class Operator {
            public final static int AXIS_CLIMB = Logitech.AXIS_LV; //Axis that controls the climber up and down
            public final static int AXIS_COLLECTOR_OUT = Logitech.AXIS_RT; //Axis that runs the collector out (actually a trigger button)
            public final static int BTN_SHOOT_WALL = Logitech.BTN_A; //Shoot from at the wall
            public final static int BTN_SHOOT_SHORT = Logitech.BTN_B; //Shoot from close to the wall
            public final static int BTN_SHOOT_LINE = Logitech.BTN_X; //Shoot from the initiation line
            public final static int BTN_SHOOT_TRENCH = Logitech.BTN_Y; //Shoot from the trench
            public final static int BTN_TOG_MIDDOWN = Logitech.BTN_LB; //Toggle collector arm between middle and down position
            public final static int BTN_COLLECTOR_IN = Logitech.BTN_RB; //Run the collector in
            public final static int BTN_MECHZERO = Logitech.BTN_BACK; //DRIVER MECHZERO and OPER MECHZERO are required for this
            public final static int DPAD_COLLECTOR_UP = Logitech.DPAD_UP; //Move collector to up
            public final static int DPAD_COLLECTOR_MID = Logitech.DPAD_RIGHT; //Move collector to middle
            public final static int DPAD_COLLECTOR_DOWN = Logitech.DPAD_DN; //Move collector down
        }
        /**
         * This class defines the hardware button and axis IDs for a Logitech Controller.
         * The buttons array is 1-based, but the axis array is 0-based
         */
        public static final class Logitech {
            //DO NOT EDIT THESE
            private final static int BTN_A = 1; //A Button
            private final static int BTN_B = 2; //B Button
            private final static int BTN_X = 3; //X Button
            private final static int BTN_Y = 4; //Y Button
            private final static int BTN_LB = 5; //Left Bumper (L1)
            private final static int BTN_RB = 6; //Right Bumper (R1)
            private final static int BTN_BACK = 7; //Back Button (Select)
            private final static int BTN_START = 8; //Start Button
            private final static int BTN_L = 9; //Left Stick Press (L3)
            private final static int BTN_R = 10; //Right Stick Press (R3)
            private final static int AXIS_LH = 0; //Left Analog Stick horizontal
            private final static int AXIS_LV = 1; //Left Analog Stick vertical
            private final static int AXIS_LT = 2; //Analog Left Trigger
            private final static int AXIS_RT = 3; //Analog Right Trigger
            private final static int AXIS_RH = 4; //Right Analog Stick horizontal
            private final static int AXIS_RV = 5; //Right Analog Stick vertical
            private final static int DPAD_UP = 0;
            private final static int DPAD_UPRIGHT = 45;
            private final static int DPAD_RIGHT = 90;
            private final static int DPAD_DNRIGHT = 135;
            private final static int DPAD_DN = 180;
            private final static int DPAD_DNLEFT = 225;
            private final static int DPAD_LEFT = 270;
            private final static int DPAD_UPLEFT = 315;
            private final static int DPAD_IDLE = -1; 
        }
    }
}

