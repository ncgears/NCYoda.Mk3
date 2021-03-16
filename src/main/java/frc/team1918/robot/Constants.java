
package frc.team1918.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

@SuppressWarnings("unused")
public class Constants {
    public static final class Global {
        //Global Constants
        public final static int kTimeoutMs = 30; //Timeout for reporting in DS if action fails, set to 0 to skip confirmation
        public final static int PID_PRIMARY = 0;  //Talon PID slot for primary loop
        public final static int PID_TURN = 1; //Talon PID slot for auxillary loop
        public final static int ROBOT_WIDTH = 23; //Width of the robot frame
        public final static int ROBOT_LENGTH = 26; //Length of the robot frame
        public final static boolean DEBUG_ENABLED_DEFAULT = true; //Default starting state of debug mode
        public final static int DEBUG_RECURRING_TICKS = 250; //Periodic cycles for recurring debug messages
        public final static int DASH_RECURRING_TICKS = 250; //Periodic cycles for dashboard updates
        public final static boolean HOME_ON_TELEOP = true; //Enable home sequence at beginning of teleop
        public final static boolean HOME_ON_AUTON = !HOME_ON_TELEOP; //Enable home sequence at beginning of auton, always opposit teleop
        public final static boolean ALLOW_CAL_IN_TELEOP = true; //Allow calibration mode during teleop
    }

    public static final class Air {
        public final static boolean AIR_DISABLED = false; //Set to true to disable compressor
        public final static int AIR_COLLECTOR1_ID = 0; //ID of solonoid for collector stage 1
        public final static int AIR_COLLECTOR2_ID = 1; //ID of solonoid for collector stage 2
        public final static int AIR_HOOD_ID = 2; //ID of solonoid for hood control
        public final static int AIR_ANTIGRAV_ID = 3; //ID of solonoid for climber anti-backdrive
        public final static int AIR_SPARE_ID = 4; //ID of solonoid for spare 
        //which state for solonoids are the knowns
        public final static boolean AIR_COLL1_DOWN = true;
        public final static boolean AIR_COLL2_DOWN = true;
        public final static boolean AIR_HOOD_UP = true;
    }
    
    public static final class Shooter {
        public final static int SHOOTER_WALL_RPM = 2650; //2650; //Wall
        public final static boolean SHOOTER_WALL_HOOD = !Air.AIR_HOOD_UP;
        public final static int SHOOTER_SHORT_RPM = 3780; //Short
        public final static boolean SHOOTER_SHORT_HOOD = Air.AIR_HOOD_UP;
        public final static int SHOOTER_LINE_RPM = 3225; //Line
        public final static boolean SHOOTER_LINE_HOOD = Air.AIR_HOOD_UP;
        public final static int SHOOTER_TRENCH_RPM = 3300; //Trench
        public final static boolean SHOOTER_TRENCH_HOOD = Air.AIR_HOOD_UP;

        public final static int SHOOTER_SHOOT_MC_ID = 12; //ID of the Shooter SparkMAX
        public final static int SHOOTER_FEED1_MC_ID = 14; //ID of the Shooter stage 1 TalonSRX
        public final static int SHOOTER_FEED2_MC_ID = 6; //ID of the Shooter stage 2 TalonSRX
        public final static boolean SHOOTER_FEED1_INVERT = true; //Should the Feed1 talon be inverted
        public final static boolean SHOOTER_FEED2_INVERT = false; //Should the Feed2 talon be inverted
        public final static boolean SHOOTER_FEED_DISABLED = false; //Disable the feed for testing
        public final static double SHOOTER_FEED1_SPEED = 1.0; //Speed for the Feed1 talon
        public final static double SHOOTER_FEED2_SPEED = 1.0; //Speed for the Feed2 talon
        public final static double SHOOTER_MAX_RPM = 5400; //Maximum RPMs for setting shooter
        public final static double SHOOTER_MIN_RPM = 2200; //Minimum RPMs for setting shooter
        public final static double SHOOTER_SPEED_INCREMENT = 25; //Incremental amounts to adjust shooter throttle
        public final static boolean SHOOTER_SHOOT_INVERT = true;
        public final static double SHOOTER_PID_P = 9e-6;
        public final static double SHOOTER_PID_I = 4e-7; 
        public final static double SHOOTER_PID_D = 0; 
        public final static double SHOOTER_PID_IZONE = 0;
        public final static double SHOOTER_PID_FF = 0; //Feed forward - This should be 1/MAX_RPM
        //Feed Forward Explanation: Reference output times desired result.. IE, if the MAX is 5400 RPM at a reference output of 1.0, then 1/5400 FF is appropriate.  
        //This supplies the controller with a known output value to get the desired target result, then the PID can handle deviations from the known behavior.
        //We should get the MAX RPM from trial and error by running with a reference power of 100% and measuring the actual result from the encoder
    }

    public static final class Mixer {
        public final static int MIXER_MC_ID = 13; //ID of the Mixer talon
        public final static double MIXER_SPEED = 0.75; //Speed for the Mixer talon
    }

    public static final class Collector {
        public final static int COLLECTOR_MC_ID = 15; //ID of the Collector talon
        public final static double COLLECTOR_SPEED = 1.0; //Speed for the Collector talon
    }

    public static final class Swerve {
        public static final boolean USE_OPTIMIZATION = false; //false to disable shortest path optimization
        public static final boolean DISABLE_FL = false; //Disable FL Module
        public static final boolean DISABLE_FR = false; //Disable FR Module
        public static final boolean DISABLE_RL = false; //Disable RL Module
        public static final boolean DISABLE_RR = false; //Disable RR Module
        
        // swerve control definitions
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
        public static final double kMaxSpeedMetersPerSecond = 4.176; //13.7fps per Mike
        public static final boolean kGyroReversed = false;
        //Forward Positive, Left Positive, Up Positive (NWU Convention)
        public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(Helpers.General.inToMeters(Global.ROBOT_LENGTH / 2), Helpers.General.inToMeters(-Global.ROBOT_WIDTH / 2)),
            new Translation2d(Helpers.General.inToMeters(Global.ROBOT_LENGTH / 2), Helpers.General.inToMeters(Global.ROBOT_WIDTH / 2)),
            new Translation2d(Helpers.General.inToMeters(-Global.ROBOT_LENGTH / 2), Helpers.General.inToMeters(-Global.ROBOT_WIDTH / 2)),
            new Translation2d(Helpers.General.inToMeters(-Global.ROBOT_LENGTH / 2), Helpers.General.inToMeters(Global.ROBOT_WIDTH / 2))
        );
    }

    public static final class DriveTrain {
        //DriveTrain definitions
        ////Home Tuning
        public final static String DT_HOMES_FILE = "/home/lvuser/swerveHomes2021.txt"; //The file where we save the homes data to persist reboots
        public final static double DT_HOME_DELAY = 0.75; //Seconds to wait for homing before reset encoders
        public final static int DT_HOME_MARGIN_OF_ERROR = 20; //Encoder ticks margin to consider home (plus or minus this amount)
        ////Turn Tuning
        public final static double DT_TURN_MULT_STATIONARY = 0.5; //Turn speed multiplier while not moving
        public final static double DT_TURN_MULT_MOVING = 0.9; //Turn speed multiplier while moving
        public final static boolean DT_TURN_MULT_BEFORE_DB = true; //Apply turn multiplier before deadband
        public final static double DT_TURN_ENCODER_FULL_ROTATION = 4096d;
        public final static boolean DT_USE_FIELD_CENTRIC = true; //Set to true to use field-centric drive
        ////Drive Tuning
        public final static boolean DT_DRIVE_DISABLED = false; //Set to true to disable the drive motors (for lab)
        public final static double DT_WHEEL_DIAM_MM = 77.1; //diameter of drive wheels in millimeters
        public final static int DT_DRIVE_FIRST_GEARONE = 21; //swerve drive first gear set input teeth
        public final static int DT_DRIVE_FIRST_GEARTWO = 36; //swerve drive first gear set output teeth
        public final static int DT_DRIVE_SECOND_GEARONE = 15; //swerve drive second gear set input teeth
        public final static int DT_DRIVE_SECOND_GEARTWO = 45; //swerve drive second gear set output teeth
        public final static double DT_DRIVE_CONVERSION_FACTOR = (DT_DRIVE_FIRST_GEARONE / DT_DRIVE_FIRST_GEARTWO) * (DT_DRIVE_SECOND_GEARONE / DT_DRIVE_SECOND_GEARTWO); //Conversion factor to correct RPM from SparkMax getVelocity()
        ////Turn PID Tuning
        public final static double DT_TURN_P = 8.0;
        public final static double DT_TURN_I = 0.0;
        public final static double DT_TURN_D = 80.0;
        public final static int DT_TURN_IZONE = 0;
        ////Front Left
        public final static int DT_FL_DRIVE_MC_ID = 2; //Front Left Drive Motor Controller ID //SPARKMAX
        public final static int DT_FL_TURN_MC_ID = 7; //Front Left Turn Motor Controller ID //TALONSRX
        public final static int DT_FL_MECHZERO = 0; //Front Left encoder value at mechanical zero, only change if mechanics broke things
        public final static double DT_FL_WHEEL_DIAM_OFFSET_MM = 0.0; //Front Left offset to wheel diam to account for wear
        ////Front Right
        public final static int DT_FR_DRIVE_MC_ID = 16; //Front Right Drive Motor Controller ID //SPARKMAX
        public final static int DT_FR_TURN_MC_ID = 4; //Front Right Turn Motor Controller ID //TALONSRX
        public final static int DT_FR_MECHZERO = 0; //Front Right encoder value at mechanical zero, only change if mechanics broke things
        public final static double DT_FR_WHEEL_DIAM_OFFSET_MM = 0.0; //Front Right offset to wheel diam to account for wear
        ////Rear Left
        public final static int DT_RL_DRIVE_MC_ID = 3; //Rear Left Drive Motor Controller ID //SPARKMAX
        public final static int DT_RL_TURN_MC_ID = 8; //Rear Left Turn Motor Controller ID //TALONSRX
        public final static int DT_RL_MECHZERO = 0; //Rear Left encoder value at mechanical zero, only change if mechanics broke things
        public final static double DT_RL_WHEEL_DIAM_OFFSET_MM = 0.0; //Rear Left offset to wheel diam to account for wear
        ////Rear Right
        public final static int DT_RR_DRIVE_MC_ID = 1; //Rear Right Drive Motor Controller ID //SPARKMAX
        public final static int DT_RR_TURN_MC_ID = 11; //Rear Right Turn Motor Controller ID //TALONSRX
        public final static int DT_RR_MECHZERO = 0; //Rear Right encoder value at mechanical zero, only change if mechanics broke things
        public final static double DT_RR_WHEEL_DIAM_OFFSET_MM = 0.0; //Rear Right offset to wheel diam to account for wear
    }
    
    public static final class OI {
        /**
         * This class is based on 2 Logitech controllers, a driver and an operator, setup for swerve drive
         */
        public final static int OI_JOY_DRIVE = 0; //ID of Driver Joystick
        public final static int OI_JOY_OPER = 1; //ID of Operator Joystick
        public final static double OI_JOY_DEADBAND = 0.1; //Deadband for analog joystick axis
        
        //Logitech Controller buttons  //In Java, buttons are 1-based array, BUT NOT AXIS
        //DO NOT EDIT THESE!!!
        private final static int LOGITECH_BTN_A = 1; //A Button
        private final static int LOGITECH_BTN_B = 2; //B Button
        private final static int LOGITECH_BTN_X = 3; //X Button
        private final static int LOGITECH_BTN_Y = 4; //Y Button
        private final static int LOGITECH_BTN_LB = 5; //Left Bumper (L1)
        private final static int LOGITECH_BTN_RB = 6; //Right Bumper (R1)
        private final static int LOGITECH_BTN_BACK = 7; //Back Button (Select)
        private final static int LOGITECH_BTN_START = 8; //Start Button
        private final static int LOGITECH_BTN_L = 9; //Left Stick Press (L3)
        private final static int LOGITECH_BTN_R = 10; //Right Stick Press (R3)
        private final static int LOGITECH_AXIS_LH = 0; //Left Analog Stick horizontal
        private final static int LOGITECH_AXIS_LV = 1; //Left Analog Stick vertical
        private final static int LOGITECH_AXIS_LT = 2; //Analog Left Trigger
        private final static int LOGITECH_AXIS_RT = 3; //Analog Right Trigger
        private final static int LOGITECH_AXIS_RH = 4; //Right Analog Stick horizontal
        private final static int LOGITECH_AXIS_RV = 5; //Right Analog Stick vertical
        private final static int LOGITECH_DPAD_UP = 0;
        private final static int LOGITECH_DPAD_UPRIGHT = 45;
        private final static int LOGITECH_DPAD_RIGHT = 90;
        private final static int LOGITECH_DPAD_DNRIGHT = 135;
        private final static int LOGITECH_DPAD_DN = 180;
        private final static int LOGITECH_DPAD_DNLEFT = 225;
        private final static int LOGITECH_DPAD_LEFT = 270;
        private final static int LOGITECH_DPAD_UPLEFT = 315;
        private final static int LOGITECH_DPAD_IDLE = -1; 
    
    //Temporary
        public final static int DRIVE_BTN_CALIBRATE_START = LOGITECH_BTN_B; //temporary for testing
        public final static int DRIVE_BTN_CALIBRATE_STOP = LOGITECH_BTN_A; //temporary for testing

    //Driver Controller
        public final static int DRIVE_AXIS_STRAFE = LOGITECH_AXIS_LH; //Axis that moves the robot side to side on the field
        public final static int DRIVE_AXIS_FWD = LOGITECH_AXIS_LV; //Axis that moves the robot up and down the field
        public final static int DRIVE_AXIS_TURN = LOGITECH_AXIS_RH; //Axis that controls the rotation of the robot
        public final static int DRIVE_BTN_ALLUP = LOGITECH_BTN_A; //Move collector to Up position
        public final static int DRIVE_BTN_ANTIGRAV = LOGITECH_BTN_B; //Engage anti-backdrive for climber
        public final static int DRIVE_BTN_MIXER_FEED = LOGITECH_BTN_X; //Run the mixer in the forward direction
        public final static int DRIVE_BTN_MIXER_FEEDSTUCK = LOGITECH_BTN_LB; //Reverse the mixer direction to unstick power cells
        public final static int DRIVE_BTN_HOMESWERVE = LOGITECH_BTN_Y; //Home the swerve modules
        public final static int DRIVE_BTN_MECHZERO = LOGITECH_BTN_BACK; //DRIVER MECHZERO and OPER MECHZERO are required for this
        public final static int DRIVE_BTN_TOG_DEBUG = LOGITECH_BTN_START; //Toggle the debugging console messages
        public final static int DRIVE_DPAD_GYRO_RESET = LOGITECH_DPAD_LEFT;
        //Drive controller DPAD used as range selector for shooter speed (top half and bottom half)
        public final static int DRIVE_DPAD_THROTUP_UL = LOGITECH_DPAD_UPLEFT;
        public final static int DRIVE_DPAD_THROTUP_UP = LOGITECH_DPAD_UP;
        public final static int DRIVE_DPAD_THROTUP_UR = LOGITECH_DPAD_UPRIGHT;
        public final static int DRIVE_DPAD_THROTDN_DL = LOGITECH_DPAD_DNLEFT;
        public final static int DRIVE_DPAD_THROTDN_DN = LOGITECH_DPAD_DN;
        public final static int DRIVE_DPAD_THROTDN_DR = LOGITECH_DPAD_DNRIGHT;
    //Operator Controller        
        public final static int OPER_AXIS_CLIMB = LOGITECH_AXIS_LV; //Axis that controls the climber up and down
        public final static int OPER_AXIS_COLLECTOR_OUT = LOGITECH_AXIS_RT; //Axis that runs the collector out (actually a trigger button)
        public final static int OPER_BTN_SHOOT_WALL = LOGITECH_BTN_A; //Shoot from at the wall
        public final static int OPER_BTN_SHOOT_SHORT = LOGITECH_BTN_B; //Shoot from close to the wall
        public final static int OPER_BTN_SHOOT_LINE = LOGITECH_BTN_X; //Shoot from the initiation line
        public final static int OPER_BTN_SHOOT_TRENCH = LOGITECH_BTN_Y; //Shoot from the trench
        public final static int OPER_BTN_TOG_MIDDOWN = LOGITECH_BTN_LB; //Toggle collector arm between middle and down position
        public final static int OPER_BTN_COLLECTOR_IN = LOGITECH_BTN_RB; //Run the collector in
        public final static int OPER_BTN_MECHZERO = LOGITECH_BTN_BACK; //DRIVER MECHZERO and OPER MECHZERO are required for this
        public final static int OPER_DPAD_COLLECTOR_UP = LOGITECH_DPAD_UP; //Move collector to up
        public final static int OPER_DPAD_COLLECTOR_MID = LOGITECH_DPAD_RIGHT; //Move collector to middle
        public final static int OPER_DPAD_COLLECTOR_DOWN = LOGITECH_DPAD_DN; //Move collector down
    }
}

