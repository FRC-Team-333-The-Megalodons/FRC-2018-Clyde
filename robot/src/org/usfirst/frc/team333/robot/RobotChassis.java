package org.usfirst.frc.team333.robot;

import org.usfirst.frc.team333.robot.RobotMap.DigitalSensorPort;
import org.usfirst.frc.team333.robot.RobotMap.MirrorMode;
import org.usfirst.frc.team333.robot.RobotMap.SolenoidPort;
import org.usfirst.frc.team333.robot.RobotMap.VictorPort;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SPI;

public class RobotChassis {

    public static final int AVERAGING_QUEUE_SIZE = 10;
    public static final double ANGLE_COMPARE_PRECISION = 1.0;
    public static final double ANGLING_DRIVE_THRESHOLD = 10.0;
    public static final double distancePerPulse = 5 * 3.1415 / 360;

    public RobotChassis(RobotMech mech) {
        /* Instantiate the compressor */
        try {
            m_compressor = new Compressor(RobotMap.CompressorPort.MAIN_COMPRESSOR);
            RobotDashboard.getInstance().setCompressor(m_compressor);
        } catch (Exception ex) {
            DriverStation.reportError("Could not start Compressor\n", false);
        }

        /* Instantiate the NavX */
        try {
            // TODO: For some reason, this doesn't throw gracefully, and causes
            // everything to crash if it's not connected.
            m_gyro = new NavXGyro(SPI.Port.kMXP);
            m_gyro.reset();
            RobotDashboard.getInstance().setGyro(m_gyro);
        } catch (Exception ex) {
            DriverStation.reportError("Could not initialize NavX\n", false);
        }

        /* Instantiate the Left and Right Encoders for the Drivetrain */
        try {
            Encoder left = new Encoder(DigitalSensorPort.ENCODER_LEFT_A, DigitalSensorPort.ENCODER_LEFT_B, true);
            Encoder right = new Encoder(DigitalSensorPort.ENCODER_RIGHT_A, DigitalSensorPort.ENCODER_RIGHT_B, false);

            m_encoder = new DriveTrainEncoder(left, right);
            m_encoder.reset();
            m_encoder.setDistancePerPulse(distancePerPulse);
            RobotDashboard.getInstance().setDriveTrainEncoder(m_encoder);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate DriveTrainEncoder\n", false);
        }

        /* Initiate the RobotDrive Chassis and Tranmission Solenoids */
        try {
            m_transmission = new Solenoidal(SolenoidPort.DRIVE_TRANS_LOW, SolenoidPort.DRIVE_TRANS_HIGH);
            RobotDashboard.getInstance().setTransmission(m_transmission);

            SpeedController leftDrive = new MultiSpeedController(new Victor(VictorPort.DRIVE_LEFT_1),
                    new Victor(VictorPort.DRIVE_LEFT_2), new Victor(VictorPort.DRIVE_LEFT_3));
            SpeedController rightDrive = new MultiSpeedController(new Victor(VictorPort.DRIVE_RIGHT_1),
                    new Victor(VictorPort.DRIVE_RIGHT_2), new Victor(VictorPort.DRIVE_RIGHT_3));
            m_rawDiffDrive = new DifferentialDrive(leftDrive, rightDrive);

            // Also construct the various other drive wrappers we'll need.
            m_teleopTransDrive = new TeleopTransDrive(m_rawDiffDrive, m_transmission);
            m_autonStraightDrive = new AutonStraightDrive(m_rawDiffDrive, m_gyro, m_encoder, mech);
            m_autonTurnDrive = new AutonTurnDrive(m_rawDiffDrive, m_gyro);
            m_autonRawForwardDrive = new AutonRawForwardDrive(m_rawDiffDrive, m_encoder);

        } catch (Exception ex) {
            DriverStation.reportError("Could not start DriveTrain/Transmission\n", false);
        }

        System.out.println("Chassis initialized.");
    }

    public void lowTransmission() {
        m_teleopTransDrive.lowTransmission();
    }

    public void highTransmission() {
        m_teleopTransDrive.highTransmission();
    }

    public void stop() {
        m_rawDiffDrive.arcadeDrive(0.0, 0.0);
    }

    public void periodic(Joystick joystick, Double abs_limit) {
        if (joystick == null) {
            DriverStation.reportError("No joystick, cannot run Chassis periodic\n", false);
            return;
        }

        if (m_teleopTransDrive != null) {
            m_teleopTransDrive.arcadeDrive(joystick, abs_limit); // m_drive with arcade style
        }

        if (m_compressor != null) {
            m_compressor.setClosedLoopControl(true);
        }
    }

    public void resetGyro() {
        if (m_gyro == null) {
            return;
        }
        m_gyro.reset();
    }

    public void resetGyro(double multiplier) {
        resetGyro();
        m_gyro.setMultiplier(multiplier);
    }

    public void resetEncoder() {
        m_encoder.reset();
    }

    public double averageEncoderDistance() {
        return -m_encoder.averageEncoderDistance();
    }

    public PIDSource getStraightPIDSource() {
        return m_autonStraightDrive;
    }

    public PIDOutput getStraightPIDOutput() {
        return m_autonStraightDrive;
    }

    public PIDSource getTurnPIDSource() {
        return m_autonTurnDrive;
    }

    public PIDOutput getTurnPIDOutput() {
        return m_autonTurnDrive;
    }

    public PIDOutput getRawForwardPIDOutput() {
        return m_autonRawForwardDrive;
    }

    private NavXGyro m_gyro;
    private DriveTrainEncoder m_encoder;
    private DifferentialDrive m_rawDiffDrive;

    // These each contain references to some/all of the above sensors/motors.
    private TeleopTransDrive m_teleopTransDrive;
    private AutonStraightDrive m_autonStraightDrive;
    private AutonTurnDrive m_autonTurnDrive;
    private AutonRawForwardDrive m_autonRawForwardDrive;

    private Compressor m_compressor;
    private Solenoidal m_transmission;
}

class RawForwardDriveController {
    public RawForwardDriveController(RobotChassis chassis) {
        this(chassis, true);
    }

    public RawForwardDriveController(RobotChassis chassis, boolean reset) {
        if (reset) {
            chassis.resetEncoder();
            chassis.resetGyro();
        }
        m_controller = new DriveTrainController(chassis.getStraightPIDSource(), chassis.getRawForwardPIDOutput());
        m_chassis = chassis;
    }

    public boolean setTarget(double target) {
        m_chassis.lowTransmission();
        return m_controller.setTarget(target);
    }

    public boolean hasArrived() {
        return m_controller.hasArrived();
    }

    private DriveTrainController m_controller;
    private RobotChassis m_chassis;
}

class StraightDriveController {
    private final double UPSHIFT_BUFFER = 150.0;
    private final double DOWNSHIFT_BUFFER = 500.0;

    public StraightDriveController(RobotChassis chassis) {
        this(chassis, true);
    }

    public StraightDriveController(RobotChassis chassis, boolean reset) {
        chassis.lowTransmission();
        if (reset) {
            chassis.resetEncoder();
            chassis.resetGyro();
        }
        m_controller = new DriveTrainController(chassis.getStraightPIDSource(), chassis.getStraightPIDOutput());
        m_startpoint = m_controller.getCurrent();
        m_chassis = chassis;
    }

    public boolean setTarget(double target) {
        return setTarget(target, null);
    }

    public boolean setTarget(double target, Boolean transState) {
        double current = m_controller.getCurrent();
        if (transState != null) {
            if (transState == true) {
                m_chassis.highTransmission();
            } else {
                m_chassis.lowTransmission();
            }
        } else { // Automatically figure out the transmission based on the buffers.
            if (current > m_startpoint + UPSHIFT_BUFFER && current < target - DOWNSHIFT_BUFFER) {
                m_chassis.highTransmission();
            } else {
                m_chassis.lowTransmission();
            }

        }
        return m_controller.setTarget(target);
    }

    public boolean hasArrived() {
        return m_controller.hasArrived();
    }

    private DriveTrainController m_controller;
    private RobotChassis m_chassis;
    private double m_startpoint;
};

class TurnDriveController {
    public TurnDriveController(RobotChassis chassis, MirrorMode mirror) {
        this(chassis, mirror, true);
    }

    public TurnDriveController(RobotChassis chassis, MirrorMode mirror, boolean reset) {
        chassis.lowTransmission();
        if (reset) {
            chassis.resetGyro();
        }
        m_controller = new DriveTrainController(chassis.getTurnPIDSource(), chassis.getTurnPIDOutput(), 0.1, 0.4, 20.0,
                1.0);
        m_chassis = chassis;
        m_mirror = mirror;
    }

    public boolean setTarget(double target) {
        m_chassis.lowTransmission();
        if (m_mirror == MirrorMode.MIRROR) { 
            target -= 2;
            target *= -1;
        }
        return m_controller.setTarget(target);
    }

    public boolean hasArrived() {
        return m_controller.hasArrived();
    }

    private DriveTrainController m_controller;
    private RobotChassis m_chassis;
    private MirrorMode m_mirror;
};

class DriveTrainController {
    public static final double DRIVE_PID_ABS_MIN = 0.3, DRIVE_PID_ABS_MAX = 0.80;

    public DriveTrainController(PIDSource source, PIDOutput output) {
        m_controller = new SimplePIDController(source, output, null, null, DRIVE_PID_ABS_MIN, DRIVE_PID_ABS_MAX, 100.0,
                40.0, 200);
    }

    public DriveTrainController(PIDSource source, PIDOutput output, double pid_abs_min, double pid_abs_max,
            double threshold, double tolerance) {
        m_controller = new SimplePIDController(source, output, null, null, pid_abs_min, pid_abs_max, threshold,
                tolerance, 200);
    }

    public boolean setTarget(double target) {
        if (m_controller.hasArrived(target)) {
            m_controller.setSetpoint(null);
            if (m_arrived == null) {
                m_arrived = new Long(System.currentTimeMillis());
            }
            return true;
        }

        m_arrived = null;
        m_controller.setSetpoint(target);
        return false;
    }

    public boolean hasArrived() {
        return (m_arrived != null);
    }

    public double getCurrent() {
        return m_controller.getCurrent();
    }

    private Long m_arrived;
    private SimplePIDController m_controller;
}
