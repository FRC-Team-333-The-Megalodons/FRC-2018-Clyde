package org.usfirst.frc.team333.robot;

import org.usfirst.frc.team333.robot.RobotMap.ADJUSTMENT;
import org.usfirst.frc.team333.robot.RobotMap.DigitalSensorPort;
import org.usfirst.frc.team333.robot.RobotMap.PotentiometerPort;
import org.usfirst.frc.team333.robot.RobotMap.ShootSpeed;
import org.usfirst.frc.team333.robot.RobotMap.SolenoidPort;
import org.usfirst.frc.team333.robot.RobotMap.VictorPort;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Victor;

public class RobotMech {
    public static final boolean DIAGNOSTIC_MODE = false;

    /* Intake Constants */
    
    public double shootSpeedToDouble(ShootSpeed speed) {
        switch (speed) {
        case DROOL: return 0.27;
        case SLOW: return 0.35;
        case MEDIUM: return 0.55;
        case FAST: return 0.85;
        case INTAKE: return -1.0;
        default: return 0.0;
        }
    }
    public static final double TELEOP_SPIN_OUT = 0.55;
    public static final double TELEOP_SPIN_IN = -1.0;
    public static final double AUTON_SPIN_OUT = 0.85;

    /* Elevator Constants */
    public static final double ELEVATOR_PID_ABS_MAX = 1.0;
    public static final double ELEVATOR_PID_ABS_MIN = 0.3;
    public static final double ELEVATOR_DIAG_UP = -0.7;
    public static final double ELEVATOR_DIAG_DOWN = 0.7;
    public static final double ELEVATOR_HOME = 2.0;
    public static final double ELEVATOR_INTAKE = 4.0;
    public static final double ELEVATOR_SWITCH = 40.0;
    public static final double ELEVATOR_LOW_SCALE = 160.0;
    public static final double ELEVATOR_HIGH_SCALE = 193.0;
    public static final double ELEVATOR_EXCHANGE = ELEVATOR_INTAKE;
    public static final double ELEVATOR_RELEASE_DELAY_MS = 300;
    public static final double ADJUSTMENT_NONE = 0.0;
    public static final double ADJUSTMENT_DOWN = -10.0;
    public static final double ADJUSTMENT_UP   = 10.0;
    

    public static final double ELEVATOR_UP_THRESHOLD = 70.0;
    public static final double ELEVATOR_WRIST_FOLDUP_THRESHOLD = 40.0;

    /* Wrist Constants */
    public static final double WRIST_PID_ABS_MAX = 1.0;
    public static final double WRIST_PID_ABS_MIN = 0.0;
    public static final double WRIST_DIAG_UP = 0.7;
    public static final double WRIST_DIAG_DOWN = -0.7;
    public static final double WRIST_HOME = 176.0;
    public static final double WRIST_SWITCH = 182.7;
    public static final double WRIST_LOW_SCALE = 182.0;
    public static final double WRIST_HIGH_SCALE = 181.0;
    public static final double WRIST_INTAKE = 186.0;
    public static final double WRIST_EXCHANGE = 185.0;

    public RobotMech() {

        /* Instantiate the Intake */
        try {
            m_intakeMotor = new Intake(new Victor(VictorPort.INTAKE));
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the Intake\n", false);
        }

        /* Instantiate the Elevator */
        try {
            m_elevatorMotor = new VictorPair(VictorPort.ELEVATOR_2, VictorPort.ELEVATOR_1, false);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the Elevator\n", false);
        }

        /* Instantiate the Claw */
        try {
            m_clawToggle = new Claw(new Solenoidal(SolenoidPort.GRIPPER_LEFT, SolenoidPort.GRIPPER_RIGHT));
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the claw\n", false);
        }

        /* Instantiate the Ratchet Release */
        try {
            m_release = new SolenoidT(SolenoidPort.RATCHET_RELEASE);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate Ratchet Release mechanism\n", false);
        }

        /* Instantiate the Wrist */
        try {
            m_wristMotor = new Victor(VictorPort.WRIST);
            // m_wrist = new Solenoidal(SolenoidPort.WRIST_UP_POSITION,
            // SolenoidPort.WRIST_DOWN_POSITION);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the wrist\n", false);
        }

        /* Instantiate the Wrist Potentiometer */
        try {
            m_wristPotentiometer = new AnalogPotentiometer(PotentiometerPort.WRIST_POTENTIOMETER, 360, 0);
            RobotDashboard.getInstance().setWristPotentiometer(m_wristPotentiometer);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate Wrist Potentiometer\n", false);
        }

        /* Instantiate the Elevator Potentiometer */
        try {
            m_elevatorPotentiometer = new AnalogPotentiometer(PotentiometerPort.ELEVATOR_POTENTIOMETER, 360, 0);
            RobotDashboard.getInstance().setElevatorPotentiometer(m_elevatorPotentiometer);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate Elevator Potentiometer\n", false);
        }

        /* Instantiate Limit Switch */
        try {
            m_lowerElevatorLimitSwitch = new DigitalInput(DigitalSensorPort.LIMITER);
            RobotDashboard.getInstance().setLimitSwitch(m_lowerElevatorLimitSwitch);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate Limit Switch\n", false);
        }

        try {
            ElevatorController elevator = new ElevatorController(m_elevatorPotentiometer, m_elevatorMotor,
                    m_lowerElevatorLimitSwitch, null, m_release);
            WristController wrist = new WristController(m_wristPotentiometer, m_wristMotor);
            m_mech = new MechController(elevator, wrist, m_clawToggle, m_intakeMotor);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate elevator controller", false);
        }
    }

    public void release() {
        m_release.set(true);
    }

    public void retract() {
        m_release.set(false);
    }

    public void elevatorUp() {
        m_release.set(true);
        long now = System.currentTimeMillis();
        if (now - m_release.getLastBecameTrue().longValue() < ELEVATOR_RELEASE_DELAY_MS) {
            return;
        }
        m_elevatorMotor.set(ELEVATOR_DIAG_UP);
    }

    public void elevatorDown() {
        if (m_lowerElevatorLimitSwitch.get()) {
            m_elevatorMotor.set(0);
            return;
        }
        m_elevatorMotor.set(ELEVATOR_DIAG_DOWN);
    }

    public void stop() {
        m_mech.stop();
        m_wristMotor.set(0);
        elevatorStop();
    }

    public void elevatorStop() {
        m_elevatorMotor.set(0);
        m_release.set(false);
    }

    public void wristIn() {
        m_wristMotor.set(WRIST_DIAG_UP);
        // m_wrist.set(true);
    }

    public void wristOut() {
        m_wristMotor.set(WRIST_DIAG_DOWN);
        // m_wrist.set(false);;
    }

    public void wristStop() {
        m_wristMotor.set(0.0);
    }

    public boolean getIsElevatorUp() {
        return m_mech.getIsElevatorUp();
    };

    public boolean goSwitch(ShootSpeed fire, ADJUSTMENT adjust) {
        return m_mech.goTarget(TARGET.SWITCH, adjust, false, fire);
    }

    public boolean goHighScale(ShootSpeed fire, ADJUSTMENT adjust) {
        return m_mech.goTarget(TARGET.HIGH_SCALE, adjust, false, fire);
    }
    
    public boolean goLowScale(ShootSpeed fire, ADJUSTMENT adjust) {
        return m_mech.goTarget(TARGET.LOW_SCALE, adjust, false, fire);
    }

    public boolean goHome(ShootSpeed fire) {
        return m_mech.goHome(fire);
    }

    public boolean goIntake(boolean open, ShootSpeed fire) {
        return m_mech.goTarget(TARGET.INTAKE, ADJUSTMENT.NONE, open, fire);
    }

    public void periodic(Joystick stick) {
        
        ADJUSTMENT adjust = ADJUSTMENT.NONE;
        double pov = stick.getPOV();
        /*
        if (pov > 270.0 || (pov >= 0.0 && pov < 90.0)) { 
            adjust = ADJUSTMENT.UP;
        } else if (pov > 90.0 && pov < 270.0) { 
            adjust = ADJUSTMENT.DOWN;
        }
        */

        if (!DIAGNOSTIC_MODE) {
            boolean openClaw = stick.getTrigger();
            ShootSpeed fire = ShootSpeed.OFF;
            if (stick.getRawButton(7) || stick.getRawButton(8)) {
                fire = ShootSpeed.DROOL;
                openClaw = true;
            } else if (stick.getRawButton(9) || stick.getRawButton(10)) {
                fire = ShootSpeed.MEDIUM;
            }

            if (stick.getRawButton(2)) { /* OPEN, FIRE */
                m_mech.goTarget(TARGET.INTAKE, adjust, openClaw, fire);
            } else if (stick.getRawButton(3)) {
                m_mech.goTarget(TARGET.SWITCH, adjust, openClaw, fire);
            } else if (stick.getRawButton(6)) {
                if (pov >= 0.0) { adjust = ADJUSTMENT.DOWN; }
                m_mech.goTarget(TARGET.LOW_SCALE, adjust, openClaw, fire);
            } else if (stick.getRawButton(4)) {
                if (pov >= 0.0) { adjust = ADJUSTMENT.DOWN; }
                m_mech.goTarget(TARGET.HIGH_SCALE, adjust, openClaw, fire);
            } else if (stick.getRawButton(5)) {
                m_mech.goTarget(TARGET.EXCHANGE, adjust, openClaw, fire);
            } else {
                m_mech.goHome(fire);
            }
            return;
        }

        // If not using Mech Controller, go to manual testing mode. Basically all
        // buttons operate components independently...
        // ...with exception of the elevator, for which we actually need to be careful
        // with the release latch.

        // 2 = Intake Motor (5 or 6 = pushout on intake wheels)
        if (stick.getRawButton(2)) {
            m_intakeMotor.run(ShootSpeed.INTAKE);
        } else if (stick.getRawButton(5) || stick.getRawButton(6)) {
            m_intakeMotor.run(ShootSpeed.MEDIUM);
        } else {
            m_intakeMotor.stop();
        }

        // 3 = Elevator Down, 4 = elevator up
        if (stick.getRawButton(3)) {
            elevatorDown();
        } else if (stick.getRawButton(4)) {
            elevatorUp();
        } else {
            elevatorStop();
        }

        if (stick.getTrigger()) {
            m_clawToggle.open();
        } else {
            m_clawToggle.close();
        }

        // Wrist on buttons 7 & 8
        if (stick.getRawButton(7)) {
            wristIn();
        } else if (stick.getRawButton(8)) {
            wristOut();
        } else {
            wristStop();
        }
    }

    public double getPotAngle() {
        return m_elevatorPotentiometer.get();
    }

    public boolean getCurrentLimit() {
        return m_lowerElevatorLimitSwitch.get();
    }

    private enum TARGET {
        HOME, INTAKE, SWITCH, LOW_SCALE, HIGH_SCALE, EXCHANGE;
    };

    class MechController {

        // Rules:
        // When the elevator is lifting up or going down, it shouldn't do it unless the
        // claw is fully in.
        // When the wrist is extending, it shouldnt do it unless the elevator is in
        // position.

        public MechController(ElevatorController elevator, WristController wrist, Claw claw, Intake intake) {
            m_wrist = wrist;
            m_elevator = elevator;
            m_intake = intake;
            m_claw = claw;
        }

        public boolean getIsElevatorUp() {
            return m_elevator.getIsElevatorUp();
        }

        public void stop() {
            m_wrist.stop();
            m_elevator.stop();
            m_intake.stop();
        }

        public boolean goHome(ShootSpeed fire) {
            m_claw.close();
            m_wrist.go(TARGET.HOME);
            // First, make sure the wrist is up!
            if (m_wrist.hasArrived(TARGET.HOME)) {
                m_elevator.go(TARGET.HOME, ADJUSTMENT.NONE);
                m_intake.run(fire);
                return m_elevator.hasArrived(TARGET.HOME, ADJUSTMENT.NONE);
            }
            m_intake.stop();
            return false;
        }

        public boolean goTarget(TARGET target, ADJUSTMENT adjust, boolean drop, ShootSpeed fire) {
            // Logical Steps:
            // 1) Fold up wrist
            // 2) Run elevator to location
            // 3) Unfold wrist

            // Actual steps:
            // A) Check if elevator has arrived, and if so, begin step 3. (If we're doing
            // intake, also start intaking.)
            boolean elevator_arrived = m_elevator.hasArrived(target, adjust);

            if (elevator_arrived) {
                m_elevator.go(target, adjust);

                if (target == TARGET.INTAKE) {
                    // If we're in intake mode, respect the claw (i.e. drop) blindly, and ignore the
                    // pushout (i.e. fire)
                    if (drop) {
                        m_claw.open();
                    } else {
                        m_claw.close();
                    }
                    m_intake.run(ShootSpeed.INTAKE);
                    m_wrist.go(TARGET.INTAKE);
                    return m_wrist.hasArrived(TARGET.INTAKE);
                }

                m_wrist.go(target);
                boolean wrist_arrived = m_wrist.hasArrived(target);

                // If we're in dropoff mode, only allow dropping or firing if we're at our
                // target.
                if (wrist_arrived) {
                    m_intake.run(fire);
                }

                if (wrist_arrived && drop) {
                    m_claw.open();
                } else {
                    m_claw.close();
                }

                return wrist_arrived;
            }

            // At this point, if we're still enroute, make sure we're at all default
            // positions for intake wheels & claw.
            m_claw.close();
            m_intake.stop();
            
            // We don't have to fold up the wrist if the distance the elevator has to travel is small enough.
            boolean skip_wrist_foldup = Math.abs(m_elevator.getCurrent() - m_elevator.elev_target_to_value(target, adjust)) < ELEVATOR_WRIST_FOLDUP_THRESHOLD;

            // B) If not, see whether we're ready to begin step 2.
            if (skip_wrist_foldup || m_wrist.hasArrived(TARGET.HOME)) {
                m_wrist.stop();
                m_elevator.go(target, adjust);
                return false;
            }

            // C) If not, start step 1.
            m_wrist.go(TARGET.HOME);
            return false;
        }

        private WristController m_wrist;
        private ElevatorController m_elevator;
        private Intake m_intake;
        private Claw m_claw;
    }

    class Claw {
        public Claw(Solenoidal solenoidal) {
            m_solenoidal = solenoidal;
        }

        public void close() {
            m_solenoidal.set(true);
        }

        public void open() {
            m_solenoidal.set(false);
        }

        private Solenoidal m_solenoidal;
    }

    class Intake {
        public Intake(Victor victor) {
            m_victor = victor;
        }

        public void stop() {
            m_victor.set(0);
        }

        public void run(ShootSpeed speed) {
            m_victor.set(shootSpeedToDouble(speed));
        }

        Victor m_victor;
    }

    class WristController {
        public WristController(PIDSource source, PIDOutput output) {
            m_controller = new SimplePIDController(source, output, null, null, WRIST_PID_ABS_MIN, WRIST_PID_ABS_MAX,
                    2.5, 0.6, 200);
        }

        public double wrist_target_to_value(TARGET target) {
            switch (target) {
            case HOME:
                return WRIST_HOME;
            case INTAKE:
                return WRIST_INTAKE;
            case SWITCH:
                return WRIST_SWITCH;
            case LOW_SCALE:
                return WRIST_LOW_SCALE;
            case HIGH_SCALE:
                return WRIST_HIGH_SCALE;
            case EXCHANGE:
                return WRIST_EXCHANGE;
            default:
                return WRIST_HOME;
            }
        }

        public void setTarget(double target) {
            if (m_controller.hasArrived(target)) {
                m_controller.setSetpoint(null);
                if (m_arrived == null) {
                    m_arrived = new Long(System.currentTimeMillis());
                }
                return;
            }

            m_arrived = null;
            m_controller.setSetpoint(target);
        }

        public void go(TARGET target) {
            double target_value = wrist_target_to_value(target);
            setTarget(target_value);
        }

        public boolean hasArrived(TARGET target) {
            double target_value = wrist_target_to_value(target);
            return m_controller.hasArrived(target_value);
        }

        public void stop() {
            m_controller.stop();
        }

        private Long m_arrived;
        private SimplePIDController m_controller;
    }

    class ElevatorController {
        public ElevatorController(PIDSource source, PIDOutput output, DigitalInput lowLimit, DigitalInput highLimit,
                SolenoidT release) {
            m_controller = new SimplePIDController(source, output, lowLimit, highLimit, ELEVATOR_PID_ABS_MIN,
                    ELEVATOR_PID_ABS_MAX, 20.0, 4.0, 150);
            m_release = release;
        }

        public boolean getIsElevatorUp() {
            return m_controller.getCurrent() > ELEVATOR_UP_THRESHOLD;
        }
        
        public double getCurrent() {
            return m_controller.getCurrent();
        }

        private void setTarget(double target) {
            long now = System.currentTimeMillis();
            if (m_controller.hasArrived(target)) {
                m_controller.setSetpoint(null);
                if (m_arrived == null) {
                    m_arrived = new Long(now);
                }
                if (now - m_arrived.longValue() > ELEVATOR_RELEASE_DELAY_MS) {
                    m_release.set(false);
                }
                return;
            } else {
                m_arrived = null;
            }

            m_release.set(true);
            if (m_controller.isProposedSetpointAbove(target)) {
                // If it hasn't been long enough, bail! (But also stop.)
                if (now - m_release.getLastBecameTrue() < ELEVATOR_RELEASE_DELAY_MS) {
                    m_controller.setSetpoint(null);
                    return;
                }
            }

            // Terrible Hack - If the target is Switch, and we're going up to it, we know
            // the speed is bananas, so cap this.
            if (target == ELEVATOR_SWITCH && m_controller.isProposedSetpointAbove(target)) {
                m_controller.setSpeedModifier(0.5);
            } else {
                m_controller.resetSpeedModifier();
            }

            m_controller.setSetpoint(target);
        }

        public double elev_target_to_value(TARGET target, ADJUSTMENT adjust) {
            double adjustment;
            
            switch (adjust) {
            case UP: adjustment = ADJUSTMENT_UP; break;
            case DOWN: adjustment = ADJUSTMENT_DOWN; break;
            default: adjustment = ADJUSTMENT_NONE; break;
            }
            
            switch (target) {
            case HOME:
                return ELEVATOR_HOME+adjustment;
            case INTAKE:
                return ELEVATOR_INTAKE+adjustment;
            case SWITCH:
                return ELEVATOR_SWITCH+adjustment;
            case LOW_SCALE:
                return ELEVATOR_LOW_SCALE+adjustment;
            case HIGH_SCALE:
                return ELEVATOR_HIGH_SCALE+adjustment;
            case EXCHANGE:
                return ELEVATOR_EXCHANGE+adjustment;
            default:
                return ELEVATOR_HOME;
            }
        }

        public void go(TARGET target, ADJUSTMENT adjust) {
            double target_value = elev_target_to_value(target, adjust);
            setTarget(target_value);
        }

        public boolean hasArrived(TARGET target, ADJUSTMENT adjust) {
            double target_value = elev_target_to_value(target, adjust);
            return m_controller.hasArrived(target_value);
        }

        public void stop() {
            m_controller.stop();
        }

        private Long m_arrived;
        private SimplePIDController m_controller;
        private SolenoidT m_release;
    }

    private AnalogPotentiometer m_wristPotentiometer;
    private AnalogPotentiometer m_elevatorPotentiometer;
    private DigitalInput m_lowerElevatorLimitSwitch;
    private Intake m_intakeMotor;
    private Claw m_clawToggle;
    private VictorPair m_elevatorMotor;
    private Victor m_wristMotor;
    private SolenoidT m_release;

    private MechController m_mech;
}
