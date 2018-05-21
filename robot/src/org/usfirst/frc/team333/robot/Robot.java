
package org.usfirst.frc.team333.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;

import org.usfirst.frc.team333.robot.RobotMap.ADJUSTMENT;
import org.usfirst.frc.team333.robot.RobotMap.AutoMode;
import org.usfirst.frc.team333.robot.RobotMap.AutoSide;
import org.usfirst.frc.team333.robot.RobotMap.JoystickPort;
import org.usfirst.frc.team333.robot.RobotMap.MirrorMode;
import org.usfirst.frc.team333.robot.RobotMap.ShootSpeed;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends IterativeRobot {
    public static final double DEFAULT_GYRO_CARPET_MULTIPLIER = 0.20;
    public static final double DEFAULT_GYRO_TILE_MULTIPLIER = 0.15;
    public static final double AUTO_DRIVE_SPEED = 0.5;
    public static final double AUTO_TURN_POWER = 0.5;
    public static final double AUTO_INITIAL_DISTANCE = 60;
    public static final double AUTO_TURN_DEGREES = 45.0;
    public static final double AUTO_TURN_SANITY_TIMER = 10.0;
    public static final double DEFAULT_TARGET_POSITION = 1000.0;
    public static final String LABEL_GYRO_MULTIPLIER = "[Auto-Config] Gyro Multiplier =";
    
    public static final double RIGHT_TURN_DEGREES = 74.5;

    double GYRO_MULTIPLIER = DEFAULT_GYRO_CARPET_MULTIPLIER; // This should always be overwritten by dashboard value
                                                             // during autonomousInit. But better to have a default than
                                                             // nothing.
    String GAME_DATA = "LLL"; // This should always be overwritten by FMS value during autonomousInit. But
                              // better to have a default than nothing.

    /**
     * This function is called when the robot starts
     */
    @Override
    public void robotInit() {
        /* Logging */
        System.out.println("Calling RobotInit");

        /* Camera */
        try {
            m_simpleCameraServer = CameraServer.getInstance();
        } catch (Exception ex) {
            DriverStation.reportError("Cannot instantiate CameraServer", false);
        }

        if (m_simpleCameraServer != null) {
            // Front camera

            try {
                UsbCamera camera = m_simpleCameraServer.startAutomaticCapture(0); //
                camera.setResolution(240, 180);// 160,120
                //camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, -240, -180, 15);

                // camera.setResolution(160,120);
                System.out.println("Front Camera valid? - " + camera.isValid());
                System.out.println("Front Camera connected? - " + camera.isConnected());
            } catch (Exception ex) {
                DriverStation.reportError("Cannot start Front CameraServer\n", false);
            }

        }

        /* Chassis */
        m_mech = new RobotMech();
        m_chassis = new RobotChassis(m_mech);
        m_driverJoystick = new Joystick(JoystickPort.PRIMARY_JOYSTICK);

        /* Dashboard */
        RobotDashboard.getInstance().setConfigValue(LABEL_GYRO_MULTIPLIER, DEFAULT_GYRO_CARPET_MULTIPLIER);
    }

    /* Autonomous movement routines */
    public void auto_test() {
        { // Turn.
            final double SCALE_TURN_DEGREES = 90.0;
            System.out.println("Turning for " + SCALE_TURN_DEGREES);
            TurnDriveController turnDrive = new TurnDriveController(m_chassis, MirrorMode.NORMAL, false);
            do {
            } while (isAutonomous() && (!turnDrive.setTarget(SCALE_TURN_DEGREES)));
        }
    }

    /* Movement: From start. */
    public void auto_do_near_scale_from_start(MirrorMode mirror) {
        System.out.println("[AUTOLOG] Do near scale from start (direction=" + mirror + ")");

        { // Drive forward.
            final double INITIAL_SCALE_DISTANCE = (mirror == MirrorMode.MIRROR ? 3230.0 : 3333.0);
            System.out.println("Drive for " + INITIAL_SCALE_DISTANCE);
            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            do {
            } while (isAutonomous() && !straightDrive.setTarget(INITIAL_SCALE_DISTANCE));
        }

        { // Turn in.
            /******
             * DO NOT TURN AND DO ELEVATOR AT THE SAME TIME!!!! (if gyro screws up, then the
             * robot falls over. it's not worth the less-than-second speedup.)
             ******/
            final double SCALE_TURN_DEGREES = 50.0;
            System.out.println("Turning for " + SCALE_TURN_DEGREES);
            TurnDriveController turnDrive = new TurnDriveController(m_chassis, mirror, false);
            do {
            } while (isAutonomous() && (!turnDrive.setTarget(SCALE_TURN_DEGREES)));
        }

        { // Scale up
            System.out.println("Mech to High Scale");
            do {
            } while (isAutonomous() && (!m_mech.goHighScale(ShootSpeed.OFF, ADJUSTMENT.NONE)));
        }

        { // Shoot for 500ms
            final double SHOOT_TIME_MS = 500.0;
            System.out.println("Shoot for " + SHOOT_TIME_MS + "ms");
            long begin = System.currentTimeMillis();
            do {
                m_mech.goHighScale(ShootSpeed.MEDIUM, ADJUSTMENT.NONE);
            } while (isAutonomous() && (System.currentTimeMillis() - begin < SHOOT_TIME_MS));
            m_mech.stop();
        }

        { // Return home
            System.out.println("Mech to Home");
            do {
            } while (isAutonomous() && !m_mech.goHome(ShootSpeed.OFF));
        }
    }

    public void auto_get_switchcube_from_near_scale(MirrorMode mirror) {
        { // Turn towards cube
            final double TURN_TO_CUBE = (mirror == MirrorMode.MIRROR ? 65.0 : 61.5);
            TurnDriveController turnDrive = new TurnDriveController(m_chassis, mirror);
            do {
            } while (isAutonomous() && (!turnDrive.setTarget(TURN_TO_CUBE)));
        }
        
        do {
        } while (isAutonomous() && !m_mech.goHome(ShootSpeed.OFF));

        Timer.delay(0.25);

        { // Drive towards cube & pick it up
            final double DRIVE_TO_CUBE = (mirror == MirrorMode.MIRROR ? 1025.0 : 950.0);

            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            boolean intakeDone = false, driveDone = false;
            do {
                intakeDone = m_mech.goIntake(true, ShootSpeed.INTAKE);
                driveDone = straightDrive.setTarget(DRIVE_TO_CUBE);
            } while (isAutonomous() && (!driveDone || !intakeDone));
            while (isAutonomous() && !m_mech.goIntake(true, ShootSpeed.INTAKE));
        }
    }
    
    public void auto_do_near_scale_from_near_scale(MirrorMode mirror) {
        auto_get_switchcube_from_near_scale(mirror);
        
        { // Start by going home.
            do {
            } while (isAutonomous() && !m_mech.goHome(ShootSpeed.INTAKE));
            // Stop the shooter.
            while (isAutonomous() && !m_mech.goHome(ShootSpeed.OFF));
        }
        
        { // Back up
            final double DRIVE_BACK_FROM_CUBE = -925.0;
            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            do {
            } while (isAutonomous() && !straightDrive.setTarget(DRIVE_BACK_FROM_CUBE));
        }

        { // Turn towards scale
            final double TURN_BACK_FROM_CUBE = -60.0;
            TurnDriveController turnDrive = new TurnDriveController(m_chassis, mirror);
            do {
            } while (isAutonomous() && (!turnDrive.setTarget(TURN_BACK_FROM_CUBE)));
        }
        
        { // Scale up
            System.out.println("Mech to High Scale");
            do {
            } while (isAutonomous() && (!m_mech.goHighScale(ShootSpeed.OFF, ADJUSTMENT.NONE)));
        }

        { // Shoot for 500ms
            final double SHOOT_TIME_MS = 500.0;
            System.out.println("Shoot for " + SHOOT_TIME_MS + "ms");
            long begin = System.currentTimeMillis();
            do {
                m_mech.goHighScale(ShootSpeed.FAST, ADJUSTMENT.NONE);
            } while (isAutonomous() && (System.currentTimeMillis() - begin < SHOOT_TIME_MS));
        }

        { // Return home
            System.out.println("Mech to Home");
            do {
            } while (isAutonomous() && !m_mech.goHome(ShootSpeed.OFF));
        }
    }

    public void auto_do_near_switch_from_near_scale(MirrorMode mirror) {
        auto_get_switchcube_from_near_scale(mirror);
        { // Go home
            do {
            } while (isAutonomous() && !m_mech.goHome(ShootSpeed.OFF));
        }
        /*
        { // Raise up towards switch
            do {
            } while (isAutonomous() && !m_mech.goSwitch(ShootSpeed.OFF, ADJUSTMENT.NONE));
        }
        */

        { // Drive to switch
            final double DRIVE_TO_SWITCH = 125.0;

            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            do {
            } while (isAutonomous() && !straightDrive.setTarget(DRIVE_TO_SWITCH));
        }

        { // Spit the cube out
            long begin = System.currentTimeMillis();
            do {
                m_mech.goHome(ShootSpeed.MEDIUM);
            } while (isAutonomous() && (System.currentTimeMillis() - begin < 500));
            while (isAutonomous() && !m_mech.goHome(ShootSpeed.OFF));
        }

        /*
        { // Spit the cube out
            long begin = System.currentTimeMillis();
            do {
                m_mech.goSwitch(ShootSpeed.MEDIUM, ADJUSTMENT.NONE);
            } while (isAutonomous() && (System.currentTimeMillis() - begin < 500));
        }
        */

        { // Back up just a bit
            final double BACKUP_DISTANCE = -200.0;

            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            do {
            } while (isAutonomous() && !straightDrive.setTarget(BACKUP_DISTANCE));
        }

        { // Go back home
            do {
            } while (isAutonomous() && !m_mech.goHome(ShootSpeed.OFF));
        }
    }
    
    public double auto_do_far_scale_turn(MirrorMode mirror) {
        { // Drive forward.
            final double INITIAL_STEPOUT_DISTANCE = 2820;
            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            do {
            } while (isAutonomous() && !straightDrive.setTarget(INITIAL_STEPOUT_DISTANCE));
        }

        { // Turn right
            TurnDriveController turnDrive = new TurnDriveController(m_chassis, mirror, false);
            do {
            } while (isAutonomous() && !turnDrive.setTarget(RIGHT_TURN_DEGREES));
        }
        return RIGHT_TURN_DEGREES;
    }

    public void auto_do_gtfo(MirrorMode mirror) {
        System.out.println("[AUTOLOG] Do far scale from start (direction=" + mirror + ")");
        // elevator to high scale
        
        auto_do_far_scale_turn(mirror);
        
        Timer.delay(0.25);

        { // Drive across
            final double CROSSOVER_DISTANCE = 1500.0;
            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            do {
            } while (isAutonomous() && !straightDrive.setTarget(CROSSOVER_DISTANCE));
        }
    }

    public void auto_do_far_scale_from_start(MirrorMode mirror) {
        System.out.println("[AUTOLOG] Do far scale from start (direction=" + mirror + ")");
        // elevator to high scale
        
        double FIRST_TURN_DEGREES = auto_do_far_scale_turn(mirror);
        
        Timer.delay(0.25);

        { // Drive across
            final double CROSSOVER_DISTANCE = (mirror == MirrorMode.MIRROR ? 2350.0 : 2450.0);
            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            do {
            } while (isAutonomous() && !straightDrive.setTarget(CROSSOVER_DISTANCE));
        }

        { // Turn back
            /******
             * DO NOT TURN AND DO ELEVATOR AT THE SAME TIME!!!! (if gyro screws up, then the
             * robot falls over. it's not worth the less-than-second speedup.)
             ******/
            final double SECOND_TURN_DEGREES = -FIRST_TURN_DEGREES + (mirror == MirrorMode.MIRROR ? -2.0 : 0.0);
            TurnDriveController turnDrive = new TurnDriveController(m_chassis, mirror);
            do {
            } while (isAutonomous() && (!turnDrive.setTarget(SECOND_TURN_DEGREES)));
        }

        { // Scale up
            do {
            } while (isAutonomous() && (!m_mech.goHighScale(ShootSpeed.OFF, ADJUSTMENT.NONE)));
            m_mech.stop();
        }

        final double STEPIN_DISTANCE = 380;
        { // Step in
            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            m_chassis.resetGyro(GYRO_MULTIPLIER);
            do {
            } while (isAutonomous() && !straightDrive.setTarget(STEPIN_DISTANCE));
        }

        { // Shoot.
            long begin = System.currentTimeMillis();
            do {
                m_mech.goHighScale(ShootSpeed.SLOW, ADJUSTMENT.NONE);
            } while (isAutonomous() && (System.currentTimeMillis() - begin < 500));
        }

        { // Step back
            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            m_chassis.resetGyro(GYRO_MULTIPLIER);
            do {
            } while (isAutonomous() && !straightDrive.setTarget(-STEPIN_DISTANCE));
        }

        { // Elevator home
            do {
            } while (isAutonomous() && !m_mech.goHome(ShootSpeed.OFF));
        }

        // Turn and go home.
        {
            TurnDriveController turnDrive = new TurnDriveController(m_chassis, mirror);
            m_chassis.resetGyro(GYRO_MULTIPLIER);
            do {
            } while (isAutonomous() && !turnDrive.setTarget(FIRST_TURN_DEGREES * 2));
        }
    }

    public void auto_do_near_switch_from_start(MirrorMode mirror) {
        System.out.println("[AUTOLOG] Do near switch from start (direction=" + mirror + ")");


        { // Drive in low towards Switch
            final double INITIAL_STEPOUT_DISTANCE = 1800.0;
            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            // We should actually raise up to Switch height while driving.
            do {
            } while (isAutonomous() && !straightDrive.setTarget(INITIAL_STEPOUT_DISTANCE));
            // goSwitch needs to come first because of short circuiting.
        }

        { // Turn 90 degrees towards Switch
            TurnDriveController turnDrive = new TurnDriveController(m_chassis, mirror);
            do {
            } while (isAutonomous() && !turnDrive.setTarget(RIGHT_TURN_DEGREES));
        }

        { // Drive forwards to Switch until bumpers are flush with the fence
            final double STEPIN_DISTANCE = 400.0;
            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            do {
            } while (isAutonomous() && !straightDrive.setTarget(STEPIN_DISTANCE));
        }

        { // Shoot the Power Cube!
            long begin = System.currentTimeMillis();
            do {
                m_mech.goHome(ShootSpeed.SLOW);
            } while (isAutonomous() && System.currentTimeMillis() - begin < 1000);
        }
        
        while (isAutonomous() && !m_mech.goHome(ShootSpeed.OFF));
    }

    public void auto_do_far_switch_from_start(MirrorMode mirror) {

    }

    public void auto_do_scale_again(MirrorMode mirror) {

    }

    public void auto_do_scale_same_side_as_current_switch(MirrorMode mirror) {

    }

    public void auto_do_scale_other_side_from_current_switch(MirrorMode mirror) {

    }

    public void auto_do_switch_again(MirrorMode mirror) {

    }

    public void auto_do_switch_near_to_current_scale(MirrorMode mirror) {

    }

    public void auto_do_switch_away_from_current_scale(MirrorMode mirror) {

    }

    /* For each of the 4 possible configs, here is our Strategy */
    public void auto_switchmine_scalemine(AutoMode ally_ability, MirrorMode mirror) {
        System.out.println("[AUTOLOG] Switch is Mine, Scale is Mine (ally=" + ally_ability.name() + ",direction=" + mirror + ")");
        auto_do_near_scale_from_start(mirror);
        auto_do_near_switch_from_near_scale(mirror);
    }

    public void auto_switchmine_scaletheirs(AutoMode ally_ability, MirrorMode mirror) {
        System.out.println("[AUTOLOG] Switch is Mine, Scale is Theirs (priority=" + ally_ability.name() + ",direction=" + mirror + ")");
                
        if (ally_ability == AutoMode.WONT_INTERFERE_WITH_ALLY_FAR_SCALE) {
            auto_do_far_scale_from_start(mirror);
        } else {
            auto_do_near_switch_from_start(mirror);
        }
    }

    public void auto_switchtheirs_scalemine(AutoMode ally_ability, MirrorMode mirror) {
        System.out.println("[AUTOLOG] Switch is Theirs, Scale is Mine (priority=" + ally_ability.name() + ",direction=" + mirror + ")");
        auto_do_near_scale_from_start(mirror);
        auto_do_near_scale_from_near_scale(mirror);
    }

    public void auto_switchtheirs_scaletheirs(AutoMode ally_ability, MirrorMode mirror) {
        System.out.println("[AUTOLOG] Switch is Theirs, Scale is Theirs (priority=" + ally_ability.name() 
                           + ",direction=" + mirror.name() + ")");
        if (ally_ability == AutoMode.WONT_INTERFERE_WITH_ALLY_FAR_SCALE) {
            auto_do_far_scale_from_start(mirror);
        } else {
            auto_do_gtfo(mirror);
        }
    }
    
    public void auto_cross_baseline() {
        System.out.println("[AUTOLOG] Cross baseline");
        { // Cross the baseline.
            final double BASELINE_DISTANCE = 800.0; // 333.0;
            System.out.println("Drive for " + BASELINE_DISTANCE);
            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            do {
            } while (isAutonomous() && !straightDrive.setTarget(BASELINE_DISTANCE, false));
        }
    }

    public void auto_center(MirrorMode mirror) {
        System.out.println("[AUTOLOG] Center! (direction=" + mirror +")");
        
        { // Go home. This should hopefully return immediately, but also hopefully should stop the arm from sometimes doing something weird.
            do {
            } while (isAutonomous() && !m_mech.goHome(ShootSpeed.OFF));
        }

        { // Step out a bit
            final double INITIAL_STEPOUT_DISTANCE = 250.0; // 333.0;
            System.out.println("Drive for " + INITIAL_STEPOUT_DISTANCE);
            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            do {
            } while (isAutonomous() && !straightDrive.setTarget(INITIAL_STEPOUT_DISTANCE, false));
        }

        { // Turn in
            final double TURN_ANGLE = -17.0;
            System.out.println("Turn for " + TURN_ANGLE);
            TurnDriveController turnDrive = new TurnDriveController(m_chassis, mirror);
            do {
                turnDrive.setTarget(TURN_ANGLE);
            } while (isAutonomous() && !turnDrive.hasArrived());
        }

        /*
         * { // Bring the elevator up do { } while (isAutonomous() &&
         * (!m_mech.goSwitch(false))); m_mech.stop(); // TODO: Probably unnecessary. }
         */

        Timer.delay(0.25);

        { // Drive in real close, and then...
            final double SWITCH_HYPOTENUSE_DISTANCE = mirror == MirrorMode.MIRROR ? 950 : 1100;
            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            do {
            } while (isAutonomous() && !straightDrive.setTarget(SWITCH_HYPOTENUSE_DISTANCE));
        }

        { // ...Kiss the wall
            final double WALL_KISS_DISTANCE = mirror == MirrorMode.MIRROR ? 200 : 300;
            RawForwardDriveController rawDrive = new RawForwardDriveController(m_chassis);
            long begin = System.currentTimeMillis();
            long timeout = 1500;
            do {
            } while (isAutonomous() && (System.currentTimeMillis() - begin < timeout) && !rawDrive.setTarget(WALL_KISS_DISTANCE));
            m_chassis.stop();
        }

        { // Shoot the Power Cube from Home
            long begin = System.currentTimeMillis();
            long timeout = 700;
            do {
                m_mech.goHome(ShootSpeed.MEDIUM);
            } while (isAutonomous() && System.currentTimeMillis() - begin < timeout);
            // Stop the shooter.
            while (isAutonomous() && !m_mech.goHome(ShootSpeed.OFF));
        }
        
        { // Back up 
            final double BACKUP_DISTANCE = -250;
            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            do {
            } while (isAutonomous() && !straightDrive.setTarget(BACKUP_DISTANCE));
        }
        { // Back up  & put down intake
            final double BACKUP_DISTANCE = -400;
            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            do {
                m_mech.goIntake(true, ShootSpeed.INTAKE);
            } while (isAutonomous() && !straightDrive.setTarget(BACKUP_DISTANCE));
        }
        
        { // Turn towards the tip cube and prep the intake
            final double CUBE_TURN_ANGLE = (mirror == MirrorMode.MIRROR ? 32.5 : 45.0);
            TurnDriveController turnDrive = new TurnDriveController(m_chassis, mirror);
            do {
                turnDrive.setTarget(CUBE_TURN_ANGLE);
                m_mech.goIntake(true, ShootSpeed.INTAKE);
            } while (isAutonomous() && !turnDrive.hasArrived());
        }
        
        m_mech.stop();
        Timer.delay(0.25);

        { // Drive towards cube & pick it up
            final double DRIVE_TO_CUBE = 425.0;

            StraightDriveController straightDrive = new StraightDriveController(m_chassis);
            boolean intakeDone = false, driveDone = false;
            do {
                intakeDone = m_mech.goIntake(true, ShootSpeed.INTAKE);
                driveDone = straightDrive.setTarget(DRIVE_TO_CUBE, false);
            } while (isAutonomous() && (!driveDone || !intakeDone));
        }
        
        { // Go home
            do {
            } while (isAutonomous() && !m_mech.goHome(ShootSpeed.INTAKE));
            while (isAutonomous() && !m_mech.goHome(ShootSpeed.OFF));
        }
        
        // If i'm in Mirror Mode, back up a wee bit.
        {
            if (mirror == MirrorMode.MIRROR) {
                final double STEP_BACK = -200.0;
                StraightDriveController straightDrive = new StraightDriveController(m_chassis);
                do {
                } while (isAutonomous() && !straightDrive.setTarget(STEP_BACK, false));
            }
        }

        { // Turn in place
            final double TURNBACK_ANGLE = -60.0;
            TurnDriveController turnDrive = new TurnDriveController(m_chassis, mirror);
            do {
                turnDrive.setTarget(TURNBACK_ANGLE);
            } while (isAutonomous() && !turnDrive.hasArrived());
        }
        
        { // Raw drive in
            final double FINAL_STEPIN = (mirror == MirrorMode.MIRROR ? 600.0 : 800.0);
            long begin = System.currentTimeMillis();
            long timeout = 2000;
            RawForwardDriveController rawDrive = new RawForwardDriveController(m_chassis);
            do {
            } while (isAutonomous() && System.currentTimeMillis() - begin < timeout && !rawDrive.setTarget(FINAL_STEPIN));
        }

        { // Shoot the Power Cube from Home
            long begin = System.currentTimeMillis();
            do {
                m_mech.goHome(ShootSpeed.MEDIUM);
            } while (isAutonomous() && System.currentTimeMillis() - begin < 700);
        }
    }

    /**
     * This function is called periodically during autonomous
     */

    @Override
    public void autonomousPeriodic() {
        if (AUTONOMOUS_COMPLETE) {
            m_chassis.stop();
            Timer.delay(0.05);
            return;
        }

        // First things first, set our wrist & elevator to home.
        do {
        } while (isAutonomous() && !m_mech.goHome(ShootSpeed.OFF));

        m_chassis.resetGyro(GYRO_MULTIPLIER);

        /*
         * { // TODO: REMOVE THIS BLOCK, FOR TESTING ONLY auto_test();
         * AUTONOMOUS_COMPLETE = true; if (0 < System.currentTimeMillis()) { return; } }
         */

        // Gather Auto-Config settings from Dashboard.
        char my_side;
        AutoSide autoSide = RobotDashboard.getInstance().getAutoSide();

        switch (autoSide) {
        case RIGHT:
            my_side = 'R';
            break;
        case CENTER:
            my_side = 'C';
            break;
        case LEFT: // fall through
            my_side = 'L';
            break;
        default:
            my_side = 'N';
        }

        AutoMode orig_ally_mode = RobotDashboard.getInstance().getAutoMode();
        AutoMode ally_ability = orig_ally_mode;
        if (ally_ability == AutoMode.NONE) { ally_ability = AutoMode.WONT_INTERFERE_WITH_ALLY_FAR_SCALE; }
        
        // Gather Game Data from FMS
        char switch_side = GAME_DATA.charAt(0);
        char scale_side = GAME_DATA.charAt(1);

        // Figure out if we're mirroring our angles.
        MirrorMode mirror = MirrorMode.NORMAL;
        if (my_side == 'R' || (my_side == 'C' && switch_side == 'R')) {
            mirror = MirrorMode.MIRROR;
        }
        
        System.out.println("[AUTOLOG] AutoSide = " + autoSide.name() + " (" + my_side + "," + mirror.name() + "), AllyMode=" + orig_ally_mode.name() + " ("+ally_ability.name()+")");

        // Reset the encoders.
        m_chassis.resetEncoder();
        m_chassis.resetGyro(GYRO_MULTIPLIER);

        if (my_side == 'N') {
            auto_cross_baseline();
        } else if (my_side == 'C') {
            auto_center(mirror);
        } else {

            // Call the various scenarios.
            if (switch_side == my_side) {
                if (scale_side == my_side) {
                    auto_switchmine_scalemine(ally_ability, mirror);
                } else {
                    auto_switchmine_scaletheirs(ally_ability, mirror);
                }
            } else {
                if (scale_side == my_side) {
                    auto_switchtheirs_scalemine(ally_ability, mirror);
                } else {
                    auto_switchtheirs_scaletheirs(ally_ability, mirror);
                }
            }

        }

        System.out.println("[AUTOLOG] Strategy complete");

        // Now that we're at the end, just go home.
        do {
        } while (isAutonomous() && !m_mech.goHome(ShootSpeed.OFF));

        m_mech.stop();

        System.out.println("End Auto");
        AUTONOMOUS_COMPLETE = true;
    }

    @Override
    public void autonomousInit() {
        AUTONOMOUS_COMPLETE = false;
        GAME_DATA = DriverStation.getInstance().getGameSpecificMessage();
        System.out.println("[AUTOLOG] FMS Input: " + GAME_DATA);
        GYRO_MULTIPLIER = RobotDashboard.getInstance().getConfigValue(LABEL_GYRO_MULTIPLIER);
    }

    /**
     * This function is called at the beginning of TeleOp control
     */
    @Override
    public void teleopInit() {
        System.out.println("teleopInit");
        AUTONOMOUS_COMPLETE = true;
        // m_driverJoystick = new Joystick(JoystickPort.PRIMARY_JOYSTICK);
        m_chassis.resetEncoder();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        final double DRIVE_LIMIT_PCT = 0.7;

        while (isOperatorControl() && isEnabled()) {
            Timer.delay(0.005);

            Double limit = null;
            if (m_mech.getIsElevatorUp()) {
                limit = DRIVE_LIMIT_PCT;
            }

            m_chassis.periodic(m_driverJoystick, limit);
            m_mech.periodic(m_driverJoystick);
        }
    }

    /** ROBOT STATE */
    private RobotChassis m_chassis;
    private RobotMech m_mech;

    /** INPUT DEVICES */
    private Joystick m_driverJoystick;
    private CameraServer m_simpleCameraServer;
    // private EncoderDrive m_encoder;

    public static boolean AUTONOMOUS_COMPLETE = false;
}
