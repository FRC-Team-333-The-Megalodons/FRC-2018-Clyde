package org.usfirst.frc.team333.robot;

public class RobotMap {

    public static class VictorPort {
        public static final int INTAKE = 0;
        public static final int DRIVE_LEFT_1 = 1;
        public static final int DRIVE_LEFT_2 = 2;
        public static final int DRIVE_LEFT_3 = 3;
        public static final int DRIVE_RIGHT_1 = 4;
        public static final int DRIVE_RIGHT_2 = 7;
        public static final int DRIVE_RIGHT_3 = 8;
        public static final int ELEVATOR_1 = 6; // To correctly wire robot, elevator UP should be:
        public static final int ELEVATOR_2 = 5; // 5 = Negative, 6 = Positive
        public static final int WRIST = 9;
    }

    public static class SolenoidPort {
        public static final int DRIVE_TRANS_LOW = 1;
        public static final int DRIVE_TRANS_HIGH = 2;
        public static final int GRIPPER_LEFT = 3;
        public static final int GRIPPER_RIGHT = 4;
        public static final int RATCHET_RELEASE = 0;
        public static final int WRIST_UP_POSITION = 5;
        public static final int WRIST_DOWN_POSITION = 6;
    }

    public static class GyroPort {
        public static final int MAIN_GYRO = 0;
    }

    public static class CompressorPort {
        public static final int MAIN_COMPRESSOR = 0;
    }

    public static class OperatorInputPort {
        public static final int DRIVER_STICK = 0;
    }

    public static class PotentiometerPort {
        public static final int WRIST_POTENTIOMETER = 0;
        public static final int ELEVATOR_POTENTIOMETER = 1;
    }

    public static class DigitalSensorPort {
        public static final int ENCODER_LEFT_A = 0;
        public static final int ENCODER_LEFT_B = 1;
        public static final int ENCODER_RIGHT_A = 2;
        public static final int ENCODER_RIGHT_B = 3;
        public static final int LIMITER = 4;
    }

    public static class RelayPort {
    }

    public static class CameraPort {
        public static final int FRONT_CAMERA = 0;
        public static final int REAR_CAMERA = 1;
    }

    public static class JoystickPort {
        public static final int PRIMARY_JOYSTICK = 0;
    }

    public static enum AutoSide {
        NONE, LEFT, CENTER, RIGHT
    }

    public static enum AutoMode {
        NONE, WILL_INTERFERE_WITH_ALLY_FAR_SCALE, WONT_INTERFERE_WITH_ALLY_FAR_SCALE
    }
    
    public static enum MirrorMode {
        NORMAL, MIRROR
    }
    
    public static enum ShootSpeed {
        OFF, DROOL, SLOW, MEDIUM, FAST, INTAKE
    }

    public static enum ADJUSTMENT {
        NONE, DOWN, UP
    };
}