package org.usfirst.frc.team333.robot;

import java.util.Map;
import java.util.TreeMap;

import org.usfirst.frc.team333.robot.RobotMap.AutoMode;
import org.usfirst.frc.team333.robot.RobotMap.AutoSide;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotDashboard {
    private final String LABEL_GYRO_READING = new String("Gyro Heading:");

    private final String LABEL_AVERAGE_ENCODER_DISTANCE = new String("Average Left Right Encoder Distance:");

    private final String LABEL_ELEVATOR_POSITION = new String("Elevator Position:");
    private final String LABEL_WRIST_POSITION = new String("Wrist Position: ");
    private final String LABEL_LIMIT_SWITCH_READING = new String("Limit switch Reading:");

    private final String LABEL_TRANSMISSION_STATE = new String("Transmission State:");

    private final String LABEL_AUTO_SIDE_CHOOSER = new String("[Auto-Config] Auto Side:");
    private final String LABEL_ACTUAL_AUTO_SIDE = new String("ACTUAL AUTO SIDE = ");
    private final String LABEL_AUTO_MODE_CHOOSER = new String("[Auto-Config] Auto Ally Behavior:");
    private final String LABEL_ACTUAL_AUTO_MODE = new String("ACTUALLY ALLY MODE = ");

    protected RobotDashboard() {
        m_configValues = new TreeMap<String, Double>();

        // Autonomous Side chooser
        m_autoSideChooser = new SendableChooser<AutoSide>();
        SmartDashboard.putData(LABEL_AUTO_SIDE_CHOOSER, m_autoSideChooser);
        m_autoSideChooser.addDefault(AutoSide.NONE.name(), AutoSide.NONE);
        m_autoSideChooser.addObject(AutoSide.LEFT.name(), AutoSide.LEFT);
        m_autoSideChooser.addObject(AutoSide.CENTER.name(), AutoSide.CENTER);
        m_autoSideChooser.addObject(AutoSide.RIGHT.name(), AutoSide.RIGHT);

        // Autonomous Mode chooser
        m_autoModeChooser = new SendableChooser<AutoMode>();
        SmartDashboard.putData(LABEL_AUTO_MODE_CHOOSER, m_autoModeChooser);
        m_autoModeChooser.addDefault(AutoMode.NONE.name(), AutoMode.NONE);
        m_autoModeChooser.addObject(AutoMode.WONT_INTERFERE_WITH_ALLY_FAR_SCALE.name(), AutoMode.WONT_INTERFERE_WITH_ALLY_FAR_SCALE);
        m_autoModeChooser.addObject(AutoMode.WILL_INTERFERE_WITH_ALLY_FAR_SCALE.name(), AutoMode.WILL_INTERFERE_WITH_ALLY_FAR_SCALE);

        m_monitor = new Monitor();
        Thread thread = new Thread(m_monitor);
        thread.start();
    }

    public static RobotDashboard getInstance() {
        if (instance == null) {
            instance = new RobotDashboard();
        }
        return instance;
    }

    public void setGyro(NavXGyro gyro) {
        m_monitor.setGyro(gyro);
    }

    public void setLimitSwitch(DigitalInput limitSwitch) {
        m_monitor.setLimitSwitch(limitSwitch);
    }

    public void setElevatorPotentiometer(AnalogPotentiometer potentiometer) {
        m_monitor.setElevatorPotentiometer(potentiometer);
    }

    public void setWristPotentiometer(AnalogPotentiometer potentiometer) {
        m_monitor.setWristPotentiometer(potentiometer);
    }

    public void setDriveTrainEncoder(DriveTrainEncoder encoder) {
        m_monitor.setDriveTrainEncoder(encoder);
    }

    public void setCompressor(Compressor compressor) {
        m_monitor.setCompressor(compressor);
    }

    public void setTransmission(Solenoidal transmission) {
        m_monitor.setTransmission(transmission);
    }

    public void setMonitoredValue(String key, double value) {
        m_monitor.setValue(key, value);
    }

    public void setConfigValue(String key, double value) {
        m_configValues.put(key, value);
        SmartDashboard.putNumber(key, value);
    }

    public double getConfigValue(String key) {
        if (m_configValues.containsKey(key)) {
            m_configValues.put(key, SmartDashboard.getNumber(key, 0.0));
            return m_configValues.get(key);
        }
        return -1.0;
    }

    public AutoMode getAutoMode() {
        return m_autoModeChooser.getSelected();
    }

    public AutoSide getAutoSide() {
        return m_autoSideChooser.getSelected();
    }

    class Monitor implements Runnable {
        public Monitor() {
            m_doubleValues = new TreeMap<String, Double>();
        }

        public void setGyro(NavXGyro gyro) {
            m_gyro = gyro;
        }

        public void setDriveTrainEncoder(DriveTrainEncoder encoder) {
            m_encoder = encoder;
        }

        public void setElevatorPotentiometer(AnalogPotentiometer potentiometer) {
            m_elevatorPotentiometer = potentiometer;
        }

        public void setWristPotentiometer(AnalogPotentiometer potentiometer) {
            m_wristPotentiometer = potentiometer;
        }

        public void setLimitSwitch(DigitalInput limitSwitch) {
            m_limitSwitch = limitSwitch;
        }

        public void setCompressor(Compressor compressor) {
            m_compressor = compressor;
        }

        public void setTransmission(Solenoidal transmission) {
            m_transmission = transmission;
        }

        public void setValue(String key, double value) {
            m_doubleValues.put(key, value);
        }

        @Override
        public void run() {
            SmartDashboard.putNumber(LABEL_GYRO_READING, -1);
            SmartDashboard.putString(LABEL_TRANSMISSION_STATE, "Unknown");

            while (true) {
                // Paint data for specific hardware
                if (m_gyro != null) {
                    // This causes race conditions with the history.
                    SmartDashboard.putNumber(LABEL_GYRO_READING, m_gyro.getRawAngle());
                }

                if (m_encoder != null) {
                    SmartDashboard.putNumber(LABEL_AVERAGE_ENCODER_DISTANCE, m_encoder.averageEncoderDistance());
                    SmartDashboard.putNumber("Left Encoder", m_encoder.leftPidGet());
                    SmartDashboard.putNumber("Right Encoder", m_encoder.rightPidGet());
                }

                if (m_wristPotentiometer != null) {
                    SmartDashboard.putNumber(LABEL_WRIST_POSITION, m_wristPotentiometer.get());
                }
                if (m_elevatorPotentiometer != null) {
                    SmartDashboard.putNumber(LABEL_ELEVATOR_POSITION, m_elevatorPotentiometer.get());
                }

                if (m_limitSwitch != null) {
                    SmartDashboard.putBoolean(LABEL_LIMIT_SWITCH_READING, m_limitSwitch.get());
                }

                if (m_compressor != null) {
                    /*
                     * SmartDashboard.putString(LABEL_COMPRESSOR_SWITCH,
                     * m_compressor.getPressureSwitchValue() ? "Stopped" : "Filling");
                     * SmartDashboard.putNumber(LABEL_COMPRESSOR_READING,
                     * m_compressor.getCompressorCurrent());
                     */
                }

                if (m_transmission != null) {
                    SmartDashboard.putString(LABEL_TRANSMISSION_STATE, m_transmission.get() ? "High" : "Low");
                }

                // Paint config values
                for (Map.Entry<String, Double> entry : m_doubleValues.entrySet()) {
                    SmartDashboard.putNumber(entry.getKey(), entry.getValue().doubleValue());
                }
                
                if (m_autoSideChooser != null) {
                    SmartDashboard.putString(LABEL_ACTUAL_AUTO_SIDE, m_autoSideChooser.getSelected().name());
                }
                
                if (m_autoModeChooser != null) {
                    SmartDashboard.putString(LABEL_ACTUAL_AUTO_MODE, m_autoModeChooser.getSelected().name());
                }

                Timer.delay(0.25);
            }
        }

        private NavXGyro m_gyro;
        private DriveTrainEncoder m_encoder;
        private TreeMap<String, Double> m_doubleValues;
        private Compressor m_compressor;
        private Solenoidal m_transmission;
        private AnalogPotentiometer m_wristPotentiometer;
        private AnalogPotentiometer m_elevatorPotentiometer;
        private DigitalInput m_limitSwitch;
    }

    private static RobotDashboard instance = null;
    private Monitor m_monitor;
    private SendableChooser<AutoMode> m_autoModeChooser;
    private SendableChooser<AutoSide> m_autoSideChooser;
    private TreeMap<String, Double> m_configValues;
}
