package org.firstinspires.ftc.teamcode.middleware;

import android.os.Environment;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HwIntake;
import org.firstinspires.ftc.teamcode.hardware.HwLift;
import org.firstinspires.ftc.teamcode.hardware.HwVera;
import org.firstinspires.ftc.teamcode.hardware.HwVision;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Drivetrain;

import java.io.FileWriter;
import java.io.IOException;

// Search for "EACH SUBSYSTEM" to see how to add a subsystem.

public class Vera implements CONSTANTS {
    // This variable can be set to "true" to bypass all hardware devices except vision. This allows
    // the code to be run on BinkyBoard (where other hardware isn't present).
    public static boolean isVisionTestMode = false;

    private final String ALLIANCE_PATH;
    private String CSV_LOG_PATH;
    private HwVera m_hwVera = new HwVera();
    private StringBuilder m_csvLogString = new StringBuilder();
    private boolean m_isAutonomous = false;
    public Alliance alliance;

    // Note: This is not ideal practice. The subsystem and alliance member data really should not
    // be public and should follow the "m_***" naming convention.
    // EACH SUBSYSTEM will have a middleware class instance (member data) defined here.
    public Drivetrain drivetrain;
    public Intake intake;
    public Lift lift;
    public Vision vision;
    // EACH SUBSYSTEM (end)


    // EACH SUBSYSTEM will have an accessor for its hardware layer object.
    public HwIntake getHwIntake() { return m_hwVera.getHwIntake(); }
    public HwLift getHwLift() { return m_hwVera.getHwLift(); }
    public HwVision getHwVision() { return m_hwVera.getHwVision(); }
    // EACH SUBSYSTEM (end)

    public Vera() {
        String extStoragePath = Environment.getExternalStorageDirectory().getAbsolutePath();
        ALLIANCE_PATH = String.format("%s/FIRST/data/alliance.txt", extStoragePath);
    }

    public void init(HardwareMap hwMap, boolean isAutonomous, boolean visionTestMode,
                     VeraPipelineType initialPipelineType, Telemetry telemetry) {
        m_isAutonomous = isAutonomous;
        Vera.isVisionTestMode = visionTestMode;
        m_hwVera.init(hwMap, true);

        // Setup CSV file logging variables
        String extStoragePath = Environment.getExternalStorageDirectory().getAbsolutePath();
        if (m_isAutonomous == true) {
            CSV_LOG_PATH = String.format("%s/CsvLogs/robot-data-a.csv", extStoragePath);
        }
        else {
            CSV_LOG_PATH = String.format("%s/CsvLogs/robot-data-t.csv", extStoragePath);
        }
        m_csvLogString.setLength(0);


        // EACH SUBSYSTEM will construct its middleware class instance here.
        if (!Vera.isVisionTestMode) {
            drivetrain = new Drivetrain(hwMap);
            intake = new Intake(this);
            lift = new Lift(this);
        }
        vision = new Vision(this, telemetry);
        // EACH SUBSYSTEM (end)

        if (m_isAutonomous) {
            vision.startStreaming(initialPipelineType);
            logCsvString("Started streaming " + initialPipelineType);
        }
    }

    public boolean isAutonomous() {
        return m_isAutonomous;
    }

    public void stopVera() {
        writeCsvLogData();
        vision.stopWebcamStreaming();
    }

    public void setAlliance(Alliance matchAlliance) {
        alliance = matchAlliance;
        vision.setAlliance(alliance);

        // Write the current alliance to a file on the robot's SD Card so that TeleOp can determine
        // the alliance last used in Autonomous.
        if (m_isAutonomous) {
            try {
                FileWriter writer = new FileWriter(ALLIANCE_PATH, false);
                writer.write(alliance + "\r\n");
                writer.close();
            } catch (Exception e) {
                logCsvString("setAlliance - file write failed.");
            }
        }
    }

    public void getInputs(boolean isAutonomous) {
        // The getInputs function must be the first function called in the main loop of TeleOp
        // or Autonomous OpModes. And clearBulkCache must be the first thing called in this
        // function.
        m_hwVera.clearBulkCache();

        // EACH SUBSYSTEM will get its inputs here.
        // Note: vision is omitted because pipelines run independently in another thread.
        if (!isVisionTestMode) {
            intake.getInputs();
            lift.getInputs();
        }
        // EACH SUBSYSTEM (end)
    }

    public void reportData(Telemetry telemetry) {
        // EACH SUBSYSTEM will report data here.
        if (!isVisionTestMode) {
            drivetrain.reportData(telemetry);
            intake.reportData(telemetry);
            lift.reportData(telemetry);
        }
        vision.reportData(telemetry);
        // EACH SUBSYSTEM (end)
    }

    public void commandVera() {
        // EACH SUBSYSTEM will process robot commands here.
        if (!isVisionTestMode) {
            drivetrain.update();
            intake.update();
            lift.update();
        }
        // EACH SUBSYSTEM (end)
    }

    public void logCsvString(String record) {
        m_csvLogString.append(record).append("\n");
    }

    public void writeCsvLogData() {
        // Include subsystem logging.
        // EACH SUBSYSTEM needs to add its log data (if any) here.
        if (drivetrain != null && drivetrain.getLogString().length() > 0) {
            logCsvString(drivetrain.getLogString().toString());
        }
        if (intake != null && intake.getLogString().length() > 0) {
            logCsvString(" Intake ");
            logCsvString(intake.getLogString().toString());
        }
        if (lift != null && lift.getLogString().length() > 0) {
            logCsvString(" lift ");
            logCsvString(lift.getLogString().toString());
        }
        if (vision != null && vision.getLogString().length() > 0) {
            logCsvString(" Vision ");
            logCsvString(vision.getLogString().toString());
        }

        // If any data was requested to be logged (non-zero length string), then write the log
        // data to a file on the robot controller's file system.

        if (m_csvLogString.length() > 0) {
            // The "false" argument indicates "do not append". We want a new file each time.
            try (FileWriter csvWriter = new FileWriter(CSV_LOG_PATH, false)) {
                csvWriter.write(m_csvLogString.toString());
            } catch (IOException e) {
                // Ignore exceptions
            }
        }
    }
}
