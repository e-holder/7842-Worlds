package org.firstinspires.ftc.teamcode.middleware;

import android.os.Environment;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HwIntake;
import org.firstinspires.ftc.teamcode.hardware.HwLift;
import org.firstinspires.ftc.teamcode.hardware.HwVera;
import org.firstinspires.ftc.teamcode.hardware.HwVision;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Drivetrain;

import java.io.BufferedReader;
import java.io.FileReader;
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
    private StringBuilder m_timerLog = new StringBuilder();
    private StringBuilder m_csvLogString = new StringBuilder();
    private boolean m_isAutonomous = false;
    private int m_loopCount = 0;
    private Alliance m_alliance;
    private ElapsedTime m_timer = new ElapsedTime();

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

    public Alliance getAlliance() { return m_alliance; }
    public int getLoopCount() { return m_loopCount; }

    public void init(HardwareMap hwMap, boolean isAutonomous, boolean visionTestMode,
                     VeraPipelineType initialPipelineType, Telemetry telemetry) {
        m_isAutonomous = isAutonomous;
        Vera.isVisionTestMode = visionTestMode;

        PhotonCore.enable();
        PhotonCore.experimental.setMaximumParallelCommands(7);
        m_hwVera.init(hwMap, true);

        // Setup CSV file logging variables
        String extStoragePath = Environment.getExternalStorageDirectory().getAbsolutePath();
        if (m_isAutonomous) {
            CSV_LOG_PATH = String.format("%s/CsvLogs/robot-data-a.csv", extStoragePath);
        } else {
            CSV_LOG_PATH = String.format("%s/CsvLogs/robot-data-t.csv", extStoragePath);
        }
        m_csvLogString.setLength(0);


        // EACH SUBSYSTEM will construct its middleware class instance here.
        if (!Vera.isVisionTestMode) {
            drivetrain = new Drivetrain(hwMap, this);
            intake = new Intake(this);
            lift = new Lift(this);
        }
        vision = new Vision(this, telemetry);  // Telemetry needed for pipelines.
        // EACH SUBSYSTEM (end)

        vision.startStreaming(initialPipelineType);
        logCsvString("Started streaming " + initialPipelineType);
    }

    public boolean isAutonomous() {
        return m_isAutonomous;
    }

    public void stopVera() {
        vision.stopWebcamStreaming();
        writeCsvLogData();
    }

    public void setAlliance(Alliance matchAlliance) {
        logCsvString("Alliance: " + matchAlliance);
        m_alliance = matchAlliance;
        vision.setAlliance(m_alliance);

        // Write the current alliance to a file on the robot's SD Card so that TeleOp can determine
        // the alliance last used in Autonomous.
        if (m_isAutonomous) {
            try {
                FileWriter writer = new FileWriter(ALLIANCE_PATH, false);
                writer.write(m_alliance + "\r\n");
                writer.close();
            } catch (Exception e) {
                logCsvString("setAlliance - file write failed.");
            }
        }
    }

    // This function is called by TeleOp during initialization to set the alliance to what was
    // used for the most recent Autonomous OpMode that was run.
    public void setAllianceFromPriorAutonomousRun() {
        String line = null;
        try {
            FileReader reader = new FileReader(ALLIANCE_PATH);
            BufferedReader bufferedReader = new BufferedReader(reader);
            line = bufferedReader.readLine();
            reader.close();

        } catch (Exception e) {
            logCsvString("determineAlliance - file read failed.");
        }

        if (line != null && line.contains("RED")) {
            setAlliance(Alliance.RED);
        } else if (line != null && line.contains("BLUE")) {
            setAlliance(Alliance.BLUE);
        } else {
            logCsvString("determineAlliance - alliance string check failed.");
            setAlliance(Alliance.RED);
        }
    }

    public void startTimer() {
        m_timer.reset();
    }

    public void getInputs(boolean isAutonomous) {
        m_loopCount++;
        // The getInputs function must be the first function called in the main loop of TeleOp
        // or Autonomous OpModes. And clearBulkCache must be the first thing called in this
        // function.
        m_hwVera.clearBulkCache();

        // EACH SUBSYSTEM will get its inputs here.
        if (!isVisionTestMode) {
            intake.getInputs();
            lift.getInputs();
        }
        vision.getInputs();

        // EACH SUBSYSTEM (end)
    }

    public void commandVera() {
        // EACH SUBSYSTEM will process robot commands here.
        if (!isVisionTestMode) {
            if(!isAutonomous()) { drivetrain.veraUpdateTeleOp(); }
            drivetrain.update();
            intake.update();
            lift.update();
        }
        // Vision is omitted since its pipelines run in another thread,
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

    // KEEP THIS - commented out code that is used to profile main loop time issues.
//    private double m_priorLoop_ms = 0;
//    private double m_priorLog_ms = 0;
//    public void logMainLoopTime() {
//        double ms = m_timer.milliseconds();
//        m_timerLog.append((int)(ms - m_priorLoop_ms))
//                .append(" Loop ")
//                .append(m_loopCount).append("\n");
//        m_priorLoop_ms = ms;
//        m_priorLog_ms = ms;
//    }
//
//    public void logTime(int level, String label) {
//        double ms = m_timer.milliseconds();
//        m_timerLog.append(new String(new char[level*2]).replace('\0', ' '))
//                .append((int)(m_timer.milliseconds() - m_priorLog_ms)).append("  ")
//                .append(label).append("\n");
//        m_priorLog_ms = ms;
//    }

    public void logCsvString(String record) {
        m_csvLogString.append(record).append("\n");
    }

    public void writeCsvLogData() {
        if (m_timerLog.length() > 0) {
            logCsvString(m_timerLog.toString());
        }

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
