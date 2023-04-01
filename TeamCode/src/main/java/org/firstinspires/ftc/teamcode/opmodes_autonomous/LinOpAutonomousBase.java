package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.middleware.Vera;

// This is a "base" class for all autonomous OpModes. All code that is shared between autonomous
// modes will go here.
public abstract class LinOpAutonomousBase extends LinearOpMode implements CONSTANTS {

    private static final int IMU_CALIBRATION_TIME_MS = 50;

    protected Alliance m_alliance;
    protected FieldSide m_fieldSide;
    protected Vera m_vera = new Vera();
    protected boolean m_isVisionTestMode;
    protected VeraPipelineType m_initialPipelineType;

    // Each autonomous OpMode (children of this base class) needs to implement initializeRoute.
    // Within initializeRoute, the setupAlliance method should be called first, followed by
    // as many addTask method calls as needed.
    protected abstract void initializeRoute();

    // Autonomous routes can override this function to specify vision test mode and/or identify a
    // a different startup pipeline.
    protected void preInitSetup() {
        m_isVisionTestMode = false;
        m_initialPipelineType = VeraPipelineType.SIGNAL;
    }

    // Each child class needs to call this method from their initializeRoute method.
    protected void setupAlliance(Alliance alliance, FieldSide fieldSide) {
        // Setup "globals" for all autonomous tasks to access.
        m_vera.setAlliance(alliance);
        m_alliance = alliance;
        m_fieldSide = fieldSide;
    }

    private void initializeVera() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        preInitSetup();  // Allows for override in various child OpModes.
        m_vera.init(hardwareMap, true,
                m_isVisionTestMode, m_initialPipelineType, telemetry);

        // Make sure the IMU gyro is calibrated before continuing.
        while (!isStopRequested() && !m_vera.isGyroCalibrated()) {
            sleep(IMU_CALIBRATION_TIME_MS);
            idle();
        }
        telemetry.addData("IMU calibration status:", m_vera.getGyroCalibrationStatus());
        telemetry.update();

        // Call abstract function defined in the autonomous OpMode.
        initializeRoute();

        if (Vera.isVisionTestMode) {
            telemetry.addData("WARNING:", "Vision test mode!");
        } else {
            telemetry.addData("Status", "Initialized.");
        }
        telemetry.update();
    }

    private void reportData() {
        m_vera.reportData(telemetry);
        telemetry.update();
    }

    private void stopVera() {
        String stopMessage = m_vera.writeCsvLogData();
        telemetry.addData("Stop", stopMessage);
        telemetry.update();

        m_vera.stopVera();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initializeVera();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until time runs out, the driver presses STOP.
        TaskStatus taskStatus = TaskStatus.RUNNING;
        while (opModeIsActive()) {

            // getInputs MUST be the first thing called at the beginning of the main loop.
            m_vera.getInputs(true);


            m_vera.commandVera();

            reportData();
        }

        stopVera();
    }
}
