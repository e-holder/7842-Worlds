package org.firstinspires.ftc.teamcode.opmodes_autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.middleware.Vera;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.AutonomousTask;

// This is a "base" class for all autonomous OpModes. All code that is shared between autonomous
// modes will go here.
public abstract class LinOpAutonomousBase extends LinearOpMode implements CONSTANTS {

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

    protected void initializeVera() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Setup old AutonomousTask (slight hack here) so it still works after eliminating the
        // route_engine approach to autonomous.
        AutonomousTask.vera = m_vera;
        AutonomousTask.telemetry = telemetry;

        preInitSetup();  // Allows for override in various child OpModes.

        m_vera.init(hardwareMap, true,
                m_isVisionTestMode, m_initialPipelineType, telemetry);

        // Call abstract function defined in the autonomous OpMode.
        initializeRoute();

        if (Vera.isVisionTestMode) {
            telemetry.addData("WARNING:", "Vision test mode!");
        } else {
            telemetry.addData("Status", "Initialized.");
        }
        telemetry.update();
    }

    protected void getInputs() {
        m_vera.getInputs(true);
    }

    protected void commandVera() {
        m_vera.commandVera();
    }

    protected void reportData() {
        m_vera.reportData(telemetry);
        telemetry.update();
    }

    protected void stopVera() {
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
        while (opModeIsActive()) {
            // getInputs MUST be the first thing called at the beginning of the main loop.
            getInputs();
            commandVera();
            reportData();
        }

        stopVera();
    }
}
