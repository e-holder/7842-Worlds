package org.firstinspires.ftc.teamcode.opmodes_teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.middleware.Vera;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskFindPole;

@TeleOp(name = "Test Find Scored Cones-High")
@Disabled
public class TestFindScoredConesH extends TestFindPoleM implements CONSTANTS {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeVera();
        initializeFindPoleTask(FindPoleMode.HIGH_SCORED_CONES);

        waitForStart();

        startFindingPole(FindPoleMode.HIGH_SCORED_CONES);

        do {
            getInputs();
            commandVera();
            reportData();
        } while (!isStopRequested());

        stopVera();
    }
}
