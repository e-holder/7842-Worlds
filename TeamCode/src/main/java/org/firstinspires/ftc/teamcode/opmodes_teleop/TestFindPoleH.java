package org.firstinspires.ftc.teamcode.opmodes_teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.middleware.CONSTANTS;
import org.firstinspires.ftc.teamcode.middleware.Vera;
import org.firstinspires.ftc.teamcode.opmodes_autonomous.tasks.TaskFindPole;

@TeleOp(name = "Test Find Pole-High")
//@Disabled
public class TestFindPoleH extends TestFindPoleM implements CONSTANTS {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeVera();
        initializeFindPoleTask(FindPoleMode.HIGH_POLE);

        waitForStart();

        startFindingPole(FindPoleMode.HIGH_POLE);

        do {
            getInputs();
            commandVera();
            reportData();
        } while (!isStopRequested());

        stopVera();
    }
}
