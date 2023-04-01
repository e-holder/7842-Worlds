package org.firstinspires.ftc.teamcode.opmodes_teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Hello World")
@Disabled
public class HelloWorld extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello","World!");
    }

    @Override
    public void loop() {
        // Do nothing
    }
}