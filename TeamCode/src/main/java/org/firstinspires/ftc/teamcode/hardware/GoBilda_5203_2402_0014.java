package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

// goBILDA 5203 (Modern Robotics 5000-0002-0001)
@MotorType(
        ticksPerRev=384.5,
        gearing=13.7,
        maxRPM=435,
        orientation= Rotation.CCW)
@DeviceProperties(
        xmlTag="ModernRobotics-GB5203-2402-0014",
        name="goBILDA-5203 (Drivetrain)",
        builtIn = true)
@DistributorInfo(
        distributor="goBILDA_distributor",
        model="goBILDA-5203-2402-0014",
        url="https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio" +
                "-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/")

public interface GoBilda_5203_2402_0014
{
}