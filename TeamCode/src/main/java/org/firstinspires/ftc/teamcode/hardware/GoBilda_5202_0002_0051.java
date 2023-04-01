package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

// goBILDA 5202 (Modern Robotics 5000-0002-0001)
@MotorType(
        ticksPerRev=28.0,  // 28 PPR (7 rises of channel A)
        gearing=1.0,
        maxRPM=6000,
        orientation= Rotation.CCW)
@DeviceProperties(
        xmlTag="ModernRobotics-GB5202-0002-0051",
        name="goBILDA-5202 (Wrist)",
        builtIn = true)
@DistributorInfo(
        distributor="goBILDA_distributor",
        model="goBILDA-5203-2402-0027",
        url="https://www.gobilda.com/5202-series-yellow-jacket-motor-1-1-ratio-" +
                "24mm-length-6mm-d-shaft-6000-rpm-3-3-5v-encoder/")

public interface GoBilda_5202_0002_0051
{
}