package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.middleware.Vera;

import java.util.List;

// Search for "EACH SUBSYSTEM" to see how to add a subsystem.

// This class encapsulates the hardware within the Robot Controller (i.e., the IMU and Hubs).
public class HwVera {
    private BNO055IMU m_imu;
    private List<LynxModule> m_allHubs = null;

    // EACH SUBSYSTEM will have a hardware class instance (member data) created by calling a
    // constructor here.
    private HwIntake m_hwIntake = new HwIntake();
    private HwLift m_hwLift = new HwLift();
    private HwPoleNav m_hwPoleNav = new HwPoleNav();
    private HwVision m_hwVision = new HwVision();
    // EACH SUBSYSTEM (end)

    public void init(HardwareMap hwMap, boolean enableManualBulkCaching) {
        // TODO: Figure out if the IMU configuration is correct. Notes from 2020 ---> For
        //  BinkyBoard we used AdaFruit IMU. But Vera is configured as REV Expansion Hub IMU. And
        //  what is odd is that the IMU appears to be working, EVEN WHEN the Expansion Hub hasn't
        //  been wired in to the Control Hub (no interface cable). Also, Vera had the IMU on I2C
        //  Bus 0, Device 0, but I couldn't get the Color Sensor to work unless it was Device 0,
        //  so I moved the IMU to Device 1.
        m_imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.mode           = BNO055IMU.SensorMode.IMU;
        imuParams.angleUnit      = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit      = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.loggingEnabled = false;
        m_imu.initialize(imuParams);

        if (enableManualBulkCaching) {
            // Get access to a list of Expansion Hub Modules to enable changing caching methods.
            m_allHubs = hwMap.getAll(LynxModule.class);
            //  Set all Expansion hubs to use the AUTO Bulk Caching mode.
            for (LynxModule module : m_allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }

        // EACH SUBSYSTEM will be initialized here.
        if (!Vera.isVisionTestMode) {
            m_hwIntake.init(hwMap);
            m_hwLift.init(hwMap);
            m_hwPoleNav.init(hwMap);
        }
        m_hwVision.init(hwMap);
        // EACH SUBSYSTEM (end)
    }

    public boolean isGyroCalibrated() {
        return m_imu.isGyroCalibrated();
    }

    public String getGyroCalibrationStatus() {
        return m_imu.getCalibrationStatus().toString();
    }

    public double getImuHeading_deg() {
        Orientation angles = m_imu.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES);
        // The "Z" angle is returned in the 'firstAngle' field, but the sign increases as the
        // heading turns counter-clockwise. This is opposite what we want. So, we need to negate
        // the angle.
        return -angles.firstAngle;
    }

    // This function must be called EVERY time through the main OpMode loop.
    // This should only be used if init was called with enableManualBulkCaching true.
    public void clearBulkCache() {
        for (LynxModule module : m_allHubs) {
            module.clearBulkCache();
        }
    }

    // EACH SUBSYSTEM will have an accessor to "get" a pointer to the hardware subsystem class.
    public HwIntake getHwIntake() { return m_hwIntake; }
    public HwLift getHwLift() { return m_hwLift; }
    public HwPoleNav getHwPoleNav() { return m_hwPoleNav; }
    public HwVision getHwVision() { return m_hwVision; }
    // EACH SUBSYSTEM (end)

}
