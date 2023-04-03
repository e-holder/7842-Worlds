package org.firstinspires.ftc.teamcode.hardware;

//  The following library needs to be in the robot controller's storage in the FIRST folder:
//  libOpenCvAndroid453.so   (MD5 checksum: 9d9a9ed11665dc92c91c475aad54ef94)
//  This library comes from the EasyOpenCV website (link is in Step 7 of the install doc).
//  Also see: https://github.com/OpenFTC/OpenCV-Repackaged

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.middleware.Vera;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class HwVision {
    // Landscape orientation (when mounted vertically, it was SIDEWAYS_LEFT).
    public static final OpenCvCameraRotation CAM_ROTATION = OpenCvCameraRotation.UPRIGHT;
    public static final int WEBCAM_HEIGHT_PIX = 288;
    public static final int WEBCAM_WIDTH_PIX = 544;

    private OpenCvCamera m_webcam;
    private Servo m_cameraServo;

    public void init(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hwMap.appContext.getPackageName());
        m_webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hwMap.get(WebcamName.class, "LogiWebcam"),
                cameraMonitorViewId);

        if (Vera.isVisionTestMode) {
            m_cameraServo = hwMap.get(Servo.class, "Servo");
        } else {
            m_cameraServo = hwMap.get(Servo.class, "CameraServo");
        }
    }

    public void startWebcamStreaming(OpenCvPipeline pipeline) {
        if (m_webcam != null && pipeline != null) {
            m_webcam.setPipeline(pipeline);
            // TODO: Is there a way to tell if the webcam is already open? If so, there may need
            //  to be a close camera async call put in somewhere before opening a new one.
            m_webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    try {
                        // TODO: Is there a way to tell if the webcam is already open? If it is,
                        //  then we wouldn't want to try to start streaming.
                        // Note: If CAM_ROTATION is changed, the height and width parameters need
                        //       to be swapped here.
                        m_webcam.startStreaming(WEBCAM_WIDTH_PIX, WEBCAM_HEIGHT_PIX, CAM_ROTATION);
                    } catch (OpenCvCameraException e) {
                        // Ignore for now. It seems that moving the pipelines to be non-static
                        // classes has introduced periodic errors where we try to startStreaming,
                        // but the camera is not opened.
                        // TODO: Consider logging errors here.
                    }
                }

                @Override
                public void onError(int errorCode) {
                    // TODO: Consider logging errors here.
                }
            });
        }
    }

    public void changeWebcamPipeline(OpenCvPipeline pipeline) {
        if (m_webcam != null && pipeline != null) {
            m_webcam.setPipeline(pipeline);
        }
    }

    public void stopWebcamStreaming() {
        if (m_webcam != null) {
            m_webcam.stopStreaming();
        }
    }

    public void setCameraTilt(double servoPos) {
        m_cameraServo.setPosition(servoPos);
    }

}
