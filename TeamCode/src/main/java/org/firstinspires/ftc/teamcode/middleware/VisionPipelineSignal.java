package org.firstinspires.ftc.teamcode.middleware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.HwVision;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class VisionPipelineSignal extends OpenCvPipeline {

    public enum Signal {
        UNKNOWN,
        ZONE1,
        ZONE2,
        ZONE3
    }

    // TODO: Adjust these two constants to adjust signal vision based on venue lighting.
    private final double BOTTOM_PAD = 0.1;
    private final double TOP_PAD = 0.1;

    // Note: The origin (0, 0) of the input image is the top-left of the screen.
    // Adjust these numbers to adjust the box size so it fits the signal cone image (with a little
    // extra on each side to account for field differences and driver robot placement).
    private final int BOX_WIDTH = 85;
    private final int BOX_HEIGHT = 80;
    // Adjust the top/left constants in these expressions to position the box.
    private final int BOX_TOP = HwVision.WEBCAM_HEIGHT_PIX - BOX_HEIGHT - 130;
    private final int BOX_LEFT = (HwVision.WEBCAM_WIDTH_PIX / 2) - (BOX_WIDTH / 2) + 42;

    // These constants are derived from the above and the camera image size.
    private final Point BOX_TOP_LEFT = new Point(BOX_LEFT, BOX_TOP);
    private final Point BOX_BOTTOM_RIGHT = new Point(BOX_LEFT + BOX_WIDTH,
            BOX_TOP + BOX_HEIGHT);
    private final Rect BOX = new Rect(BOX_TOP_LEFT, BOX_BOTTOM_RIGHT);
    private final Point BOX_MID_LEFT = new Point(BOX_LEFT, BOX_TOP+BOX_HEIGHT/2);
    private final Point BOX_MID_RIGHT = new Point(BOX_LEFT+BOX_WIDTH, BOX_TOP+BOX_HEIGHT/2);

    private final Scalar WHITE = new Scalar(255);
    private final int BORDER_THICKNESS = 2;

    private Telemetry m_telemetry;


    // Volatile: possible access by OpMode thread w/o synchronization.
    private volatile int m_frameCount = 0;
    private volatile double m_avgBox;
    private volatile double m_avgTop;
    private volatile double m_avgBottom;
    private volatile Signal m_signal = Signal.UNKNOWN;

    // Accessors to allow retrieving the results.
    public Signal getSignal() {
        return m_signal;
    }

    public double getSignalBoxAvg() {
        return m_avgBox;
    }

    public double getSignalTopAvg() {
        return m_avgTop;
    }

    public double getSignalBottomAvg() {
        return m_avgBottom;
    }

    public int getFrameCount() {
        return m_frameCount;
    }

    public VisionPipelineSignal(Telemetry telemetry) {
        m_telemetry = telemetry;
    }

    @Override
    public void init(Mat firstFrame) {
        // Nothing to do here.
    }

    @Override
    public Mat processFrame(Mat matInput) {
        m_frameCount++;

        // Simulate input image from a static file.
        //matInput = Imgcodecs.imread("/sdcard/FIRST/data/Zone1.jpg");

        Mat box = matInput.submat(BOX);
        //Convert the box to grayscale.
        Imgproc.cvtColor(box, box, Imgproc.COLOR_RGB2GRAY);

        // Get average pixel value for each component.
        m_avgBox = Core.mean(box).val[0];
        m_avgTop = Core.mean(box.rowRange(0,BOX_HEIGHT/2)).val[0];
        m_avgBottom = Core.mean(box.rowRange(BOX_HEIGHT/2, BOX_HEIGHT-1)).val[0];

        double sum = m_avgTop + m_avgBottom + 1.0e-15;  // Add tiny value to defend divide by zero
        double topRatio = m_avgTop / sum;
        double bottomRatio = m_avgBottom / sum;

        if (topRatio > (bottomRatio + BOTTOM_PAD)) {
            m_signal = Signal.ZONE1;
        } else if (bottomRatio > (topRatio + TOP_PAD)) {
            m_signal = Signal.ZONE2;
        } else  {
            m_signal = Signal.ZONE3;
        }

        //=========================================================================================
        // Draw a white rectangle around the cone box on the image.
        Imgproc.rectangle(
                matInput, // Buffer to draw on
                BOX_TOP_LEFT, // First point which defines the rectangle
                BOX_BOTTOM_RIGHT, // Second point which defines the rectangle
                WHITE, // The color the rectangle is drawn in
                BORDER_THICKNESS);
        Imgproc.line(matInput,
                BOX_MID_LEFT,BOX_MID_RIGHT,WHITE);

        // To see the output image: Use phone to select opmode, press Init, wait a bit,
        // then  ...->Camera Stream
        //=========================================================================================

        return matInput;
    }
}