package org.firstinspires.ftc.teamcode.middleware;

import static org.firstinspires.ftc.teamcode.hardware.HwVision.WEBCAM_HEIGHT_PIX;
import static org.firstinspires.ftc.teamcode.hardware.HwVision.WEBCAM_WIDTH_PIX;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

public class VisionPipelineFindPole extends OpenCvPipeline implements CONSTANTS {

    // In the Cb plane of a YCrCb image, the yellow poles should be very dark.
    public final double POLE_LIGHT_THRESH = 0.0;
    public final double POLE_DARK_THRESH = 100.0;


    // Note: The origin (0, 0) of the input image is the top-left of the screen.

    // Define a box for the area where we expect to see the pole we are about to score on. It needs
    // to be at the very top of the image area, but not too wide (we should have pretty accurate
    // robot navigation). The height of the box probably doesn't need to be really tall. If there
    // are 3 or 4 cones already on the pole we'd like to see over the top of those, but we do need
    // enough delta between the top and bottom of the box to detect how much the pole is leaning
    // left or right.
    private final int BOX_CENTER_ADJ = 0;
    public static final int BOX_WIDTH = 400;
    private final int BOX_HEIGHT = 150;
    private final int BOX_TOP = 0;
    private final int BOX_LEFT = (WEBCAM_WIDTH_PIX / 2) - (BOX_WIDTH / 2) + BOX_CENTER_ADJ;
    private final Point BOX_TOP_LEFT = new Point(BOX_LEFT, BOX_TOP);
    private final Point BOX_BOTTOM_RIGHT =
            new Point(BOX_LEFT + BOX_WIDTH, BOX_TOP + BOX_HEIGHT);
    private final Rect BOX = new Rect(BOX_TOP_LEFT, BOX_BOTTOM_RIGHT);

    private final Telemetry m_telemetry;

    private final Mat m_matYCrCb = new Mat(WEBCAM_HEIGHT_PIX, WEBCAM_WIDTH_PIX, CvType.CV_8UC3);
    private final Mat m_matOutputCb = new Mat(WEBCAM_HEIGHT_PIX, WEBCAM_WIDTH_PIX, CvType.CV_8UC1);
    private final Mat m_matThreshCb = new Mat(BOX_HEIGHT, BOX_WIDTH, CvType.CV_8UC1);

    // Volatile since accessed by OpMode thread w/o synchronization.
    private final AtomicInteger m_frameCount = new AtomicInteger(0);
    private final AtomicInteger m_poleCol_pix = new AtomicInteger(-999);
    private final AtomicInteger m_poleWidth_pix = new AtomicInteger(-999);
    private final AtomicInteger m_minPoleWidth_pix = new AtomicInteger(9999);
    private final AtomicInteger m_maxPoleWidth_pix = new AtomicInteger(9999);

    private volatile boolean m_isFrameBlack = false;

    private final StringBuilder m_csvLogString = new StringBuilder();

    private void logCsvString(String record) {
        m_csvLogString.append(record).append("\n");
    }

    public StringBuilder getLogString() {
        return m_csvLogString;
    }

    // Accessors to allow retrieving the results.
    public int getFrameCount() {
        return m_frameCount.get();
    }

    public boolean isFrameBlack() {
        return m_isFrameBlack;
    }

    public void setMinMaxPoleWidth(int minPoleWidth_pix, int maxPoleWidth_pix) {
        m_minPoleWidth_pix.set(minPoleWidth_pix);
        m_maxPoleWidth_pix.set(maxPoleWidth_pix);
    }

    public int getPoleCol_pix() {
        return m_poleCol_pix.get();
    }
    public int getPoleWidth_pix() {
        return m_poleWidth_pix.get();
    }

    public VisionPipelineFindPole(Telemetry telemetry) {
        m_telemetry = telemetry;
    }

    private VisionPipelineFindPole() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Default constructor not supported.");
    }

    @Override
    public void init(Mat firstFrame) {
        // Nothing to do here.
    }

    @Override
    public Mat processFrame(Mat matInput) {

        // We will search for the pole in three separate rows of the image. ROW_A will be the lower
        // edge of the image, ROW_B will be the middle of the image, and ROW_C will be the upper edge
        // of the image.
        final int BLUR_SIZE = 21;   // Blur window size. Must be an odd number.

        // Simulate input image from a static file.
//        matInput = Imgcodecs.imread("/sdcard/FIRST/data/image.jpg");

        // Check an arbitrary pixel to ensure we aren't getting black images.
        m_isFrameBlack = false;
        double[] pixel = matInput.get(10, 10);
        if (pixel[0] == 0.0 && pixel[1] == 0.0 && pixel[2] == 0.0) {
            m_isFrameBlack = true;
            return matInput;   // Just bail if we don't have an image.
        }

        // Convert the image from RGB to YCrCb color space.
        Imgproc.cvtColor(matInput, m_matYCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Channel 0 is Y (luma component), Channel 1 is Cr (red-difference [red to green] chroma
        // component); Channel 2 is Cb (blue-difference [blue to yellow] chroma component).
        // In the Cb component, the yellow pole will appear darker than everything else we
        // expect to see in the frame.
        Core.extractChannel(m_matYCrCb, m_matOutputCb, 2);

        // Extract just the portion of the image we care about.
        Mat matBox = m_matOutputCb.submat(BOX);

        // Remove noise from the image data in the sample box using a blur filter. This is so stray
        // dark pixel will not be seen as the pole and stray lighter pixels within the pole are not
        // seen as background. We want to produce a solid column of dark pixels.
        // Note: The old Gaussian blur is preserved in comments at the end of this file.
        Imgproc.medianBlur(matBox, matBox, BLUR_SIZE);

        Scalar lowerB = new Scalar(POLE_LIGHT_THRESH);
        Scalar upperB = new Scalar(POLE_DARK_THRESH);
        Core.inRange(matBox, lowerB, upperB, m_matThreshCb);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(m_matThreshCb, contours, hierarchy,
                Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        int rectWidth = 0;
        Rect rectangle = new Rect();
        for (MatOfPoint c : contours) {
            MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
            Rect rect = Imgproc.boundingRect(copy);

            int w = rect.width;
            if (w > rectWidth) {
                rectWidth = w;
                rectangle = rect;
            }
            c.release();
            copy.release();
        }

        if (rectangle != null &&
                rectWidth > m_minPoleWidth_pix.get() &&
                rectWidth < m_maxPoleWidth_pix.get()) {
            m_poleCol_pix.set(rectangle.x);
        } else {
            m_poleCol_pix.set(-1);
        }
        m_poleWidth_pix.set(rectWidth);
        m_frameCount.incrementAndGet(); // Ignore return value from "Get".

        return m_matThreshCb;
    }
}
