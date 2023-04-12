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
    private final Mat m_matCb = new Mat(WEBCAM_HEIGHT_PIX, WEBCAM_WIDTH_PIX, CvType.CV_8UC1);
    private final Mat m_matWithinThresh = new Mat(BOX_HEIGHT, BOX_WIDTH, CvType.CV_8UC1);
    private final Mat m_matOutput = new Mat(WEBCAM_HEIGHT_PIX, WEBCAM_WIDTH_PIX, CvType.CV_8UC3);

    // Volatile since accessed by OpMode thread w/o synchronization.
    private final AtomicInteger m_frameCount = new AtomicInteger(0);
    private final AtomicInteger m_poleCol_pix = new AtomicInteger(-999);
    private final AtomicInteger m_poleWidth_pix = new AtomicInteger(-999);
    private final AtomicInteger m_minPoleWidth_pix = new AtomicInteger(9999);
    private final AtomicInteger m_maxPoleWidth_pix = new AtomicInteger(9999);

    private Rect m_detectRectangle;
    private double m_poleRow_pix;
    private int m_outputSelect = 0;
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

    public void cycleOutputSelect() {
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
    public void onViewportTapped() {
        super.onViewportTapped();
        // TODO: This doesn't seem to be working.
        m_outputSelect++;
        if (m_outputSelect >= 3) {
            m_outputSelect = 0;
        }
    }

    @Override
    public void init(Mat firstFrame) {
        // Nothing to do here.
    }

    @Override
    public Mat processFrame(Mat matInput) {

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
        Core.extractChannel(m_matYCrCb, m_matCb, 2);

        // Extract just the portion of the image we care about.
        Mat matBox = m_matCb.submat(BOX);

        // Remove noise from the image data in the sample box using a blur filter. This is so stray
        // dark pixel will not be seen as the pole and stray lighter pixels within the pole are not
        // seen as background. We want to produce a solid column of dark pixels.
        // Note: The old Gaussian blur is preserved in comments at the end of this file.
        Imgproc.medianBlur(matBox, matBox, BLUR_SIZE);

        Scalar lowerB = new Scalar(POLE_LIGHT_THRESH);
        Scalar upperB = new Scalar(POLE_DARK_THRESH);
        Core.inRange(matBox, lowerB, upperB, m_matWithinThresh);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(m_matWithinThresh, contours, hierarchy,
                Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        int rectWidth = 0;
        m_detectRectangle = null;
        for (MatOfPoint c : contours) {
            MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
            Rect rect = Imgproc.boundingRect(copy);

            int w = rect.width;
            if (w > rectWidth) {
                rectWidth = w;
                m_detectRectangle = rect;
            }
            c.release();
            copy.release();
        }

        if (m_detectRectangle != null &&
                m_detectRectangle.width > m_minPoleWidth_pix.get() &&
                m_detectRectangle.width < m_maxPoleWidth_pix.get()) {
            m_poleWidth_pix.set(m_detectRectangle.width);
            m_poleCol_pix.set(m_detectRectangle.x + (m_detectRectangle.width / 2));
            m_poleRow_pix = m_detectRectangle.y + (m_detectRectangle.height / 2);
        } else {
            m_poleWidth_pix.set(-1);
            m_poleCol_pix.set(-1);
            m_poleRow_pix = -1;
        }
        m_frameCount.incrementAndGet(); // Ignore return value from "Get".

        Mat output;
        if (Vera.isVisionTestMode) {
            switch (2) {  // TODO: use m_outputSelect if onViewportTap is fixed
                case 0:
                    output = drawDetectionAnnotations(matInput, false);
                    break;
                case 1:
                    output = drawDetectionAnnotations(m_matCb, false);
                    break;
                case 2:
                default:
                    m_matCb.copyTo(m_matOutput);
                    m_matWithinThresh.copyTo(m_matOutput.submat(BOX));
                    output = drawDetectionAnnotations(m_matOutput, true);
                    break;
            }
        } else {
            // TODO: Put in a full size output frame with color annotations.
            output = m_matWithinThresh;
        }
        return output;
    }

    private final Scalar WHITE = new Scalar(255, 255, 255);
    private final Scalar RED = new Scalar(255, 0, 0);
    private final Scalar GRAY = new Scalar(128, 0, 0);

    private Mat drawDetectionAnnotations(Mat srcMat, boolean isGrayScale) {
        Scalar color = (isGrayScale ? GRAY : WHITE);
        Imgproc.rectangle(
                srcMat, // Buffer to draw on
                BOX_TOP_LEFT, // First point which defines the rectangle
                BOX_BOTTOM_RIGHT, // Second point which defines the rectangle
                color, // The color the rectangle is drawn in
                1);

        // Draw the nominal high pole position and width.
        Point point1 = new Point(BOX_LEFT +
                Vision.NOMINAL_HIGH_POLE_CENTER_PIX - Vision.NOMINAL_HIGH_POLE_WIDTH_PIX / 2.0,
                15);
        Point point2 = new Point(BOX_LEFT +
                Vision.NOMINAL_HIGH_POLE_CENTER_PIX + Vision.NOMINAL_HIGH_POLE_WIDTH_PIX / 2.0,
                15);
        Imgproc.line(srcMat, point1, point2, color, 2);

        // Draw the nominal mid pole position and width.
        point1 = new Point(BOX_LEFT +
                Vision.NOMINAL_MID_POLE_CENTER_PIX - Vision.NOMINAL_MID_POLE_WIDTH_PIX / 2.0,25);
        point2 = new Point(BOX_LEFT +
                Vision.NOMINAL_MID_POLE_CENTER_PIX + Vision.NOMINAL_MID_POLE_WIDTH_PIX / 2.0,25);
        Imgproc.line(srcMat, point1, point2, color, 2);

        // Draw the pole "detection line"
        color = (isGrayScale ? GRAY : WHITE);
        if (m_poleRow_pix >= 0) {
            point1 = new Point(BOX_LEFT + m_poleCol_pix.get() - (m_poleWidth_pix.get() / 2),
                    m_poleRow_pix);
            point2 = new Point(BOX_LEFT + m_poleCol_pix.get() + (m_poleWidth_pix.get() / 2),
                    m_poleRow_pix);
            Imgproc.line(srcMat, point1, point2, color, 10);
        }


        return srcMat;
    }
}

