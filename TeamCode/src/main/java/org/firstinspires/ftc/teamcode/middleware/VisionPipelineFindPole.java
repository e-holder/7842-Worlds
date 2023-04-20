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
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class VisionPipelineFindPole extends OpenCvPipeline implements CONSTANTS {

    // In the Cb plane of a YCrCb image, the yellow poles should be very dark.
    private final double POLE_DARK_THRESH = 0.0;
    private final double POLE_LIGHT_THRESH = 100.0;

    private final double CONES_BLUE_LOWER_THRESH = 140.0;
    private final double CONES_RED_LOWER_THRESH = 150.0;
    private final double CONES_UPPER_THRESH = 255.0;

    private final int BLUR_SIZE = 21;   // Blur window size. Must be an odd number.

    // Note: The origin (0, 0) of the input image is the top-left of the screen.

    // Define a box for the area where we expect to see the pole we are about to score on. It needs
    // to be at the very top of the image area, but not too wide (we should have pretty accurate
    // robot navigation). The height of the box probably doesn't need to be really tall. If there
    // are 3 or 4 cones already on the pole we'd like to see over the top of those, but we do need
    // enough delta between the top and bottom of the box to detect how much the pole is leaning
    // left or right.
    private final int BOX_CENTER_ADJ = 0;
    public static final int BOX_WIDTH = WEBCAM_WIDTH_PIX - 2;
    private final int BOX_HEIGHT = 141;
    private final int BOX_TOP = 142;
    private final int BOX_LEFT = (WEBCAM_WIDTH_PIX / 2) - (BOX_WIDTH / 2) + BOX_CENTER_ADJ;
    private final Point BOX_TOP_LEFT = new Point(BOX_LEFT, BOX_TOP);
    private final Point BOX_BOTTOM_RIGHT =
            new Point(BOX_LEFT + BOX_WIDTH, BOX_TOP + BOX_HEIGHT);
    private final Rect BOX = new Rect(BOX_TOP_LEFT, BOX_BOTTOM_RIGHT);

    public static final int CONE_BOX_WIDTH = WEBCAM_WIDTH_PIX;
    private final int CONE_BOX_HEIGHT = WEBCAM_HEIGHT_PIX / 3;
    private final int CONE_BOX_TOP = WEBCAM_HEIGHT_PIX - CONE_BOX_HEIGHT - 1;
    private final int CONE_BOX_LEFT = 0;
    private final Point CONE_BOX_TOP_LEFT = new Point(CONE_BOX_LEFT, CONE_BOX_TOP);
    private final Point CONE_BOX_BOTTOM_RIGHT =
            new Point(CONE_BOX_WIDTH, CONE_BOX_TOP + CONE_BOX_HEIGHT);
    private final Rect CONE_BOX = new Rect(CONE_BOX_TOP_LEFT, CONE_BOX_BOTTOM_RIGHT);

    private final Mat m_matYCrCb = new Mat(WEBCAM_HEIGHT_PIX, WEBCAM_WIDTH_PIX, CvType.CV_8UC3);
    private final Mat m_matPlaneOfInterest = new Mat(WEBCAM_HEIGHT_PIX, WEBCAM_WIDTH_PIX, CvType.CV_8UC1);
    private final Mat m_matWithinThreshP = new Mat(BOX_HEIGHT, BOX_WIDTH, CvType.CV_8UC1);
    private final Mat m_matWithinThreshC = new Mat(CONE_BOX_HEIGHT, CONE_BOX_WIDTH, CvType.CV_8UC1);
    private final Mat m_matOutput = new Mat(WEBCAM_HEIGHT_PIX, WEBCAM_WIDTH_PIX, CvType.CV_8UC3);

    // Volatile since accessed by OpMode thread w/o synchronization.
    private final AtomicInteger m_frameCount = new AtomicInteger(0);
    private final AtomicInteger m_poleCol_pix = new AtomicInteger(-999);
    private final AtomicInteger m_width_pix = new AtomicInteger(-999);
    private final AtomicInteger m_minPoleWidth_pix = new AtomicInteger(9999);
    private final AtomicInteger m_maxPoleWidth_pix = new AtomicInteger(9999);
    private final AtomicBoolean m_isFindScoredConesMode = new AtomicBoolean(false);
    private final AtomicBoolean m_isFrameBlack = new AtomicBoolean(true);
    private final AtomicInteger m_alliance = new AtomicInteger(Alliance.BLUE.ordinal());

    private final Telemetry m_telemetry;
    private final StringBuilder m_csvLogString = new StringBuilder();

    public VisionPipelineFindPole(Telemetry telemetry) {
        m_telemetry = telemetry;
    }

    private VisionPipelineFindPole() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("Default constructor not supported.");
    }

    private Rect m_detectRectangle;
    private int m_poleRow_pix;
    private int m_outputSelect = 0;

    // Accessors to allow retrieving the results.
    public int getFrameCount() {
        return m_frameCount.get();
    }
    public boolean isFrameBlack() { return m_isFrameBlack.get(); }
    public int getPoleCol_pix() {
        return m_poleCol_pix.get();
    }
    public int getPoleWidth_pix() {
        return m_width_pix.get();
    }
    public StringBuilder getLogString() {
        return m_csvLogString;
    }

    private void logCsvString(String record) {
        m_csvLogString.append(record).append("\n");
    }

    public void setFindPoleMode(FindPoleMode findPoleMode, Alliance alliance) {
        switch (findPoleMode) {
            case MID_SCORED_CONES:
            case HIGH_SCORED_CONES:
                m_isFindScoredConesMode.set(true);
                break;
            default:
                m_isFindScoredConesMode.set(false);
                break;
        }
        m_alliance.set(alliance.ordinal());
    }

    public void setMinMaxWidth(int minPoleWidth_pix, int maxPoleWidth_pix) {
        m_minPoleWidth_pix.set(minPoleWidth_pix);
        m_maxPoleWidth_pix.set(maxPoleWidth_pix);
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
        if (m_isFindScoredConesMode.get()) {
            return processFrameForScoredCones(matInput);
        } else {
            return processFrameForPole(matInput);
        }
    }

    private Mat processFrameForPole(Mat matInput) {

        // Simulate input image from a static file.
//        matInput = Imgcodecs.imread("/sdcard/FIRST/data/image.jpg");

        // Check an arbitrary pixel to ensure we aren't getting black images.
        m_isFrameBlack.set(false);
        double[] pixel = matInput.get(10, 10);
        if (pixel[0] == 0.0 && pixel[1] == 0.0 && pixel[2] == 0.0) {
            m_isFrameBlack.set(true);
            return matInput;   // Just bail if we don't have an image.
        }

        // Convert the image from RGB to YCrCb color space.
        Imgproc.cvtColor(matInput, m_matYCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Channel 0 is Y (luma component), Channel 1 is Cr (red-difference [red to green] chroma
        // component); Channel 2 is Cb (blue-difference [blue to yellow] chroma component).
        // In the Cb component, the yellow pole will appear darker than everything else we
        // expect to see in the frame.
        Core.extractChannel(m_matYCrCb, m_matPlaneOfInterest, 2);

        // Extract just the portion of the image we care about.
        Mat matBox = m_matPlaneOfInterest.submat(BOX);

        // Remove noise from the image data in the sample box using a blur filter. This is so stray
        // dark pixel will not be seen as the pole and stray lighter pixels within the pole are not
        // seen as background. We want to produce a solid column of dark pixels.
        // Note: The old Gaussian blur is preserved in comments at the end of this file.
        Imgproc.medianBlur(matBox, matBox, BLUR_SIZE);

        Scalar lowerB = new Scalar(POLE_DARK_THRESH);
        Scalar upperB = new Scalar(POLE_LIGHT_THRESH);
        Core.inRange(matBox, lowerB, upperB, m_matWithinThreshP);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(m_matWithinThreshP, contours, hierarchy,
                Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        int rectWidth = 0;
        int rectCenterX = 0;
        int rectCenterY = 0;
        m_detectRectangle = null;
        for (MatOfPoint c : contours) {
            MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
            Rect rect = Imgproc.boundingRect(copy);

            // TODO: High not implemented explicitly (should work okay, though)
            int w = rect.width;
            int x = (rect.x + w / 2);
            int deltaX_pix = x - Vision.NOMINAL_MID_POLE_CENTER_PIX;
            if ((w > rectWidth) || (Math.abs(deltaX_pix) < 250)) {
                rectWidth = w;
                rectCenterX = x;
                rectCenterY = rect.y + (rect.height / 2);
                m_detectRectangle = rect;
            }
            c.release();
            copy.release();
        }

        if (m_detectRectangle != null &&
                m_detectRectangle.width > m_minPoleWidth_pix.get() &&
                m_detectRectangle.width < m_maxPoleWidth_pix.get()) {
            m_width_pix.set(rectWidth);
            m_poleCol_pix.set(rectCenterX);
            m_poleRow_pix = rectCenterY;
        } else {
            m_width_pix.set(-1);
            m_poleCol_pix.set(-1);
            m_poleRow_pix = -1;
        }
        m_frameCount.incrementAndGet(); // Ignore return value from "Get".

        Mat output;
        if (Vera.isVisionTestMode) {
            switch (2) {  // TODO: use m_outputSelect if onViewportTap is fixed
                case 0:
                    output = drawPoleDetectionAnnotations(matInput, false);
                    break;
                case 1:
                    output = drawPoleDetectionAnnotations(m_matPlaneOfInterest, false);
                    break;
                case 2:
                default:
                    m_matPlaneOfInterest.copyTo(m_matOutput);
                    m_matWithinThreshP.copyTo(m_matOutput.submat(BOX));
                    output = drawPoleDetectionAnnotations(m_matOutput, true);
                    break;
            }
        } else {
            // TODO: Put in a full size output frame with color annotations.
            output = m_matWithinThreshP;
        }
        return output;
    }

    private Mat processFrameForScoredCones(Mat matInput) {

        final int BLUR_SIZE = 21;   // Blur window size. Must be an odd number.

        // Check an arbitrary pixel to ensure we aren't getting black images.
        m_isFrameBlack.set(false);
        double[] pixel = matInput.get(10, 10);
        if (pixel[0] == 0.0 && pixel[1] == 0.0 && pixel[2] == 0.0) {
            m_isFrameBlack.set(true);
            return matInput;   // Just bail if we don't have an image.
        }

        // Convert the image from RGB to YCrCb color space.
        Imgproc.cvtColor(matInput, m_matYCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Channel 0 is Y (luma component), Channel 1 is Cr (red-difference [red to green] chroma
        // component); Channel 2 is Cb (blue-difference [blue to yellow] chroma component).
        // In the Cb component, the blue cones will appear whiter than everything else we
        // expect to see in the frame, and likewise for Cr and red cones.
        if (m_alliance.get() == Alliance.BLUE.ordinal()) {
            Core.extractChannel(m_matYCrCb, m_matPlaneOfInterest, 2);
        } else {
            Core.extractChannel(m_matYCrCb, m_matPlaneOfInterest, 1);
        }

        // Extract just the portion of the image we care about.
        Mat matConeBox = m_matPlaneOfInterest.submat(CONE_BOX);

        // Remove noise from the image data in the sample box using a blur filter.
        Imgproc.medianBlur(matConeBox, matConeBox, BLUR_SIZE);

        Scalar lowerB = new Scalar(m_alliance.get() == Alliance.BLUE.ordinal() ?
                CONES_BLUE_LOWER_THRESH : CONES_RED_LOWER_THRESH);
        Scalar upperB = new Scalar(CONES_UPPER_THRESH);
        Core.inRange(matConeBox, lowerB, upperB, m_matWithinThreshC);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(m_matWithinThreshC, contours, hierarchy,
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
            m_width_pix.set(m_detectRectangle.width);
            m_poleCol_pix.set(m_detectRectangle.x + (m_detectRectangle.width / 2));
            m_poleRow_pix = CONE_BOX_TOP + m_detectRectangle.y + (m_detectRectangle.height / 2);
        } else {
            // If we get no rectangle detection, check to see if we are so close the cones are
            // filling the FOV and we will just assume we are centered.
            if (Core.mean(m_matWithinThreshC).val[0] > 200) {
                m_width_pix.set(CONE_BOX_WIDTH);
                m_poleCol_pix.set(Vision.NOMINAL_MID_CONES_CENTER_PIX);  // TODO: HIGH not implemented
                m_poleRow_pix = CONE_BOX_TOP + (CONE_BOX_HEIGHT / 2);
            } else {
                m_width_pix.set(-1);
                m_poleCol_pix.set(-1);
                m_poleRow_pix = -1;
            }
        }
        m_frameCount.incrementAndGet(); // Ignore return value from "Get".

        Mat output;
        if (Vera.isVisionTestMode) {
            switch (2) {  // TODO: use m_outputSelect if onViewportTap is fixed
                case 0:
                    output = drawConeDetectionAnnotations(matInput, false);
                    break;
                case 1:
                    output = drawConeDetectionAnnotations(m_matPlaneOfInterest, false);
                    break;
                case 2:
                default:
                    m_matPlaneOfInterest.copyTo(m_matOutput);
                    m_matWithinThreshC.copyTo(m_matOutput.submat(CONE_BOX));
                    output = drawConeDetectionAnnotations(m_matOutput, true);
                    break;
            }
        } else {
            // TODO: Put in a full size output frame with color annotations.
            output = m_matWithinThreshC;
        }

        return output;
    }

    //======== ANNOTATION DRAWING METHODS ==================================

    private final Scalar WHITE = new Scalar(255, 255, 255);
    private final Scalar GREEN = new Scalar(0, 255, 0);
    private final Scalar GRAY = new Scalar(100, 100, 100);

    private Mat drawPoleDetectionAnnotations(Mat srcMat, boolean isGrayScale) {
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
        color = (isGrayScale ? GRAY : GREEN);
        if (m_poleRow_pix >= 0) {
            point1 = new Point((int)(BOX_LEFT + m_poleCol_pix.get() - (m_width_pix.get() / 2)),
                    m_poleRow_pix);
            point2 = new Point((int)(BOX_LEFT + m_poleCol_pix.get() + (m_width_pix.get() / 2)),
                    m_poleRow_pix);
            Imgproc.line(srcMat, point1, point2, color, 10);
        }
        return srcMat;
    }

    private Mat drawConeDetectionAnnotations(Mat srcMat, boolean isGrayScale) {
        Scalar color = (isGrayScale ? GRAY : WHITE);
        Imgproc.rectangle(
                srcMat, // Buffer to draw on
                CONE_BOX_TOP_LEFT,     // First point which defines the rectangle
                CONE_BOX_BOTTOM_RIGHT, // Second point which defines the rectangle
                color, // The color the rectangle is drawn in
                1);

        // Draw the nominal high cones position and width.
        Point point1 = new Point(CONE_BOX_LEFT +
                Vision.NOMINAL_HIGH_CONES_CENTER_PIX - Vision.NOMINAL_HIGH_CONES_WIDTH_PIX / 2.0,
                CONE_BOX_TOP + 5);
        Point point2 = new Point(BOX_LEFT +
                Vision.NOMINAL_HIGH_CONES_CENTER_PIX + Vision.NOMINAL_HIGH_CONES_CENTER_PIX / 2.0,
                CONE_BOX_TOP + 5);
        Imgproc.line(srcMat, point1, point2, color, 2);

        // Draw the nominal mid cones position and width.
        point1 = new Point(CONE_BOX_LEFT +
                Vision.NOMINAL_MID_CONES_CENTER_PIX - Vision.NOMINAL_MID_CONES_WIDTH_PIX / 2.0,
                CONE_BOX_TOP + 15);
        point2 = new Point(CONE_BOX_LEFT +
                Vision.NOMINAL_MID_CONES_CENTER_PIX + Vision.NOMINAL_MID_CONES_WIDTH_PIX / 2.0,
                CONE_BOX_TOP + 15);
        Imgproc.line(srcMat, point1, point2, color, 2);

        // Draw the cones "detection line"
        color = (isGrayScale ? GRAY : GREEN);
        if (m_poleRow_pix >= 0) {
            point1 = new Point(
                    (int)(CONE_BOX_LEFT + m_poleCol_pix.get() - (m_width_pix.get() / 2)),
                    m_poleRow_pix);
            point2 = new Point(
                    (int)(CONE_BOX_LEFT + m_poleCol_pix.get() + (m_width_pix.get() / 2)),
                    m_poleRow_pix);
            Imgproc.line(srcMat, point1, point2, color, 10);
        }
        return srcMat;
    }
}

