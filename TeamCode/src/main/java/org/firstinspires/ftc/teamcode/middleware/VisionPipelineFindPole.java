package org.firstinspires.ftc.teamcode.middleware;

import static org.firstinspires.ftc.teamcode.hardware.HwVision.WEBCAM_HEIGHT_PIX;
import static org.firstinspires.ftc.teamcode.hardware.HwVision.WEBCAM_WIDTH_PIX;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.atomic.AtomicInteger;

public class VisionPipelineFindPole extends OpenCvPipeline implements CONSTANTS {

    // In the Cb plane of a YCrCb image, the yellow poles should be very dark. Define a percentage
    // of the range between the minimum pixel value and the average pixel value.
    public final double POLE_LIGHT_THRESH_PERCENT = 0.60;
    // Poles further away are darker. May need a low threshold to try and eliminate those from
    // consideration.
    public final double POLE_DARK_THRESH_PERCENT = 0.00;


    // Note: The origin (0, 0) of the input image is the top-left of the screen.

    // Define a box for the area where we expect to see the pole we are about to score on. It needs
    // to be at the very top of the image area, but not too wide (we should have pretty accurate
    // robot navigation). The height of the box probably doesn't need to be really tall. If there
    // are 3 or 4 cones already on the pole we'd like to see over the top of those, but we do need
    // enough delta between the top and bottom of the box to detect how much the pole is leaning
    // left or right.
    private final int BOX_CENTER_ADJ = 0;
    public static final int BOX_WIDTH = 500;
    private final int BOX_HEIGHT = 150;     // This will only see over the top of 2 stacked cones!
    private final int BOX_TOP = 0;
    private final int BOX_LEFT = (WEBCAM_WIDTH_PIX / 2) - (BOX_WIDTH / 2) + BOX_CENTER_ADJ;
    private final Point BOX_TOP_LEFT = new Point(BOX_LEFT, BOX_TOP);
    private final Point BOX_BOTTOM_RIGHT =
            new Point(BOX_LEFT + BOX_WIDTH, BOX_TOP + BOX_HEIGHT);
    private final Rect BOX = new Rect(BOX_TOP_LEFT, BOX_BOTTOM_RIGHT);

    // We will search a row at the upper edge and the lower edge of the box (these two rows).
    // Upper row needs to be > (BLUR_SIZE / 2) to sample from "fully blurred" image data.
    private final int POLE_UPPER_ROW = 3;
    // Likewise, lower row needs to be < (BOX_HEIGHT - 1 - BLUR_SIZE / 2).
    private final int POLE_LOWER_ROW = BOX_HEIGHT - 5;

    private final Scalar WHITE = new Scalar(255);

    // Gaussian blur parameters. Size must be an odd number.
    private final int BLUR_SIZE = 5;
    private final double SIGMA = 0.0;
    private final Size GAUSSIAN_BLUR_SIZE = new Size(BLUR_SIZE, BLUR_SIZE);

    private Telemetry m_telemetry;


    private Mat m_matYCrCb = new Mat(WEBCAM_HEIGHT_PIX, WEBCAM_WIDTH_PIX, CvType.CV_8UC3);
    private Mat m_matOutputCb = new Mat(WEBCAM_HEIGHT_PIX, WEBCAM_WIDTH_PIX, CvType.CV_8UC1);
    private Mat m_matBox = new Mat(BOX_HEIGHT, BOX_WIDTH, CvType.CV_8UC1);

    // Volatile since accessed by OpMode thread w/o synchronization.
    private AtomicInteger m_frameCount = new AtomicInteger(0);
    private AtomicInteger m_poleUpperCol_pix = new AtomicInteger(-1);
    private AtomicInteger m_poleLowerCol_pix = new AtomicInteger(-1);
    private AtomicInteger m_poleUpperWidth_pix = new AtomicInteger(-1);
    private AtomicInteger m_poleLowerWidth_pix = new AtomicInteger(-1);

    // Any sequence of horizontal pixels "at least this wide" that are dark (falling below the
    // threshold percentage) will be considered the pole.
    private AtomicInteger m_minPoleWidth_pix = new AtomicInteger(9999);

    private volatile boolean m_isFrameNull = false;

    private volatile short[] m_upperRow = new short[BOX_WIDTH];
    private volatile short[] m_lowerRow = new short[BOX_WIDTH];
    private volatile short m_poleLightThresh;
    private volatile short m_poleDarkThresh;

    private StringBuilder m_csvLogString = new StringBuilder();
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

    public boolean isFrameNull() {
        return m_isFrameNull;
    }

    public void setMinPoleWidth(int minPoleWidth_pix) {
        m_minPoleWidth_pix.set(minPoleWidth_pix);
    }

    public int getPoleUpperCol_pix() {
        return m_poleUpperCol_pix.get();
    }

    public int getPoleLowerCol_pix() {
        return m_poleLowerCol_pix.get();
    }

    public int getUpperPoleWidth_pix() {
        return m_poleUpperWidth_pix.get();
    }

    public int getLowerPoleWidth_pix() {
        return m_poleLowerWidth_pix.get();
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
        m_frameCount.incrementAndGet();  // Ignore return value.
        m_poleUpperCol_pix.set(-1);
        m_poleLowerCol_pix.set(-1);
        m_poleUpperWidth_pix.set(-1);
        m_poleLowerWidth_pix.set(-1);

        // Simulate input image from a static file.
        //matInput = Imgcodecs.imread("/sdcard/FIRST/data/image.jpg");

        // Check an arbitrary pixel to ensure we aren't getting black images.
        double[] pixel = matInput.get(10, 10);
        if (pixel[0] == 0.0 && pixel[1] == 0.0 && pixel[2] == 0.0) {
            m_isFrameNull = true;
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
        m_matBox = m_matOutputCb.submat(BOX);

        // Remove noise from the image data in the sample box using a Gaussian blur filter. This
        // is so one stray dark pixel will not be seen as the pole. We are looking for a column
        // of dark pixels.
        Imgproc.GaussianBlur(m_matBox, m_matBox, GAUSSIAN_BLUR_SIZE, SIGMA, SIGMA);

        // First get the row data and find average and minimum of the pixel values in both rows.
        short pixVal;
        short minPix = 999;
        int sumPix = 0;
        for (int col = 0; col < BOX_WIDTH; col++) {
            pixel = m_matBox.get(POLE_UPPER_ROW, col);
            pixVal = (short) pixel[0];
            m_upperRow[col] = pixVal;
            minPix = (short) Math.min(minPix, pixVal);
            sumPix += pixVal;
            pixel = m_matBox.get(POLE_LOWER_ROW, col);
            pixVal = (short) pixel[0];
            m_lowerRow[col] = pixVal;
            minPix = (short) Math.min(minPix, pixVal);
            sumPix += pixVal;
        }
        int pixAvg = sumPix / (BOX_WIDTH * 2);
        m_poleLightThresh = (short)(((pixAvg - minPix) * POLE_LIGHT_THRESH_PERCENT) + minPix);
        m_poleDarkThresh = (short)(((pixAvg - minPix) * POLE_DARK_THRESH_PERCENT) + minPix);
//        logCsvString("dkThresh, " + m_poleDarkThresh +
//                ", ltThresh, " + m_poleLightThresh +
//                ", minPoleW, " + m_minPoleWidth_pix.get());

        // Now identify where the pole is in the upper and lower rows.
        int upperStart = 0;
        int upperWidth = 0;
        int lowerStart = 0;
        int lowerWidth = 0;
        for (int col = 0; col < BOX_WIDTH; col++) {
            // See where the upper part of the pole is.
            if (m_upperRow[col] <= m_poleLightThresh && m_upperRow[col] >= m_poleDarkThresh) {
                upperWidth++;
                if (upperWidth == 1) {
                    upperStart = col;
                }
            } else if (upperWidth >= m_minPoleWidth_pix.get()) {
                m_poleUpperCol_pix.set(upperStart + (upperWidth / 2));
                m_poleUpperWidth_pix.set(upperWidth);
            } else {
                upperWidth = 0;
            }

            // See where the lower part of the pole is.
            if (m_lowerRow[col] <= m_poleLightThresh && m_lowerRow[col] >= m_poleDarkThresh) {
                lowerWidth++;
                if (lowerWidth == 1) {
                    lowerStart = col;
                }
            } else if (lowerWidth >= m_minPoleWidth_pix.get()) {
                m_poleLowerCol_pix.set(lowerStart + (lowerWidth / 2));
                m_poleLowerWidth_pix.set(lowerWidth);
            } else {
                lowerWidth = 0;
            }
        }

//        if (m_frameCount.get() % 100 == 0) {
//            for (int i = 0; i < BOX_WIDTH; i++) {
//                m_csvLogString.append(", " + m_lowerRow[i]);
//            }
//            m_csvLogString.append("\n");
//        }
//        logCsvString("FINAL, u, " + m_poleUpperCol_pix.get() +
//                ", l, " + m_poleLowerCol_pix.get() +
//                ", uw, " + m_poleWidth_pix.get() +
//                ", lw, " + lowerWidth);

        if (Vera.isVisionTestMode) {
            // TODO: Disabled this telemetry (which shows up nicely in Init/Camera Stream mode so
            //  I could see the telemetry produced when running the test opmodes. When running the
            //  opmode, you can see the deltas from expected instead of absolute pixel columns.
//            m_telemetry.addData("POLE", m_poleDarkThresh + "/" + m_poleLightThresh +
//                    " upper " + m_poleUpperCol_pix.get() +
//                    " lower " + m_poleLowerCol_pix.get() +
//                    " width " + m_poleWidth_pix.get());
//            m_telemetry.update();

            // To display the output image: Use phone to select opmode, press Init, wait until
            // initialization completes, then  ...->Camera Stream
            //=========================================================================================

            Imgproc.rectangle(
                    m_matOutputCb, // Buffer to draw on
                    BOX_TOP_LEFT, // First point which defines the rectangle
                    BOX_BOTTOM_RIGHT, // Second point which defines the rectangle
                    WHITE, // The color the rectangle is drawn in
                    1);

            // Draw the nominal high pole position and width.
            Point point1 = new Point(BOX_LEFT +
                    Vision.NOMINAL_HIGH_POLE_CENTER_PIX - Vision.NOMINAL_HIGH_POLE_WIDTH_PIX / 2.0,
                    POLE_UPPER_ROW + 5);
            Point point2 = new Point(BOX_LEFT +
                    Vision.NOMINAL_HIGH_POLE_CENTER_PIX + Vision.NOMINAL_HIGH_POLE_WIDTH_PIX / 2.0,
                    POLE_UPPER_ROW + 5);
            Imgproc.line(m_matOutputCb, point1, point2, WHITE, 2);

            // Draw the nominal medium pole position and width.
            point1 = new Point(BOX_LEFT +
                    Vision.NOMINAL_MED_POLE_CENTER_PIX - Vision.NOMINAL_MED_POLE_WIDTH_PIX / 2.0,
                    POLE_UPPER_ROW + 15);
            point2 = new Point(BOX_LEFT +
                    Vision.NOMINAL_MED_POLE_CENTER_PIX + Vision.NOMINAL_MED_POLE_WIDTH_PIX / 2.0,
                    POLE_UPPER_ROW + 15);
            Imgproc.line(m_matOutputCb, point1, point2, WHITE, 2);

            // Draw the nominal low pole position and width.
            point1 = new Point(BOX_LEFT +
                    Vision.NOMINAL_LOW_POLE_CENTER_PIX - Vision.NOMINAL_LOW_POLE_WIDTH_PIX / 2.0,
                    POLE_UPPER_ROW + 25);
            point2 = new Point(BOX_LEFT +
                    Vision.NOMINAL_LOW_POLE_CENTER_PIX + Vision.NOMINAL_LOW_POLE_WIDTH_PIX / 2.0,
                    POLE_UPPER_ROW + 25);
            Imgproc.line(m_matOutputCb, point1, point2, WHITE, 2);

            if (m_poleUpperWidth_pix.get() > 0) {
                // Draw the pole center "detection line"
                point1 = new Point(BOX_LEFT + m_poleUpperCol_pix.get(), POLE_UPPER_ROW);
                point2 = new Point(BOX_LEFT + m_poleLowerCol_pix.get(), POLE_LOWER_ROW);
                Imgproc.line(m_matOutputCb, point1, point2, WHITE, 3);

            }
        }

        return m_matOutputCb;
    }
}
