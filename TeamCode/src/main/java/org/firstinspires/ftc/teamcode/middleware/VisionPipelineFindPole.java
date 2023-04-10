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
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.atomic.AtomicInteger;

public class VisionPipelineFindPole extends OpenCvPipeline implements CONSTANTS {

    // In the Cb plane of a YCrCb image, the yellow poles should be very dark. Define a percentage
    // of the range between the minimum pixel value and the average pixel value.
    public final double POLE_LIGHT_THRESH_PERCENT = 0.75;   // Original: 0.60
    // Poles further away can be darker. May need a low threshold to try and eliminate those from
    // consideration.
    public final double POLE_DARK_THRESH_PERCENT = 0.0;     // Original: 0.0


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

    private final Scalar WHITE = new Scalar(255);

    private final Telemetry m_telemetry;

    private final Mat m_matYCrCb = new Mat(WEBCAM_HEIGHT_PIX, WEBCAM_WIDTH_PIX, CvType.CV_8UC3);
    private final Mat m_matOutputCb = new Mat(WEBCAM_HEIGHT_PIX, WEBCAM_WIDTH_PIX, CvType.CV_8UC1);

    // Volatile since accessed by OpMode thread w/o synchronization.
    private final AtomicInteger m_frameCount = new AtomicInteger(0);
    private final AtomicInteger m_poleRowACol_pix = new AtomicInteger(-999);
    private final AtomicInteger m_poleRowBCol_pix = new AtomicInteger(-999);
    private final AtomicInteger m_poleRowCCol_pix = new AtomicInteger(-999);
    private final AtomicInteger m_poleRowAWidth_pix = new AtomicInteger(-999);
    private final AtomicInteger m_poleRowBWidth_pix = new AtomicInteger(-999);
    private final AtomicInteger m_poleRowCWidth_pix = new AtomicInteger(-999);

    // Any sequence of horizontal pixels "at least this wide" that are dark (falling below the
    // threshold percentage) will be considered the pole.
    private final AtomicInteger m_minPoleWidth_pix = new AtomicInteger(9999);

    private volatile boolean m_isFrameBlack = false;

    private volatile short[] m_rowC = new short[BOX_WIDTH];
    private volatile short[] m_rowB = new short[BOX_WIDTH];
    private volatile short[] m_rowA = new short[BOX_WIDTH];
    private volatile short m_poleLightThresh;
    private volatile short m_poleDarkThresh;

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

    public void setMinPoleWidth(int minPoleWidth_pix) {
        m_minPoleWidth_pix.set(minPoleWidth_pix);
    }

    public int getPoleRowACol_pix() {
        return m_poleRowACol_pix.get();
    }
    public int getPoleRowBCol_pix() {
        return m_poleRowBCol_pix.get();
    }
    public int getPoleRowCCol_pix() {
        return m_poleRowCCol_pix.get();
    }

    public int getRowAPoleWidth_pix() {
        return m_poleRowAWidth_pix.get();
    }
    public int getRowBPoleWidth_pix() {
        return m_poleRowBWidth_pix.get();
    }
    public int getRowCPoleWidth_pix() {
        return m_poleRowCWidth_pix.get();
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

    private int poleRowACol_pix;
    private int poleRowBCol_pix;
    private int poleRowCCol_pix;
    private int poleRowAWidth_pix;
    private int poleRowBWidth_pix;
    private int poleRowCWidth_pix;

    @Override
    public Mat processFrame(Mat matInput) {

        // We will search for the pole in three separate rows of the image. ROW_A will be the lower
        // edge of the image, ROW_B will be the middle of the image, and ROW_C will be the upper edge
        // of the image.
        final int BLUR_SIZE = 21;   // Blur window size. Must be an odd number.
        // Use fully blurred rows (not close to an edge of the sample window).
        final int ROW_A = BOX_HEIGHT - 1 - BLUR_SIZE / 2;
        final int ROW_B = BOX_HEIGHT / 2;
        final int ROW_C = BLUR_SIZE / 2;

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

        // First get the row data and find average and minimum of the pixel values in both rows.
        short pixVal;
        short minPix = 999;
        int sumPix = 0;
        for (int col = 0; col < BOX_WIDTH; col++) {
            pixel = matBox.get(ROW_A, col);
            pixVal = (short) pixel[0];
            m_rowA[col] = pixVal;
            minPix = (short) Math.min(minPix, pixVal);
            sumPix += pixVal;

            pixel = matBox.get(ROW_B, col);
            pixVal = (short) pixel[0];
            m_rowB[col] = pixVal;
            minPix = (short) Math.min(minPix, pixVal);
            sumPix += pixVal;

            pixel = matBox.get(ROW_C, col);
            pixVal = (short) pixel[0];
            m_rowC[col] = pixVal;
            minPix = (short) Math.min(minPix, pixVal);
            sumPix += pixVal;
        }
        int pixAvg = sumPix / (BOX_WIDTH * 3);
        m_poleLightThresh = (short) (((pixAvg - minPix) * POLE_LIGHT_THRESH_PERCENT) + minPix);
        m_poleDarkThresh = (short) (((pixAvg - minPix) * POLE_DARK_THRESH_PERCENT) + minPix);
//        logCsvString("dkThresh, " + m_poleDarkThresh +
//                ", ltThresh, " + m_poleLightThresh +
//                ", min, " + minPix +
//                ", avg, " + pixAvg +
//                ", minPoleW, " + m_minPoleWidth_pix.get());

        // Now identify where the pole is in rows A, B, and C.
        poleRowACol_pix = -1;
        poleRowBCol_pix = -1;
        poleRowCCol_pix = -1;
        poleRowAWidth_pix = -1;
        poleRowBWidth_pix = -1;
        poleRowCWidth_pix = -1;

        int rowAStart = 0;
        int rowAWidth = 0;
        int rowBStart = 0;
        int rowBWidth = 0;
        int rowCStart = 0;
        int rowCWidth = 0;
        for (int col = 0; col < BOX_WIDTH; col++) {
            // See where the pole is in Row A (lower pole).
            if (m_rowA[col] <= m_poleLightThresh && m_rowA[col] >= m_poleDarkThresh) {
                rowAWidth++;
                if (rowAWidth == 1) {
                    rowAStart = col;
                }
            } else if (rowAWidth >= m_minPoleWidth_pix.get()) {
                poleRowACol_pix = rowAStart + (rowAWidth / 2);
                poleRowAWidth_pix = rowAWidth;
            } else {
                rowAWidth = 0;
            }

            // See where the pole is in Row B (middle pole).
            if (m_rowB[col] <= m_poleLightThresh && m_rowB[col] >= m_poleDarkThresh) {
                rowBWidth++;
                if (rowBWidth == 1) {
                    rowBStart = col;
                }
            } else if (rowBWidth >= m_minPoleWidth_pix.get()) {
                poleRowBCol_pix = rowBStart + (rowBWidth / 2);
                poleRowBWidth_pix = rowBWidth;
            } else {
                rowBWidth = 0;
            }

            // See where the pole is in Row C (upper pole).
            if (m_rowC[col] <= m_poleLightThresh && m_rowC[col] >= m_poleDarkThresh) {
                rowCWidth++;
                if (rowCWidth == 1) {
                    rowCStart = col;
                }
            } else if (rowCWidth >= m_minPoleWidth_pix.get()) {
                poleRowCCol_pix = rowCStart + (rowCWidth / 2);
                poleRowCWidth_pix = rowCWidth;
            } else {
                rowCWidth = 0;
            }
        }

//        if (m_frameCount.get() % 100 == 0) {
//            for (int i = 0; i < BOX_WIDTH; i++) {
//                m_csvLogString.append(", " + m_rowB[i]);
//            }
//            m_csvLogString.append("\n");
//        }
//        logCsvString("FINAL, " + m_poleRowBCol_pix.get() +
//                ", w, " + rowBWidth);

        if (Vera.isVisionTestMode) {
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
                    ROW_C + 5);
            Point point2 = new Point(BOX_LEFT +
                    Vision.NOMINAL_HIGH_POLE_CENTER_PIX + Vision.NOMINAL_HIGH_POLE_WIDTH_PIX / 2.0,
                    ROW_C + 5);
            Imgproc.line(m_matOutputCb, point1, point2, WHITE, 2);

            // Draw the nominal mid pole position and width.
            point1 = new Point(BOX_LEFT +
                    Vision.NOMINAL_MID_POLE_CENTER_PIX - Vision.NOMINAL_MID_POLE_WIDTH_PIX / 2.0,
                    ROW_C + 15);
            point2 = new Point(BOX_LEFT +
                    Vision.NOMINAL_MID_POLE_CENTER_PIX + Vision.NOMINAL_MID_POLE_WIDTH_PIX / 2.0,
                    ROW_C + 15);
            Imgproc.line(m_matOutputCb, point1, point2, WHITE, 2);

            if (poleRowAWidth_pix > 0) {
                // Draw the pole center "detection spot"
                point1 = new Point(BOX_LEFT + poleRowACol_pix, ROW_A - 5);
                point2 = new Point(BOX_LEFT + poleRowACol_pix, ROW_A + 5);
                Imgproc.line(m_matOutputCb, point1, point2, WHITE, 5);
            }

            if (poleRowBWidth_pix > 0) {
                // Draw the pole center "detection spot"
                point1 = new Point(BOX_LEFT + poleRowBCol_pix, ROW_B - 5);
                point2 = new Point(BOX_LEFT + poleRowBCol_pix, ROW_B + 5);
                Imgproc.line(m_matOutputCb, point1, point2, WHITE, 5);
            }

            if (poleRowCWidth_pix > 0) {
                // Draw the pole center "detection spot"
                point1 = new Point(BOX_LEFT + poleRowCCol_pix, ROW_C - 5);
                point2 = new Point(BOX_LEFT + poleRowCCol_pix, ROW_C + 5);
                Imgproc.line(m_matOutputCb, point1, point2, WHITE, 5);
            }
        }

        m_poleRowACol_pix.set(poleRowACol_pix);
        m_poleRowBCol_pix.set(poleRowBCol_pix);
        m_poleRowCCol_pix.set(poleRowCCol_pix);
        m_poleRowAWidth_pix.set(poleRowAWidth_pix);
        m_poleRowBWidth_pix.set(poleRowBWidth_pix);
        m_poleRowCWidth_pix.set(poleRowCWidth_pix);
        m_frameCount.incrementAndGet();  // Ignore return value.

        return m_matOutputCb;
    }

}

// Gaussian blur used prior to camera lower position and tilt up.
//        import org.opencv.core.Size;
//        private final double SIGMA_X = 0.0;
//        private final double SIGMA_Y = 0.0;
//        private final Size BLUR_WINDOW = new Size(BLUR_SIZE, BLUR_SIZE);
//        Imgproc.GaussianBlur(m_matBox, m_matBox, BLUR_WINDOW, 0, 0);

