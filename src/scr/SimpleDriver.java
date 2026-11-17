package scr;

import java.awt.event.KeyEvent;
import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.SwingUtilities;

/**
 * Driver for TORCS based on behavioral cloning with a k-NN classifier.
 *
 * <p>The driver works in one of two modes, selected with the {@code torcs.mode}
 * system property:</p>
 * <ul>
 *   <li><b>drive</b> (default): at every simulation step the current sensor
 *       reading is normalized, classified by a k-NN built on the recorded
 *       dataset, and the predicted class (accelerate, brake, steer left,
 *       steer right, reverse, default) is translated into car commands.
 *       Off-track and stuck-car recovery routines override the prediction
 *       when needed.</li>
 *   <li><b>train</b>: the human driver controls the car with the keyboard
 *       through a small Swing window ({@link CharReader}) and every
 *       (sensors, key) pair is appended to a CSV file.</li>
 * </ul>
 *
 * <p>Configuration (JVM system properties):</p>
 * <ul>
 *   <li>{@code -Dtorcs.mode=drive|train} - operating mode (default {@code drive})</li>
 *   <li>{@code -Dtorcs.data=<file>} - dataset used by the k-NN in drive mode
 *       (default {@code data/Torcs_data.csv})</li>
 *   <li>{@code -Dtorcs.output=<file>} - file the recording is written to in
 *       train mode (default {@code data/recorded_session.csv}); it is
 *       overwritten at the start of every recording session</li>
 * </ul>
 *
 * <p>Classes of the dataset: 0 = accelerate, 1 = brake, 2 = steer left,
 * 3 = steer right, 4 = reverse, 5 = no key pressed.</p>
 */
public class SimpleDriver extends Controller {

    /* ---------------------------------------------------------------- */
    /* Configuration                                                    */
    /* ---------------------------------------------------------------- */

    private static final String MODE_PROPERTY = "torcs.mode";
    private static final String DATA_PROPERTY = "torcs.data";
    private static final String OUTPUT_PROPERTY = "torcs.output";
    private static final String DEFAULT_DATA_FILE = "data/Torcs_data.csv";
    private static final String DEFAULT_OUTPUT_FILE = "data/recorded_session.csv";

    /** Number of neighbours used by the k-NN classifier. */
    private static final int K = 3;

    private final boolean train;
    private final boolean autonomousDriving;

    /** Action sent to the simulator; in train mode it is also driven by the keyboard. */
    public Action trainingAction;

    /** Last key code pressed by the human driver (written by the Swing thread, read by the main one). */
    public volatile int ch;

    private NearestNeighbor knn;
    private String trainingFilename;

    /* ---------------------------------------------------------------- */
    /* Normalization ranges                                             */
    /* ---------------------------------------------------------------- */

    private final int minTrackEdgeSensor = 0;
    private final int maxTrackEdgeSensor = 200;
    private final double minSpeed = 0;
    private final double maxSpeed1 = 310.00;
    private final double minSpeedReverse = -60.0;
    private final double maxSpeedReverse = -0.001;

    /* ---------------------------------------------------------------- */
    /* Recovery state                                                   */
    /* ---------------------------------------------------------------- */

    /** True while the off-track correction is overriding the k-NN prediction. */
    private boolean correct = false;
    private final int stuckTimeAutonomous = 40;
    private final float stuckAngleAutonomous = (float) 0.523598775;
    private int stuckAutonomous = 0;
    private final float steerLockAutonomous = (float) 0.10;
    private boolean carStuck = false;

    /* ---------------------------------------------------------------- */
    /* Gear shifting thresholds (RPM)                                   */
    /* ---------------------------------------------------------------- */

    final int[] gearUp = {7500, 8000, 8500, 9000, 9500, 0};
    final int[] gearDown = {0, 2800, 3200, 3500, 5200, 5000};

    /* ---------------------------------------------------------------- */
    /* Clutch parameters                                                */
    /* ---------------------------------------------------------------- */

    final float clutchMax = (float) 0.5;
    final float clutchDelta = (float) 0.05;
    final float clutchDeltaTime = (float) 0.02;
    final float clutchDeltaRaced = 10;
    final float clutchDec = (float) 0.01;
    final float clutchMaxModifier = (float) 1.3;
    final float clutchMaxTime = (float) 1.5;

    /** Current clutch value. */
    private float clutch = 0;

    public SimpleDriver() {
        String mode = System.getProperty(MODE_PROPERTY, "drive").trim().toLowerCase();
        switch (mode) {
            case "drive":
                train = false;
                autonomousDriving = true;
                break;
            case "train":
                train = true;
                autonomousDriving = false;
                break;
            default:
                throw new IllegalArgumentException("Invalid -D" + MODE_PROPERTY + "=" + mode
                        + " (expected 'drive' or 'train')");
        }

        trainingAction = new Action();

        if (train) {
            trainingFilename = System.getProperty(OUTPUT_PROPERTY, DEFAULT_OUTPUT_FILE);
            writeCsvHeader();
            SwingUtilities.invokeLater(() -> new CharReader(this));
        }

        if (autonomousDriving) {
            String prototypesFilename = System.getProperty(DATA_PROPERTY, DEFAULT_DATA_FILE);
            if (!Files.isRegularFile(Paths.get(prototypesFilename))) {
                String message = "Dataset not found: " + Paths.get(prototypesFilename).toAbsolutePath()
                        + System.lineSeparator()
                        + "Run the client from the repository root or set -D" + DATA_PROPERTY + "=<path>.";
                System.err.println(message);
                throw new IllegalStateException(message);
            }
            knn = new NearestNeighbor(prototypesFilename);
        }
    }

    @Override
    public void reset() {
        System.out.println("Restarting the race!");
    }

    @Override
    public void shutdown() {
        System.out.println("Bye bye!");
    }

    /* ---------------------------------------------------------------- */
    /* Main control loop                                                */
    /* ---------------------------------------------------------------- */

    @Override
    public Action control(SensorModel sensors) {

        // Gear handling: automatic gearbox unless the driver is reversing from standstill
        if (trainingAction.gear != -1.0 || sensors.getSpeed() > 1) {
            trainingAction.gear = getGear(sensors);
        } else if (trainingAction.gear == -1 || sensors.getSpeed() < 1) {
            trainingAction.accelerate = 1.0;
        }
        trainingAction.clutch = clutching(sensors, (float) trainingAction.clutch);

        if (train) {
            // Record the (sensors, key) pair and replay the human commands
            try {
                exportToCSV(sensors);
            } catch (IOException ex) {
                System.err.println("Error: " + ex.getMessage());
            }
            return trainingAction;
        }

        if (autonomousDriving) {
            recovery(sensors);
            if (!carStuck) {
                predictControl(sensors);
            }
        }

        return trainingAction;
    }

    /* ---------------------------------------------------------------- */
    /* Gearbox and clutch                                               */
    /* ---------------------------------------------------------------- */

    private int getGear(SensorModel sensors) {
        int gear = sensors.getGear();
        double rpm = sensors.getRPM();

        // Neutral (0) or reverse (-1): go to first gear
        if (gear < 1) {
            return 1;
        }

        // Shift up if the RPM is above the threshold of the current gear
        if (gear < 6 && rpm >= gearUp[gear - 1]) {
            return gear + 1;
        }
        // Shift down if the RPM is below the threshold of the current gear
        if (gear > 1 && rpm <= gearDown[gear - 1]) {
            return gear - 1;
        }
        // Otherwise keep the current gear
        return gear;
    }

    float clutching(SensorModel sensors, float clutch) {

        float maxClutch = clutchMax;

        // Check whether this is the start of the race
        if (sensors.getCurrentLapTime() < clutchDeltaTime && getStage() == Stage.RACE
                && sensors.getDistanceRaced() < clutchDeltaRaced) {
            clutch = maxClutch;
        }

        // Update the current clutch value
        if (clutch > 0) {
            double delta = clutchDelta;
            if (sensors.getGear() < 2) {

                // Release the clutch more slowly in first gear right after the start
                delta /= 2;
                maxClutch *= clutchMaxModifier;
                if (sensors.getCurrentLapTime() < clutchMaxTime) {
                    clutch = maxClutch;
                }
            }

            // The clutch cannot exceed its maximum value
            clutch = Math.min(maxClutch, clutch);

            // If the clutch is not at its maximum, release it quickly
            if (clutch != maxClutch) {
                clutch -= delta;
                clutch = Math.max((float) 0.0, clutch);
            }
            // If it is at its maximum, release it very slowly
            else {
                clutch -= clutchDec;
            }
        }
        return clutch;
    }

    /* ---------------------------------------------------------------- */
    /* Sensor setup                                                     */
    /* ---------------------------------------------------------------- */

    /**
     * Angles (in degrees) of the 19 track edge sensors:
     * {-90,-75,-60,-45,-30,-20,-15,-10,-5,0,5,10,15,20,30,45,60,75,90}.
     */
    @Override
    public float[] initAngles() {

        float[] angles = new float[19];

        for (int i = 0; i < 5; i++) {
            angles[i] = -90 + i * 15;
            angles[18 - i] = 90 - i * 15;
        }

        for (int i = 5; i < 9; i++) {
            angles[i] = -20 + (i - 5) * 5;
            angles[18 - i] = 20 - (i - 5) * 5;
        }
        angles[9] = 0;
        return angles;
    }

    /* ---------------------------------------------------------------- */
    /* Training data recording                                          */
    /* ---------------------------------------------------------------- */

    private BufferedWriter openTrainingWriter(boolean append) throws IOException {
        return new BufferedWriter(new OutputStreamWriter(
                new FileOutputStream(trainingFilename, append), StandardCharsets.UTF_8));
    }

    /** Creates (or truncates) the recording file and writes the CSV header. */
    private void writeCsvHeader() {
        Path parent = Paths.get(trainingFilename).toAbsolutePath().getParent();
        try {
            if (parent != null) {
                Files.createDirectories(parent);
            }
        } catch (IOException ex) {
            Logger.getLogger(SimpleDriver.class.getName()).log(Level.SEVERE, null, ex);
        }

        StringBuilder header = new StringBuilder("Velocità;DistanzaLineaCentrale");
        for (int i = 0; i < 20; i++) {
            header.append(";TrackEdgeSensors[").append(i).append("]");
        }
        header.append(";cls\n");

        try (BufferedWriter bw = openTrainingWriter(false)) {
            bw.append(header);
        } catch (IOException ex) {
            Logger.getLogger(SimpleDriver.class.getName()).log(Level.SEVERE, null, ex);
        }
        System.out.println("Recording training data to: " + Paths.get(trainingFilename).toAbsolutePath());
    }

    /**
     * Appends one row to the recording file: normalized speed, track position,
     * the 19 track edge sensors, the angle to the track axis and the class of
     * the key currently pressed.
     */
    private void exportToCSV(SensorModel sensors) throws IOException {

        try (BufferedWriter bw = openTrainingWriter(true)) {
            if (sensors.getSpeed() >= 0) {
                bw.append(normalize(sensors.getSpeed(), minSpeed, maxSpeed1) + ";");
            } else if (sensors.getSpeed() < 0) {
                bw.append(-normalize(sensors.getSpeed(), minSpeedReverse, maxSpeedReverse) + ";");
            }
            bw.append(normalize(sensors.getTrackPosition(), -1, +1) + ";");

            double[] edges = sensors.getTrackEdgeSensors();
            for (int i = 0; i < 19; i++) {
                bw.append(normalize(edges[i], minTrackEdgeSensor, maxTrackEdgeSensor) + ";");
            }
            bw.append(normalize(sensors.getAngleToTrackAxis(), -(Math.PI), +(Math.PI)) + ";");

            int cls = ch == KeyEvent.VK_UP ? 0
                    : ch == KeyEvent.VK_DOWN ? 1
                    : ch == KeyEvent.VK_LEFT ? 2
                    : ch == KeyEvent.VK_RIGHT ? 3
                    : ch == KeyEvent.VK_SHIFT ? 4
                    : 5;
            bw.append(String.valueOf(cls));
            bw.append('\n');

        } catch (IOException ex) {
            Logger.getLogger(SimpleDriver.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    private double normalize(double value, double min, double max) {
        return (value - min) / (max - min);
    }

    /* ---------------------------------------------------------------- */
    /* k-NN prediction                                                  */
    /* ---------------------------------------------------------------- */

    /** Classifies the current sensor reading with the k-NN and applies the predicted action. */
    public void predictControl(SensorModel sensors) {
        double speed = 0;

        if (sensors.getSpeed() >= 0) {
            speed = normalize(sensors.getSpeed(), minSpeed, maxSpeed1);
        } else if (sensors.getSpeed() < 0) {
            speed = normalize(sensors.getSpeed(), minSpeedReverse, maxSpeedReverse);
        }

        double[] e = sensors.getTrackEdgeSensors();
        DrivingData dd = new DrivingData(
                speed,
                normalize(sensors.getTrackPosition(), -1, +1),
                normalize(e[0], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[1], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[2], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[3], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[4], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[5], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[6], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[7], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[8], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[9], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[10], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[11], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[12], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[13], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[14], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[15], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[16], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[17], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(e[18], minTrackEdgeSensor, maxTrackEdgeSensor),
                normalize(sensors.getAngleToTrackAxis(), -(Math.PI), +(Math.PI)));

        int predictedClass = knn.classify(dd, K);
        autoControl(predictedClass, sensors);
    }

    /** Translates a predicted class into car commands, unless the off-track correction is active. */
    private void autoControl(int cls, SensorModel sensors) {

        // Correct the trajectory when the car is off track
        correctOffTrack(sensors);

        if (!correct) {

            switch (cls) {
                case 0:
                    accelera();
                    break;
                case 1:
                    frena();
                    break;
                case 2:
                    sterzaSX(sensors);
                    break;
                case 3:
                    sterzaDX(sensors);
                    break;
                case 4:
                    retro();
                    break;
                case 5:
                    setDefault();
                    break;
                default:
                    break;
            }
        }
    }

    /* ---------------------------------------------------------------- */
    /* Actions associated with each class                               */
    /* ---------------------------------------------------------------- */

    /** Class 0: full throttle, wheels straight. */
    public void accelera() {
        trainingAction.accelerate = 1.0;
        trainingAction.steering = 0.0;
        trainingAction.brake = 0.0;
    }

    /** Class 1: brake, wheels straight. */
    public void frena() {
        trainingAction.brake = 0.8;
        trainingAction.accelerate = 0.0;
        trainingAction.steering = 0.0;
    }

    /** Class 2: steer left (positive steering); the faster the car, the gentler the command. */
    public void sterzaSX(SensorModel sensors) {

        if (sensors.getSpeed() > 150) {
            trainingAction.steering = +0.45;
            trainingAction.accelerate = 0.10;
        } else if (sensors.getSpeed() > 100) {
            trainingAction.steering = +0.50;
            trainingAction.accelerate = 0.30;
        } else {
            trainingAction.steering = +0.55;
            trainingAction.accelerate = 0.60;
        }
        trainingAction.brake = 0.0;
    }

    /**
     * Class 3: steer right (negative steering); the faster the car, the gentler the command.
     *
     * <p>Known issue (kept as-is to preserve the behaviour of the submitted driver): in the
     * low-speed branch the steering is {@code +0.55}, i.e. towards the left.</p>
     */
    public void sterzaDX(SensorModel sensors) {

        if (sensors.getSpeed() > 150) {
            trainingAction.steering = -0.45;
            trainingAction.accelerate = 0.10;
        } else if (sensors.getSpeed() > 100) {
            trainingAction.steering = -0.50;
            trainingAction.accelerate = 0.30;
        } else {
            trainingAction.steering = +0.55;
            trainingAction.accelerate = 0.60;
        }
        trainingAction.brake = 0.0;
    }

    /** Class 4: reverse gear. */
    public void retro() {
        trainingAction.gear = -1;
        trainingAction.accelerate = 0.6;
        trainingAction.steering = 0.0;
        trainingAction.brake = 0.0;
    }

    /** Class 5: no key pressed, moderate throttle. */
    public void setDefault() {
        trainingAction.accelerate = 0.7;
        trainingAction.steering = 0.0;
        trainingAction.brake = 0.0;
    }

    /* ---------------------------------------------------------------- */
    /* Recovery routines                                                */
    /* ---------------------------------------------------------------- */

    /** Detects an off-track car (|trackPos| &gt; 1.2) and steers it back towards the track. */
    private void correctOffTrack(SensorModel sensors) {
        // Off track on the left edge
        if (sensors.getTrackPosition() > 1.20) {
            correct = true;
            offTrackSterzaDX(sensors);
        }
        // Off track on the right edge
        else if (sensors.getTrackPosition() < -1.20) {
            correct = true;
            offTrackSterzaSX(sensors);
        } else {
            correct = false;
        }
    }

    public void offTrackSterzaSX(SensorModel sensors) {
        if (sensors.getSpeed() > 125) {
            trainingAction.steering = +0.15;
            trainingAction.accelerate = 0.20;
        } else {
            trainingAction.steering = +0.25;
            trainingAction.accelerate = 0.40;
        }
        trainingAction.brake = 0.0;
    }

    public void offTrackSterzaDX(SensorModel sensors) {
        if (sensors.getSpeed() > 125) {
            trainingAction.steering = -0.15;
            trainingAction.accelerate = 0.20;
        } else {
            trainingAction.steering = -0.25;
            trainingAction.accelerate = 0.40;
        }
        trainingAction.brake = 0.0;
    }

    /**
     * Stuck-car recovery: when the angle to the track axis stays above 30 degrees
     * for more than {@code stuckTimeAutonomous} steps the car is considered stuck
     * and it is driven (in reverse or forward) back parallel to the track axis.
     */
    public void recovery(SensorModel sensors) {

        if (Math.abs(sensors.getAngleToTrackAxis()) > stuckAngleAutonomous) {
            stuckAutonomous++;
        } else {
            stuckAutonomous = 0;
        }
        // Car stuck
        if (stuckAutonomous > stuckTimeAutonomous) {
            carStuck = true;
            // Steer to bring the car parallel to the track axis
            float steer = (float) (-sensors.getAngleToTrackAxis() / steerLockAutonomous);
            int gear = -1;
            // If the car already points in the right direction, go forward and invert the steering
            if (sensors.getAngleToTrackAxis() * sensors.getTrackPosition() > 0) {
                gear = 1;
                steer = -steer;
            }
            clutch = clutching(sensors, clutch);
            trainingAction.gear = gear;
            trainingAction.steering = steer;
            trainingAction.accelerate = 0.6;
            trainingAction.brake = 0.0;
            trainingAction.clutch = clutch;
        } else {
            carStuck = false;
        }
    }

}
