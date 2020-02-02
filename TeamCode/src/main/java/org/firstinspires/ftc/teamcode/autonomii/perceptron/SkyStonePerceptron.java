package org.firstinspires.ftc.teamcode.autonomii.perceptron;

import android.content.Context;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;

public class SkyStonePerceptron {

    private float trainingStep;

    private float wCS; // weight for the color sensor
    private float wIMU; // weight for IMU
    public final String WEIGHTS_FILE_NAME = "weights.txt";

    /**
     * Creates a new perceptron for the SkyStonePerceptron
     * @param trainingStep
     * @param hardwareMap
     */
    public SkyStonePerceptron(float trainingStep, HardwareMap hardwareMap) {
        this.trainingStep = trainingStep;

        // we need to retrieve the weights
        // file is of the format 'w1,w2' where w1 = wCS and w2 = wIMU

        // read the file
        InputStream stream;
        InputStreamReader rawReader;
        BufferedReader reader;

        try {
            stream = hardwareMap.appContext.openFileInput(this.WEIGHTS_FILE_NAME);
            rawReader = new InputStreamReader(stream);
            reader = new BufferedReader(rawReader);
            String text = reader.readLine(); // of the format w1,w2
            this.wCS = Integer.parseInt(text.split(",")[0]);
            this.wIMU = Float.parseFloat(text.split(",")[1]);

        } catch (FileNotFoundException e) {
            this.wCS = (float)0.5;
            this.wIMU = (float)0.5;
        }
        catch (IOException f) {
            // TODO: we will need to resort to default values and/or ignore the weights and fallback to our original algorithm
        }

        this.wCS = wCS;
        this.wIMU = wIMU;
    }


    /**
     * Get the weights as an array
     * @return the weights as an array
     */
    public float[] getWeights() {
        return new float[] {this.wCS, this.wIMU};
    }

    /**
     * Trains the perceptron, for now, we're really just finding the values we need because the values are pretty constant
     * @param rawInput
     */
    public void train(SSPPoint rawInput) {
        this.wCS = this.wCS + (this.trainingStep * (rawInput.getColorSensorPoint() - (rawInput.getColorSensorPoint() * this.wCS)) * rawInput.getColorSensorPoint());
        this.wIMU = this.wIMU + (this.trainingStep * (rawInput.getIMUPoint() - (rawInput.getIMUPoint() * this.wIMU)) * rawInput.getIMUPoint());
    }

    /**
     * Write the new weights off to a file. Needs to be done due to the fact we are on linear op modes
     * @param hardwareMap
     */
    public void save(HardwareMap hardwareMap) {
        // write the new weights off to the file
        try {
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(hardwareMap.appContext.openFileOutput(this.WEIGHTS_FILE_NAME, Context.MODE_PRIVATE));
            outputStreamWriter.write(String.valueOf(this.wCS) + "," + String.valueOf(this.wIMU));
            outputStreamWriter.close();
        }
        catch (IOException e) {

        }
    }
}
