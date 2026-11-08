package org.firstinspires.ftc.teamcode;

import com.google.gson.Gson;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

/**
 * Simplified LimelightHelpers for FTC.
 * Fetches and parses the JSON results string directly from the Limelight 3A web server.
 */
public class LimelightHelpers {

    public static class LimelightResults {
        public Results vpr;
    }

    public static class Results {
        public boolean isValid;
        public RetroOutput[] retroOutputs;


    }

    public static class RetroOutput {
        public double[] tcornx;
        public double[] tcorny;
        public double ta;
    }

    /**
     * Downloads the latest JSON tracking frame from the camera and parses it.
     * @param limelightName If left blank "", defaults to the standard 'limelight.local' address.
     */
    public static LimelightResults getLatestResults(String limelightName) {
        String host = (limelightName == null || limelightName.isEmpty()) ? "limelight.local" : limelightName + ".local";
        String urlString = "http://" + host + ":5807/results";

        try {
            URL url = new URL(urlString);
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("GET");
            conn.setConnectTimeout(250);
            conn.setReadTimeout(250);

            if (conn.getResponseCode() == 200) {
                BufferedReader in = new BufferedReader(new InputStreamReader(conn.getInputStream()));
                StringBuilder response = new StringBuilder();
                String inputLine;

                while ((inputLine = in.readLine()) != null) {
                    response.append(inputLine);
                }
                in.close();

                // Uses FTC's built-in Gson framework to convert the JSON string into the object structures above
                Gson gson = new Gson();
                return gson.fromJson(response.toString(), LimelightResults.class);
            }
        } catch (Exception e) {
            // Rest returns a safe fallback empty result object if the network connection slips
        }

        LimelightResults emptyResults = new LimelightResults();
        emptyResults.vpr = new Results();
        emptyResults.vpr.isValid = false;
        emptyResults.vpr.retroOutputs = new RetroOutput[0];
        return emptyResults;
    }
}
