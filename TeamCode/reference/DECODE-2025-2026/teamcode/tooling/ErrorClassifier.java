package org.firstinspires.ftc.teamcode.tooling;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Set;
import java.util.TreeSet;
import java.util.concurrent.Callable;

public final class ErrorClassifier {
    private static ErrorClassifier instance;
    public static boolean errorDetected = false;
    private final Telemetry telemetry; // Instance variable instead of static
    private final Set<String> errors = new TreeSet<>();

    public static ErrorClassifier getInstance() {
        if (instance == null) {
            throw new IllegalStateException("ErrorClassifier instance not created. Call createInstance() first.");
        }
        return instance;
    }

    public static void createInstance(Telemetry telemetry) {
        if (instance == null) {
            instance = new ErrorClassifier(telemetry);
        }
    }

    public void addErrorsToTelemetry() {
        for (String error : errors) {
            telemetry.addLine(error);
        }
    }

    public void addError(String error) {
        errorDetected = true;
        errors.add(error);
    }

    public static boolean assertStatement(Callable<Boolean> conditional, String error) {
        if (instance == null) {
            throw new IllegalStateException("ErrorClassifier instance not created. Call createInstance() first.");
        }
        try {
            if (conditional.call()) {
                return true;
            } else {
                instance.addError(error);
                return false;
            }
        } catch (Exception e) {
            instance.addError("Exception in assertStatement: " + e);
            return false;
        }
    }

    private ErrorClassifier(Telemetry telemetry) {
        this.telemetry = telemetry; // Use instance variable
    }
}