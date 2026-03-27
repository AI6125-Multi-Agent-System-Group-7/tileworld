package tileworld.agent.utils;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

/**
 * Lightweight append-only runtime logger for agent-side analytics.
 */
public final class RuntimeFileLogger {
    private final Path logPath;

    public RuntimeFileLogger(String filePath) {
        this.logPath = Paths.get(filePath);
    }

    public synchronized void log(String line) {
        try {
            Path parent = logPath.getParent();
            if (parent != null) {
                Files.createDirectories(parent);
            }
            String message = line + System.lineSeparator();
            Files.write(
                    logPath,
                    message.getBytes(StandardCharsets.UTF_8),
                    StandardOpenOption.CREATE,
                    StandardOpenOption.APPEND);
        } catch (IOException ignored) {
            // Logging must never break the simulation step.
        }
    }
}
