package org.firstinspires.ftc.teamcode.aim.utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

public class Logging
{
    private FileWriter fileWriter;
    private BufferedWriter bufferedWriter;
    private String fieldsLine = null;
    private boolean fieldsLineOutputted = false;

    public Logging(String filename) throws IOException {
        String filepath = String.format("/sdcard/FIRST/java/src/Datalogs/%s.txt", filename);
        File tmp = new File(filepath);

        // Ensure parent directory exists
        if (!tmp.getParentFile().exists()) {
            tmp.getParentFile().mkdirs();
        }

        // Delete existing file to ensure clean start
        if (tmp.exists()) {
            tmp.delete();
        }

        fileWriter = new FileWriter(filepath, false);
        bufferedWriter = new BufferedWriter(fileWriter);
    }

    public void setFieldsLine(String line) {
        this.fieldsLine = "timestamp," + line;
    }

    public void write(String format, Object... args) throws IOException {
        if (!this.fieldsLineOutputted) {
            bufferedWriter.write(this.fieldsLine);
            bufferedWriter.newLine();
            this.fieldsLineOutputted = true;
        }

        long timestamp = System.currentTimeMillis();
        String formattedLine = String.format(Locale.US, format, args);
        bufferedWriter.write(timestamp + "," + formattedLine);
        bufferedWriter.newLine();
    }

    public void close() throws IOException
    {
        bufferedWriter.close();
    }
}
