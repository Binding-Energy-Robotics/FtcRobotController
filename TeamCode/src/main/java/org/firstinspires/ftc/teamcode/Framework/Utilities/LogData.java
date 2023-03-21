package org.firstinspires.ftc.teamcode.Framework.Utilities;

import android.annotation.SuppressLint;
import android.os.Environment;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;

public class LogData {
	private static FileWriter fileWriter;
	private static ElapsedTime startTimer;
	private static StringBuilder line;
	private static boolean isOpen = false;

	public static void open() {
		if (isOpen)
			return;

		@SuppressLint("SimpleDateFormat") String timestamp =
				new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss").format(new java.util.Date());
		String filePath = Environment.getExternalStorageDirectory().getAbsolutePath() +
				"/FIRST/data/log_" + timestamp + ".txt";

		try {
			File logFile = new File(filePath);
			logFile.createNewFile();
			fileWriter = new FileWriter(logFile);
		}
		catch (IOException e) {
			e.printStackTrace();
		}

		startTimer = new ElapsedTime();

		line = new StringBuilder("time: ");
		line.append(0);

		isOpen = true;
	}

	public static void close() {
		if (!isOpen)
			return;

		try {
			fileWriter.close();
		}
		catch (IOException e) {
			e.printStackTrace();
		}

		isOpen = false;
	}

	public static void update() {
		if (!isOpen)
			open();

		line.append('\n');

		try {
			fileWriter.write(line.toString());
		}
		catch (IOException e) {
			e.printStackTrace();
		}

		clearLine();
	}

	private static void clearLine() {
		line = new StringBuilder(line.capacity());
		line.append("time: ");
		line.append(startTimer.time());
	}

	private static void separate() {
		line.append("; ");
	}

	private static void space() {
		line.append(": ");
	}

	public static void addData(String data) {
		if (!isOpen)
			open();

		separate();
		line.append(data);
	}

	public static void addData(String label, Object data) {
		if (!isOpen)
			open();

		separate();
		line.append(label);
		space();
		line.append(data);
	}
}
