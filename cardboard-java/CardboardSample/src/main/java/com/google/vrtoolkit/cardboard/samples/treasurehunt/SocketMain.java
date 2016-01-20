package com.google.vrtoolkit.cardboard.samples.treasurehunt;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.util.Scanner;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.widget.TextView;
import android.widget.Toast;

public class SocketMain  {

    public float []pos = new float[2];

    public static final String TAG = "Connection";
    public static final int TIMEOUT = 10;

    private String connectionStatus = null;
    private String socketData = null;
    private Handler mHandler = null;
    ServerSocket server = null;

    public SocketMain() {
        mHandler = new Handler();
        new Thread(initializeConnection).start();

    }

    private Runnable initializeConnection = new Thread() {
        public void run() {

            Log.e(TAG, "start");
            Socket client = null;
            // initialize server socket
            while (true) {
                try {
                    //server = new ServerSocket(38300);
                    server = new ServerSocket(38300);
                    server.setSoTimeout(TIMEOUT * 1000);

                    // attempt to accept a connection
                    Log.d(TAG, "wait for connection");
                    client = server.accept();

                    Globals.socketIn = new Scanner(client.getInputStream());
                    Globals.socketOut = new PrintWriter(client.getOutputStream(),
                            true);
                } catch (SocketTimeoutException e) {
                    Log.d(TAG, "Connection has timed out! Please try again");
                } catch (IOException e) {
                    Log.e(TAG, "" + e);
                } finally {
                    // close the server socket
                    try {
                        if (server != null)
                            server.close();
                    } catch (IOException ec) {
                        Log.e(TAG, "Cannot close server socket" + ec);
                    }
                }

                if (client != null) {
                    Globals.connected = true;
                    Log.d(TAG, "connected!");
                    //mHandler.post(showConnectionStatus);
                    while (Globals.socketIn.hasNext()) {
                        socketData = Globals.socketIn.next();
                        Log.d(TAG, "get data " + socketData);
                        mHandler.post(socketStatus);
                    }
                    Log.d(TAG, "hangup");
                    client = null;
                    // startActivity(i);
                }
            }
        }
    };

    private Runnable socketStatus = new Runnable() {
        public void run() {
            String tok[] = socketData.split(",");
            if (tok.length == 2) {
                try {
                    pos[0] = Float.parseFloat(tok[0]);
                    pos[1] = Float.parseFloat(tok[1]);
                } catch (NumberFormatException e) {

                }
            }
            Log.e(TAG, socketData);
        }
    };

    public static class Globals {
        public static boolean connected;
        public static Scanner socketIn;
        public static PrintWriter socketOut;
    }
}