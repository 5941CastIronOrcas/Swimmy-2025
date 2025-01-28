package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;

public class Arduino {

    //checks if arduino connected or not
    public static boolean ArduinoConnected = false;

    //holds the serial port info, which is used to talk with arudino
    public static SerialPort arduinoSerialPort;

    //BaudRate
    public static int BaudRate = 500000;

    //call this functino at the start
    //tries to establish connectino with arduino (not communication but connection, find the port that arduino is connected to)
    public static void ArduinoConnect()
    {

        ArduinoConnected = true;

        //try to connected to arduino through this port0
        try {
            arduinoSerialPort = new SerialPort(BaudRate, SerialPort.Port.kUSB);
            System.out.println("Connected on kUSB");

        } catch (Exception e) {

            //try to connected to arduino through this port1
            try {
                arduinoSerialPort = new SerialPort(BaudRate, SerialPort.Port.kUSB1);
                System.out.println("Connected on kUSB1");

            } catch (Exception e1) {

                //try to connected to arduino through this port2
                try {
                    arduinoSerialPort = new SerialPort(BaudRate, SerialPort.Port.kUSB2);
                    System.out.println("Connected on kUSB2");

                } catch (Exception e2) {
                    System.out.println("ERROR: File Arduino.java; CONNECTION FAILED");
                    ArduinoConnected = false;
                }
            }
        }
    }


    //sends byte to arudino thus activating specifc functino in arduino that returns some value.
    public static double getCallArduino(byte adressToPing) {
        

        //check if arudino is connected
        if (ArduinoConnected) {
            //if connected...
            
            //defult, will not update if transmission failed
            double finalValue = -1;

            //try to call specific function inside arudino, send that byte
            arduinoSerialPort.write(new byte[] {adressToPing}, 1);

            //Check if there is anything availble in serial
            if (arduinoSerialPort.getBytesReceived() > 0) {
                //if availble...

                //Read data from the serial
                String readed = arduinoSerialPort.readString();  //s33.65e
                //System.out.println(readed); //debug
                String filtered = "";

                //start reading data when char 's' is noticed
                if((readed.indexOf("s")) != -1) 
                {
                    filtered = readed.substring((readed.indexOf("s"))+1,(readed.indexOf("e"))-1);

                }

                //try to convert recieved string data into double type variable
                try {
                    finalValue = Double.valueOf(filtered);
                } catch (Exception e) {
                    System.out.println(" \u001b[31;1m.The communication is out of sync!!!!");
                    //failed to convert, usualy means trash data recieved
                }
            }
            return finalValue;
        }

        //connection failed error
        return -3;
    }




    
}
