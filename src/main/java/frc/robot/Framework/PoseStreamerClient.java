package frc.robot.Framework;

import java.net.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.io.*;

public class PoseStreamerClient extends Thread {
    String host;
    int port;
    
    Socket m_socket;

    DataInputStream input;
    DataOutputStream output;

    boolean connected = false;

    public class StreamRequest {
        StreamRequestStatus status;
        int pose_class;
        int object_id;
        Consumer <List<Double>> callback;

        public StreamRequest(int pose_class, int object_id) {
            this.pose_class = pose_class;
            this.object_id = object_id;
            status = StreamRequestStatus.none;
        }

        public StreamRequest setCallback(Consumer <List<Double>> callback) {
            this.callback = callback;
            return this;
        }
    }

    enum StreamRequestStatus {
        none,
        sent,
        accepted
    }

    List<StreamRequest> wantedPoses;

    int requestid;

    long lastHeartBeat;

    public PoseStreamerClient(String host, int port) {
        this.host = host;
        this.port = port;
        wantedPoses = new LinkedList<StreamRequest>();
    }

    public void requestPose(int poseClass, int objectId, Consumer<List<Double>> poseCallback) {
        wantedPoses.add(new StreamRequest(poseClass, objectId).setCallback(poseCallback));
    }

    void sendPacketHeader(int packetType, int headerLength) throws IOException {
        output.writeByte(packetType);
        output.writeByte(headerLength);
    }

    void sendStreamRequest(StreamRequest streamRequest) throws IOException {
        sendPacketHeader(0x01, 3);

        output.writeByte(0x10); //request type: initiate stream
        output.writeByte(0); //request id
        output.writeByte(2); //Request size

        output.writeByte(streamRequest.pose_class);
        output.writeByte(streamRequest.object_id);
        System.out.println("PoseStreamer: sent stream request");
    }

    void readStreamData() throws IOException {
        int received_pose_class = input.readByte();
        int received_object_id = input.readByte();
        int num_doubles = input.readByte() / 8;

        List<Double> values = new ArrayList<Double>();

        ByteBuffer buf = ByteBuffer.allocate(8);
        buf.order(ByteOrder.LITTLE_ENDIAN);

        for(int i = 0; i<num_doubles; i++) {
            buf.put(0, input.readNBytes(8));
            values.add(buf.getDouble(0));
        }

        for (StreamRequest streamRequest : wantedPoses) {
            if(
                streamRequest.pose_class == received_pose_class &&
                (streamRequest.object_id == received_object_id ||
                streamRequest.object_id == 0)
            ) {
                streamRequest.callback.accept(values);
            }
        }
    }

    public boolean connect() {
        try {
            m_socket = new Socket(host, port);
            System.out.println("PoseStreamer: connected to server");

            input = new DataInputStream(m_socket.getInputStream());
            output = new DataOutputStream(m_socket.getOutputStream());
        } catch (UnknownHostException u) {
            System.out.println("PoseStreamer: unknown host");
            return false;
        } catch (IOException i) {
            System.out.println(i);
            return false;
        }

        lastHeartBeat = System.nanoTime();

        return true;
    }

    public void close() {
        for (StreamRequest streamRequest : wantedPoses) {
            streamRequest.status = StreamRequestStatus.none;
        }
        try {
            input.close();
            output.close();
            m_socket.close();
        } catch (IOException i) {
            System.out.println("unable to close");
        } catch (NullPointerException n) {

        }
        connected = false;
    }

    public void run() {
        while(true) {
            if(!connected || m_socket.isClosed()) {
                close();
                connected = connect();
                if(!connected) {
                    System.out.println("PoseStreamer: connection failed... trying again in 2s");
                    try {
                        sleep(2000);
                    } catch(InterruptedException i) {
                        return;
                    }
                    continue;
                }
            }

            try {
                if((System.nanoTime() - lastHeartBeat) > 500000) {
                    output.writeByte(0x03);
                    output.writeByte(0);
                    lastHeartBeat = System.nanoTime();
                }

                if(input.available() >= 2){
                    int packetType = input.read();
                    int headerLength = input.read();
                    switch (packetType) {
                        case 0x10:
                            readStreamData();
                            break;
                        default:
                            break;
                    }
                }

                for (StreamRequest streamRequest : wantedPoses) {
                    if(streamRequest.status == StreamRequestStatus.none) {
                        sendStreamRequest(streamRequest);
                        streamRequest.status = StreamRequestStatus.sent;
                    }
                }
            } catch (IOException i) {
                connected = false;
                System.out.println("disconnecting");
            }
        }
    }
}