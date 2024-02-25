package frc.robot.Framework;

import java.net.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.io.*;

public class PoseStreamerClient extends Thread {
    String host;
    int port;
    
    Socket m_socket;

    DataInputStream input;
    DataOutputStream output;

    boolean connected = false;

    long clock_request_ts;
    long clock_offset;
    long estimated_latency;
    public boolean awaiting_clock_request = false;

    public class StreamRequest {
        StreamRequestStatus status;
        int pose_class;
        int object_id;
        Consumer <List<Double>> callback;
        long send_ts;
        int id;

        public StreamRequest(int pose_class, int object_id) {
            this.pose_class = pose_class;
            this.object_id = object_id;
            status = StreamRequestStatus.none;
        }

        public StreamRequest setCallback(Consumer <List<Double>> callback) {
            this.callback = callback;
            return this;
        }
        public StreamRequest setId(int id) {
            this.id = id;
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
        wantedPoses.add(new StreamRequest(poseClass, objectId).setCallback(poseCallback).setId(requestid));
        requestid++;
        if(requestid > 255) requestid = 0;
    }

    void sendPacketHeader(int packetType, int headerLength) throws IOException {
        output.writeByte(0x05);
        output.writeByte(0xb3);
        
        output.writeByte(packetType);
        output.writeByte(headerLength);
    }

    void sendStreamRequest(StreamRequest streamRequest) throws IOException {
        sendPacketHeader(0x01, 3);

        output.writeByte(0x10); //request type: initiate stream
        output.writeByte(streamRequest.id); //request id
        output.writeByte(2); //Request size

        output.writeByte(streamRequest.pose_class);
        output.writeByte(streamRequest.object_id);
        output.flush();
        streamRequest.send_ts = System.nanoTime();
        System.out.println("PoseStreamer: sent stream request");
    }

    void sendClockRequest() throws IOException {
        sendPacketHeader(0x31, 0);
        clock_request_ts = System.nanoTime();
    }

    void handleClockResponse() throws IOException {
        long clock_response_ts = System.nanoTime();
        
        estimated_latency = (clock_response_ts - clock_request_ts);
        System.out.println("Estimating one-way latency at " + (estimated_latency / 2000000.0) + "ms");

        ByteBuffer buf = ByteBuffer.allocate(8);
        buf.order(ByteOrder.LITTLE_ENDIAN);

        buf.put(0, input.readNBytes(8));

        long remote_time = buf.getLong();

        long clock_offset = (remote_time - (clock_response_ts - (estimated_latency / 2)));
        System.out.println("Set clock offset to " + clock_offset);

        double remote_send_estimate = (clock_response_ts - (remote_time - clock_offset))/1000000.0;

        System.out.println("Got time: " + remote_time);
        System.out.println("Got estimate: " + remote_send_estimate);
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

    void readRequestResponse() throws IOException {
        int request_type = input.readByte();
        int request_id = input.readByte();
        int status_code = input.readByte();

        System.out.println("streamreq id " + request_id);
        System.out.println("streamreq type" + request_type);

        if(request_type != 16) return;
        if(status_code != 0) {
            System.out.println("PoseStreamer: server returned response " +  status_code + " to stream request");
            return;
        }
        for (StreamRequest streamRequest : wantedPoses) {
            if(request_id == streamRequest.id) {
                System.out.println("PoseStreamer: request response received");
                streamRequest.status = StreamRequestStatus.accepted;
            }
        }
    }

    void parseIncoming() throws IOException {
        if(input.available() >= 2){
            int preamble1 = input.read();
            if(preamble1 != 0x05) return;

            int preamble2 = input.read();
            if(preamble2 != 0xb3) return;

            int packetType = input.read();
            int headerLength = input.read();
            switch (packetType) {
                case 0x10:
                    readStreamData();
                    break;
                case 0x02:
                    readRequestResponse();
                    break;
                case 0x30:
                    handleClockResponse();
                    break;
                default:
                    break;
            }
        }
    }

    public boolean connect() {
        try {
            m_socket = new Socket(host, port);
            System.out.println("PoseStreamer: connected to server");

            input = new DataInputStream(m_socket.getInputStream());
            output = new DataOutputStream(m_socket.getOutputStream());
            try {
                sleep(250);

            } catch (InterruptedException e) {
                return false;
            }
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
        requestid = 0;
        while(true) {
            if(!connected || m_socket.isClosed()) {
                requestid = 0;
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
                if((System.nanoTime() - lastHeartBeat) > 500000000) {
                    sendPacketHeader(0x03, 0);
                    lastHeartBeat = System.nanoTime();
                }

                parseIncoming();

                for (StreamRequest streamRequest : wantedPoses) {
                    if(streamRequest.status == StreamRequestStatus.none) {
                        sendStreamRequest(streamRequest);
                        streamRequest.status = StreamRequestStatus.sent;
                    }
                    if(streamRequest.status == StreamRequestStatus.sent) {
                        if((System.nanoTime() - streamRequest.send_ts) > 800000000) {
                            streamRequest.status = StreamRequestStatus.none;
                            System.out.println("PoseStreamer: Stream request timed out");
                        }
                    }
                }
                if(awaiting_clock_request) {
                    sendClockRequest();
                    awaiting_clock_request = false;
                }
            } catch (IOException i) {
                connected = false;
                System.out.println("disconnecting");
            }
            try {
                        sleep(1);
            } catch(InterruptedException i) {
                        return;
            }
        }
    }
}
