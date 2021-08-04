//function called by arduino to read any MAVlink messages sent by serial communication from flight controller to arduino
void MavLink_receive()
{
  mavlink_message_t msg;
  mavlink_status_t status;

  while (SerialTEL.available())
  {
    uint8_t c = SerialTEL.read();

    //Get new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {

      //Handle new message from autopilot
      switch (msg.msgid)
      {

        // Step 2 uploading a new waypoint - Check for mission replies
        case MAVLINK_MSG_ID_MISSION_REQUEST:
          {
            mavlink_mission_request_t missionreq;
            mavlink_msg_mission_request_decode(&msg, &missionreq);

            SerialMon.print("\nMission Req Sequence: "); SerialMon.println(missionreq.seq);
            SerialMon.print("\SysID: "); SerialMon.println(missionreq.target_system);
            SerialMon.print("\Compid: "); SerialMon.println(missionreq.target_component);


            switch (missionreq.seq)
            {
              case 0:
                {
                  create_waypoint(0, 0, MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, x_home, y_home, 20);
                  SerialMon.print("Sent Home: \n");
                }
                break;

              case 1:
                {
                  create_waypoint(1, 0, MAV_CMD_NAV_TAKEOFF, 1, 0, 0, 0, 0, 0, 0, 10);
                  SerialMon.print("Sent Waypoint: \n");
                }
                break;

              case 2:
                {
                  create_waypoint(2, 0, MAV_CMD_NAV_WAYPOINT, 1, 0, 0, 0, 0, x_des, y_des, 20);
                  SerialMon.print("Sent Waypoint: \n");
                }
                break;

              case 3:
                {
                  create_waypoint(3, 0, MAV_CMD_NAV_WAYPOINT, 1, 0, 0, 0, 0, x_des, y_des, 3);
                  SerialMon.print("Sent Waypoint: \n");
                }
                break;

              //          case 4:
              //          {
              //            create_waypoint(4, 0, MAV_CMD_DO_SET_CAM_TRIGG_DIST, 1, 0, 0, 0, 0, x_des, y_des, 3);
              //            Serial.print("Sent Waypoint: \n");
              //          }
              //          break;

              case 4:
                {
                  create_waypoint(4, 0, MAV_CMD_NAV_LOITER_TIME, 1, 30, 0, 0, 0, x_des, y_des, 3);
                  SerialMon.print("Sent Waypoint: \n");
                }
                break;

              case 5:
                {
                  create_waypoint(5, 0, MAV_CMD_NAV_WAYPOINT, 1, 0, 0, 0, 0, x_des, y_des, 20);
                  SerialMon.print("Sent Waypoint: \n");
                }
                break;

              case 6:
                {
                  create_waypoint(6, 0, MAV_CMD_NAV_RETURN_TO_LAUNCH, 1, 0, 0, 0, 0, 0, 0, 0);
                  SerialMon.print("Sent Waypoint: \n");
                }
                break;
            }
          }
          break;

        case MAVLINK_MSG_ID_MISSION_ACK:
          // Step 4 uploading a new waypoint - Receive Mission Ack Message
          {
            mavlink_mission_ack_t missionack;
            mavlink_msg_mission_ack_decode(&msg, &missionack);

            SerialMon.print("\nMission Ack Sequence: "); SerialMon.println(missionack.type);
            SerialMon.print("\SysID: "); SerialMon.println(missionack.target_system);
            SerialMon.print("\CompID: "); SerialMon.println(missionack.target_component);

            if (missionack.type == 1) {
              SerialMon.print("\nMission upload FAILED: "); SerialMon.println(missionack.type);
            }

            if (missionack.type == 0) {
              SerialMon.print("\nMission upload SUCCESSFULL: "); SerialMon.println(missionack.type);
              MissionUpload_SUCCESSFULL = true;
              //        delay(100);
              //        Command_long_ARM(1);
              //        delay(3000);
              //        setmode_Auto();

            }
          }
          break;

        case MAVLINK_MSG_ID_GPS_RAW_INT:
          {
            mavlink_gps_raw_int_t packet;
            mavlink_msg_gps_raw_int_decode(&msg, &packet);

            SerialMon.print("\nGPS Fix: "); SerialMon.println(packet.fix_type);
            SerialMon.print("GPS Latitude: "); SerialMon.println(packet.lat/10000000.0f);
            SerialMon.print("GPS Longitude: "); SerialMon.println(packet.lon/10000000.0f);
            SerialMon.print("GPS Speed: "); SerialMon.println(packet.vel);
            SerialMon.print("Sats Visible: "); SerialMon.println(packet.satellites_visible);
            x_cur = packet.lat/10000000.0f;
            y_cur = packet.lon/10000000.0f;
            SerialMon.print(x_cur,10);SerialMon.print(" ; ");SerialMon.println(y_cur,10);
          }
          break;

      }
    }
  }
}

void mission_count(uint16_t count) {
  //Step #1 of uploading a new waypoint
  uint8_t _system_id = 255; // system id of sending station. 255 is Ground control software
  uint8_t _component_id = 2; // component id of sending station 2 works fine
  uint8_t _target_system = 1; // Pixhawk id
  uint8_t _target_component = 0; // Pixhawk component id, 0 = all (seems to work fine)

  //uint16_t count = 2; // How many items to upload (HOME coordinates are always the first way-point)

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_mission_count_pack(_system_id, _component_id, &msg, _target_system, _target_component, count);
  //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t count

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message (.write sends as bytes)
  SerialTEL.write(buf, len);

}

void create_waypoint(uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, float param1, float param2, float param3, float param4, float x, float y, float z ) {
  //Step 3 continuation of uploading a new waypoint (send 1st coordinates)
  uint8_t _system_id = 255; // system id of sending station. 255 is Ground control software
  uint8_t _component_id = 2; // component id of sending station 2 works fine
  uint8_t _target_system = 1; // Pixhawk id
  uint8_t _target_component = 0; // Pixhawk component id, 0 = all (seems to work fine)

  //  uint16_t seq = 1; // Sequence number
  //  uint8_t frame = 0; // Set target frame to global default
  //  uint16_t command = MAV_CMD_NAV_WAYPOINT; // Specific command for PX4
  //  uint8_t current = 1; // Guided mode waypoint
  uint8_t autocontinue = 0; // Always 0
  //  float param1 = 0; // Loiter time
  //  float param2 = 0; // Acceptable range from target - radius in meters
  //  float param3 = 0; // Pass through waypoint
  //  float param4 = 0; // Desired yaw angle
  //  float x = 11.6537718; // Latitude - degrees
  //  float y = 104.9116391; // Longitude - degrees
  //  float z = 20; // Altitude - meters

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_mission_item_pack(_system_id, _component_id, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z);
  //uint16_t mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message (.write sends as bytes)
  SerialTEL.write(buf, len);

}

void setmode_Auto() {
  //Set message variables
  uint8_t _system_id = 255; // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t _base_mode = 1;
  uint32_t _custom_mode = 3; //3 = auto mode

  /*
    Flight / Driving Modes (change custom mode above)
    0 - Stabilize
    1 - Acro
    2 - AltHold
    3 - Auto
    4 - Guided
    5 - Loiter
    6 - RTL
    7 - Circle
    9 - Land
    11 - Drift
  */

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_set_mode_pack(_system_id, _component_id, &msg, _target_system, _base_mode, _custom_mode);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)
  SerialMon.print("\nsending set mode command...");
  SerialTEL.write(buf, len); //Write data to serial port
}

void Command_long_ARM(float param1) {
  uint8_t _system_id = 255; // system id of sending station. 255 is Ground control software
  uint8_t _component_id = 2; // component id of sending station 2 works fine
  uint8_t _target_system = 1; // Pixhawk id
  uint8_t _target_component = 0; // Pixhawk component id, 0 = all (seems to work fine)

  //float param1 = 0; // to arm the Pixhawk
  float param2 = 0;
  float param3 = 0;
  float param4 = 0;
  float param5 = 0;
  float param6 = 0;
  float param7 = 0;

  uint16_t CMD_LONG_command = MAV_CMD_COMPONENT_ARM_DISARM; //this is the type of command i.e. disamr/arm
  uint8_t CMD_LONG_confirmation = 0;

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_command_long_pack(_system_id, _component_id, &msg, _target_system, _target_component, CMD_LONG_command, CMD_LONG_confirmation, param1, param2, param3, param4, param5, param6, param7);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)
  SerialMon.print("\nsending set mode command...");
  SerialTEL.write(buf, len); //Write data to serial port

}

void request_datastream(uint8_t _start_stop) {
//Request Data from Pixhawk
  uint8_t _system_id = 255; // id of computer which is sending the command (ground control software has id of 255)
  uint8_t _component_id = 2; // seems like it can be any # except the number of what Pixhawk sys_id is
  uint8_t _target_system = 1; // Id # of Pixhawk (should be 1)
  uint8_t _target_component = 0; // Target component, 0 = all (seems to work with 0 or 1
  uint8_t _req_stream_id = MAV_DATA_STREAM_ALL;
  uint16_t _req_message_rate = 0x01; //number of times per second to request the data in hex
  //uint8_t _start_stop = 1; //1 = start, 0 = stop

// STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM and more importantly at:
     https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_request_data_stream_pack(_system_id, _component_id, &msg, _target_system, _target_component, _req_stream_id, _req_message_rate, _start_stop);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);  // Send the message (.write sends as bytes)

  SerialTEL.write(buf, len); //Write data to serial port
}
