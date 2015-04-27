#include <mavlink.h>
#include <mavlink_types.h>
#include <mavlink_helpers.h>
#include <checksum.h>
#include <protocol.h>


void comm_send_ch(mavlink_channel_t chan, uint8_t ch) {
  Serial.write(ch);
}

void comm_receive() { 
	mavlink_message_t msg; 
	mavlink_status_t status;
	
	//receive data over serial 
	while(Serial.available() > 0) { 
		uint8_t c = Serial.read();
		
		//try to get a new message 
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) { 
			// Handle message
                        mavlink_msg_named_value_int_send(MAVLINK_COMM_0, millis(), "LastMsg", msg.msgid);
 			switch(msg.msgid) {
			        case MAVLINK_MSG_ID_SET_MODE: {
			        	// set mode
			        }
			        break;
			        case  MAVLINK_MSG_ID_COMMAND_LONG:
					// EXECUTE ACTION
				break;
				default:
					//Do nothing
				break;
			}
		} 
		// And get the next one
	}
}


