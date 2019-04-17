#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <vector>
#include "mavlinkTask.h"
#include "gps.h"
#include "imu.h"

#include "mavlink/mavlink_types.h"
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes
void mavlink_send_uart_bytes(uint8_t chan, const uint8_t *buf, uint8_t len);
mavlink_system_t mavlink_system;
#include "mavlink/standard/mavlink.h"

// Define the system type, in this case a rover
uint8_t system_type = MAV_TYPE_GROUND_ROVER;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC_MISSION_FULL;

uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

std::vector<mavlink_mission_item_int_t> missions[4];
uint16_t currentMissionItem = 0;
uint16_t itemsToAdd = 0;

uint8_t pingSeq = 0;

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define MS_TO_G 0.101972

void mavlink_send_uart_bytes(uint8_t chan, const uint8_t *buf, uint8_t len)
{
  Serial1.write(buf, len);
}

void mavlinkRecive()
{
  while (Serial1.available())
  {
    uint8_t chan = 0;
    uint8_t byte = Serial1.read();
    mavlink_message_t msg;
    mavlink_status_t status;
    if (mavlink_parse_char(chan, byte, &msg, &status))
    {
      switch (msg.msgid)
      {
      case MAVLINK_MSG_ID_HEARTBEAT:
      {
      }
      break;

      case MAVLINK_MSG_ID_SYS_STATUS:
      {
        mavlink_sys_status_t sys_status;
        mavlink_msg_sys_status_decode(&msg, &sys_status);
      }
      break;

      case MAVLINK_MSG_ID_PING:
      {
        mavlink_ping_t ping;
        mavlink_msg_ping_decode(&msg, &ping);
        if (ping.target_system == mavlink_system.sysid && ping.target_component)
        {
          pingSeq = ping.seq + 1;
          mavlink_msg_ping_send(MAVLINK_COMM_0, ping.time_usec, pingSeq, msg.sysid, 0);
        }
      }
      break;

      case MAVLINK_MSG_ID_SET_MODE:
      {
        mavlink_set_mode_t mode_value;
        mavlink_msg_set_mode_decode(&msg, &mode_value);
        if (mode_value.target_system == mavlink_system.sysid)
        {
          system_mode = mode_value.base_mode;
          custom_mode = mode_value.custom_mode;
        }
      }
      break;

      case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
      {
        mavlink_msg_param_value_send(MAVLINK_COMM_0, 0, 0, 0, 0, 0);
      }
      break;

      case MAVLINK_MSG_ID_PARAM_VALUE:
      {
        mavlink_param_value_t param_value;
        mavlink_msg_param_value_decode(&msg, &param_value);
      }
      break;

      case MAVLINK_MSG_ID_MISSION_ITEM:
      {
        mavlink_mission_item_t mission_item;
        mavlink_msg_mission_item_decode(&msg, &mission_item);
        Serial.print(mission_item.mission_type);
      }
      break;

      case MAVLINK_MSG_ID_MISSION_REQUEST:
      {
        mavlink_mission_request_t mission_request;
        mavlink_msg_mission_request_decode(&msg, &mission_request);
        
        if(mission_request.target_system == mavlink_system.sysid && mission_request.target_component == mavlink_system.compid)
        {
          if(mission_request.mission_type < 3) 
          {
            mavlink_msg_mission_item_int_send_struct(MAVLINK_COMM_0, &missions[mission_request.mission_type].at(mission_request.seq));
          }
        }
      }
      break;

      case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
      {
        mavlink_mission_set_current_t mission_current;
        mavlink_msg_mission_set_current_decode(&msg, &mission_current);
        
        if(mission_current.target_system == mavlink_system.sysid && mission_current.target_component == mavlink_system.compid)
        {
          currentMissionItem = mission_current.seq;
          mavlink_msg_mission_current_send(MAVLINK_COMM_0, currentMissionItem);
        }
      }
      break;

      case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
      {
        mavlink_mission_request_list_t mission_list;
        mavlink_msg_mission_request_list_decode(&msg, &mission_list);
        
        if(mission_list.target_system == mavlink_system.sysid && mission_list.target_component == mavlink_system.compid)
        {
          if(mission_list.mission_type < 3) 
          {
            mavlink_msg_mission_count_send(MAVLINK_COMM_0, msg.sysid, msg.compid, missions[mission_list.mission_type].size(), mission_list.mission_type);
          }
        }
      }
      break;

      case MAVLINK_MSG_ID_MISSION_COUNT:
      {
        mavlink_mission_count_t mission_count;
        mavlink_msg_mission_count_decode(&msg, &mission_count);
        
        if(mission_count.target_system == mavlink_system.sysid && mission_count.target_component == mavlink_system.compid)
        {
          if(mission_count.mission_type < 3) 
          {
            itemsToAdd = mission_count.count;
            missions[mission_count.mission_type].clear();
            mavlink_msg_mission_request_int_send(MAVLINK_COMM_0, msg.sysid, msg.compid, 0, mission_count.mission_type);
          }
        }
      }
      break;
      
      case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
      {
        mavlink_mission_clear_all_t mission_clear_all;
        mavlink_msg_mission_clear_all_decode(&msg, &mission_clear_all);
        
        if(mission_clear_all.target_system == mavlink_system.sysid && mission_clear_all.target_component == mavlink_system.compid)
        {
          if(mission_clear_all.mission_type < 3) 
          {
            itemsToAdd = 0;
            missions[mission_clear_all.mission_type].clear();
            mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg.sysid, msg.compid, 0, mission_clear_all.mission_type);
          }
          else
          {
            itemsToAdd = 0;
            for(int i = 0; i < 4; i++) missions[i].clear();
            mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg.sysid, msg.compid, 0, mission_clear_all.mission_type);
          }
          
        }
      }
      break;

      case MAVLINK_MSG_ID_MANUAL_CONTROL:
      {
        mavlink_manual_control_t manual_control;
        mavlink_msg_manual_control_decode(&msg, &manual_control);
        
        if(manual_control.target == mavlink_system.sysid)
        {
          // Drive rover
        }
      }
      break;

      case MAVLINK_MSG_ID_MISSION_ITEM_INT:
      {
        mavlink_mission_item_int_t mission_item;
        mavlink_msg_mission_item_int_decode(&msg, &mission_item);
        
        if(mission_item.target_system == mavlink_system.sysid && mission_item.target_component == mavlink_system.compid)
        {
          if(mission_item.mission_type < 3) 
          {
            if(mission_item.current) currentMissionItem = mission_item.seq;
            
            if(mission_item.seq < itemsToAdd - 1)
            {
              // Add mission item and request next
              missions[mission_item.mission_type].emplace_back(mission_item);
              mavlink_msg_mission_request_int_send(MAVLINK_COMM_0, msg.sysid, msg.compid, mission_item.seq + 1, mission_item.mission_type);
            }
            else
            {
              // Add last mission item
              itemsToAdd = 0;
              missions[mission_item.mission_type].emplace_back(mission_item);
              mavlink_msg_mission_ack_send(MAVLINK_COMM_0, msg.sysid, msg.compid, 0, mission_item.mission_type);
            }
            
          }
        }
      }
      break;

      case MAVLINK_MSG_ID_COMMAND_LONG:
      {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(&msg, &cmd);

        if(cmd.target_system == mavlink_system.sysid && cmd.target_component == mavlink_system.compid)
        {
          switch (cmd.command)
          {
          case MAV_CMD_REQUEST_PROTOCOL_VERSION:
            mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, MAVLINK_VERSION, 0, msg.sysid, 0);
            break;

          case MAV_CMD_COMPONENT_ARM_DISARM:
          {
            if (cmd.param1) system_mode |= 128; // Arm
            if (!cmd.param1) system_mode &= 127; // Disarm
            mavlink_msg_command_ack_send(MAVLINK_COMM_0, cmd.command, MAV_RESULT_ACCEPTED, cmd.param1, 0, msg.sysid, 0);
          }
          break;

          default:
          {
            Serial.print("command: ");
            Serial.println(cmd.command);
          }
          break;

          }
        }
      }
      break;

      default:
        Serial.print("message ID: ");
        Serial.println(msg.msgid);
        break;
      }
    }
  }
}

void mavlinkTask(void *ctx)
{
  // Start mavlink serial port
  Serial1.begin(115200);

  mavlink_system.sysid = 1;
  mavlink_system.compid = MAV_COMP_ID_AUTOPILOT1;
  system_mode = MAV_MODE_GUIDED_DISARMED;
  system_state = MAV_STATE_ACTIVE;

  while (1)
  {
    mavlinkRecive();

    mavlink_msg_heartbeat_send(MAVLINK_COMM_0, system_type, autopilot_type, system_mode, custom_mode, system_state);
    //mavlink_msg_sys_status_send(MAVLINK_COMM_0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

    if (xSemaphoreTake(imuSemaphore, (TickType_t)portMAX_DELAY) == pdTRUE)
    {
      mavlink_msg_attitude_send(MAVLINK_COMM_0, millis(), pitch, roll, yaw, 0, 0, 0);
      mavlink_msg_highres_imu_send(MAVLINK_COMM_0, micros(), a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x * DEG_TO_RAD, g.gyro.y * DEG_TO_RAD, g.gyro.z * DEG_TO_RAD, m.magnetic.x, m.magnetic.y, m.magnetic.z, 0, 0, 0, temp.temperature, 0xFFFF);

      xSemaphoreGive(imuSemaphore);
    }

    if (xSemaphoreTake(gpsSemaphore, (TickType_t)portMAX_DELAY) == pdTRUE)
    {
      if(gps.location.isUpdated()) mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0, gps.time.value(), 3, gps.location.lat() * 1E7, gps.location.lng() * 1E7, gps.altitude.meters() * 1000, gps.hdop.value(), UINT16_MAX, gps.speed.mps() * 100, gps.course.deg() * 100, gps.satellites.value(), UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX);

      xSemaphoreGive(gpsSemaphore);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
