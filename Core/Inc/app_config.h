#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#define APP_DEBUG_ENABLE 0
#define APP_MODE_DEBUG_ENABLE 0
#define BUZZER_DEBUG_ENABLE 0
#define TRACKING_DEBUG_ENABLE 0
#define TRANSPORT_DEBUG_ENABLE 0
#define TRANSPORT_DEBUG_PERIOD_MS 500U
#define APP_HEALTH_LOOP_WARN_MS 20U

#define UWB_FOLLOW_TARGET_ID                    0x0000UL
/* Enable target filtering after the real tag ID has been measured. */
#define UWB_FOLLOW_FILTER_TARGET_ID             0U
#define UWB_FOLLOW_PROTOCOL_SIMPLE_ENABLE       1U
#define UWB_FOLLOW_PROTOCOL_GC_P2304_ENABLE     1U
#define UWB_FOLLOW_RX_BUFFER_SIZE               256U
#define UWB_FOLLOW_GC_MAX_PAYLOAD_LEN           95U
#define UWB_FOLLOW_DEBUG_ENABLE                 0U
#define UWB_FOLLOW_DEBUG_PERIOD_MS              1000U
#define UWB_FOLLOW_EXPECTED_PERIOD_MS           50U
#define UWB_FOLLOW_LOST_PERIOD_LIMIT            8U

#define FOLLOW_TARGET_DISTANCE_CM               100U
#define FOLLOW_DISTANCE_DEADBAND_CM             20U
#define FOLLOW_NEAR_STOP_CM                     60U
#define FOLLOW_LOST_TIMEOUT_MS                  (UWB_FOLLOW_EXPECTED_PERIOD_MS * UWB_FOLLOW_LOST_PERIOD_LIMIT)
#define FOLLOW_MIN_RSSI_DBM                     (-90)
#define FOLLOW_ANGLE_DEADBAND_DEG               10
#define FOLLOW_MAX_SPEED                        250
#define FOLLOW_MAX_TURN                         180
#define FOLLOW_TURN_GAIN                        4
#define FOLLOW_DISTANCE_GAIN                    3
#define FOLLOW_SPEED_STEP                       15
#define FOLLOW_DEBUG_ENABLE                     0U
#define FOLLOW_DEBUG_PERIOD_MS                  500U

#endif /* __APP_CONFIG_H */
