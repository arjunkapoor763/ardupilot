/*
 * RFMService.h
 *
 *  Created on: 08-Aug-2019
 *      Author: zomato
 */

#ifndef LIBRARIES_RFMSERVICE_RFMSERVICE_H_
#define LIBRARIES_RFMSERVICE_RFMSERVICE_H_

#include <AP_HAL/AP_HAL.h>
#include <string.h>
#include <time.h>
#include <AP_Math/Ap_Math.h>
#include <float.h>
#include <stdint.h>
#include <limits.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AC_Fence/AC_PolyFence_loader.h>
#include <GCS_MAVLink/GCS.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_InertialNav/AP_InertialNav.h>

//for time boundaries
#define TYPE_BITS(type) (sizeof (type)* CHAR_BIT)
#define TYPE_FLOATING(type) ((type) 0.5 == 0.5)
#define TYPE_SIGNED(type) ((type) -1 < 0)
class RFMService
{
private:
	AC_PolyFence_loader _poly_loader;
	uint8_t   fence_count;
	Vector2l return_point;
	double ret_lat,ret_lng;
	uint8_t lcl_count;

public:

	//constructor for this class
	RFMService();

	//handshake between sbc and fc
	struct sbc_hb
	{
		uint8_t PA_received_status;
		uint8_t PA_verification_status;
		uint8_t PA_sent_to_fc_status;
	};

	struct date_time
	{
		double start_time;
		double end_time;
	};

	//variable of structure
	struct sbc_hb sbc_hb;
	struct date_time dt;

	//member functions
	//for difference between two times in UTC
	static double subtract(time_t time1, time_t time0);
	double diff_time(time_t time1, time_t time0);
	bool get_datetime_restriction();
	bool get_internal_id(char uin[40]);


	void handle_pa_geofence_points(mavlink_channel_t chan, mavlink_message_t *msg, AC_Fence &fence);

	//data-member
	double curr_time_gps;
	bool geo_fence_return_set;
	int8_t total_vertices;

	bool sbc_alive;
	bool sbc_alive_control;
	bool sbc_ready_to_send_PA;
};

#endif /* LIBRARIES_RFMSERVICE_RFMSERVICE_H_ */
