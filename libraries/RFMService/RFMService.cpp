/*
 * RFMService.cpp
 *
 *  Created on: 08-Aug-2019
 *      Author: zomato
 */
#include "RFMService.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

RFMService::RFMService()
{
	fence_count=0;
	return_point.x=0;
	return_point.y=0;
	ret_lat=0;
	ret_lng=0;
	lcl_count=1;
	curr_time_gps=0;
	geo_fence_return_set=false;
	total_vertices=0;
	sbc_alive=false;
	sbc_alive_control=false;
	sbc_ready_to_send_PA=false;

}

/*this is the handling of polygon geofence packet coming from SBC and saving it to eeprom.
Also if GCS asks for a polygon geofence, this is where it sends back polygon geofence.*/
void RFMService::handle_pa_geofence_points(GCS_MAVLINK &link, mavlink_message_t *msg, AC_Fence &fence)
{
	// exit immediately if null message
	if (msg == nullptr) {
		return;
	}
	uint8_t result = MAV_RESULT_FAILED;

	switch (msg->msgid)
	{
	//	    case MAVLINK_MSG_ID_FENCE_POINT:
	case MAVLINK_MSG_ID_PA_GEOFENCE:
	{
		/* sanity check!
		 * SBC should be ready with heartbeat as 111,means ready to sent PA arteface!
		 */
		if(!sbc_ready_to_send_PA)
		{
			return;
		}
		mavlink_pa_geofence_t packet;
		mavlink_msg_pa_geofence_decode(msg,&packet);

		/* sanity check!
		 * id should never be 0, as 0 is assigned to
		 * Geo-Fence return point, nothing should overwrite it.
		 */
		if(packet.waypoint_id == 0)
		{
			return;
		}

		if (!check_latlng(packet.Latitude,packet.Longitude))
		{
			gcs().send_text(MAV_SEVERITY_WARNING, "Invalid fence point, lat or lng too large");

		}
		else
		{
			Vector2l point;

			static bool once =false;

			//calculate the total vertices.
			total_vertices=(packet.total_count-1);
			fence_count=(packet.total_count+1);

			//this is the setting of geo-fence return location as cendroid of polygon.
			if(once == false && lcl_count<=total_vertices)
			{
				ret_lat = ret_lat + packet.Latitude;
				ret_lng = ret_lng + packet.Longitude;

				if(lcl_count++ == total_vertices)
				{
					ret_lat=(ret_lat / total_vertices);
					ret_lng=(ret_lng / total_vertices);
					return_point.x=ret_lat*1.0e7f;
					return_point.y=ret_lng*1.0e7f;
					lcl_count=1;
					total_vertices=0;
					_poly_loader.save_point_to_eeprom(0, return_point);
					return_point.x=0;
					return_point.y=0;
					ret_lat=0;
					ret_lng=0;
					once = true;
					geo_fence_return_set=true; //currently not being used anywhere!
				}
				geo_fence_return_set=false;
			}
			if(packet.Latitude == 12.934158 && packet.Longitude ==77.609316)
			{
				gcs().send_text(MAV_SEVERITY_CRITICAL, "packet value ok");
			}

			point.x = packet.Latitude*1.0e7f;
			point.y = packet.Longitude*1.0e7f;

			if(packet.waypoint_id == packet.total_count)
			{
				once = false;
			}

			if (!_poly_loader.save_point_to_eeprom(packet.waypoint_id, point))
			{
				gcs().send_text(MAV_SEVERITY_WARNING,"Failed to save polygon point, too many points?");
			}
			else
			{
				// trigger reload of points //this is important to reload boundary from eeprom!
				fence._boundary_loaded = false;
			}

			gcs().send_text(MAV_SEVERITY_CRITICAL,"Points receiving");

			//this is the ack/nak to every polygon point coming from sbc
			result=MAV_RESULT_ACCEPTED;
		}
		//sending ack for received point from SBC
		mavlink_msg_message_ack_send(link.get_chan(),result,MAVLINK_MSG_ID_PA_GEOFENCE);
		break;
	}

	// download geofence from GCS!
	// send a fence point to GCS
	case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
	{

		mavlink_fence_fetch_point_t packet;
		mavlink_msg_fence_fetch_point_decode(msg, &packet);
		// attempt to retrieve from eeprom
		Vector2l point;
		if (_poly_loader.load_point_from_eeprom(packet.idx, point))
		{
			//			setting fence variables while downloading geofence to GCS!
			fence._total.set_and_save(fence_count); //setting the total points to what is coming from sbce packet.
			fence._enabled.set_and_save(1);
			fence._enabled_fences.set_and_save(AC_FENCE_TYPE_CIRCLE_POLYGON);
			fence._action.set_and_save(0);
			mavlink_msg_fence_point_send_buf(msg, link.get_chan(), msg->sysid, msg->compid, packet.idx, fence._total, point.x*1.0e-7f, point.y*1.0e-7f);
			gcs().send_text(MAV_SEVERITY_CRITICAL, "In fetch Point send point");
		}
		else
		{
			gcs().send_text(MAV_SEVERITY_WARNING, "Bad fence point");
		}
		break;
	}

	default:
		// do nothing
		break;
	}

}

double RFMService::subtract(time_t time1, time_t time0)
{
	if (! TYPE_SIGNED (time_t))
		return time1 - time0;
	else
	{
		/* Optimize the common special cases where time_t
	         can be converted to uintmax_t without losing information.  */
		uintmax_t dt = (uintmax_t) time1 - (uintmax_t) time0;
		double delta = dt;
		if (UINTMAX_MAX / 2 < INTMAX_MAX)
		{
			/* This is a rare host where uintmax_t has padding bits, and possibly
	             information was lost when converting time_t to uintmax_t.
	             Check for overflow by comparing dt/2 to (time1/2 - time0/2).
	             Overflow occurred if they differ by more than a small slop.
	             Thanks to Clive D.W. Feather for detailed technical advice about
	             hosts with padding bits.
	             In the following code the "h" prefix means half.  By range
	             analysis, we have:
	                  -0.5 <= ht1 - 0.5*time1 <= 0.5
	                  -0.5 <= ht0 - 0.5*time0 <= 0.5
	                  -1.0 <= dht - 0.5*(time1 - time0) <= 1.0
	             If overflow has not occurred, we also have:
	                  -0.5 <= hdt - 0.5*(time1 - time0) <= 0
	                  -1.0 <= dht - hdt <= 1.5
	             and since dht - hdt is an integer, we also have:
	                  -1 <= dht - hdt <= 1
	             or equivalently:
	                  0 <= dht - hdt + 1 <= 2
	             In the above analysis, all the operators have their exact
	             mathematical semantics, not C semantics.  However, dht - hdt +
	             1 is unsigned in C, so it need not be compared to zero.  */
			uintmax_t hdt = dt / 2;
			time_t ht1 = time1 / 2;
			time_t ht0 = time0 / 2;
			time_t dht = ht1 - ht0;
			if (2 < dht - hdt + 1)
			{
				/* Repair delta overflow.
	                 The following expression contains a second rounding,
	                 so the result may not be the closest to the true answer.
	                 This problem occurs only with very large differences.
	                 It's too painful to fix this portably.  */
				delta = dt + 2.0L * (UINTMAX_MAX - UINTMAX_MAX / 2);
			}
		}
		return delta;
	}
}

/* Return the difference between TIME1 and TIME0.  */
double RFMService:: diff_time (time_t time1, time_t time0)
{
	/* Convert to double and then sub_tract if no double-rounding error could
     result.  */
	if (TYPE_BITS (time_t) <= DBL_MANT_DIG
			|| (TYPE_FLOATING (time_t) && sizeof (time_t) < sizeof (long double)))
		return (double) time1 - (double) time0;
	/* Likewise for long double.  */
	if (TYPE_BITS (time_t) <= LDBL_MANT_DIG || TYPE_FLOATING (time_t))
		return (long double) time1 - (long double) time0;
	/* Subtract the smaller integer from the larger, convert the difference to
     double, and then negate if needed.  */
	return time1 < time0 ? - subtract (time0, time1) : subtract (time1, time0);
}


/* This is a boolean function that returns true if current time lies in bound of start and end time|else false.
 * Time is needed in Epoch UTC format for comaparison.*/
bool RFMService::get_datetime_restriction(AP_GPS &Gps)
{
	bool date_time_verified=false;

	//getting the system clock in milliseconds converting it seconds,epcoh UTC
	static uint32_t currdtime_s;
//	currdtime_s=(hal.util->get_hw_rtc()/1000);
	currdtime_s=Gps.time_epoch_usec();
	currdtime_s=currdtime_s / 1000000U;

	//this variable is for the rest of the classes using it as logging etc.
	curr_time_gps=currdtime_s;

	//difference_C_S is the difference of current time and start time,should be +ve. This program subtracts 2nd from 1st argument.
	double difference_C_S= diff_time(currdtime_s,dt.start_time);
	//difference_C_E is the difference of current time and end time,should be -ve.
	double difference_C_E= diff_time(currdtime_s,dt.end_time);

	if (difference_C_S >=0 && difference_C_E <= 0)
	{
		date_time_verified=true;
	}
	return date_time_verified;
}

/*this function gets the internal_id from the chip id*/
bool RFMService::get_internal_id(char uin[40])
{
	char sysid[40];

	if (hal.util->get_system_id(sysid))
	{
		sysid[0]='z';
		sysid[1]='o';
		sysid[2]='m';
		sysid[3]='a';
		sysid[4]='t';
		sysid[5]='o';
		sysid[6]=' ';
		sysid[7]=' ';
		sysid[8]=' ';
		strncpy(uin, sysid,40);
		return true;
	}
	return false;
}
