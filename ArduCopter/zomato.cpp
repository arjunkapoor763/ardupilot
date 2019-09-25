/*
 * zomato.cpp
 *
 *  Created on: 22-Aug-2019
 *      Author: arjun
 */

#include"zomato.h"


/*This is time restriction service of rfm, it keeps on checking whether the current time lies between the limits of the time given in PA.
 * called at 10hz
 * Times are in EPOC
 */
void Copter::rfm_NPNT_restrictions()
{
	/*****************************Time Verification*****************************/

	//local counter used for controlling the sbc_alive variable values.
	static uint8_t lcl_count=0;

	//this is controlling the sbc_alive variable turning it false on receiving
	if(rfm.sbc_alive_control)
	{
		lcl_count=0;
		rfm.sbc_alive_control=false;
	}

	//waiting for 5 sec to turn sbc_alive false.
	if(!rfm.sbc_alive_control && lcl_count++ >=50)
	{
		lcl_count=0;
		rfm.sbc_alive=false;
	}

	//this ensures that nothing is to be done until artifact is not received.
	//	if(!rfm.sbc_ready_to_send_PA)
	//	{
	//		return;
	//	}

	static uint8_t temp = 0;
	//this is the function call to check the date/time restrictions.
	//hold for 300ms so that the background variable gets updated.
	if(temp++ > 3)
	{
		temp = 0;
		res_time_veri=rfm.get_datetime_restriction();
	}

	//this is the packet which is logging date time required fields.
	struct log_rfm_time_bound pkt =
	{
			LOG_PACKET_HEADER_INIT(LOG_RFM_TIME_MSG),
			current		 : rfm.curr_time_gps,
			start   	 : rfm.dt.start_time,
			end     	 : rfm.dt.end_time,
			result       : res_time_veri,
	};
	//writing a packet of time onto logs.
	DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

/*
 * This function sends the Internal id of 40 bytes for the UIN registration to SBC.
 * The Internal id consists of "AAxxx'followed by pixhawk hardware chip id.
 */
void Copter::send_internal_id(mavlink_channel_t chan)
{
	//sanity check whether sbc is responsing or not
	if(!rfm.sbc_alive)
	{
		return;
	}
	//sanity check if we have already sent internal_id and recieved internal_id_ack, return
	if(!internal_id_ack)
	{
		return;
	}
	static char internal_id[40];

	//calling internal id function and passing string that needs to be filled.
	if(rfm.get_internal_id(internal_id))
	{
		static uint8_t count=0;

		if(count++>20)
		{
			count=0;
			mavlink_msg_internal_id_send(chan,internal_id); //send UIN to SBC
		}
	}
}

/*
 * This function sends fence breach lat/lng, start_breach_time/ end_breach_time, to sbc for logging .
 */
void Copter::send_log_geofence_breach(mavlink_channel_t chan)
{
	static int8_t geo_breach = -1;
	static bool breach_started = false;
	static uint64_t start_breach_time=0,end_breach_time = 0;
	int32_t breach_lat = 0,breach_lng = 0;
	float geo_fence_breach_alt=0;

	static uint8_t timeCount = 0;

	//for getting the altitude AMSL
	const struct Location &loc = gps.location();

	geo_breach=copter.fence.get_breaches();

	if(timeCount++ >=10) //for sending @1hz as its MAVLink stream is being set at @10hz already.
	{
		if((geo_breach & AC_FENCE_TYPE_POLYGON) || (geo_breach & AC_FENCE_TYPE_CIRCLE_POLYGON))
		{
			if(breach_started == false)
			{
				AP::rtc().get_utc_usec(start_breach_time);
				start_breach_time=start_breach_time / 1000000U;
				start_breach_time=start_breach_time+19800; //converting to ist
				breach_started = true;
			}
			breach_lat=current_loc.lat;
			breach_lng=current_loc.lng;
			geo_fence_breach_alt=(loc.alt/100.0f);
			mavlink_msg_pa_geofence_breach_log_send(chan,breach_lat,breach_lng,start_breach_time,end_breach_time,geo_fence_breach_alt);

		}
		else
		{
			if(breach_started == true)
			{
				AP::rtc().get_utc_usec(end_breach_time);
				end_breach_time=end_breach_time / 1000000U;
				end_breach_time=end_breach_time+19800; //converting to ist

				breach_started = false;
				breach_lat=current_loc.lat;
				breach_lng=current_loc.lng;
				start_breach_time=0;
				geo_fence_breach_alt=(loc.alt/100.0f);
				mavlink_msg_pa_geofence_breach_log_send(chan,breach_lat,breach_lng,start_breach_time,end_breach_time,geo_fence_breach_alt);
				end_breach_time = 0;
			}
		}
		timeCount = 0;
	}
}
/*
 * This function sends Take off essential fields to sbc for logging .
 */
void Copter::send_log_takeoff(mavlink_channel_t chan)
{
	//sanity check if motors are not armed, return
	if(!motors->armed())
	{
		return;
	}
	//sanity check if we have already sent Takeoff and recieved its ack, return
	if(!takeoff_ack)
	{
		return;
	}
	int32_t takeoff_lat,takeoff_lng;
	uint64_t takeoff_timestamp;
	float takeoff_altitude;

	//we need to get clarification on this whether to use EKF origin or home(where it arms) as home location
	takeoff_lat=ahrs.get_home().lat;
	takeoff_lng=ahrs.get_home().lng;

	//the home alt is in cm,converting it to m
	takeoff_altitude=(ahrs.get_home().alt/100.0f);

	//get timestamp
	AP::rtc().get_utc_usec(takeoff_timestamp);
	takeoff_timestamp = takeoff_timestamp / 100000U;
	takeoff_timestamp=takeoff_timestamp+19800;//converting to ist

	mavlink_msg_pa_takeoff_log_send(chan,takeoff_lat,takeoff_lng,takeoff_altitude,takeoff_timestamp);

	gcs().send_text(MAV_SEVERITY_CRITICAL,"Takeoff Sent");

	//reset land ack so that we send land again on this current takeoff
	land_ack=true;
}

/*
 * This function sends Land essential fields to sbc for logging .
 */
void Copter::send_log_land(mavlink_channel_t chan)
{
	static bool temp=false;

	/*sanity check if we have already sent Land and recieved its ack, return
	 * we are also checking whether temp=true and reseting it for another arm disarm cycle if any.
	 */
	if(!land_ack && temp == true)
	{
		//temp=false means the one arm disarm cycle has been completed!
		temp=false;
		return;
	}

	//means the quad has been armed first and now if it disarms that would be the land location.
	if(motors->armed())
	{
		temp=true;
	}
	/*checking whether the motors=disarmed and
	 *temp=true means as it was armed before and now has been landed
	 */
	if(!motors->armed() && temp)
	{
		int32_t land_lat,land_lng;
		uint64_t land_timestamp;
		float land_altitude;
		const struct Location &loc = gps.location();

		land_lat=current_loc.lat;
		land_lng=current_loc.lng;

		//converting cm it to m
		land_altitude=(loc.alt/100.0f);

		//get time
		AP::rtc().get_utc_usec(land_timestamp);
		land_timestamp= land_timestamp / 1000000U;
		land_timestamp=land_timestamp+19800;

		mavlink_msg_pa_land_log_send(chan,land_lat,land_lng,land_altitude,land_timestamp);

		//reset takeoff ack so that we send takeoff again on the next arm and takeoff
		takeoff_ack=true;

		gcs().send_text(MAV_SEVERITY_CRITICAL,"Land Sent");
	}
	else
		return;
}

void Copter::send_time_breach(mavlink_channel_t chan)
{
	if(!res_time_veri)
	{
		static uint64_t start_time_breach=0,end_breach_time = 0;
		static uint8_t timeCount = 0;
		static bool armed_once=false;

		if(motors->armed())
		{

			if(timeCount++ >=10) //for sending @1hz as its MAVLink stream is being set at @10hz already.
			{
				armed_once=true;
				//new packet is being sent from send_time_breach to GCS telling the date/time is breached.

				//get time
				AP::rtc().get_utc_usec(start_time_breach);
				start_time_breach=start_time_breach / 1000000U;
				start_time_breach=start_time_breach+19800;
				end_breach_time=0;
				mavlink_msg_rfm_status_send(chan,start_time_breach,end_breach_time);
				timeCount=0;
			}
		}
		else
		{
			if(armed_once)
			{
				if(timeCount++ >=10) //for sending @1hz as its MAVLink stream is being set at @10hz already.
				{
					start_time_breach=0;
					AP::rtc().get_utc_usec(end_breach_time);
					end_breach_time=end_breach_time / 1000000U;
					end_breach_time=end_breach_time+19800;
					mavlink_msg_rfm_status_send(chan,start_time_breach,end_breach_time);
					timeCount=0;
					armed_once=false;
				}
			}
		}
	}
}




