#include <Arduino.h>

#include <ros.h>
#include <smart_home_msgs/ModeChange.h>
#include <smart_home_msgs/CountdownState.h>

int TLON = LOW;
int TLOFF = HIGH;

int PIN_RED = 10;
int PIN_YLW = 11;
int PIN_GRN = 12;

uint8_t current_mode_type = smart_home_msgs::ModeChange::FULL_OFF;

void set_lights(int r, int y, int g)
{
	digitalWrite(PIN_RED, r);
	digitalWrite(PIN_YLW, y);
	digitalWrite(PIN_GRN, g);
}

void mode_change_callback(const smart_home_msgs::ModeChange& msg)
{
	current_mode_type = msg.mode_type;
	if (current_mode_type == smart_home_msgs::ModeChange::FULL_ON)
	{
		set_lights(TLON, TLON, TLON);
	}
	else
	{
		set_lights(TLOFF, TLOFF, TLOFF);
	}
}

void countdown_state_callback(const smart_home_msgs::CountdownState& msg)
{
	if (current_mode_type == smart_home_msgs::ModeChange::MORNING_COUNTDOWN)
	{
		switch(msg.state)
		{
			case smart_home_msgs::CountdownState::TO_RED:
				set_lights(TLON, TLOFF, TLOFF);
				break;
			case smart_home_msgs::CountdownState::TO_YELLOW:
				set_lights(TLOFF, TLON, TLOFF);
				break;
			case smart_home_msgs::CountdownState::TO_GREEN:
				set_lights(TLOFF, TLOFF, TLON);
				break;
			case smart_home_msgs::CountdownState::TO_ALL:
				set_lights(TLON, TLON, TLON);
				break;
			default:
				set_lights(TLOFF, TLOFF, TLOFF);
				break;
		}
	}
}

ros::NodeHandle node_handle;
ros::Subscriber<smart_home_msgs::ModeChange> mode_change_sub(
	"smart_home/mode_change_chatter",
	&mode_change_callback
);
ros::Subscriber<smart_home_msgs::CountdownState> countdown_state_sub(
	"smart_home/countdown_state_chatter",
	&countdown_state_callback
);

void setup()
{
	pinMode(PIN_RED, OUTPUT);
	pinMode(PIN_YLW, OUTPUT);
	pinMode(PIN_GRN, OUTPUT);
	digitalWrite(PIN_RED, TLOFF);
	digitalWrite(PIN_YLW, TLOFF);
	digitalWrite(PIN_GRN, TLOFF);
	Serial.begin(57600);
	node_handle.initNode();
	node_handle.subscribe(mode_change_sub);
	node_handle.subscribe(countdown_state_sub);
}

void loop()
{
	node_handle.spinOnce();
	delay(1);
}
