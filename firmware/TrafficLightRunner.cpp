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

void mode_change_callback(const smart_home_msgs::ModeChange& msg)
{
	current_mode_type = msg.mode_type;
}

void countdown_state_callback(const smart_home_msgs::CountdownState& msg)
{
	if (current_mode_type == smart_home_msgs::ModeChange::MORNING_COUNTDOWN)
	{
		int r = TLOFF, y = TLOFF, g = TLOFF;
		switch(msg.state)
		{
			case smart_home_msgs::CountdownState::TO_RED:
				r = TLON; y = TLOFF; g = TLOFF;
				break;
			case smart_home_msgs::CountdownState::TO_YELLOW:
				r = TLOFF; y = TLON; g = TLOFF;
				break;
			case smart_home_msgs::CountdownState::TO_GREEN:
				r = TLOFF; y = TLOFF; g = TLON;
				break;
			case smart_home_msgs::CountdownState::TO_ALL:
				r = TLON; y = TLON; g = TLON;
				break;
		}
		digitalWrite(PIN_RED, r);
		digitalWrite(PIN_YLW, y);
		digitalWrite(PIN_GRN, g);
	}
}

ros::NodeHandle node_handle;
ros::Subscriber<smart_home_msgs::ModeChange> mode_change_sub(
	"smart_home_msgs/mode_change_chatter",
	&mode_change_callback
);
ros::Subscriber<smart_home_msgs::CountdownState> countdown_state_sub(
	"smart_home_msgs/countdown_state_chatter",
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
