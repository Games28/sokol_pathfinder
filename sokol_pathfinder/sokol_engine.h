#pragma once
#ifndef SOKOL_ENGINE_CLASS_H
#define SOKOL_ENGINE_CLASS_H

#define SOKOL_IMPL
#include "sokol/sokol_app.h"  

//for memset & memcpy
#include <string>

//for snprintf
#include <cstdio>

class SokolEngine {
	static const int _num_keys=512;
	bool _keys_old[_num_keys],
		_keys_curr[_num_keys],
		_keys_new[_num_keys];

	static const int _num_buttons=256;
	bool _buttons_old[_num_buttons],
		_buttons_curr[_num_buttons],
		_buttons_new[_num_buttons];

	bool _mouse_moving_new=false;
	bool _mouse_moving=false;

	float _mouse_x_new=0;
	float _mouse_y_new=0;

	float _mouse_dx_new=0;
	float _mouse_dy_new=0;

protected:
	float mouse_x=0;
	float mouse_y=0;

	float mouse_dx=0;
	float mouse_dy=0;

	std::string app_title="Untitled";

public:
	virtual void userCreate()=0;

	virtual void userDestroy() {};

	virtual void userUpdate(float dt)=0;

	virtual void userRender()=0;

	void init() {
		std::memset(_keys_old, false, sizeof(bool)*_num_keys);
		std::memset(_keys_curr, false, sizeof(bool)*_num_keys);
		std::memset(_keys_new, false, sizeof(bool)*_num_keys);

		std::memset(_buttons_old, false, sizeof(bool)*_num_buttons);
		std::memset(_buttons_curr, false, sizeof(bool)*_num_buttons);
		std::memset(_buttons_new, false, sizeof(bool)*_num_buttons);

		userCreate();
	}

	void cleanup() {
		userDestroy();
	}

	//dont immediately send new key states.
	void input(const sapp_event* e) {
		switch(e->type) {
			case SAPP_EVENTTYPE_KEY_DOWN:
				_keys_new[e->key_code]=true;
				break;
			case SAPP_EVENTTYPE_KEY_UP:
				_keys_new[e->key_code]=false;
				break;
			case SAPP_EVENTTYPE_MOUSE_DOWN:
				_buttons_new[e->mouse_button]=true;
				break;
			case SAPP_EVENTTYPE_MOUSE_UP:
				_buttons_new[e->mouse_button]=false;
				break;
			case SAPP_EVENTTYPE_MOUSE_MOVE:
				_mouse_moving=true;

				//absolute invalid when locked
				if(!sapp_mouse_locked()) {
					_mouse_x_new=e->mouse_x;
					_mouse_y_new=e->mouse_y;
				}

				//delta always valid
				_mouse_dx_new=e->mouse_dx;
				_mouse_dy_new=e->mouse_dy;
				break;
		}
	}

	//fun little helpers
	struct EventState { bool pressed, held, released; };
	EventState getKey(const sapp_keycode& kc) const {
		bool curr=_keys_curr[kc], prev=_keys_old[kc];
		return {curr&&!prev, curr, !curr&&prev};
	}

	EventState getMouse(const int& mb) const {
		bool curr=_buttons_curr[mb], prev=_buttons_old[mb];
		return {curr&&!prev, curr, !curr&&prev};
	}

	void frame() {
		//update curr key values
		std::memcpy(_keys_curr, _keys_new, sizeof(bool)*_num_keys);

		//update curr button values
		std::memcpy(_buttons_curr, _buttons_new, sizeof(bool)*_num_buttons);

		//update mouse coords
		mouse_x=_mouse_x_new;
		mouse_y=_mouse_y_new;

		//hmm...
		if(_mouse_moving) {
			mouse_dx=_mouse_dx_new;
			mouse_dy=_mouse_dy_new;
		} else {
			mouse_dx=0;
			mouse_dy=0;
		}

		const float dt=sapp_frame_duration();

		userUpdate(dt);

		//update title with fps
		{
			char buf[256];
			std::snprintf(
				buf,
				sizeof(buf),
				"%s - FPS: %d",
				app_title.c_str(),
				int(1/dt)
			);
			sapp_set_window_title(buf);
		}

		userRender();

		_mouse_moving=false;

		//update prev key values
		std::memcpy(_keys_old, _keys_curr, sizeof(bool)*_num_keys);

		//update prev key values
		std::memcpy(_buttons_old, _buttons_curr, sizeof(bool)*_num_buttons);
	}
};
#endif