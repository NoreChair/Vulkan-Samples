#include "forward_plus.h"

forward_plus::forward_plus() 
{

}

forward_plus::~forward_plus()
{
}


void forward_plus::input_event(const vkb::InputEvent &input_event)
{
	Application::input_event(input_event);

	bool gui_captures_event = false;

	if (gui)
	{
		gui_captures_event = gui->input_event(input_event);
	}

	if (!gui_captures_event)
	{
		if (input_event.get_source() == vkb::EventSource::Mouse)
		{
			const auto &mouse_button = static_cast<const vkb::MouseButtonInputEvent &>(input_event);

			handle_mouse_move(static_cast<int32_t>(mouse_button.get_pos_x()), static_cast<int32_t>(mouse_button.get_pos_y()));

			if (mouse_button.get_action() == vkb::MouseAction::Down)
			{
				switch (mouse_button.get_button())
				{
				case vkb::MouseButton::Left:
					mouse_buttons.left = true;
					break;
				case vkb::MouseButton::Right:
					mouse_buttons.right = true;
					break;
				case vkb::MouseButton::Middle:
					mouse_buttons.middle = true;
					break;
				default:
					break;
				}
			}
			else if (mouse_button.get_action() == vkb::MouseAction::Up)
			{
				switch (mouse_button.get_button())
				{
				case vkb::MouseButton::Left:
					mouse_buttons.left = false;
					break;
				case vkb::MouseButton::Right:
					mouse_buttons.right = false;
					break;
				case vkb::MouseButton::Middle:
					mouse_buttons.middle = false;
					break;
				default:
					break;
				}
			}
		}
		else if (input_event.get_source() == vkb::EventSource::Touchscreen)
		{
			const auto &touch_event = static_cast<const vkb::TouchInputEvent &>(input_event);

			if (touch_event.get_action() == vkb::TouchAction::Down)
			{
				touch_down = true;
				touch_pos.x = static_cast<int32_t>(touch_event.get_pos_x());
				touch_pos.y = static_cast<int32_t>(touch_event.get_pos_y());
				mouse_pos.x = touch_event.get_pos_x();
				mouse_pos.y = touch_event.get_pos_y();
				mouse_buttons.left = true;
			}
			else if (touch_event.get_action() == vkb::TouchAction::Up)
			{
				touch_pos.x = static_cast<int32_t>(touch_event.get_pos_x());
				touch_pos.y = static_cast<int32_t>(touch_event.get_pos_y());
				touch_timer = 0.0;
				touch_down = false;
				camera.keys.up = false;
				mouse_buttons.left = false;
			}
			else if (touch_event.get_action() == vkb::TouchAction::Move)
			{
				bool handled = false;
				if (gui)
				{
					ImGuiIO &io = ImGui::GetIO();
					handled = io.WantCaptureMouse;
				}
				if (!handled)
				{
					int32_t eventX = static_cast<int32_t>(touch_event.get_pos_x());
					int32_t eventY = static_cast<int32_t>(touch_event.get_pos_y());

					float deltaX = (float)(touch_pos.y - eventY) * rotation_speed * 0.5f;
					float deltaY = (float)(touch_pos.x - eventX) * rotation_speed * 0.5f;

					camera.rotate(glm::vec3(deltaX, 0.0f, 0.0f));
					camera.rotate(glm::vec3(0.0f, -deltaY, 0.0f));

					rotation.x += deltaX;
					rotation.y -= deltaY;

					touch_pos.x = eventX;
					touch_pos.y = eventY;
				}
			}
		}
		else if (input_event.get_source() == vkb::EventSource::Keyboard)
		{
			const auto &key_button = static_cast<const vkb::KeyInputEvent &>(input_event);

			if (key_button.get_action() == vkb::KeyAction::Down)
			{
				switch (key_button.get_code())
				{
				case vkb::KeyCode::W:
					camera.keys.up = true;
					break;
				case vkb::KeyCode::S:
					camera.keys.down = true;
					break;
				case vkb::KeyCode::A:
					camera.keys.left = true;
					break;
				case vkb::KeyCode::D:
					camera.keys.right = true;
					break;
				default:
					break;
				}
			}
			else if (key_button.get_action() == vkb::KeyAction::Up)
			{
				switch (key_button.get_code())
				{
				case vkb::KeyCode::W:
					camera.keys.up = false;
					break;
				case vkb::KeyCode::S:
					camera.keys.down = false;
					break;
				case vkb::KeyCode::A:
					camera.keys.left = false;
					break;
				case vkb::KeyCode::D:
					camera.keys.right = false;
					break;
				default:
					break;
				}
			}
		}
	}
}

void forward_plus::handle_mouse_move(int32_t x, int32_t y)
{
	int32_t dx = (int32_t)mouse_pos.x - x;
	int32_t dy = (int32_t)mouse_pos.y - y;

	bool handled = false;

	if (gui)
	{
		ImGuiIO &io = ImGui::GetIO();
		handled = io.WantCaptureMouse;
	}

	if (handled)
	{
		mouse_pos = glm::vec2((float)x, (float)y);
		return;
	}

	if (mouse_buttons.left)
	{
		rotation.x += dy * 1.25f * rotation_speed;
		rotation.y -= dx * 1.25f * rotation_speed;
		camera.rotate(glm::vec3(dy * camera.rotation_speed, -dx * camera.rotation_speed, 0.0f));
		view_updated = true;
	}
	if (mouse_buttons.right)
	{
		zoom += dy * .005f * zoom_speed;
		camera.translate(glm::vec3(-0.0f, 0.0f, dy * .005f * zoom_speed));
		view_updated = true;
	}
	if (mouse_buttons.middle)
	{
		camera_pos.x -= dx * 0.01f;
		camera_pos.y -= dy * 0.01f;
		camera.translate(glm::vec3(-dx * 0.01f, -dy * 0.01f, 0.0f));
		view_updated = true;
	}
	mouse_pos = glm::vec2((float)x, (float)y);
}
