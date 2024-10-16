// (c) scratchapixel - 2024
// Distributed under the terms of the CC BY-NC-ND 4.0 License.
// https://creativecommons.org/licenses/by-nc-nd/4.0/
// clang++ -std=c++23 -Wall -Wextra -luser32 -lgdi32 -o 3d-nav-controls.exe 3d-nav-controls.cc -O3

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdint>
#include <cassert>
#include <iostream>

namespace gfx {

struct Point {
    int x, y;
    constexpr Point() : x(0), y(0) {}
    constexpr Point(int x, int y) : x(x), y(y) {}
    constexpr Point operator-(const Point& pt) const { 
        return Point(x - pt.x, y - pt.y);
    }
};

}

template<typename T>
class Vec3 {
public:
    Vec3() : x(T(0)), y(T(0)), z(T(0)) {}
	Vec3(T xx) : x(T(xx)), y(T(xx)), z(T(xx)) {}
	Vec3(T xx, T yy, T zz) : x(T(xx)), y(T(yy)), z(T(zz)) {}
	
    constexpr Vec3<T> Normalized() const {
        T l = std::sqrt(x * x + y * y + z * z);
        if (l == 0) [[unlikely]] return Vec3(T(0));
        return Vec3(x / l, y / l, z / l);
    }
	constexpr Vec3<T>& Normalize() {
        T l = std::sqrt(x * x + y * y + z * z);
        if (l != 0) [[likely]] {
			x /= l;
			y /= l;
			z /= l;
		}
		return *this;
    }
    constexpr T Dot(const Vec3<T>& v) const {
        return x * v.x + y * v.y + z * v.z;
    }
    constexpr T operator^ (const Vec3<T>& v) const {
        return Dot(v);
    }
    constexpr Vec3<T> Cross(const Vec3<T>& v) const {
        return Vec3(
            y * v.z - z * v.y,
            z * v.x - x * v.z, 
            x * v.y - y * v.x);
    }
    constexpr Vec3<T> operator%(const Vec3<T>& v) const {
        return Cross(v);
    }
	constexpr Vec3<T> operator*(T r) const {
		return Vec3<T>(x * r, y * r, z * r);
	}
    friend constexpr Vec3<T> operator*(T r, const Vec3<T>& v) {
        return Vec3<T>(v.x * r, v.y * r, v.z * r);
    }
	constexpr Vec3<T> operator-(const Vec3<T>& v) const {
        return Vec3<T>(x - v.x, y - v.y, z - v.z);
    }
    constexpr Vec3<T> operator+(const Vec3<T>& v) const {
        return Vec3<T>(x + v.x, y + v.y, z + v.z);
    }
	constexpr Vec3<T>& operator+=(const Vec3<T>& v) {
        x += v.x, y += v.y, z += v.z;
		return *this;
    }
	friend std::ostream& operator<<(std::ostream& os, const Vec3<T>& v) {
		return os << v.x << " " << v.y << " " << v.z;
	}
    T x, y, z;
};

template<typename T>
class Matrix44 {
public:
	Matrix44() {
		x[0][0] = 1; x[0][1] = 0; x[0][2] = 0; x[0][3] = 0;
		x[1][0] = 0; x[1][1] = 1; x[1][2] = 0; x[1][3] = 0;
		x[2][0] = 0; x[2][1] = 0; x[2][2] = 1; x[2][3] = 0;
		x[3][0] = 0; x[3][1] = 0; x[3][2] = 0; x[3][3] = 1;
	}
	constexpr Matrix44 (
		T a, T b, T c, T d,
		T e, T f, T g, T h,
		T i, T j, T k, T l,
		T m, T n, T o, T p) {
		x[0][0] = a; x[0][1] = b; x[0][2] = c; x[0][3] = d;
		x[1][0] = e; x[1][1] = f; x[1][2] = g; x[1][3] = h;
		x[2][0] = i; x[2][1] = j; x[2][2] = k; x[2][3] = l;
		x[3][0] = m; x[3][1] = n; x[3][2] = o; x[3][3] = p;
	}
    constexpr T* operator[](size_t i) {
        return x[i];
    }
    constexpr const T* operator[](size_t i) const {
        return x[i];
    }
	constexpr void MultVecMatrix(const Vec3<T>& src, Vec3<T>& dst) const {
		T a, b, c, w;

		a = src.x * x[0][0] + src.y * x[1][0] + src.z * x[2][0] + x[3][0];
		b = src.x * x[0][1] + src.y * x[1][1] + src.z * x[2][1] + x[3][1];
		c = src.x * x[0][2] + src.y * x[1][2] + src.z * x[2][2] + x[3][2];
		w = src.x * x[0][3] + src.y * x[1][3] + src.z * x[2][3] + x[3][3];

		dst.x = a / w;
		dst.y = b / w;
		dst.z = c / w;
	}
	constexpr void MultDirMatrix(const Vec3<T>& src, Vec3<T>& dst) const {
        T a, b, c;

		a = src.x * x[0][0] + src.y * x[1][0] + src.z * x[2][0];
        b = src.x * x[0][1] + src.y * x[1][1] + src.z * x[2][1];
        c = src.x * x[0][2] + src.y * x[1][2] + src.z * x[2][2];

		dst.x = a, dst.y = b, dst.z = c;
    }
    T x[4][4];
};

template<typename T>
class Quat {
public:
	Quat() = default;
	constexpr Quat(T s, T i, T j, T k) 
		: r(s), v(i, j, k) {
	}
	constexpr Quat(T s, Vec3<T> d) 
		: r(s), v(d) {
	}
    constexpr Quat<T>& SetAxisAngle(const Vec3<T>& axis, T radians) {
        v = axis.Normalized() * std::sin(radians / 2);
        r = std::cos(radians / 2);
        return *this;
    }
    constexpr Matrix44<T> ToMatrix44() const {
        return Matrix44<T>(
            1 - 2 * (v.y * v.y + v.z * v.z),
            2 * (v.x * v.y + v.z * r),
            2 * (v.z * v.x - v.y * r),
            0,
            2 * (v.x * v.y - v.z * r),
            1 - 2 * (v.z * v.z + v.x * v.x),
            2 * (v.y * v.z + v.x * r),
            0,
            2 * (v.z * v.x + v.y * r),
            2 * (v.y * v.z - v.x * r),
            1 - 2 * (v.y * v.y + v.x * v.x),
            0,
            0,
            0,
            0,
            1);
    }
	T r{1}; // The real part
    Vec3<T> v{0,0,0}; // The imaginary vector
};

// Quaternion multiplication
template<typename T>
constexpr inline Quat<T> operator* (const Quat<T>& q1, const Quat<T>& q2) {
    return Quat<T>(
        q1.r * q2.r - (q1.v ^ q2.v), q1.r * q2.v + q1.v * q2.r + q1.v % q2.v);
}

struct FreeCameraModel {
    enum class CameraMoveType : uint8_t {
        NONE,
        TUMBLE,
        TRACK,
        DOLLY,
    };
	FreeCameraModel() = default;
    CameraMoveType move_type{CameraMoveType::NONE};
    gfx::Point mouse_pos;
    float theta{0};
    float phi{0};
    Vec3<float> look_at{0};
    float distance_to_target{10};
    Quat<float> camera_rotation;
};

FreeCameraModel freecam_model;
Matrix44<float> rotation_mat;
constexpr float kRotateAmplitude = 0.01;
constexpr float kPanAmplitude = 0.01;
constexpr float kScrollAmplitude = 0.1;

Matrix44<float> cam_to_world;

const Vec3<float> points[8] = {
	{-0.5, -0.5,  0.5},
	{ 0.5, -0.5,  0.5},
	{-0.5,  0.5,  0.5},
	{ 0.5,  0.5,  0.5},
	{-0.5,  0.5, -0.5},
	{ 0.5,  0.5, -0.5},
	{-0.5, -0.5, -0.5},
	{ 0.5, -0.5, -0.5},
};

const uint32_t tri_vertex_indices[36] = {
	0, 1, 2, 2, 1, 3, 2, 3, 4, 
	4, 3, 5, 4, 5, 6, 6, 5, 7, 
	6, 7, 0, 0, 7, 1, 1, 7, 3, 
	3, 7, 5, 6, 0, 4, 4, 0, 2
};

enum EventType {
	ET_UNKNOWN = 0,
	ET_MOUSE_PRESSED,
	ET_MOUSE_RELEASED,
};

enum EventFlags {
	EF_NONE					= 0,
	EF_SHIFT_DOWN			= 1 << 0,
	EF_CONTROL_DOWN			= 1 << 1,
	EF_ALT_DOWN				= 1 << 2,
	EF_LEFT_BUTTON_DOWN		= 1 << 3,
	EF_MIDDLE_BUTTON_DOWN	= 1 << 4,
	EF_RIGHT_BUTTON_DOWN	= 1 << 5
};

void OnMousePressed(int flags, gfx::Point location) {
	freecam_model.mouse_pos = location;
	if (flags & EF_ALT_DOWN) {
		freecam_model.move_type =
			(flags & EF_LEFT_BUTTON_DOWN) ? FreeCameraModel::CameraMoveType::TUMBLE :
			(flags & EF_MIDDLE_BUTTON_DOWN) ? FreeCameraModel::CameraMoveType::TRACK :
			(flags & EF_RIGHT_BUTTON_DOWN) ? FreeCameraModel::CameraMoveType::DOLLY :
			(assert(false), FreeCameraModel::CameraMoveType::NONE);
	}
}

void OnMouseReleased() {
	freecam_model.move_type = FreeCameraModel::CameraMoveType::NONE;
}

void UpdateCameraRotation() {
    Quat<float> q1; q1.SetAxisAngle(Vec3<float>(1,0,0), freecam_model.phi);
    Quat<float> q2; q2.SetAxisAngle(Vec3<float>(0,1,0), freecam_model.theta);
    freecam_model.camera_rotation = q2 * q1;
    rotation_mat = freecam_model.camera_rotation.ToMatrix44();
}

void SetCameraMatrices() {
    Vec3<float> camera_orient;
    rotation_mat.MultDirMatrix(Vec3<float>(0,0,1), camera_orient);
    Vec3<float> camera_position = freecam_model.look_at +
        freecam_model.distance_to_target * camera_orient;
    cam_to_world = rotation_mat;
    cam_to_world[3][0] = camera_position.x;
    cam_to_world[3][1] = camera_position.y;
    cam_to_world[3][2] = camera_position.z;
}

void OnMouseWheel(int scroll_amount) {
    freecam_model.distance_to_target -= scroll_amount * kScrollAmplitude;
    SetCameraMatrices();
}

void OnMouseMoved(const gfx::Point& location) {
    gfx::Point delta = location - freecam_model.mouse_pos;
    freecam_model.mouse_pos = location;
    if (freecam_model.move_type == FreeCameraModel::CameraMoveType::NONE)
        return;
    switch (freecam_model.move_type) {
        case FreeCameraModel::CameraMoveType::TUMBLE: {
            freecam_model.theta -= delta.x * kRotateAmplitude;
            freecam_model.phi -= delta.y * kRotateAmplitude;
            UpdateCameraRotation();
		} break;
        case FreeCameraModel::CameraMoveType::DOLLY:
            OnMouseWheel(delta.x + delta.y);
            return;
        case FreeCameraModel::CameraMoveType::TRACK: {
				Vec3<float> target_offset;
				rotation_mat.MultDirMatrix(
					Vec3<float>(-kPanAmplitude * delta.x, kPanAmplitude * delta.y, 0),
					target_offset);
				freecam_model.look_at += target_offset;
			}
            break;
        default:
            break;
	}
	SetCameraMatrices();
}

// Rendering

#include <chrono>
#include <fstream>

constexpr uint32_t width = 640;
constexpr uint32_t height = 480;

constexpr float super_far = 1.e6;

float angle = 50.f;

#define UNICODE
#define NOMINMAX
#include <windows.h>
#include <windowsx.h> // GET_X_LPARAM

const wchar_t* CLASSNAME = L"myapp_window";
HWND hwnd;
HDC hdcBuffer;
void* pvBits; // Pointer to the bitmap's pixel bits
HBITMAP hbmBuffer;

template<typename T>
T DegreesToRadians(const T& degrees) {
	return M_PI * degrees / T(180);
}

struct Hit {
	float t{super_far};
	float u, v;
	Vec3<float> Ng;
};

inline float Xorf(const float x, const float y) {
    std::uint32_t ix, iy;

    std::memcpy(&ix, &x, sizeof(float));
    std::memcpy(&iy, &y, sizeof(float));

    std::uint32_t resultInt = ix ^ iy;

    float result;
    std::memcpy(&result, &resultInt, sizeof(float));

    return result;
}

void Intersect(const Vec3<float>& ray_orig, 
			   const Vec3<float>& ray_dir,
			   const Vec3<float>& p0, 
			   const Vec3<float>& p1, 
			   const Vec3<float>& p2, 
			   Hit& hit) {
	const float ray_near = 0.1;
	const Vec3<float> e1 = p0 - p1;
	const Vec3<float> e2 = p2 - p0;
	const Vec3<float> Ng = e1.Cross(e2);
	
	const Vec3<float> C = p0 - ray_orig;
	const Vec3<float> R = ray_dir.Cross(C);
	const float det = Ng.Dot(ray_dir);
	const float abs_det = std::abs(det);
	const float sign_det = std::copysign(0.f, det);
	if (det == 0) [[unlikely]] return;

	const float U = Xorf(R.Dot(e2), sign_det);
	if (U < 0) [[likely]] return;
	
	const float V = Xorf(R.Dot(e1), sign_det);
	if (V < 0) [[likely]] return;

	const float W = abs_det - U - V;
	if (W < 0) [[likely]] return;

	const float T = Xorf(Ng.Dot(C), sign_det); 
	if (T < abs_det * ray_near || abs_det * hit.t < T) [[unlikely]] return;

	const float rcp_abs_det = 1.f / abs_det;
	hit.u = U * rcp_abs_det;
	hit.v = V * rcp_abs_det;
	hit.t = T * rcp_abs_det;
	hit.Ng = Ng.Normalized();
}

float fps;

void Render() {
	auto start = std::chrono::steady_clock::now();
	float aspect_ratio = width / static_cast<float>(height);
	float scale = std::tan(DegreesToRadians(0.5f * angle));
	Vec3<float> ray_orig;
	cam_to_world.MultVecMatrix(Vec3<float>(0,0,0), ray_orig);
	char* pixel = (char*)pvBits;
	memset(pixel, 0x0, width * height * 3);
	for (uint32_t j = 0; j < height; ++j) {
		float y = (1 - 2 * (j + 0.5f) / static_cast<float>(height)) * scale * 1 / aspect_ratio;
		for (uint32_t i = 0; i < width; ++i) {
			float x = (2 * (i + 0.5f) / static_cast<float>(width) - 1) * scale;
			Vec3<float> ray_dir(x, y, -1);
			ray_dir.Normalize();
			cam_to_world.MultDirMatrix(ray_dir, ray_dir);
			float t = super_far;
			for (size_t n = 0, ni = 0; n < 12; n++, ni += 3) {
				Hit hit;
				const Vec3<float>& v0 = points[tri_vertex_indices[ni]];
				const Vec3<float>& v1 = points[tri_vertex_indices[ni + 1]];
				const Vec3<float>& v2 = points[tri_vertex_indices[ni + 2]];
				Intersect(ray_orig, ray_dir, v0, v1, v2, hit);
				if (hit.t < t) {
					t = hit.t;
					char color = static_cast<char>(255 * std::max(0.f, ray_dir.Dot(hit.Ng)));
					pixel[(i + j * width) * 3] = 
					pixel[(i + j * width) * 3 + 1 ] =
					pixel[(i + j * width) * 3 + 2 ] = color;
				}
			}
		}
	}
	auto end = std::chrono::steady_clock::now();
	fps = 1000.f / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	InvalidateRect(hwnd, NULL, TRUE);
	UpdateWindow(hwnd);
}

// Windows related stuff

LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
	switch(msg) {
		case WM_CLOSE:
			DeleteObject(hbmBuffer);
			DestroyWindow(hWnd);
			break;
		case WM_CREATE:
			//Render();
			break;
		case WM_DESTROY:
			PostQuitMessage(0);
			break;
		case WM_LBUTTONUP:
		case WM_MBUTTONUP:
		case WM_RBUTTONUP:
			OnMouseReleased();
			InvalidateRect(hwnd, NULL, TRUE);
			UpdateWindow(hwnd);
			break;
		case WM_LBUTTONDOWN:
		case WM_MBUTTONDOWN:
		case WM_RBUTTONDOWN: {
				gfx::Point location(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));

				unsigned int flags = 0;
				if (GetKeyState(VK_SHIFT) & 0x8000) flags |= EF_SHIFT_DOWN;
				if (GetKeyState(VK_CONTROL) & 0x8000) flags |= EF_CONTROL_DOWN;
				if (GetKeyState(VK_MENU) & 0x8000) flags |= EF_ALT_DOWN; // VK_MENU is the Alt key
				if (wParam & MK_LBUTTON) flags |= EF_LEFT_BUTTON_DOWN;
				if (wParam & MK_MBUTTON) flags |= EF_MIDDLE_BUTTON_DOWN;
				if (wParam & MK_RBUTTON) flags |= EF_RIGHT_BUTTON_DOWN;

				OnMousePressed(flags, location);
				Render();
			}
			break;
		case WM_MOUSEMOVE: {
				int xpos = GET_X_LPARAM(lParam);
				int ypos = GET_Y_LPARAM(lParam);
				OnMouseMoved(gfx::Point(xpos, ypos));
				Render();
			}
			break;
		case WM_MOUSEWHEEL: {
				int delta = GET_WHEEL_DELTA_WPARAM(wParam) / WHEEL_DELTA;
				OnMouseWheel(delta);
				Render();
			} 
			break;
		case WM_ERASEBKGND:
			return 1; // Indicate that background erase is handled
		case WM_PAINT: {
				const std::wstring mode[4] = { L"None", 
											   L"Tumble (ALT+LMB)", 
											   L"Track (Alt+MMB)", 
											   L"Dolly (ALT+RMB/Wheel)"};
				PAINTSTRUCT ps;
				HDC hdcWindow = BeginPaint(hwnd, &ps);
				BitBlt(hdcWindow, 0, 0, width, height, hdcBuffer, 0, 0, SRCCOPY);
				std::wstring text = L"fps: " + std::to_wstring(fps);
				SetTextColor(hdcWindow, RGB(255, 255, 255)); // White text
				SetBkMode(hdcWindow, TRANSPARENT);
				TextOut(hdcWindow, 10, 10, text.c_str(), text.length());
				TextOut(hdcWindow, 10, 28, mode[(int)freecam_model.move_type].c_str(), 
					mode[(int)freecam_model.move_type].length());
			} break;
		default:
			return DefWindowProc(hWnd, msg, wParam, lParam);
	}
	return 0;
}

void CreateAndRegisterWindow(HINSTANCE hInstance) {
	WNDCLASSEX wc = {};
	wc.cbSize = sizeof(WNDCLASSEX);
	wc.lpfnWndProc = WndProc;
	wc.hInstance = hInstance;
	wc.lpszClassName = CLASSNAME;
	wc.hCursor = LoadCursor(nullptr, IDC_ARROW); // Set the default arrow cursor
	wc.hIcon = LoadIcon(hInstance, IDI_APPLICATION); // Load the default application icon
	wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wc.lpszMenuName = nullptr;
	wc.hIconSm = LoadIcon(hInstance, IDI_APPLICATION); // Load the small icon for the application

	if (!RegisterClassEx(&wc)) {
		MessageBox(nullptr, L"Window Registration Failed", L"Error",
			MB_ICONEXCLAMATION | MB_OK);
	}

	hwnd = CreateWindowEx(
		WS_EX_CLIENTEDGE,
		CLASSNAME,
		L"3D Navigation Controls",
		WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME & ~WS_MAXIMIZEBOX, // non-resizable
		CW_USEDEFAULT, CW_USEDEFAULT, width, height,
		nullptr, nullptr, hInstance, nullptr);

	if (hwnd == nullptr) {
		MessageBox(nullptr, L"Window Creation Failed", L"Error",
			MB_ICONEXCLAMATION | MB_OK);
	}

	HDC hdcScreen = GetDC(hwnd); // Obtain the screen/device context
	hdcBuffer = CreateCompatibleDC(hdcScreen); // Create a compatible device context for off-screen drawing

	BITMAPINFO bmi;
	ZeroMemory(&bmi, sizeof(bmi)); // Ensure the structure is initially empty
	bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	bmi.bmiHeader.biWidth = width; // Specify the width of the bitmap
	bmi.bmiHeader.biHeight = -height; // Negative height for a top-down DIB
	bmi.bmiHeader.biPlanes = 1;
	bmi.bmiHeader.biBitCount = 24; // 24 bits per pixel (RGB)
	bmi.bmiHeader.biCompression = BI_RGB; // No compression

	hbmBuffer = CreateDIBSection(hdcBuffer, &bmi, DIB_RGB_COLORS, &pvBits, NULL, 0);
	SelectObject(hdcBuffer, hbmBuffer);
	ReleaseDC(hwnd, hdcScreen);

	ShowWindow(hwnd, SW_SHOWDEFAULT); // or use WS_VISIBLE but more control with this option
	Render();
}

int main() {
	HINSTANCE hInstance = GetModuleHandle(NULL);

	freecam_model.look_at = Vec3<float>(0); //points[0];

	UpdateCameraRotation();
	SetCameraMatrices();

	CreateAndRegisterWindow(hInstance);

	MSG msg;
	while (1) {
		while(PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE) != 0) {
			TranslateMessage(&msg);
			DispatchMessage(&msg);
			if (msg.message == WM_QUIT) {
				break;
			}
		}
		if (msg.message == WM_QUIT)
			break;
	}
    return 0;
}
