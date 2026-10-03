// ------------------------------------------------- //
// ----------- Last-Touched : Aug 2026  ------------ //
// ------------------------------------------------- //

// -------------------- Libraries ------------------ //

#pragma comment(lib, "winmm") // for timeBeginPeriod
#pragma comment (lib, "Ws2_32.lib") // networking
#pragma comment(lib, "opengl32")
#pragma comment(lib, "dependencies/external/GLEW/glew32s") // opengl extensions
#pragma comment(lib, "dependencies/external/GLFW/glfw3") // window & input
#pragma comment(lib, "dependencies/external/OpenAL/OpenAL32.lib") //  audio

// --------------------- includes ------------------ //

#define _CRT_SECURE_NO_WARNINGS // because printf is "too dangerous"

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../external/stb_image.h"
#include "../external/stb_image_write.h"

#define GLEW_STATIC
#include "../external/GLEW\glew.h" // OpenGL functions
#include "../external/GLFW\glfw3.h"// window & input

#include "../external/OpenAL/al.h" // for audio
#include "../external/OpenAL/alc.h"

#include <winsock2.h> // rearranging these includes breaks everything; idk why
#include <ws2tcpip.h>
#include <Windows.h>

#include <fileapi.h>
#include <iostream>

// ------------------------------------------------- //
// --------------------- Helpers ------------------- //
// ------------------------------------------------- //

#define out(val) std::cout << ' ' << val << '\n'
#define stop std::cin.get()
#define print printf
#define printvec(vec) printf("%f %f %f\n", vec.x, vec.y, vec.z)
#define Alloc(type, count) (type *)calloc(count, sizeof(type))

typedef signed   char      int8, i8;
typedef signed   short     int16, i16;
typedef signed   int       int32, i32;
typedef signed   long long int64, i64;

typedef unsigned char      uint8, u8, byte;
typedef unsigned short     uint16, u16;
typedef unsigned int       uint32, u32, uint;
typedef unsigned long long uint64, u64;

typedef char string32[32];
typedef char string64[64];
typedef char string128[128];

struct bvec3 { union { struct { byte x, y, z; }; struct { byte r, g, b; }; }; };

// ------------------------------------------------- //
// ------------------- Mathematics ----------------- //
// ------------------------------------------------- //

#include "mathematics.h" // this is pretty much GLM for now

// ------------------------------------------------- //
// --------------------- Timers -------------------- // // might be broken idk
// ------------------------------------------------- //

// while the raw timestamp can be used for relative performence measurements,
// it does not necessarily correspond to any external measure of time
struct Timer
{
	LARGE_INTEGER win32_ticks_per_second;
	LARGE_INTEGER start_timestamp, end_timestamp;

	void init() { QueryPerformanceFrequency(&win32_ticks_per_second); }
	void start();
	int64 microseconds_elapsed();
	void print_microseconds(const char* label) { out(label << microseconds_elapsed() << " us"); };
	void print_milliseconds(const char* label) { out(label << microseconds_elapsed() << " ms"); };
};

void Timer::start()
{
	// Get the current time's timestamp
	QueryPerformanceCounter(&start_timestamp);
}

int64 Timer::microseconds_elapsed()
{
	QueryPerformanceCounter(&end_timestamp);

	LARGE_INTEGER elapsed_microseconds;
	elapsed_microseconds.QuadPart = end_timestamp.QuadPart - start_timestamp.QuadPart;

	// We now have the elapsed number of ticks, along with the
	// number of ticks-per-second. We use these values
	// to convert to the number of elapsed microseconds.
	// To guard against loss-of-precision, we convert
	// to microseconds *before* dividing by ticks-per-second.

	elapsed_microseconds.QuadPart *= 1000000;
	elapsed_microseconds.QuadPart /= win32_ticks_per_second.QuadPart;

	return elapsed_microseconds.QuadPart;
}

void os_sleep(uint milliseconds)
{
#define DESIRED_SCHEDULER_GRANULARITY 1 // milliseconds
	HRESULT SchedulerResult = timeBeginPeriod(DESIRED_SCHEDULER_GRANULARITY);
#undef DESIRED_SCHEDULER_GRANULARITY

	Sleep(milliseconds);
}

// Use this for quick timing needs!
#define DEBUG_TIMER_BEGIN() Timer d; d.init(); d.start();
#define DEBUG_TIMER_END() d.print_microseconds("debug timer : ");

// ------------------------------------------------- //
// ---------------------- Audio -------------------- //
// ------------------------------------------------- //

/* -- how 2 play a sound --

	Audio sound = load_audio("sound.audio");
	play_sudio(sound);
*/

typedef ALuint Audio;

// ------------------------------------------------- //
// -------------------- 3D Camera ------------------ //
// ------------------------------------------------- //

enum CAM_DIR {
	FWD, BCK, LFT, RGT
};

struct Camera
{
	vec3 position;
	vec3 front, right, up;
	float yaw, pitch;
	float trauma;

	void update_dir(float dx, float dy, float dtime, float sensitivity = 0.003)
	{
		yaw   += (dx * sensitivity) / TWOPI;
		pitch += (dy * sensitivity) / TWOPI;

		if (pitch >  PI / 2.01) pitch =  PI / 2.01;
		if (pitch < -PI / 2.01) pitch = -PI / 2.01;

		front.y = sin(pitch);
		front.x = cos(pitch) * cos(yaw);
		front.z = cos(pitch) * sin(yaw);

		front = normalize(front);
		right = normalize(cross(front, vec3(0, 1, 0)));
		up    = normalize(cross(right, front));
	}
	void update_pos(int direction, float distance)
	{
		if (direction == CAM_DIR::FWD) position += front * distance;
		if (direction == CAM_DIR::LFT) position -= right * distance;
		if (direction == CAM_DIR::RGT) position += right * distance;
		if (direction == CAM_DIR::BCK) position -= front * distance;
	}
};

// ------------------------------------------------- //
// ----------------- Multi-Threading --------------- //
// ------------------------------------------------- //

typedef DWORD WINAPI thread_function(LPVOID); // what is this sorcery?
DWORD WINAPI thread_func(LPVOID param)
{
	printf("Thread Started");
	return 0;
}
uint64 create_thread(thread_function function, void* params = NULL)
{
	DWORD thread_id = 0;
	CreateThread(0, 0, function, params, 0, &thread_id);
	
	// CreateThread returns a handle to the thread.
	// im not sure if i should be keeping it
	return thread_id;
}
//uint test(float a) { out(a); return 0; }
//typedef uint temp(float);
//void wtf(temp func) { func(7); return; }

// ------------------------------------------------- //
// --------------- Files & Directories ------------- //
// ------------------------------------------------- //

enum FILETYPE {
	NONE = 0,
	C, H, CPP, HPP,
	JPG, PNG, BMP, R32,
	OBJ, FBX,
	BLENDER, MD,
	VERT, FRAG,
	MESH, MESH_UV, MESH_ANIM, MESH_ANIM_UV,
	DIRECTORY
};

FILETYPE get_filetype(char* name)
{
	char type[32] = {};
	sscanf(name, "%*[^.].%s", type);
	//print("."); print(type); print("\n");

	// determine file type
	if (!strcmp(type, "h"))
		return FILETYPE::H;
	else if (!strcmp(type, "cpp"))
		return FILETYPE::CPP;
	else if (!strcmp(type, "mesh"))
		return FILETYPE::MESH;
	else if (!strcmp(type, "mesh_uv"))
		return FILETYPE::MESH_UV;
	else if (!strcmp(type, "mesh_anim"))
		return FILETYPE::MESH_ANIM;
	else if (!strcmp(type, "mesh_anim_uv"))
		return FILETYPE::MESH_ANIM_UV;
	else
		return FILETYPE::NONE; // mark as directory instead?
}

byte* read_text_file_into_memory(const char* path)
{
	DWORD BytesRead;
	HANDLE os_file = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, NULL, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

	if (os_file == INVALID_HANDLE_VALUE)
	{
		// return an empty string instead of an uninitialized size :
		// callers (shaders) fail gracefully and the error shows in the console
		return (byte*)calloc(1, 1);
	}

	LARGE_INTEGER size;
	GetFileSizeEx(os_file, &size);

	byte* memory = (byte*)calloc(size.QuadPart + 1, sizeof(byte));
	ReadFile(os_file, memory, size.QuadPart, &BytesRead, NULL);

	CloseHandle(os_file);

	return memory;
}
void load_file_r32(const char* path, float* memory, uint n)
{
	float* temp = Alloc(float, n * n); // n should always be a power of 2

	DWORD BytesRead;
	HANDLE os_file = CreateFileA(path, GENERIC_READ, NULL, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	ReadFile(os_file, (byte*)temp, n * n * sizeof(float), &BytesRead, NULL);
	CloseHandle(os_file); // assert(BytesRead == n * n);

	uint half_n = n / 2; // n << something? is this faster?

	for (uint x = 0; x < n; x++) { // rotate about horizontal
	for (uint y = 0; y < half_n; y++)
	{
		uint index1 = (y * n) + x;

		uint distance_to_center = half_n - 1 - y;
		uint y2 = distance_to_center + half_n;
		uint index2 = (y2 * n) + x; // index of point to flip with

		float tmp = temp[index2];

		temp[index2] = temp[index1];
		temp[index1] = tmp;

	} }

	for (uint x = 0; x < n; x++) { // rotate about y = -x
	for (uint y = 0; y < n; y++)
	{
		memory[y * n + x] = temp[x * n + y];

	} }

	free(temp); // for some reason we have to rotate the image
}

#define DIRECTORY_ERROR(str) std::cout << "DIRECTORY ERROR: " << str << '\n';
#define MAX_DIRECTORY_FILES 64 //WARNING: harcoded file name limit here
#define MAX_PATH_LENGTH 250 // last 6 bytes are reserved for parsing operation needs

struct Directory
{
	uint num_files;
	char* names[MAX_DIRECTORY_FILES];
};

// allocates memory & fills directory struct. free the memory with free_directory
void parse_directory(Directory* dir, const char* path)
{
	char filepath[256] = {};
	snprintf(filepath, 256, "%s\\*.*", path); // file mask: *.* = get everything

	WIN32_FIND_DATAA FoundFile;
	HANDLE Find = FindFirstFileA(filepath, &FoundFile);
	if (Find == INVALID_HANDLE_VALUE) { print("Path not found: [%s]\n", path); return; }

	//FindFirstFile always returns "." & ".." as first two directories
	while (!strcmp(FoundFile.cFileName, ".") || !strcmp(FoundFile.cFileName, "..")) FindNextFileA(Find, &FoundFile);

	uint num_files = 0; // for readability
	do
	{
		uint length = strlen(FoundFile.cFileName);

		dir->names[num_files] = Alloc(char, length + 1);
		dir->names[num_files][length] = 0;
		memcpy(dir->names[num_files], FoundFile.cFileName, length);

		++num_files;

	} while (FindNextFileA(Find, &FoundFile));

	FindClose(Find);

	dir->num_files = num_files;

	//	out("Finished parsing " << path << "/ and found " << num_files << " files");
}

// prints file count, names, and extensions to std output
void print_directory(Directory dir)
{
	print("directory contains %d files\n", dir.num_files);
	for (uint i = 0; i < dir.num_files; ++i)
		print(" %d: %s\n", i + 1, dir.names[i]);
}

// frees all memory used by this directory struct
void free_directory(Directory* dir)
{
	for (uint i = 0; i < dir->num_files; ++i) free(dir->names[i]);
	*dir = {};
}

uint get_file_size(const char* path)
{
	HANDLE file_handle = CreateFileA(path, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	
	if (file_handle == INVALID_HANDLE_VALUE)
		return -1; // call GetLastError() to find out more

	LARGE_INTEGER size;
	if (!GetFileSizeEx(file_handle, &size))
	{
		CloseHandle(file_handle);
		return -1; // error condition, could call GetLastError to find out more
	}

	CloseHandle(file_handle);
	return size.QuadPart; // file size in bytes!
}
uint get_directory_size(Directory* dir, const char* path)
{
	uint directory_size = 0; // in bytes
	for (uint i = 0; i < dir->num_files; i++)
	{
		directory_size += get_file_size(dir->names[i]);
	}

	return directory_size; // in bytes!
}
uint get_directory_size(const char* path)
{
	Directory* dir = NULL;
	parse_directory(dir, path);

	uint directory_size = 0;
	for (uint i = 0; i < dir->num_files; i++)
	{
		directory_size += get_file_size(dir->names[i]);
	}

	free_directory(dir);

	return directory_size; // in bytes!
}

// TODO : helper functions to get file extentions and names seperately?

// ------------------------------------------------- //
// --------------------- Physics ------------------- //
// ------------------------------------------------- //

#include "../BULLET/btBulletDynamicsCommon.h"
#include "../BULLET/BulletSoftBody/btSoftRigidDynamicsWorld.h"

// ------------------------------------------------- //
// ------------------- Networking ------------------ //
// ------------------------------------------------- //

#include "networking.h"

// ------------------------------------------------- //
// ----------------------- UX ---------------------- //
// ------------------------------------------------- //

#include "../external/IMGUI/imgui.h"
#include "../external/IMGUI/backends/imgui_impl_glfw.h"
#include "../external/IMGUI/backends/imgui_impl_opengl3.h"

#define KiloByte(n) (n * 1024)
#define MegaByte(n) (KiloByte(n) * 1024)

// IMGUI
void apply_imgui_style(ImGuiIO& io)
{
	// Darkest  0.14f, 0.18f, 0.14f
	// Lighter  0.2f , 0.29f, 0.33f
	// Lightest 0.29f, 0.39f, 0.45f
	// Orange   0.98f, 0.66f, 0.2f

	// Setup Dear ImGui style
	ImGuiStyle* style = &ImGui::GetStyle();
	ImVec4* colors = style->Colors;

	// --- Text ---
	colors[ImGuiCol_Text] = ImVec4(0.95f, 0.95f, 0.95f, 1.00f);
	colors[ImGuiCol_TextDisabled] = ImVec4(0.55f, 0.55f, 0.55f, 1.00f);

	// --- Windows / Panels ---
	colors[ImGuiCol_WindowBg] = ImVec4(0.06f, 0.06f, 0.07f, 0.35f);  // smoky glass
	colors[ImGuiCol_ChildBg] = ImVec4(0.05f, 0.05f, 0.06f, 0.45f);
	colors[ImGuiCol_PopupBg] = ImVec4(0.08f, 0.08f, 0.09f, 0.70f);

	// --- Borders ---
	colors[ImGuiCol_Border] = ImVec4(0.20f, 0.20f, 0.22f, 0.35f);
	colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);

	// --- Frames / Inputs ---
	colors[ImGuiCol_FrameBg] = ImVec4(0.12f, 0.12f, 0.14f, 0.35f);
	colors[ImGuiCol_FrameBgHovered] = ImVec4(0.30f, 0.30f, 0.32f, 0.40f);
	colors[ImGuiCol_FrameBgActive] = ImVec4(0.35f, 0.35f, 0.38f, 0.50f);

	// --- Titles ---
	colors[ImGuiCol_TitleBg] = ImVec4(0.08f, 0.08f, 0.09f, 0.75f);
	colors[ImGuiCol_TitleBgActive] = ImVec4(0.10f, 0.10f, 0.12f, 0.85f);
	colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.05f, 0.05f, 0.05f, 0.65f);

	// --- Tabs ---
	colors[ImGuiCol_Tab] = ImVec4(0.12f, 0.12f, 0.14f, 0.65f);
	colors[ImGuiCol_TabHovered] = ImVec4(0.25f, 0.20f, 0.10f, 0.55f); // hint of orange warmth
	colors[ImGuiCol_TabActive] = ImVec4(0.30f, 0.25f, 0.15f, 0.70f);
	colors[ImGuiCol_TabUnfocused] = ImVec4(0.10f, 0.10f, 0.11f, 0.60f);
	colors[ImGuiCol_TabUnfocusedActive] = colors[ImGuiCol_TabActive];

	// --- Buttons ---
	colors[ImGuiCol_Button] = ImVec4(0.16f, 0.16f, 0.17f, 0.40f);
	colors[ImGuiCol_ButtonHovered] = ImVec4(0.45f, 0.30f, 0.15f, 0.55f); // faint amber glow
	colors[ImGuiCol_ButtonActive] = ImVec4(0.60f, 0.45f, 0.20f, 0.60f);

	// --- Headers ---
	colors[ImGuiCol_Header] = ImVec4(0.20f, 0.20f, 0.22f, 0.45f);
	colors[ImGuiCol_HeaderHovered] = ImVec4(0.40f, 0.30f, 0.20f, 0.55f);
	colors[ImGuiCol_HeaderActive] = ImVec4(0.50f, 0.35f, 0.20f, 0.60f);

	// --- Sliders / Grabs ---
	colors[ImGuiCol_SliderGrab] = ImVec4(0.80f, 0.55f, 0.25f, 0.45f);
	colors[ImGuiCol_SliderGrabActive] = ImVec4(1.00f, 0.70f, 0.30f, 0.60f);

	// --- Checkmarks / Separators ---
	colors[ImGuiCol_CheckMark] = ImVec4(1.00f, 0.70f, 0.25f, 0.65f);
	colors[ImGuiCol_Separator] = ImVec4(0.25f, 0.25f, 0.27f, 0.50f);
	colors[ImGuiCol_SeparatorHovered] = ImVec4(0.70f, 0.55f, 0.30f, 0.55f);
	colors[ImGuiCol_SeparatorActive] = ImVec4(0.80f, 0.65f, 0.35f, 0.70f);

	// --- Misc ---
	colors[ImGuiCol_NavHighlight] = ImVec4(1.00f, 0.70f, 0.25f, 0.35f);
	colors[ImGuiCol_TextSelectedBg] = ImVec4(1.00f, 0.60f, 0.10f, 0.25f);
	colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.03f, 0.03f, 0.03f, 0.50f);

	style->WindowRounding = 8.0f;
	style->ChildRounding = 8.0f;
	style->FrameRounding = 5.0f;
	style->GrabRounding = 4.0f;
	style->TabRounding = 6.0f;

	style->WindowBorderSize = 1.0f;
	style->FrameBorderSize = 1.0f;

	style->Alpha = 0.95f;
	style->WindowPadding = ImVec2(8, 6);
	style->FramePadding = ImVec2(6, 4);
}