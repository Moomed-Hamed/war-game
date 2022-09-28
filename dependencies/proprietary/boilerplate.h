// ------------------------------------------------- //
// -------  boilerplate version 1 : 16.9.22  ------- //
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

struct bvec3 { union { struct { byte x, y, z; }; struct { byte r, g, b; }; }; };

// ------------------------------------------------- //
// ------------------- Mathematics ----------------- //
// ------------------------------------------------- //

#include "mathematics.h" // this is pretty much GLM for now

// ------------------------------------------------- //
// --------------------- Timers -------------------- // // might be broken idk
// ------------------------------------------------- //

// while the raw timestamp can be used for relative performence measurements,
// it does not necessarily correspond to any external notion of time
typedef uint64 Timestamp;

Timestamp get_timestamp()
{
	LARGE_INTEGER win32_timestamp;
	QueryPerformanceCounter(&win32_timestamp);

	return win32_timestamp.QuadPart;
}

int64 calculate_milliseconds_elapsed(Timestamp start, Timestamp end)
{
	//Get CPU clock frequency for Timing
	LARGE_INTEGER win32_performance_frequency;
	QueryPerformanceFrequency(&win32_performance_frequency);

	return (1000 * (end - start)) / win32_performance_frequency.QuadPart;
}
int64 calculate_microseconds_elapsed(Timestamp start, Timestamp end)
{
	//Get CPU clock frequency for Timing
	LARGE_INTEGER win32_performance_frequency;
	QueryPerformanceFrequency(&win32_performance_frequency);

	// i think (end - start) corresponds directly to cpu clock cycles but i'm not sure
	return (1000000 * end - start) / win32_performance_frequency.QuadPart;
}

void os_sleep(uint milliseconds)
{
#define DESIRED_SCHEDULER_GRANULARITY 1 // milliseconds
	HRESULT SchedulerResult = timeBeginPeriod(DESIRED_SCHEDULER_GRANULARITY);
#undef DESIRED_SCHEDULER_GRANULARITY

	Sleep(milliseconds);
}

// ------------------------------------------------- //
// ---------------------- Audio -------------------- //
// ------------------------------------------------- //

/* -- how 2 play a sound --

	Audio sound = load_audio("sound.audio");
	play_sudio(sound);
*/

typedef ALuint Audio;

// ------------------------------------------------- //
// ----------------- Multithreading ---------------- //
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

byte* read_text_file_into_memory(const char* path)
{
	DWORD BytesRead;
	HANDLE os_file = CreateFile(path, GENERIC_READ | GENERIC_WRITE, NULL, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

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
	HANDLE os_file = CreateFile(path, GENERIC_READ, NULL, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
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
#define MAX_DIRECTORY_FILES 256 //WARNING: harcoded file name limit here

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

	WIN32_FIND_DATA FoundFile;
	HANDLE Find = FindFirstFile(filepath, &FoundFile);
	if (Find == INVALID_HANDLE_VALUE) { print("Path not found: [%s]\n", path); return; }

	//FindFirstFile always returns "." & ".." as first two directories
	while (!strcmp(FoundFile.cFileName, ".") || !strcmp(FoundFile.cFileName, "..")) FindNextFile(Find, &FoundFile);

	uint num_files = 0; // for readability
	do
	{
		uint length = strlen(FoundFile.cFileName);

		dir->names[num_files] = Alloc(char, length + 1);
		dir->names[num_files][length] = 0;
		memcpy(dir->names[num_files], FoundFile.cFileName, length);

		++num_files;

	} while (FindNextFile(Find, &FoundFile));

	FindClose(Find);

	dir->num_files = num_files;

	//	out("Finished parsing " << path << "/ and found " << num_files << " files");
}

// prints file count, names, and extensions to std output
void print_directory(Directory dir)
{
	print("directory contains %d files", dir.num_files);
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
	HANDLE file_handle = CreateFile(path, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	
	if (file_handle == INVALID_HANDLE_VALUE)
		return -1; // call GetLastError() to find out more

	LARGE_INTEGER size;
	if (!GetFileSizeEx(file_handle, &size))
	{
		CloseHandle(file_handle);
		return -1; // error condition, could call GetLastError to find out more
	}

	CloseHandle(file_handle);
	return size.QuadPart;
}
uint get_directory_size(Directory* dir, const char* path)
{
	uint directory_size = 0;
	for (uint i = 0; i < dir->num_files; i++)
	{
		directory_size += get_file_size(dir->names[i]);
	}

	return directory_size;
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

	return directory_size;
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