#include "loader.h"

/* ------------------------- *
   -         Audio         -
 * ------------------------- */

// OpenAL mixer : a 3 uint header (format, rate, size), alBufferData and a real
// alSourcePlay. on top : named sounds, a few voices each so a retrigger does
// not cut the one before it, master x per-sound gain, a trim down to the take's
// useful span, and an envelope for the ux graph. the device and context are
// opened in GameWindow::init

const uint MAX_SOUNDS = 24;
const uint MAX_VOICES = 3;
const uint WAVE_POINTS = 96;
const float MAX_SOUND_SECONDS = 3.f; // safety cap : the head/tail trim does the work
const float TRIM_HEAD = .03f;        // sample thresholds, fraction of the peak
const float TRIM_TAIL = .06f;

typedef ALuint Audio;

// raw load : one file in, one ready source out
Audio load_audio(const char* path)
{
	uint format = 0, size = 0, sample_rate = 0;
	byte* audio_data = NULL;

	FILE* file = fopen(path, "rb"); // rb = read binary
	if (file == NULL)
	{
		char msg[MAX_LOG_MSG_LENGTH];
		snprintf(msg, MAX_LOG_MSG_LENGTH, "audio not found : %s", path);
		console->add_entry(msg, FIXME, AUDI);
		return 0;
	}

	fread(&format     , sizeof(uint), 1, file);
	fread(&sample_rate, sizeof(uint), 1, file);
	fread(&size       , sizeof(uint), 1, file);

	audio_data = Alloc(byte, size);
	fread(audio_data, sizeof(byte), size, file);
	fclose(file);

	ALuint buffer_id = NULL;
	alGenBuffers(1, &buffer_id);
	alBufferData(buffer_id, format, audio_data, size, sample_rate);

	ALuint source_id = NULL;
	alGenSources(1, &source_id);
	alSourcei(source_id, AL_BUFFER, buffer_id);

	free(audio_data);
	return source_id;
}

void play_audio(Audio source_id)
{
	alSourcePlay(source_id);
}

struct GameAudio
{
	struct Sound
	{
		char   name[24];
		ALuint buffer;
		ALuint voice[MAX_VOICES]; // played round robin
		uint   next_voice;
		float  gain, pitch;
		bool   loop;
		uint   rate, bytes;
		float  seconds;
		float  wave[WAVE_POINTS];
	};

	Sound sounds[MAX_SOUNDS];
	uint  num_sounds;
	float master; // 0..1, scales everything

	void init()
	{
		master = 1.f;
		num_sounds = 0;

		char msg[MAX_LOG_MSG_LENGTH];
		snprintf(msg, MAX_LOG_MSG_LENGTH, "audio | %s | %s", alGetString(AL_VENDOR), alGetString(AL_RENDERER));
		console->add_entry(msg, SUCCESS, AUDI);
	}

	// the span a take is worth playing : silence off the front, tail off the
	// back, a little run-up and tail kept. 16 bit samples, stride = bytes/frame
	uint trim_span(void* pcm, uint frames, uint rate, uint stride, uint* first)
	{
		*first = 0;
		if (frames == 0) return 0;

		short* s = (short*)pcm;
		uint per_frame = stride / 2;

		uint peak = 0;
		for (uint i = 0; i < frames; i++)
		{
			uint v = (uint)abs((int)s[i * per_frame]);
			if (v > peak) peak = v;
		}

		uint a = 0, b = frames - 1;
		if (peak)
		{
			uint head = (uint)(peak * TRIM_HEAD), tail = (uint)(peak * TRIM_TAIL);
			for (uint i = 0; i < frames && a == 0; i++)
				if ((uint)abs((int)s[i * per_frame]) > head) a = i;
			for (uint i = frames; i > 0 && b == frames - 1; i--)
				if ((uint)abs((int)s[(i - 1) * per_frame]) > tail) b = i - 1;

			uint run_up = (uint)(rate * .005f), keep = (uint)(rate * .08f);
			a = (a > run_up) ? a - run_up : 0;
			b += keep;
			if (b >= frames) b = frames - 1;
		}

		uint cap = (uint)(rate * MAX_SOUND_SECONDS);
		if (b - a + 1 > cap) b = a + cap - 1;

		*first = a;
		return b;
	}

	// load a .audio take as a named sound with MAX_VOICES sources. returns the
	// sound's slot (0 = failed, error already logged) - play it with play()
	uint load(const char* name, const char* path)
	{
		if (num_sounds >= MAX_SOUNDS)
		{
			console->add_entry("audio : sound slots full", FIXME, AUDI);
			return 0;
		}

		FILE* file = fopen(path, "rb");
		if (!file)
		{
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "audio not found : %s", path);
			console->add_entry(msg, FIXME, AUDI);
			return 0;
		}

		uint format = 0, rate = 0, size = 0;
		fread(&format, sizeof(uint), 1, file);
		fread(&rate,   sizeof(uint), 1, file);
		fread(&size,   sizeof(uint), 1, file);

		// a corrupt header (a 0 or nonsense rate) makes alBufferData refuse the
		// whole take : fall back to a sane rate and say so
		if (rate < 4000 || rate > 192000)
		{
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "audio | %s | bad rate %u -> 44100", name, rate);
			console->add_entry(msg, FIXME, AUDI);
			rate = 44100;
		}

		byte* pcm = Alloc(byte, size);
		fread(pcm, sizeof(byte), size, file);
		fclose(file);

		bool pcm16 = (format == AL_FORMAT_MONO16 || format == AL_FORMAT_STEREO16);
		uint stride = (format == AL_FORMAT_STEREO16) ? 4 : 2;
		uint first = 0, play_bytes = size;
		// trim only the long takes : a short file plays whole, so a quiet one-off
		// (a decay tail, a shot with a long lead-in) can never be cut to nothing
		if (pcm16 && size / stride > (uint)(rate * MAX_SOUND_SECONDS))
		{
			uint last = trim_span(pcm, size / stride, rate, stride, &first);
			play_bytes = (last - first + 1) * stride;
		}

		Sound& s = sounds[num_sounds];
		snprintf(s.name, sizeof(s.name), "%s", name);
		s.gain = 1.f;
		s.pitch = 1.f;
		s.loop = false;
		s.rate = rate;
		s.bytes = play_bytes;
		s.seconds = (float)(play_bytes / stride) / (float)rate;
		s.next_voice = 0;

		// envelope for the ux : peaks across the trimmed span, normalised
		for (uint k = 0; k < WAVE_POINTS; k++) s.wave[k] = .5f;
		if (pcm16)
		{
			short* samples = (short*)(pcm + first * stride);
			uint frames = play_bytes / stride, per = stride / 2, loudest = 1;

			for (uint k = 0; k < WAVE_POINTS; k++)
			{
				uint from = frames * k / WAVE_POINTS, to = frames * (k + 1) / WAVE_POINTS, peak = 1;
				for (uint f = from; f < to; f++)
				{
					uint v = (uint)abs((int)samples[f * per]);
					if (v > peak) peak = v;
				}
				s.wave[k] = (float)peak;
				if (peak > loudest) loudest = peak;
			}
			for (uint k = 0; k < WAVE_POINTS; k++) s.wave[k] /= (float)loudest;
		}

		alGenBuffers(1, &s.buffer);
		alBufferData(s.buffer, format, pcm + first * stride, play_bytes, rate);
		free(pcm);

		for (uint v = 0; v < MAX_VOICES; v++)
		{
			alGenSources(1, &s.voice[v]);
			alSourcei(s.voice[v], AL_BUFFER, s.buffer);
			alSourcef(s.voice[v], AL_GAIN, s.gain * master);
		}

		char msg[MAX_LOG_MSG_LENGTH];
		snprintf(msg, MAX_LOG_MSG_LENGTH, "audio | %s | %.2fs %uhz %ukb%s",
			s.name, s.seconds, s.rate, play_bytes / 1024,
			alGetError() ? " | REFUSED" : "");
		console->add_entry(msg, alGetError() ? FIXME : SUCCESS, AUDI);

		return ++num_sounds;
	}

	uint find(const char* name)
	{
		for (uint i = 0; i < num_sounds; i++)
			if (strcmp(sounds[i].name, name) == 0) return i + 1;
		return 0;
	}

	// one shot per voice, round robin : a retrigger never cuts the last one
	void play(uint i)
	{
		if (i == 0 || i > num_sounds) return;

		Sound& s = sounds[i - 1];
		ALuint v = s.voice[s.next_voice];
		s.next_voice = (s.next_voice + 1) % MAX_VOICES;

		alSourcei(v, AL_LOOPING, s.loop);
		alSourcef(v, AL_PITCH, s.pitch);
		alSourceRewind(v);
		alSourcePlay(v);
	}

	void play_name(const char* name) { play(find(name)); }

	void halt(uint i)
	{
		if (i == 0 || i > num_sounds) return;
		for (uint v = 0; v < MAX_VOICES; v++) alSourceStop(sounds[i - 1].voice[v]);
	}

	void halt_all()
	{
		for (uint i = 1; i <= num_sounds; i++) halt(i);
	}

	bool playing(uint i)
	{
		if (i == 0 || i > num_sounds) return false;
		for (uint v = 0; v < MAX_VOICES; v++)
		{
			ALint state = 0;
			alGetSourcei(sounds[i - 1].voice[v], AL_SOURCE_STATE, &state);
			if (state == AL_PLAYING) return true;
		}
		return false;
	}

	// how far into its buffer the loudest voice is, 0..1 : the ux playhead
	float head(uint i)
	{
		if (i == 0 || i > num_sounds || sounds[i - 1].seconds <= 0) return 0;
		for (uint v = 0; v < MAX_VOICES; v++)
		{
			ALint state = 0;
			alGetSourcei(sounds[i - 1].voice[v], AL_SOURCE_STATE, &state);
			if (state != AL_PLAYING) continue;

			ALfloat offset = 0;
			alGetSourcef(sounds[i - 1].voice[v], AL_SEC_OFFSET, &offset);
			return offset / sounds[i - 1].seconds;
		}
		return 0;
	}

	void apply(uint i)
	{
		Sound& s = sounds[i - 1];
		for (uint v = 0; v < MAX_VOICES; v++)
		{
			alSourcef(s.voice[v], AL_GAIN, s.gain * master);
			alSourcef(s.voice[v], AL_PITCH, s.pitch);
			alSourcei(s.voice[v], AL_LOOPING, s.loop);
		}
	}

	void set_gain(uint i, float g)  { if (i && i <= num_sounds) { sounds[i - 1].gain = g;  apply(i); } }
	void set_pitch(uint i, float p) { if (i && i <= num_sounds) { sounds[i - 1].pitch = p; apply(i); } }
	void set_loop(uint i, bool on)  { if (i && i <= num_sounds) { sounds[i - 1].loop = on;  apply(i); } }

	void set_master(float m)
	{
		master = m;
		for (uint i = 1; i <= num_sounds; i++) apply(i);
	}
};

GameAudio audio; // one mixer for the process
Audio pop_sound, shot_sound; // the Audio tab's ad-hoc takes (raw sources)
