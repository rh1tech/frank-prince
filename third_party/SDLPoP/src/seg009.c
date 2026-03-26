/*
SDLPoP, a port/conversion of the DOS game Prince of Persia.
Copyright (C) 2013-2025  Dávid Nagy

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

The authors of this program may be contacted at https://forum.princed.org
*/

#include "common.h"
#ifdef POP_RP2350
#include "psram_allocator.h"
#include "HDMI.h"
#include "pico/stdlib.h"  // for sleep_us
#include <setjmp.h>
// Jump buffer for returning to main loop on quit (instead of exit())
jmp_buf pop_quit_jmp_buf;
int pop_quit_requested = 0;
#endif
#include <time.h>
#include <errno.h>

#ifdef POP_RP2350
#include "pop_fs.h"
#include "ff.h"
#endif

#ifdef _WIN32
#include <windows.h>
#include <wchar.h>
#else
#include "dirent.h"
#endif

#ifdef __amigaos4__
#include <workbench/startup.h>
#include <proto/dos.h>
#endif

#ifdef POP_RP2350
// PSRAM-backed file I/O wrappers (FatFS)
typedef FIL pop_file_t;
static inline pop_file_t* pop_open(const char* path, const char* mode) { return pop_fs_open(path, mode); }
static inline int pop_seek(pop_file_t* fp, long ofs, int whence) { return pop_fs_seek(fp, ofs, whence); }
static inline size_t pop_read(void* ptr, size_t sz, size_t n, pop_file_t* fp) { return pop_fs_read(ptr, sz, n, fp); }
static inline int pop_close(pop_file_t* fp) { return pop_fs_close(fp); }
static inline long pop_size(pop_file_t* fp) { return (long)f_size(fp); }

#define IS_PSRAM(ptr) ((uintptr_t)(ptr) >= 0x11000000 && (uintptr_t)(ptr) < 0x12000000)
static inline void* pop_heap_alloc(size_t size) {
	void* p = psram_malloc(size);
	if (!p) {
		printf("pop_heap_alloc: OOM (psram_malloc failed) size=%d\n", (int)size);
		fflush(stdout);
		return NULL;
	}
	return p;
}
static inline void pop_heap_free(void* p) {
	if (!p) return;
	if (IS_PSRAM(p)) psram_free(p);
	else free(p);
}

static byte lzg_window[0x400];
#else
// Desktop file I/O wrappers (stdio)
typedef FILE pop_file_t;
static inline pop_file_t* pop_open(const char* path, const char* mode) { return fopen(path, mode); }
static inline int pop_seek(pop_file_t* fp, long ofs, int whence) { return fseek(fp, ofs, whence); }
static inline size_t pop_read(void* ptr, size_t sz, size_t n, pop_file_t* fp) { return fread(ptr, sz, n, fp); }
static inline int pop_close(pop_file_t* fp) { return fclose(fp); }
static inline long pop_size(pop_file_t* fp) {
	struct stat buf;
	if (fstat(fileno(fp), &buf) != 0) return -1;
	return (long)buf.st_size;
}
#endif

// Most functions in this file are different from those in the original game.

void sdlperror(const char* header) {
	const char* error = SDL_GetError();
	printf("%s: %s\n",header,error);
	//quit(1);
}

char exe_dir[POP_MAX_PATH] = ".";
bool found_exe_dir = false;
#if ! (defined WIN32 || _WIN32 || WIN64 || _WIN64)
char home_dir[POP_MAX_PATH];
bool found_home_dir = false;
char share_dir[POP_MAX_PATH];
bool found_share_dir = false;
#endif

void find_exe_dir(void) {
	if (found_exe_dir) return;
#ifdef __amigaos4__
	if(g_argc == 0) { // from Workbench
		struct WBStartup *WBenchMsg = (struct WBStartup *)g_argv;
		NameFromLock( WBenchMsg->sm_ArgList->wa_Lock, exe_dir, sizeof(exe_dir) );
	}
	else { // from Shell/CLI
		NameFromLock( GetProgramDir(), exe_dir, sizeof(exe_dir) );
	}
#else
	snprintf_check(exe_dir, sizeof(exe_dir), "%s", g_argv[0]);
	char* last_slash = NULL;
	char* pos = exe_dir;
	for (char c = *pos; c != '\0'; ++pos, c = *pos) {
		if (c == '/' || c == '\\') {
			last_slash = pos;
		}
	}
	if (last_slash != NULL) {
		*last_slash = '\0';
	}
#endif
	found_exe_dir = true;
}

#if ! (defined WIN32 || _WIN32 || WIN64 || _WIN64)
void find_home_dir(void) {
	if (found_home_dir) return;
	const char* home_path = getenv("HOME");
	snprintf_check(home_dir, POP_MAX_PATH - 1, "%s/.%s", home_path, POP_DIR_NAME);
	if(file_exists(home_dir))
		found_home_dir = true;
}

void find_share_dir(void) {
	if (found_share_dir) return;
	snprintf_check(share_dir, POP_MAX_PATH - 1, "%s/%s", SHARE_PATH, POP_DIR_NAME);
	if(file_exists(share_dir))
		found_share_dir = true;
}
#endif

bool file_exists(const char* filename) {
	#ifdef POP_RP2350
	return pop_fs_exists(filename);
	#else
	return (access(filename, F_OK) != -1);
	#endif
}

const char* find_first_file_match(char* dst, int size, char* format, const char* filename) {
	find_exe_dir();
#if defined WIN32 || _WIN32 || WIN64 || _WIN64
	snprintf_check(dst, size, format, exe_dir, filename);
#else
	find_home_dir();
	find_share_dir();
	char* dirs[3] = {home_dir, share_dir, exe_dir};
	for (int i = 0; i < 3; i++) {
		snprintf_check(dst, size, format, dirs[i], filename);
		if(file_exists(dst))
			break;
	}
#endif
	return (const char*) dst;
}

const char* locate_save_file_(const char* filename, char* dst, int size) {
	#ifdef POP_RP2350
	(void)dst;
	(void)size;
	return filename;
	#else
	find_exe_dir();
#if defined WIN32 || _WIN32 || WIN64 || _WIN64
	snprintf_check(dst, size, "%s/%s", exe_dir, filename);
#else
	find_home_dir();
	find_share_dir();
	char* dirs[3] = {home_dir, share_dir, exe_dir};
	for (int i = 0; i < 3; i++) {
		struct stat path_stat;
		int result = stat(dirs[i], &path_stat);
		if (result == 0 && S_ISDIR(path_stat.st_mode) && access(dirs[i], W_OK) == 0) {
			snprintf_check(dst, size, "%s/%s", dirs[i], filename);
			break;
		}
	}
#endif
	return (const char*) dst;
	#endif
}
const char* locate_file_(const char* filename, char* path_buffer, int buffer_size) {
	#ifdef POP_RP2350
	(void)path_buffer;
	(void)buffer_size;
	return filename;
	#else
	if(file_exists(filename)) {
		return filename;
	} else {
		// If failed, it may be that SDLPoP is being run from the wrong different working directory.
		// We can try to rescue the situation by loading from the directory of the executable.
		return find_first_file_match(path_buffer, buffer_size, "%s/%s", filename);
	}
	#endif
}

#ifdef _WIN32
// These macros are from the SDL2 source. (src/core/windows/SDL_windows.h)
// The pointers returned by these macros must be freed with SDL_free().
#define WIN_StringToUTF8(S) SDL_iconv_string("UTF-8", "UTF-16LE", (char *)(S), (SDL_wcslen(S)+1)*sizeof(WCHAR))
#define WIN_UTF8ToString(S) (WCHAR *)SDL_iconv_string("UTF-16LE", "UTF-8", (char *)(S), SDL_strlen(S)+1)

// This hack is needed because SDL uses UTF-8 everywhere (even in argv!), but fopen on Windows uses whatever code page is currently set.
FILE* fopen_UTF8(const char* filename_UTF8, const char* mode_UTF8) {
	WCHAR* filename_UTF16 = WIN_UTF8ToString(filename_UTF8);
	WCHAR* mode_UTF16 = WIN_UTF8ToString(mode_UTF8);
	FILE* result = _wfopen(filename_UTF16, mode_UTF16);
	SDL_free(mode_UTF16);
	SDL_free(filename_UTF16);
	return result;
}

int chdir_UTF8(const char* path_UTF8) {
	WCHAR* path_UTF16 = WIN_UTF8ToString(path_UTF8);
	int result = _wchdir(path_UTF16);
	SDL_free(path_UTF16);
	return result;
}

int mkdir_UTF8(const char* path_UTF8) {
	WCHAR* path_UTF16 = WIN_UTF8ToString(path_UTF8);
	int result = _wmkdir(path_UTF16);
	SDL_free(path_UTF16);
	return result;
}

int access_UTF8(const char* filename_UTF8, int mode) {
	WCHAR* filename_UTF16 = WIN_UTF8ToString(filename_UTF8);
	int result = _waccess(filename_UTF16, mode);
	SDL_free(filename_UTF16);
	return result;
}

int stat_UTF8(const char *filename_UTF8, struct stat *_Stat) {
	WCHAR* filename_UTF16 = WIN_UTF8ToString(filename_UTF8);
#ifdef _MSC_VER
	int result = _wstat(filename_UTF16, _Stat);
#else
	// There is a _wstat() function on Dev-C++ as well, but it expects the second argument to be a different type than stat().
	int result = wstat(filename_UTF16, _Stat);
#endif
	SDL_free(filename_UTF16);
	return result;
}

#endif //_WIN32

// OS abstraction for listing directory contents
// Directory listing using dirent.h is available using MinGW on Windows, but not using MSVC (need to use Win32 API).
// - Under GNU/Linux, etc (or if compiling with MinGW on Windows), we can use dirent.h
// - Under Windows, we'd like to directly call the Win32 API. (Note: MSVC does not include dirent.h)
// NOTE: If we are using MinGW, we'll opt to use the Win32 API as well: dirent.h would just wrap Win32 anyway!

#ifdef _WIN32
struct directory_listing_type {
	WIN32_FIND_DATAW find_data;
	HANDLE search_handle;
	char* current_filename_UTF8;
};

directory_listing_type* create_directory_listing_and_find_first_file(const char* directory, const char* extension) {
	directory_listing_type* directory_listing = calloc(1, sizeof(directory_listing_type));
	char search_pattern[POP_MAX_PATH];
	snprintf_check(search_pattern, POP_MAX_PATH, "%s/*.%s", directory, extension);
	WCHAR* search_pattern_UTF16 = WIN_UTF8ToString(search_pattern);
	directory_listing->search_handle = FindFirstFileW( search_pattern_UTF16, &directory_listing->find_data );
	SDL_free(search_pattern_UTF16);
	if (directory_listing->search_handle != INVALID_HANDLE_VALUE) {
		return directory_listing;
	} else {
		free(directory_listing);
		return NULL;
	}
}

char* get_current_filename_from_directory_listing(directory_listing_type* data) {
	SDL_free(data->current_filename_UTF8);
	data->current_filename_UTF8 = NULL;
	data->current_filename_UTF8 = WIN_StringToUTF8(data->find_data.cFileName);
	return data->current_filename_UTF8;
}

bool find_next_file(directory_listing_type* data) {
	return (bool) FindNextFileW( data->search_handle, &data->find_data );
}

void close_directory_listing(directory_listing_type* data) {
	FindClose(data->search_handle);
	SDL_free(data->current_filename_UTF8);
	data->current_filename_UTF8 = NULL;
	free(data);
}

#else // use dirent.h API for listing files

struct directory_listing_type {
	DIR* dp;
	char* found_filename;
	const char* extension;
};

directory_listing_type* create_directory_listing_and_find_first_file(const char* directory, const char* extension) {
	directory_listing_type* data = calloc(1, sizeof(directory_listing_type));
	bool ok = false;
	data->dp = opendir(directory);
	if (data->dp != NULL) {
		struct dirent* ep;
		while ((ep = readdir(data->dp))) {
			char *ext = strrchr(ep->d_name, '.');
			if (ext != NULL && strcasecmp(ext+1, extension) == 0) {
				data->found_filename = ep->d_name;
				data->extension = extension;
				ok = true;
				break;
			}
		}
	}
	if (ok) {
		return data;
	} else {
		free(data);
		return NULL;
	}
}

char* get_current_filename_from_directory_listing(directory_listing_type* data) {
	return data->found_filename;
}

bool find_next_file(directory_listing_type* data) {
	bool ok = false;
	struct dirent* ep;
	while ((ep = readdir(data->dp))) {
		char *ext = strrchr(ep->d_name, '.');
		if (ext != NULL && strcasecmp(ext+1, data->extension) == 0) {
			data->found_filename = ep->d_name;
			ok = true;
			break;
		}
	}
	return ok;
}

void close_directory_listing(directory_listing_type *data) {
	closedir(data->dp);
	free(data);
}

#endif //_WIN32

dat_type* dat_chain_ptr = NULL;

char last_text_input;

// seg009:000D
int read_key() {
	// stub
	int key = last_key_scancode;
	last_key_scancode = 0;
	return key;
}

// seg009:019A
void clear_kbd_buf() {
	// stub
	last_key_scancode = 0;
	last_text_input = 0;
}

// seg009:040A
word prandom(word max) {
	if (!seed_was_init) {
		// init from current time
		random_seed = (dword)time(NULL);
		seed_was_init = 1;
	}
	random_seed = random_seed * 214013 + 2531011;
	return (random_seed >> 16) % (max + 1);
}

// seg009:0467
int round_xpos_to_byte(int xpos,int round_direction) {
	// stub
	return xpos;
}

// seg009:0C7A
void quit(int exit_code) {
	restore_stuff();
#ifdef POP_RP2350
	// On RP2350, longjmp back to main loop instead of exit()
	// This allows returning to the start screen
	pop_quit_requested = 1;
	longjmp(pop_quit_jmp_buf, exit_code ? exit_code : 1);
#else
	exit(exit_code);
#endif
}

// seg009:0C90
void restore_stuff() {
	SDL_Quit();
}

// seg009:0E33
int key_test_quit() {
	word key = read_key();
	if (key == (SDL_SCANCODE_Q | WITH_CTRL)) { // Ctrl+Q

		#ifdef USE_REPLAY
		if (recording) save_recorded_replay_dialog();
		#endif
		#ifdef USE_MENU
		if (is_menu_shown) menu_was_closed();
		#endif

		quit(0);
	}
	return key;
}

// seg009:0E54
const char* check_param(const char* param) {
	// stub
	for (short arg_index = 1; arg_index < g_argc; ++arg_index) {

		char* curr_arg = g_argv[arg_index];

		// Filenames (e.g. replays) should never be a valid 'normal' param so we should skip these to prevent conflicts.
		// We can lazily distinguish filenames from non-filenames by checking whether they have a dot in them.
		// (Assumption: all relevant files, e.g. replay files, have some file extension anyway)
		if (strchr(curr_arg, '.') != NULL) {
			continue;
		}

		// List of params that expect a specifier ('sub-') arg directly after it (e.g. the mod's name, after "mod" arg)
		// Such sub-args may conflict with the normal params (so, we should 'skip over' them)
		static const char params_with_one_subparam[][16] = { "mod", "validate", /*...*/ };

		bool curr_arg_has_one_subparam = false;
		for (int i = 0; i < COUNT(params_with_one_subparam); ++i) {
			if (strncasecmp(curr_arg, params_with_one_subparam[i], strlen(params_with_one_subparam[i])) == 0) {
				curr_arg_has_one_subparam = true;
				break;
			}
		}

		if (curr_arg_has_one_subparam) {
			// Found an arg that has one sub-param, so we want to:
			// 1: skip over the next arg                (if we are NOT checking for this specific param)
			// 2: return a pointer below to the SUB-arg (if we ARE checking for this specific param)
			++arg_index;
			if (!(arg_index < g_argc)) return NULL; // not enough arguments
		}

		if (/*strnicmp*/strncasecmp(curr_arg, param, strlen(param)) == 0) {
			return g_argv[arg_index];
		}
	}
	return NULL;
}

// seg009:0EDF
int pop_wait(int timer_index,int time) {
	start_timer(timer_index, time);
	return do_wait(timer_index);
}

#ifdef POP_RP2350
static FIL* open_dat_from_root_or_data_dir(const char* filename) {
	FIL* fp = pop_fs_open(filename, "rb");
	if (fp == NULL) {
		char data_path[POP_MAX_PATH];
		snprintf_check(data_path, sizeof(data_path), "prince/%s", filename);
		fp = pop_fs_open(data_path, "rb");
	}
	return fp;
}
#else
static FILE* open_dat_from_root_or_data_dir(const char* filename) {
	FILE* fp = NULL;
	fp = fopen(filename, "rb");

	// if failed, try if the DAT file can be opened in the prince/ directory, instead of the main folder
	if (fp == NULL) {
		char data_path[POP_MAX_PATH];
		snprintf_check(data_path, sizeof(data_path), "prince/%s", filename);

		if (!file_exists(data_path)) {
			find_first_file_match(data_path, sizeof(data_path), "%s/prince/%s", filename);
		}

		// verify that this is a regular file and not a directory (otherwise, don't open)
		struct stat path_stat;
		stat(data_path, &path_stat);
		if (S_ISREG(path_stat.st_mode)) {
			fp = fopen(data_path, "rb");
		}
	}
	return fp;
}
#endif

int showmessage(char* text,int arg_4,void* arg_0);

// seg009:0F58
dat_type* open_dat(const char* filename, int optional) {
	#ifdef POP_RP2350
	FIL* fp = NULL;
	#else
	FILE* fp = NULL;
	#endif
	if (!use_custom_levelset) {
		fp = open_dat_from_root_or_data_dir(filename);
	}
	else {
		// Don't complain about missing data files if we are only looking in the mod folder, because they might exist in the data folder.
		// (This is possible only if open_dat() was called by load_all_sounds().)
		if (!skip_mod_data_files && skip_normal_data_files) optional = 1;

		if (!skip_mod_data_files && !(always_use_original_graphics && optional == 'G')) {
			char filename_mod[POP_MAX_PATH];
			// before checking the root directory, first try mods/MODNAME/
			snprintf_check(filename_mod, sizeof(filename_mod), "%s/%s", mod_data_path, filename);
			#ifdef POP_RP2350
			fp = pop_fs_open(filename_mod, "rb");
			#else
			fp = fopen(filename_mod, "rb");
			#endif
		}
		if (fp == NULL && !skip_normal_data_files) {
			fp = open_dat_from_root_or_data_dir(filename);
		}
	}
	dat_header_type dat_header;
	dat_table_type* dat_table = NULL;

	dat_type* pointer = (dat_type*) calloc(1, sizeof(dat_type));
	snprintf_check(pointer->filename, sizeof(pointer->filename), "%s", filename);
	pointer->next_dat = dat_chain_ptr;
	dat_chain_ptr = pointer;

	if (fp != NULL) {
		#ifdef POP_RP2350
		if (pop_fs_read(&dat_header, 6, 1, fp) != 1) goto failed;
		dat_table = (dat_table_type*) malloc(SDL_SwapLE16(dat_header.table_size));
		if (dat_table == NULL ||
		    pop_fs_seek(fp, (long)SDL_SwapLE32(dat_header.table_offset), SEEK_SET) ||
		    pop_fs_read(dat_table, SDL_SwapLE16(dat_header.table_size), 1, fp) != 1) {
			goto failed;
		}
		pointer->handle = (void*)fp;
		pointer->dat_table = dat_table;
		#else
		if (fread(&dat_header, 6, 1, fp) != 1)
			goto failed;
		dat_table = (dat_table_type*) malloc(SDL_SwapLE16(dat_header.table_size));
		if (dat_table == NULL ||
		    fseek(fp, SDL_SwapLE32(dat_header.table_offset), SEEK_SET) ||
		    fread(dat_table, SDL_SwapLE16(dat_header.table_size), 1, fp) != 1)
			goto failed;
		pointer->handle = fp;
		pointer->dat_table = dat_table;
		#endif
	} else if (optional == 0) {
		// showmessage will crash if we call it before certain things are initialized!
		// Solution: In pop_main(), I moved the first open_dat() call after init_copyprot_dialog().
		// /*
		// There is no DAT file, verify whether the corresponding directory exists.
		char filename_no_ext[POP_MAX_PATH];
		// strip the .DAT file extension from the filename (use folders simply named TITLE, KID, VPALACE, etc.)
		strncpy(filename_no_ext, pointer->filename, sizeof(filename_no_ext));
		size_t len = strlen(filename_no_ext);
		if (len >= 5 && filename_no_ext[len-4] == '.') {
			filename_no_ext[len-4] = '\0'; // terminate, so ".DAT" is deleted from the filename
		}
		char foldername[POP_MAX_PATH];
		snprintf_check(foldername,sizeof(foldername),"prince/%s",filename_no_ext);
		const char* data_path = locate_file(foldername);
		#ifdef POP_RP2350
		FILINFO fno;
		char full[POP_MAX_PATH];
		pop_fs_make_path(full, sizeof(full), data_path);
		FRESULT fr = f_stat(full, &fno);
		if (fr != FR_OK || (fno.fattrib & AM_DIR) == 0) {
		#else
		struct stat path_stat;
		int result = stat(data_path, &path_stat);
		if (result != 0 || !S_ISDIR(path_stat.st_mode)) {
		#endif
			char error_message[256];
			snprintf_check(error_message, sizeof(error_message), "Cannot find a required data file: %s or folder: %s\nPress any key to quit.", filename, foldername);
			#ifdef POP_RP2350
			// showmessage() is RP2350-safe and non-blocking.
			if (onscreen_surface_ != NULL) {
				showmessage(error_message, 1, &key_test_quit);
				quit(1);
			}
			#else
			if (onscreen_surface_ != NULL && copyprot_dialog != NULL) { // otherwise showmessage will crash
				showmessage(error_message, 1, &key_test_quit);
				quit(1);
			}
			#endif
		}
		// */
	}
out:
	// stub
	return pointer;
failed:
	DBG_PRINTF("open_dat failed: %s\n", filename);
	if (fp) pop_fs_close(fp);
#ifndef POP_RP2350
	perror(filename);
	if (fp)
		fclose(fp);
	#endif
	if (dat_table)
		free(dat_table);
	goto out;
}

// seg009:9CAC
void set_loaded_palette(dat_pal_type* palette_ptr) {
	int dest_row, dest_index, source_row;
	for (dest_row = dest_index = source_row = 0; dest_row < 16; ++dest_row, dest_index += 0x10) {
		if (palette_ptr->row_bits & (1 << dest_row)) {
			set_pal_arr(dest_index, 16, palette_ptr->vga + source_row*0x10);
			++source_row;
		}
	}
}

// data:3356
word chtab_palette_bits = 1;

// seg009:104E
chtab_type* load_sprites_from_file(int resource,int palette_bits, int quit_on_error) {
	//int has_palette_bits = 1;
	dat_shpl_type* shpl = (dat_shpl_type*) load_from_opendats_alloc(resource, "pal", NULL, NULL);
	if (shpl == NULL) {
		printf("Can't load sprites from resource %d.\n", resource);
		if (quit_on_error) {
			char error_message[256];
			// Unfortunately we don't know at this point which data file is missing. So we use the name of the last opened DAT file.
			// It's also possible that the DAT file exists and it just doesn't contain the needed resource.
			snprintf_check(error_message, sizeof(error_message), "Can't load sprites from resource %d.\nThe last opened data file is: %s\nPress any key to quit.", resource, dat_chain_ptr->filename);
			showmessage(error_message, 1, &key_test_quit);
			quit(1);
		}
		return NULL;
	}

	dat_pal_type* pal_ptr = &shpl->palette;
	if (graphics_mode == gmMcgaVga) {
		if (palette_bits == 0) {
			/*
			palette_bits = add_palette_bits(pal_ptr->n_colors);
			if (palette_bits == 0) {
				quit(1);
			}
			*/
		} else {
			chtab_palette_bits |= palette_bits;
			//has_palette_bits = 0;
		}
		pal_ptr->row_bits = palette_bits;
	}

	int n_images = shpl->n_images;
	size_t alloc_size = sizeof(chtab_type) + sizeof(void *) * n_images;
	#ifdef POP_RP2350
	chtab_type* chtab = (chtab_type*) psram_malloc(alloc_size);
	#else
	chtab_type* chtab = (chtab_type*) malloc(alloc_size);
	#endif
	if (chtab == NULL) {
		printf("load_sprites_from_file: OOM allocating chtab (size=%d)\n", (int)alloc_size);
		return NULL;
	}
	memset(chtab, 0, alloc_size);
	chtab->n_images = n_images;
	for (int i = 1; i <= n_images; i++) {
		#ifdef POP_RP2350
		if ((i == 1) || ((i & 0x0F) == 0) || (i == n_images)) {
			DBG_PRINTF("[load_sprites_from_file] res %d: %d/%d\n", resource, i, n_images);
			fflush(stdout);
		}
		// Give HDMI DMA time to access PSRAM (avoid bus contention during heavy loading)
		// The 500us delay allows several scanlines worth of DMA to complete
		sleep_us(500);
		#endif
		SDL_Surface* image = load_image(resource + i, pal_ptr);
//		if (image == NULL) printf(" failed");
		if (image != NULL) {
/*
			if (SDL_SetSurfaceAlphaMod(image, 0) != 0) {
				sdlperror("load_sprites_from_file: SDL_SetAlpha");
				quit(1);
			}
*/
			/*
			if (SDL_SetColorKey(image, SDL_SRCCOLORKEY, 0) != 0) {
				sdlperror("load_sprites_from_file: SDL_SetColorKey");
				quit(1);
			}
			*/
		}
//		printf("\n");
		chtab->images[i-1] = image;
	}
	set_loaded_palette(pal_ptr);
	return chtab;
}

// seg009:11A8
void free_chtab(chtab_type *chtab_ptr) {
	image_type* curr_image;
	if (graphics_mode == gmMcgaVga && chtab_ptr->has_palette_bits) {
		chtab_palette_bits &= ~ chtab_ptr->chtab_palette_bits;
	}
	word n_images = chtab_ptr->n_images;
	for (word id = 0; id < n_images; ++id) {
		curr_image = chtab_ptr->images[id];
		if (curr_image) {
			SDL_FreeSurface(curr_image);
		}
	}
	#ifdef POP_RP2350
	psram_free(chtab_ptr);
	#else
	free(chtab_ptr);
	#endif
}

// seg009:8CE6
void decompress_rle_lr(byte* destination,const byte* source,int dest_length) {
	const byte* src_pos = source;
	byte* dest_pos = destination;
	short rem_length = dest_length;
	while (rem_length) {
		sbyte count = *src_pos;
		src_pos++;
		if (count >= 0) { // copy
			++count;
			do {
				*dest_pos = *src_pos;
				dest_pos++;
				src_pos++;
				--rem_length;
				--count;
			} while (count && rem_length);
		} else { // repeat
			byte al = *src_pos;
			src_pos++;
			count = -count;
			do {
				*dest_pos = al;
				dest_pos++;
				--rem_length;
				--count;
			} while (count && rem_length);
		}
	}
}

// seg009:8D1C
void decompress_rle_ud(byte* destination,const byte* source,int dest_length,int width,int height) {
	short rem_height = height;
	const byte* src_pos = source;
	byte* dest_pos = destination;
	short rem_length = dest_length;
	--dest_length;
	--width;
	while (rem_length) {
		sbyte count = *src_pos;
		src_pos++;
		if (count >= 0) { // copy
			++count;
			do {
				*dest_pos = *src_pos;
				dest_pos++;
				src_pos++;
				dest_pos += width;
				--rem_height;
				if (rem_height == 0) {
					dest_pos -= dest_length;
					rem_height = height;
				}
				--rem_length;
				--count;
			} while (count && rem_length);
		} else { // repeat
			byte al = *src_pos;
			src_pos++;
			count = -count;
			do {
				*dest_pos = al;
				dest_pos++;
				dest_pos += width;
				--rem_height;
				if (rem_height == 0) {
					dest_pos -= dest_length;
					rem_height = height;
				}
				--rem_length;
				--count;
			} while (count && rem_length);
		}
	}
}

// seg009:90FA
byte* decompress_lzg_lr(byte* dest,const byte* source,int dest_length) {
	#ifdef POP_RP2350
	byte* window = lzg_window;
	memset(window, 0, 0x400);
	#else
	byte* window = (byte*) malloc(0x400);
	if (window == NULL) return NULL;
	memset(window, 0, 0x400);
	#endif
	byte* window_pos = window + 0x400 - 0x42; // bx
	short remaining = dest_length; // cx
	byte* window_end = window + 0x400; // dx
	const byte* source_pos = source;
	byte* dest_pos = dest;
	word mask = 0;
	do {
		mask >>= 1;
		if ((mask & 0xFF00) == 0) {
			mask = *source_pos | 0xFF00;
			source_pos++;
		}
		if (mask & 1) {
			*window_pos = *dest_pos = *source_pos;
			window_pos++;
			dest_pos++;
			source_pos++;
			if (window_pos >= window_end) window_pos = window;
			--remaining;
		} else {
			word copy_info = *source_pos;
			source_pos++;
			copy_info = (copy_info << 8) | *source_pos;
			source_pos++;
			byte* copy_source = window + (copy_info & 0x3FF);
			byte copy_length = (copy_info >> 10) + 3;
			do {
				*window_pos = *dest_pos = *copy_source;
				window_pos++;
				dest_pos++;
				copy_source++;
				if (copy_source >= window_end) copy_source = window;
				if (window_pos >= window_end) window_pos = window;
				--remaining;
				--copy_length;
			} while (remaining && copy_length);
		}
	} while (remaining);
//	end:
	#ifndef POP_RP2350
	free(window);
	#endif
	return dest;
}

// seg009:91AD
byte* decompress_lzg_ud(byte* dest,const byte* source,int dest_length,int stride,int height) {
	#ifdef POP_RP2350
	byte* window = lzg_window;
	memset(window, 0, 0x400);
	#else
	byte* window = (byte*) malloc(0x400);
	if (window == NULL) return NULL;
	memset(window, 0, 0x400);
	#endif
	byte* window_pos = window + 0x400 - 0x42; // bx
	short remaining = height; // cx
	byte* window_end = window + 0x400; // dx
	const byte* source_pos = source;
	byte* dest_pos = dest;
	word mask = 0;
	short dest_end = dest_length - 1;
	do {
		mask >>= 1;
		if ((mask & 0xFF00) == 0) {
			mask = *source_pos | 0xFF00;
			source_pos++;
		}
		if (mask & 1) {
			*window_pos = *dest_pos = *source_pos;
			window_pos++;
			source_pos++;
			dest_pos += stride;
			--remaining;
			if (remaining == 0) {
				dest_pos -= dest_end;
				remaining = height;
			}
			if (window_pos >= window_end) window_pos = window;
			--dest_length;
		} else {
			word copy_info = *source_pos;
			source_pos++;
			copy_info = (copy_info << 8) | *source_pos;
			source_pos++;
			byte* copy_source = window + (copy_info & 0x3FF);
			byte copy_length = (copy_info >> 10) + 3;
			do {
				*window_pos = *dest_pos = *copy_source;
				window_pos++;
				copy_source++;
				dest_pos += stride;
				--remaining;
				if (remaining == 0) {
					dest_pos -= dest_end;
					remaining = height;
				}
				if (copy_source >= window_end) copy_source = window;
				if (window_pos >= window_end) window_pos = window;
				--dest_length;
				--copy_length;
			} while (dest_length && copy_length);
		}
	} while (dest_length);
//	end:
	#ifndef POP_RP2350
	free(window);
	#endif
	return dest;
}

// seg009:938E
void decompr_img(byte* dest,const image_data_type* source,int decomp_size,int cmeth, int stride) {
	switch (cmeth) {
		case 0: // RAW left-to-right
			memcpy(dest, &source->data, decomp_size);
		break;
		case 1: // RLE left-to-right
			decompress_rle_lr(dest, source->data, decomp_size);
		break;
		case 2: // RLE up-to-down
			decompress_rle_ud(dest, source->data, decomp_size, stride, SDL_SwapLE16(source->height));
		break;
		case 3: // LZG left-to-right
			decompress_lzg_lr(dest, source->data, decomp_size);
		break;
		case 4: // LZG up-to-down
			decompress_lzg_ud(dest, source->data, decomp_size, stride, SDL_SwapLE16(source->height));
		break;
	}
}

int calc_stride(image_data_type* image_data) {
	int width = SDL_SwapLE16(image_data->width);
	int flags = SDL_SwapLE16(image_data->flags);
	int depth = ((flags >> 12) & 7) + 1;
	return (depth * width + 7) / 8;
}

byte* conv_to_8bpp(byte* in_data, int width, int height, int stride, int depth) {
	#ifdef POP_RP2350
	byte* out_data = (byte*) psram_get_scratch_2((size_t)width * (size_t)height);
	#else
	byte* out_data = (byte*) malloc(width * height);
	#endif
	if (out_data == NULL) return NULL;
	int pixels_per_byte = 8 / depth;
	int mask = (1 << depth) - 1;
	for (int y = 0; y < height; ++y) {
		byte* in_pos = in_data + y*stride;
		byte* out_pos = out_data + y*width;
		for (int x_pixel = 0, x_byte = 0; x_byte < stride; ++x_byte) {
			byte v = *in_pos;
			int shift = 8;
			for (int pixel_in_byte = 0; pixel_in_byte < pixels_per_byte && x_pixel < width; ++pixel_in_byte, ++x_pixel) {
				shift -= depth;
				*out_pos = (v >> shift) & mask;
				++out_pos;
			}
			++in_pos;
		}
	}
	return out_data;
}

image_type* decode_image(image_data_type* image_data, dat_pal_type* palette) {
	int height = SDL_SwapLE16(image_data->height);
	if (height == 0) return NULL;
	int width = SDL_SwapLE16(image_data->width);
	int flags = SDL_SwapLE16(image_data->flags);
	int depth = ((flags >> 12) & 7) + 1;
	int cmeth = (flags >> 8) & 0x0F;
	int stride = calc_stride(image_data);
	int dest_size = stride * height;
	#ifdef POP_RP2350
	byte* dest = (byte*) psram_get_scratch_1((size_t)dest_size);
	#else
	byte* dest = (byte*) malloc(dest_size);
	#endif
	if (dest == NULL) {
		printf("decode_image: OOM allocating decompression buffer (%d)\n", dest_size);
		return NULL;
	}
	memset(dest, 0, dest_size);
	decompr_img(dest, image_data, dest_size, cmeth, stride);
	byte* image_8bpp = conv_to_8bpp(dest, width, height, stride, depth);
	if (image_8bpp == NULL) {
		printf("decode_image: OOM allocating conversion buffer (%d)\n", width * height);
		return NULL;
	}
	#ifndef POP_RP2350
	free(dest); dest = NULL;
	#endif
	image_type* image = SDL_CreateRGBSurface(0, width, height, 8, 0, 0, 0, 0);
	if (image == NULL) {
		sdlperror("decode_image: SDL_CreateRGBSurface");
		quit(1);
	}
	if (SDL_LockSurface(image) != 0) {
		sdlperror("decode_image: SDL_LockSurface");
	}
	for (int y = 0; y < height; ++y) {
		// fill image with data
		memcpy((byte*)image->pixels + y*image->pitch, image_8bpp + y*width, width);
	}
	SDL_UnlockSurface(image);
	#ifndef POP_RP2350
	free(image_8bpp); image_8bpp = NULL;
	#endif
	SDL_Color colors[16];
	for (int i = 0; i < 16; ++i) {
		colors[i].r = palette->vga[i].r << 2;
		colors[i].g = palette->vga[i].g << 2;
		colors[i].b = palette->vga[i].b << 2;
		colors[i].a = SDL_ALPHA_OPAQUE;   // SDL2's SDL_Color has a fourth alpha component
	}
	// Force 0th color to be black for non-transparent blitters. (hitpoints, shadow)
	// This is needed to remove the colored rectangles around hitpoints and the shadow, when using Brain's SNES graphics for example.
	colors[0].r = 0;
	colors[0].g = 0;
	colors[0].b = 0;
	colors[0].a = SDL_ALPHA_TRANSPARENT;
	SDL_SetPaletteColors(image->format->palette, colors, 0, 16); // SDL_SetColors = deprecated
	return image;
}

// seg009:121A
static const char* get_sprite_name(int res_id) {
	// Map resource IDs to sprite names
	if (res_id >= 800 && res_id <= 817) return "KID";
	if (res_id >= 850 && res_id <= 888) return "GUARD/JAFFAR";
	if (res_id >= 950 && res_id <= 962) return "CUTSCENE";
	if (res_id == 980 || res_id == 981) return "PRINCESS";
	if (res_id >= 1000 && res_id <= 1255) return "FONT";
	return "?";
}

image_type* load_image(int resource_id, dat_pal_type* palette) {
	// stub
	DBG_PRINTF("[load_image] res=%d (%s)\n", resource_id, get_sprite_name(resource_id));
	
	data_location result;
	int size;
	void* image_data = load_from_opendats_alloc(resource_id, "png", &result, &size);
	image_type* image = NULL;
	switch (result) {
		case data_none:
			return NULL;
		break;
		case data_DAT: { // DAT
			image = decode_image((image_data_type*) image_data, palette);
		} break;
		case data_directory: { // directory
			SDL_RWops* rw = SDL_RWFromConstMem(image_data, size);
			if (rw == NULL) {
				sdlperror("load_image: SDL_RWFromConstMem");
				return NULL;
			}
			image = IMG_Load_RW(rw, 0);
			if (image == NULL) {
				printf("load_image: IMG_Load_RW: %s\n", IMG_GetError());
			}
			if (SDL_RWclose(rw) != 0) {
				sdlperror("load_image: SDL_RWclose");
			}
#ifdef POP_RP2350
			// RP2350: Convert 32bpp RGBA to 8bpp indexed to save PSRAM (4x memory reduction).
			// PNG files from directories are loaded as 32bpp RGBA by stb_image.
			// We convert them to 8bpp indexed using the same palette as DAT images.
			// EXCEPTION: Fonts (resource 1000-1255) have n_colors=0 and are monochrome.
			// They need special handling - convert to 8bpp but use a simple black/white palette.
			DBG_PRINTF("[load_image] res=%d PNG: img=%p bpp=%d pal=%p n_colors=%d\n", 
				resource_id, (void*)image, 
				image ? image->format->BytesPerPixel : 0, 
				(void*)palette,
				palette ? palette->n_colors : -1);
			if (image != NULL && image->format->BytesPerPixel == 4 && palette != NULL) {
				int w = image->w;
				int h = image->h;
				DBG_PRINTF("[load_image] res=%d converting RGBA %dx%d to 8bpp\n", resource_id, w, h);
				// Create 8bpp indexed surface
				SDL_Surface* indexed = SDL_CreateRGBSurface(0, w, h, 8, 0, 0, 0, 0);
				if (indexed != NULL) {
					// Set palette from DAT palette
					SDL_Color colors[16];
					int is_font = (palette->n_colors == 0);
					
					if (is_font) {
						// Font images: monochrome, index 0 = transparent black, index 1 = white
						// The actual color is applied at render time by method_3_blit_mono
						colors[0].r = 0;   colors[0].g = 0;   colors[0].b = 0;   colors[0].a = SDL_ALPHA_TRANSPARENT;
						colors[1].r = 255; colors[1].g = 255; colors[1].b = 255; colors[1].a = SDL_ALPHA_OPAQUE;
						for (int i = 2; i < 16; ++i) {
							colors[i] = colors[1]; // Fill rest with white
						}
					} else {
						// Normal sprites: use DAT palette
						for (int i = 0; i < 16; ++i) {
							colors[i].r = palette->vga[i].r << 2;
							colors[i].g = palette->vga[i].g << 2;
							colors[i].b = palette->vga[i].b << 2;
							colors[i].a = (i == 0) ? SDL_ALPHA_TRANSPARENT : SDL_ALPHA_OPAQUE;
						}
					}
					SDL_SetPaletteColors(indexed->format->palette, colors, 0, 16);
					
					// Convert each pixel: find closest palette match or use alpha for transparency
					uint32_t* src = (uint32_t*)image->pixels;
					uint8_t* dst = (uint8_t*)indexed->pixels;
					int src_pitch = image->pitch / 4;
					int dst_pitch = indexed->pitch;
					
					// Debug: print first few RGBA values for fonts
					if (is_font && resource_id >= 1076 && resource_id <= 1078) {
						DBG_PRINTF("[load_image] res=%d RGBA row0:", resource_id);
						for (int x = 0; x < w && x < 6; ++x) {
							uint32_t px = src[x];
							DBG_PRINTF(" %08X", px);
						}
						DBG_PRINTF("\n");
					}
					
					for (int y = 0; y < h; ++y) {
						// Yield every 16 rows to let HDMI DMA access PSRAM
						if ((y & 0x0F) == 0) sleep_us(50);
						for (int x = 0; x < w; ++x) {
							uint32_t pixel = src[y * src_pitch + x];
							// Extract RGBA (assuming RGBA8888 byte order from stb_image)
							uint8_t r = (pixel >> 0) & 0xFF;
							uint8_t g = (pixel >> 8) & 0xFF;
							uint8_t b = (pixel >> 16) & 0xFF;
							uint8_t a = (pixel >> 24) & 0xFF;
							
							if (a < 128) {
								// Transparent pixel -> index 0
								dst[y * dst_pitch + x] = 0;
							} else if (is_font) {
								// Font PNGs are white text on black background (both opaque)
								// Detect foreground by brightness: white pixels are foreground
								int brightness = (int)r + (int)g + (int)b;
								dst[y * dst_pitch + x] = (brightness > 384) ? 1 : 0; // threshold ~50% gray
							} else {
								// Find closest palette color (simple brute force)
								int best_idx = 1; // Skip 0 (transparent)
								int best_dist = 999999;
								for (int i = 1; i < 16; ++i) {
									int dr = (int)colors[i].r - (int)r;
									int dg = (int)colors[i].g - (int)g;
									int db = (int)colors[i].b - (int)b;
									int dist = dr*dr + dg*dg + db*db;
									if (dist < best_dist) {
										best_dist = dist;
										best_idx = i;
									}
								}
								dst[y * dst_pitch + x] = (uint8_t)best_idx;
							}
						}
					}
					
					// Replace RGBA surface with indexed surface
					SDL_FreeSurface(image);
					image = indexed;
					DBG_PRINTF("[load_image] res=%d converted to 8bpp OK\n", resource_id);
				} else {
					DBG_PRINTF("[load_image] res=%d convert FAILED (indexed surface NULL)\n", resource_id);
				}
			}
#endif
		} break;
	}
	if (image_data != NULL) {
		#ifdef POP_RP2350
		pop_heap_free(image_data);
		#else
		free(image_data);
		#endif
	}


	if (image != NULL) {
		// should immediately start using the onscreen pixel format, so conversion will not be needed

		#ifdef POP_RP2350
		// Debug: Check the actual pixel data
		if (!image->format->palette && image->format->BytesPerPixel == 4) {
			// RGBA surface - check the actual alpha values in first few pixels
			uint32_t* pixels = (uint32_t*)image->pixels;
			DBG_PRINTF("[load_image] res=%d RGBA pixels (hex): ", resource_id);
			for (int i = 0; i < 4 && i < image->w * image->h; ++i) {
				DBG_PRINTF("%08x ", pixels[i]);
			}
			DBG_PRINTF("\n");
		}
		#endif
		
		// Apply colorkey to all surfaces for transparency
		// Black (value 0) is the transparent color
		if (SDL_SetColorKey(image, SDL_TRUE, 0) != 0) {
			// Ignore errors - not all surfaces support colorkey
		}
/*
		if (SDL_SetSurfaceAlphaMod(image, 0) != 0) { //sdl 1.2: SDL_SetAlpha removed
			sdlperror("load_image: SDL_SetAlpha");
			quit(1);
		}
*/
//		image_type* colored_image = SDL_ConvertSurfaceFormat(image, SDL_PIXELFORMAT_ARGB8888, 0);
//		if (!colored_image) {
//			sdlperror("load_image: SDL_ConvertSurfaceFormat");
//			quit(1);
//		}
//		SDL_FreeSurface(image);
//		image = colored_image;
	}
	return image;
}

// seg009:13C4
void draw_image_transp(image_type* image,image_type* mask,int xpos,int ypos) {
	if (graphics_mode == gmMcgaVga) {
		draw_image_transp_vga(image, xpos, ypos);
	} else {
		// ...
	}
}

// seg009:157E
int set_joy_mode() {
	// stub
	if (SDL_NumJoysticks() < 1) {
		is_joyst_mode = 0;
	} else {
		if (gamecontrollerdb_file[0] != '\0') {
			SDL_GameControllerAddMappingsFromFile(gamecontrollerdb_file);
		}

		if (SDL_IsGameController(0)) {
			sdl_controller_ = SDL_GameControllerOpen(0);
			if (sdl_controller_ == NULL) {
				is_joyst_mode = 0;
			} else {
				is_joyst_mode = 1;
			}
		}
		// We have a joystick connected, but it's NOT compatible with the SDL_GameController
		// interface, so we resort to the classic SDL_Joystick interface instead
		else {
			sdl_joystick_ = SDL_JoystickOpen(0);
			is_joyst_mode = 1;
			using_sdl_joystick_interface = 1;
		}
	}
	if (enable_controller_rumble && is_joyst_mode) {
		sdl_haptic = SDL_HapticOpen(0);
		SDL_HapticRumbleInit(sdl_haptic); // initialize the device for simple rumble
	} else {
		sdl_haptic = NULL;
	}

	is_keyboard_mode = !is_joyst_mode;
	return is_joyst_mode;
}

// seg009:178B
surface_type* make_offscreen_buffer(const rect_type* rect) {
	// stub
#ifndef USE_ALPHA
	// Bit order matches onscreen buffer, good for fading.
	// On RP2350 we run an 8bpp indexed onscreen buffer; the offscreen buffer must
	// match that format or fade/blit routines will treat 24/32bpp pixel bytes as
	// palette indices and corrupt the onscreen buffer.
	#ifdef POP_RP2350
	surface_type* surface = SDL_CreateRGBSurface(0, rect->right, rect->bottom, 8, 0, 0, 0, 0);
	if (surface && onscreen_surface_ && onscreen_surface_->format) {
		SDL_SurfaceAdoptPalette(surface, onscreen_surface_->format->palette);
	}
	return surface;
	#else
	return SDL_CreateRGBSurface(0, rect->right, rect->bottom, 24, Rmsk, Gmsk, Bmsk, 0);
	#endif
#else
	return SDL_CreateRGBSurface(0, rect->right, rect->bottom, 32, Rmsk, Gmsk, Bmsk, Amsk);
#endif
	//return surface;
}

// seg009:17BD
void free_surface(surface_type* surface) {
	SDL_FreeSurface(surface);
}

// seg009:17EA
void free_peel(peel_type* peel_ptr) {
	SDL_FreeSurface(peel_ptr->peel);
	free(peel_ptr);
}

// seg009:182F
void set_hc_pal() {
	// stub
	if (graphics_mode == gmMcgaVga) {
		set_pal_arr(0, 16, custom->vga_palette);
	} else {
		// ...
	}
}

// seg009:2446
void flip_not_ega(byte* memory,int height,int stride) {
	byte* row_buffer = (byte*) malloc(stride);
	byte* top_ptr;
	byte* bottom_ptr;
	bottom_ptr = top_ptr = memory;
	bottom_ptr += (height - 1) * stride;
	short rem_rows = height >> 1;
	do {
		memcpy(row_buffer, top_ptr, stride);
		memcpy(top_ptr, bottom_ptr, stride);
		memcpy(bottom_ptr, row_buffer, stride);
		top_ptr += stride;
		bottom_ptr -= stride;
		--rem_rows;
	} while (rem_rows);
	free(row_buffer);
}

// seg009:19B1
void flip_screen(surface_type* surface) {
	// stub
	if (graphics_mode != gmEga) {
		if (SDL_LockSurface(surface) != 0) {
			sdlperror("flip_screen: SDL_LockSurface");
			quit(1);
		}
		flip_not_ega((byte*) surface->pixels, surface->h, surface->pitch);
		SDL_UnlockSurface(surface);
	} else {
		// ...
	}
}

#ifndef USE_FADE
// seg009:19EF
void fade_in_2(surface_type* source_surface,int which_rows) {
	// stub
	method_1_blit_rect(onscreen_surface_, source_surface, &screen_rect, &screen_rect, 0);
#ifdef POP_RP2350
	// Reset HDMI fade level to full brightness since we skipped software fade
	graphics_set_fade_level(0, 0);
	update_screen();
#endif
}

// seg009:1CC9
void fade_out_2(int rows) {
	// stub
}
#endif // USE_FADE

// seg009:2288
void draw_image_transp_vga(image_type* image,int xpos,int ypos) {
	// stub
	method_6_blit_img_to_scr(image, xpos, ypos, blitters_10h_transp);
}

#ifdef USE_TEXT


/*const*/ byte hc_font_data[] = {
0x20,0x83,0x07,0x00,0x02,0x00,0x01,0x00,0x01,0x00,0xD2,0x00,0xD8,0x00,0xE5,0x00,
0xEE,0x00,0xFA,0x00,0x07,0x01,0x14,0x01,0x21,0x01,0x2A,0x01,0x37,0x01,0x44,0x01,
0x50,0x01,0x5C,0x01,0x6A,0x01,0x74,0x01,0x81,0x01,0x8E,0x01,0x9B,0x01,0xA8,0x01,
0xB5,0x01,0xC2,0x01,0xCF,0x01,0xDC,0x01,0xE9,0x01,0xF6,0x01,0x03,0x02,0x10,0x02,
0x1C,0x02,0x2A,0x02,0x37,0x02,0x42,0x02,0x4F,0x02,0x5C,0x02,0x69,0x02,0x76,0x02,
0x83,0x02,0x90,0x02,0x9D,0x02,0xAA,0x02,0xB7,0x02,0xC4,0x02,0xD1,0x02,0xDE,0x02,
0xEB,0x02,0xF8,0x02,0x05,0x03,0x12,0x03,0x1F,0x03,0x2C,0x03,0x39,0x03,0x46,0x03,
0x53,0x03,0x60,0x03,0x6D,0x03,0x7A,0x03,0x87,0x03,0x94,0x03,0xA1,0x03,0xAE,0x03,
0xBB,0x03,0xC8,0x03,0xD5,0x03,0xE2,0x03,0xEB,0x03,0xF9,0x03,0x02,0x04,0x0F,0x04,
0x1C,0x04,0x29,0x04,0x36,0x04,0x43,0x04,0x50,0x04,0x5F,0x04,0x6C,0x04,0x79,0x04,
0x88,0x04,0x95,0x04,0xA2,0x04,0xAF,0x04,0xBC,0x04,0xC9,0x04,0xD8,0x04,0xE7,0x04,
0xF4,0x04,0x01,0x05,0x0E,0x05,0x1B,0x05,0x28,0x05,0x35,0x05,0x42,0x05,0x51,0x05,
0x5E,0x05,0x6B,0x05,0x78,0x05,0x85,0x05,0x8D,0x05,0x9A,0x05,0xA7,0x05,0xBB,0x05,
0xD9,0x05,0x00,0x00,0x03,0x00,0x00,0x00,0x07,0x00,0x02,0x00,0x01,0x00,0xC0,0xC0,
0xC0,0xC0,0xC0,0x00,0xC0,0x03,0x00,0x05,0x00,0x01,0x00,0xD8,0xD8,0xD8,0x06,0x00,
0x07,0x00,0x01,0x00,0x00,0x6C,0xFE,0x6C,0xFE,0x6C,0x07,0x00,0x07,0x00,0x01,0x00,
0x10,0x7C,0xD0,0x7C,0x16,0x7C,0x10,0x07,0x00,0x08,0x00,0x01,0x00,0xC3,0xC6,0x0C,
0x18,0x30,0x63,0xC3,0x07,0x00,0x08,0x00,0x01,0x00,0x38,0x6C,0x38,0x7A,0xCC,0xCE,
0x7B,0x03,0x00,0x03,0x00,0x01,0x00,0x60,0x60,0xC0,0x07,0x00,0x04,0x00,0x01,0x00,
0x30,0x60,0xC0,0xC0,0xC0,0x60,0x30,0x07,0x00,0x04,0x00,0x01,0x00,0xC0,0x60,0x30,
0x30,0x30,0x60,0xC0,0x06,0x00,0x07,0x00,0x01,0x00,0x00,0x6C,0x38,0xFE,0x38,0x6C,
0x06,0x00,0x06,0x00,0x01,0x00,0x00,0x30,0x30,0xFC,0x30,0x30,0x08,0x00,0x03,0x00,
0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x60,0xC0,0x04,0x00,0x04,0x00,0x01,0x00,
0x00,0x00,0x00,0xF0,0x07,0x00,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0xC0,
0xC0,0x07,0x00,0x08,0x00,0x01,0x00,0x03,0x06,0x0C,0x18,0x30,0x60,0xC0,0x07,0x00,
0x06,0x00,0x01,0x00,0x78,0xCC,0xCC,0xCC,0xCC,0xCC,0x78,0x07,0x00,0x06,0x00,0x01,
0x00,0x30,0x70,0xF0,0x30,0x30,0x30,0xFC,0x07,0x00,0x06,0x00,0x01,0x00,0x78,0xCC,
0x0C,0x18,0x30,0x60,0xFC,0x07,0x00,0x06,0x00,0x01,0x00,0x78,0xCC,0x0C,0x18,0x0C,
0xCC,0x78,0x07,0x00,0x07,0x00,0x01,0x00,0x1C,0x3C,0x6C,0xCC,0xFE,0x0C,0x0C,0x07,
0x00,0x06,0x00,0x01,0x00,0xF8,0xC0,0xC0,0xF8,0x0C,0x0C,0xF8,0x07,0x00,0x06,0x00,
0x01,0x00,0x78,0xC0,0xC0,0xF8,0xCC,0xCC,0x78,0x07,0x00,0x06,0x00,0x01,0x00,0xFC,
0x0C,0x18,0x30,0x30,0x30,0x30,0x07,0x00,0x06,0x00,0x01,0x00,0x78,0xCC,0xCC,0x78,
0xCC,0xCC,0x78,0x07,0x00,0x06,0x00,0x01,0x00,0x78,0xCC,0xCC,0x7C,0x0C,0xCC,0x78,
0x06,0x00,0x02,0x00,0x01,0x00,0x00,0xC0,0xC0,0x00,0xC0,0xC0,0x08,0x00,0x03,0x00,
0x01,0x00,0x00,0x60,0x60,0x00,0x00,0x60,0x60,0xC0,0x07,0x00,0x05,0x00,0x01,0x00,
0x18,0x30,0x60,0xC0,0x60,0x30,0x18,0x05,0x00,0x04,0x00,0x01,0x00,0x00,0x00,0xF0,
0x00,0xF0,0x07,0x00,0x05,0x00,0x01,0x00,0xC0,0x60,0x30,0x18,0x30,0x60,0xC0,0x07,
0x00,0x06,0x00,0x01,0x00,0x78,0xCC,0x0C,0x18,0x30,0x00,0x30,0x07,0x00,0x06,0x00,
0x01,0x00,0x78,0xCC,0xDC,0xDC,0xD8,0xC0,0x78,0x07,0x00,0x06,0x00,0x01,0x00,0x78,
0xCC,0xCC,0xFC,0xCC,0xCC,0xCC,0x07,0x00,0x06,0x00,0x01,0x00,0xF8,0xCC,0xCC,0xF8,
0xCC,0xCC,0xF8,0x07,0x00,0x06,0x00,0x01,0x00,0x78,0xCC,0xC0,0xC0,0xC0,0xCC,0x78,
0x07,0x00,0x06,0x00,0x01,0x00,0xF8,0xCC,0xCC,0xCC,0xCC,0xCC,0xF8,0x07,0x00,0x05,
0x00,0x01,0x00,0xF8,0xC0,0xC0,0xF0,0xC0,0xC0,0xF8,0x07,0x00,0x05,0x00,0x01,0x00,
0xF8,0xC0,0xC0,0xF0,0xC0,0xC0,0xC0,0x07,0x00,0x06,0x00,0x01,0x00,0x78,0xCC,0xC0,
0xDC,0xCC,0xCC,0x78,0x07,0x00,0x06,0x00,0x01,0x00,0xCC,0xCC,0xCC,0xFC,0xCC,0xCC,
0xCC,0x07,0x00,0x04,0x00,0x01,0x00,0xF0,0x60,0x60,0x60,0x60,0x60,0xF0,0x07,0x00,
0x06,0x00,0x01,0x00,0x0C,0x0C,0x0C,0x0C,0x0C,0xCC,0x78,0x07,0x00,0x07,0x00,0x01,
0x00,0xC6,0xCC,0xD8,0xF0,0xD8,0xCC,0xC6,0x07,0x00,0x05,0x00,0x01,0x00,0xC0,0xC0,
0xC0,0xC0,0xC0,0xC0,0xF8,0x07,0x00,0x08,0x00,0x01,0x00,0xC3,0xE7,0xFF,0xDB,0xC3,
0xC3,0xC3,0x07,0x00,0x06,0x00,0x01,0x00,0xCC,0xCC,0xEC,0xFC,0xDC,0xCC,0xCC,0x07,
0x00,0x06,0x00,0x01,0x00,0x78,0xCC,0xCC,0xCC,0xCC,0xCC,0x78,0x07,0x00,0x06,0x00,
0x01,0x00,0xF8,0xCC,0xCC,0xF8,0xC0,0xC0,0xC0,0x07,0x00,0x06,0x00,0x01,0x00,0x78,
0xCC,0xCC,0xCC,0xCC,0xD8,0x6C,0x07,0x00,0x06,0x00,0x01,0x00,0xF8,0xCC,0xCC,0xF8,
0xD8,0xCC,0xCC,0x07,0x00,0x06,0x00,0x01,0x00,0x78,0xCC,0xC0,0x78,0x0C,0xCC,0x78,
0x07,0x00,0x06,0x00,0x01,0x00,0xFC,0x30,0x30,0x30,0x30,0x30,0x30,0x07,0x00,0x06,
0x00,0x01,0x00,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0x7C,0x07,0x00,0x06,0x00,0x01,0x00,
0xCC,0xCC,0xCC,0xCC,0xCC,0x78,0x30,0x07,0x00,0x08,0x00,0x01,0x00,0xC3,0xC3,0xC3,
0xDB,0xFF,0xE7,0xC3,0x07,0x00,0x06,0x00,0x01,0x00,0xCC,0xCC,0x78,0x30,0x78,0xCC,
0xCC,0x07,0x00,0x06,0x00,0x01,0x00,0xCC,0xCC,0xCC,0x78,0x30,0x30,0x30,0x07,0x00,
0x08,0x00,0x01,0x00,0xFF,0x06,0x0C,0x18,0x30,0x60,0xFF,0x07,0x00,0x04,0x00,0x01,
0x00,0xF0,0xC0,0xC0,0xC0,0xC0,0xC0,0xF0,0x07,0x00,0x08,0x00,0x01,0x00,0xC0,0x60,
0x30,0x18,0x0C,0x06,0x03,0x07,0x00,0x04,0x00,0x01,0x00,0xF0,0x30,0x30,0x30,0x30,
0x30,0xF0,0x03,0x00,0x06,0x00,0x01,0x00,0x30,0x78,0xCC,0x08,0x00,0x06,0x00,0x01,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x03,0x00,0x04,0x00,0x01,0x00,0xC0,
0x60,0x30,0x07,0x00,0x06,0x00,0x01,0x00,0x00,0x00,0x78,0x0C,0x7C,0xCC,0x7C,0x07,
0x00,0x06,0x00,0x01,0x00,0xC0,0xC0,0xF8,0xCC,0xCC,0xCC,0xF8,0x07,0x00,0x06,0x00,
0x01,0x00,0x00,0x00,0x78,0xCC,0xC0,0xCC,0x78,0x07,0x00,0x06,0x00,0x01,0x00,0x0C,
0x0C,0x7C,0xCC,0xCC,0xCC,0x7C,0x07,0x00,0x06,0x00,0x01,0x00,0x00,0x00,0x78,0xCC,
0xFC,0xC0,0x7C,0x07,0x00,0x05,0x00,0x01,0x00,0x38,0x60,0xF8,0x60,0x60,0x60,0x60,
0x09,0x00,0x06,0x00,0x01,0x00,0x00,0x00,0x78,0xCC,0xCC,0xCC,0x7C,0x0C,0x78,0x07,
0x00,0x06,0x00,0x01,0x00,0xC0,0xC0,0xF8,0xCC,0xCC,0xCC,0xCC,0x07,0x00,0x02,0x00,
0x01,0x00,0xC0,0x00,0xC0,0xC0,0xC0,0xC0,0xC0,0x09,0x00,0x04,0x00,0x01,0x00,0x30,
0x00,0x30,0x30,0x30,0x30,0x30,0x30,0xE0,0x07,0x00,0x06,0x00,0x01,0x00,0xC0,0xC0,
0xCC,0xD8,0xF0,0xD8,0xCC,0x07,0x00,0x02,0x00,0x01,0x00,0xC0,0xC0,0xC0,0xC0,0xC0,
0xC0,0xC0,0x07,0x00,0x08,0x00,0x01,0x00,0x00,0x00,0xFE,0xDB,0xDB,0xDB,0xDB,0x07,
0x00,0x06,0x00,0x01,0x00,0x00,0x00,0xF8,0xCC,0xCC,0xCC,0xCC,0x07,0x00,0x06,0x00,
0x01,0x00,0x00,0x00,0x78,0xCC,0xCC,0xCC,0x78,0x09,0x00,0x06,0x00,0x01,0x00,0x00,
0x00,0xF8,0xCC,0xCC,0xCC,0xF8,0xC0,0xC0,0x09,0x00,0x06,0x00,0x01,0x00,0x00,0x00,
0x78,0xCC,0xCC,0xCC,0x7C,0x0C,0x0C,0x07,0x00,0x06,0x00,0x01,0x00,0x00,0x00,0x78,
0xCC,0xC0,0xC0,0xC0,0x07,0x00,0x06,0x00,0x01,0x00,0x00,0x00,0x78,0xC0,0x78,0x0C,
0xF8,0x07,0x00,0x05,0x00,0x01,0x00,0x60,0x60,0xF8,0x60,0x60,0x60,0x38,0x07,0x00,
0x06,0x00,0x01,0x00,0x00,0x00,0xCC,0xCC,0xCC,0xCC,0x7C,0x07,0x00,0x06,0x00,0x01,
0x00,0x00,0x00,0xCC,0xCC,0xCC,0x78,0x30,0x07,0x00,0x08,0x00,0x01,0x00,0x00,0x00,
0xC3,0xC3,0xDB,0xFF,0x66,0x07,0x00,0x06,0x00,0x01,0x00,0x00,0x00,0xCC,0x78,0x30,
0x78,0xCC,0x09,0x00,0x06,0x00,0x01,0x00,0x00,0x00,0xCC,0xCC,0xCC,0xCC,0x7C,0x0C,
0x78,0x07,0x00,0x06,0x00,0x01,0x00,0x00,0x00,0xFC,0x18,0x30,0x60,0xFC,0x07,0x00,
0x04,0x00,0x01,0x00,0x30,0x60,0x60,0xC0,0x60,0x60,0x30,0x07,0x00,0x02,0x00,0x01,
0x00,0xC0,0xC0,0xC0,0x00,0xC0,0xC0,0xC0,0x07,0x00,0x04,0x00,0x01,0x00,0xC0,0x60,
0x60,0x30,0x60,0x60,0xC0,0x02,0x00,0x07,0x00,0x01,0x00,0x76,0xDC,0x07,0x00,0x07,
0x00,0x01,0x00,0x00,0x00,0x70,0xC4,0xCC,0x8C,0x38,0x07,0x00,0x07,0x00,0x01,0x00,
0x00,0x06,0x0C,0xD8,0xF0,0xE0,0xC0,0x08,0x00,0x10,0x00,0x02,0x00,0x7F,0xFE,0xCD,
0xC7,0xB5,0xEF,0xB5,0xEF,0x85,0xEF,0xB5,0xEF,0xB4,0x6F,0x08,0x00,0x13,0x00,0x03,
0x00,0x7F,0xFF,0xC0,0xCC,0x46,0xE0,0xB6,0xDA,0xE0,0xBE,0xDA,0xE0,0xBE,0xC6,0xE0,
0xB6,0xDA,0xE0,0xCE,0xDA,0x20,0x7F,0xFF,0xC0,0x08,0x00,0x11,0x00,0x03,0x00,0x7F,
0xFF,0x00,0xC6,0x73,0x80,0xDD,0xAD,0x80,0xCE,0xEF,0x80,0xDF,0x6F,0x80,0xDD,0xAD,
0x80,0xC6,0x73,0x80,0x7F,0xFF,0x00
};

static void load_font_character_offsets(rawfont_type* data) {
	int n_chars = data->last_char - data->first_char + 1;
	byte* pos = (byte*) &data->offsets[n_chars];
	for (int index = 0; index < n_chars; ++index) {
		data->offsets[index] = SDL_SwapLE16(pos - (byte*) data);
		image_data_type* image_data = (image_data_type*) pos;
		int image_bytes = SDL_SwapLE16(image_data->height) * calc_stride(image_data);
		pos = (byte*) &image_data->data + image_bytes;
	}
}

font_type load_font_from_data(/*const*/ rawfont_type* data) {
	font_type font;
	font.first_char = data->first_char;
	font.last_char = data->last_char;
	font.height_above_baseline = SDL_SwapLE16(data->height_above_baseline);
	font.height_below_baseline = SDL_SwapLE16(data->height_below_baseline);
	font.space_between_lines = SDL_SwapLE16(data->space_between_lines);
	font.space_between_chars = SDL_SwapLE16(data->space_between_chars);
	int n_chars = font.last_char - font.first_char + 1;
	// Allow loading a font even if the offsets for each character image were not supplied in the raw data.
	if (SDL_SwapLE16(data->offsets[0]) == 0) {
		load_font_character_offsets(data);
	}
	chtab_type* chtab = malloc(sizeof(chtab_type) + sizeof(image_type*) * n_chars);
	// Make a dummy palette for decode_image().
	dat_pal_type dat_pal;
	memset(&dat_pal, 0, sizeof(dat_pal));
	dat_pal.vga[1].r = dat_pal.vga[1].g = dat_pal.vga[1].b = 0x3F; // white
	for (int index = 0, chr = data->first_char; chr <= data->last_char; ++index, ++chr) {
		/*const*/ image_data_type* image_data = (/*const*/ image_data_type*)((/*const*/ byte*)data + SDL_SwapLE16(data->offsets[index]));
		//image_data->flags=0;
		if (image_data->height == SDL_SwapLE16(0)) image_data->height = SDL_SwapLE16(1); // HACK: decode_image() returns NULL if height==0.
		image_type* image;
		chtab->images[index] = image = decode_image(image_data, &dat_pal);
		if (SDL_SetColorKey(image, SDL_TRUE, 0) != 0) {
			sdlperror("load_font_from_data: SDL_SetColorKey");
			quit(1);
		}
	}
	font.chtab = chtab;
	return font;
}

// Small font data (hardcoded), defined in menu.c
extern byte hc_small_font_data[];

void load_font(void) {
	// Try to load font from files (DAT and/or data/font directory).
	const char* const dat_name = "font";
	DBG_PRINTF("[load_font] open_dat(%s)\n", dat_name);
	dat_type* dathandle = open_dat(dat_name, 1);
	DBG_PRINTF("[load_font] load_sprites_from_file(1000)\n");
	hc_font.chtab = load_sprites_from_file(1000, 1<<1, 0);
	DBG_PRINTF("[load_font] close_dat(%s)\n", dat_name);
	close_dat(dathandle);
	if (hc_font.chtab == NULL) {
		// Use built-in font.
		DBG_PRINTF("[load_font] falling back to built-in font\n");
		hc_font = load_font_from_data((/*const*/ rawfont_type*)hc_font_data);
	}

#ifdef USE_MENU
	DBG_PRINTF("[load_font] build small font\n");
	hc_small_font = load_font_from_data((rawfont_type*)hc_small_font_data);
#endif
	DBG_PRINTF("[load_font] done\n");

}

// seg009:35C5
int get_char_width(byte character) {
	font_type* font = textstate.ptr_font;
	int width = 0;
	if (character <= font->last_char && character >= font->first_char) {
		image_type* image = font->chtab->images[character - font->first_char];
		if (image != NULL) {
			width += image->w; //char_ptrs[character - font->first_char]->width;
			if (width) width += font->space_between_chars;
		}
	}
	return width;
}

// seg009:3E99
int find_linebreak(const char* text,int length,int break_width,int x_align) {
	int curr_char_pos = 0;
	short last_break_pos = 0; // in characters
	short curr_line_width = 0; // in pixels
	const char* text_pos = text;
	while (curr_char_pos < length) {
		curr_line_width += get_char_width(*text_pos);
		if (curr_line_width <= break_width) {
			++curr_char_pos;
			char curr_char = *text_pos;
			text_pos++;
			if (curr_char == '\n') {
				return curr_char_pos;
			}
			if (curr_char == '-' ||
				(x_align <= 0 && (curr_char == ' ' || *text_pos == ' ')) ||
				(*text_pos == ' ' && curr_char == ' ')
			) {
				// May break here.
				last_break_pos = curr_char_pos;
			}
		} else {
			if (last_break_pos == 0) {
				// If the first word is wider than break_width then break it.
				return curr_char_pos;
			} else {
				// Otherwise break at the last space.
				return last_break_pos;
			}
		}
	}
	return curr_char_pos;
}

// seg009:403F
int get_line_width(const char* text,int length) {
	int width = 0;
	const char* text_pos = text;
	while (--length >= 0) {
		width += get_char_width(*text_pos);
		text_pos++;
	}
	return width;
}

// seg009:3706
int draw_text_character(byte character) {
	//printf("going to do draw_text_character...\n");
	#ifdef POP_RP2350
	static int rp2350_text_chars = 0;
	if (rp2350_text_chars < 32) {
		DBG_PRINTF("[draw_text_character] #%d ch=0x%02X\n", rp2350_text_chars, (unsigned)character);
	}
	++rp2350_text_chars;
	#endif
	font_type* font = textstate.ptr_font;
	int width = 0;
	if (character <= font->last_char && character >= font->first_char) {
		image_type* image = font->chtab->images[character - font->first_char]; //char_ptrs[character - font->first_char];
		if (image != NULL) {
			#ifdef POP_RP2350
			if (rp2350_text_chars < 32) {
				DBG_PRINTF("[draw_text_character] blit ch=0x%02X img=%p %dx%d to (%d,%d)\n",
					(unsigned)character, image, image->w, image->h,
					(int)textstate.current_x,
					(int)(textstate.current_y - font->height_above_baseline));
			}
			#endif
			method_3_blit_mono(image, textstate.current_x, textstate.current_y - font->height_above_baseline, textstate.textblit, textstate.textcolor);
			width = font->space_between_chars + image->w;
		}
	}
	textstate.current_x += width;
	return width;
}

// seg009:377F
int draw_text_line(const char* text,int length) {
	//hide_cursor();
	int width = 0;
	const char* text_pos = text;
	while (--length >= 0) {
		width += draw_text_character(*text_pos);
		text_pos++;
	}
	//show_cursor();
	return width;
}

// seg009:3755
int draw_cstring(const char* string) {
	//hide_cursor();
	int width = 0;
	const char* text_pos = string;
	while ('\0' != *text_pos) {
		width += draw_text_character(*text_pos);
		text_pos++;
	}
	//show_cursor();
	return width;
}

// seg009:3F01
const rect_type* draw_text(const rect_type* rect_ptr,int x_align,int y_align,const char* text,int length) {
	//printf("going to do draw_text()...\n");
	short rect_top;
	short rect_height;
	short rect_width;
	//textinfo_type var_C;
	short num_lines;
	short font_line_distance;
	//hide_cursor();
	//get_textinfo(&var_C);
	set_clip_rect(rect_ptr);
	rect_width = rect_ptr->right - rect_ptr->left;
	rect_top = rect_ptr->top;
	rect_height = rect_ptr->bottom - rect_ptr->top;
	num_lines = 0;
	int rem_length = length;
	const char* line_start = text;
	#define MAX_LINES 100
	const char* line_starts[MAX_LINES];
	int line_lengths[MAX_LINES];
	do {
		int line_length = find_linebreak(line_start, rem_length, rect_width, x_align);
		if (line_length == 0) break;
		if (num_lines >= MAX_LINES) {
			//... ERROR!
			printf("draw_text(): Too many lines!\n");
			quit(1);
		}
		line_starts[num_lines] = line_start;
		line_lengths[num_lines] = line_length;
		++num_lines;
		line_start += line_length;
		rem_length -= line_length;
	} while(rem_length);
	font_type* font = textstate.ptr_font;
	font_line_distance = font->height_above_baseline + font->height_below_baseline + font->space_between_lines;
	int text_height = font_line_distance * num_lines - font->space_between_lines;
	int text_top = rect_top;
	if (y_align >= 0) {
		if (y_align <= 0) {
			// middle
			// The +1 is for simulating SHR + ADC/SBB.
			text_top += (rect_height+1)/2 - (text_height+1)/2;
		} else {
			// bottom
			text_top += rect_height - text_height;
		}
	}
	textstate.current_y = text_top + font->height_above_baseline;
	for (int i = 0; i < num_lines; ++i) {
		const char* line_pos = line_starts[i];
		int line_length = line_lengths[i];
		if (x_align < 0 &&
			*line_pos == ' ' &&
			i != 0 &&
			*(line_pos-1) != '\n'
		) {
			// Skip over space if it's not at the beginning of a line.
			++line_pos;
			--line_length;
			if (line_length != 0 &&
				*line_pos == ' ' &&
				*(line_pos-2) == '.'
			) {
				// Skip over second space after point.
				++line_pos;
				--line_length;
			}
		}
		int line_width = get_line_width(line_pos,line_length);
		int text_left = rect_ptr->left;
		if (x_align >= 0) {
			if (x_align <= 0) {
				// center
				text_left += rect_width/2 - line_width/2;
			} else {
				// right
				text_left += rect_width - line_width;
			}
		}
		textstate.current_x = text_left;
		//printf("going to draw text line...\n");
		draw_text_line(line_pos,line_length);
		textstate.current_y += font_line_distance;
	}
	reset_clip_rect();
	//set_textinfo(...);
	//show_cursor();
	return rect_ptr;
}

// seg009:3E4F
void show_text(const rect_type* rect_ptr,int x_align,int y_align,const char* text) {
	// stub
	//printf("show_text: %s\n",text);
	#ifdef POP_RP2350
	static int rp2350_show_text_calls = 0;
	if (rp2350_show_text_calls < 8) {
		DBG_PRINTF("[show_text] enter call=%d text=\"%s\"\n", rp2350_show_text_calls, text ? text : "(null)");
	}
	++rp2350_show_text_calls;
	#endif
	draw_text(rect_ptr, x_align, y_align, text, (int)strlen(text));
	#ifdef POP_RP2350
	static int rp2350_show_text_returns = 0;
	if (rp2350_show_text_returns < 8) {
		DBG_PRINTF("[show_text] exit ret=%d\n", rp2350_show_text_returns);
	}
	++rp2350_show_text_returns;
	#endif
}

// seg009:04FF
void show_text_with_color(const rect_type* rect_ptr,int x_align,int y_align, const char* text,int color) {
	short saved_textcolor;
	saved_textcolor = textstate.textcolor;
	textstate.textcolor = color;
	show_text(rect_ptr, x_align, y_align, text);
	textstate.textcolor = saved_textcolor;
}

// seg009:3A91
void set_curr_pos(int xpos,int ypos) {
	textstate.current_x = xpos;
	textstate.current_y = ypos;
}

// seg009:145A
void init_copyprot_dialog() {
	#ifdef POP_RP2350
	// On RP2350 we don't use the interactive dialog system yet (no keyboard).
	// Also, reading the dialog peel from the screen requires robust cross-format
	// blitting very early during startup. Skip it for now.
	copyprot_dialog = NULL;
	return;
	#endif

	copyprot_dialog = make_dialog_info(&dialog_settings, &dialog_rect_1, &dialog_rect_1, NULL);
	copyprot_dialog->peel = read_peel_from_screen(&copyprot_dialog->peel_rect);
}

// seg009:0838
int showmessage(char* text,int arg_4,void *arg_0) {
	#ifdef POP_RP2350
	// No keyboard/UI on RP2350 yet; never block waiting for input.
	// Also avoid dereferencing dialog pointers that may not be initialized.
	(void)arg_4;
	(void)arg_0;
	printf("SDLPoP message: %s\n", text ? text : "(null)");
	return 0;
	#endif

	word key;
	rect_type rect;
	//font_type* saved_font_ptr;
	//surface_type* old_target;
	//old_target = current_target_surface;
	//current_target_surface = onscreen_surface_;
	// In the disassembly there is some messing with the current_target_surface and font (?)
	// However, this does not seem to be strictly necessary
	if (NULL == offscreen_surface) offscreen_surface = make_offscreen_buffer(&screen_rect); // In case we get an error before there is an offsceen buffer
	method_1_blit_rect(offscreen_surface, onscreen_surface_, &copyprot_dialog->peel_rect, &copyprot_dialog->peel_rect, 0);
	draw_dialog_frame(copyprot_dialog);
	//saved_font_ptr = textstate.ptr_font;
	//saved_font_ptr = current_target_surface->ptr_font;
	//current_target_surface->ptr_font = ptr_font;
	shrink2_rect(&rect, &copyprot_dialog->text_rect, 2, 1);
	show_text_with_color(&rect, halign_center, valign_middle, text, color_15_brightwhite);
	//textstate.ptr_font = saved_font_ptr;
	//current_target_surface->ptr_font = saved_font_ptr;
	clear_kbd_buf();
	do {
		idle();
		key = key_test_quit(); // Press any key to continue...
	} while(key == 0);
	//restore_dialog_peel_2(copyprot_dialog->peel);
	//current_target_surface = old_target;
	need_full_redraw = 1; // lazy: instead of neatly restoring only the relevant part, just redraw the whole screen
	return key;
}

// seg009:08FB
dialog_type* make_dialog_info(dialog_settings_type* settings, rect_type* dialog_rect,
                                            rect_type* text_rect, peel_type* dialog_peel) {
	dialog_type* dialog_info;
	dialog_info = malloc(sizeof(dialog_type));
	dialog_info->settings = settings;
	dialog_info->has_peel = 0;
	dialog_info->peel = dialog_peel;
	if (text_rect != NULL)
		dialog_info->text_rect = *text_rect;
	calc_dialog_peel_rect(dialog_info);
	if (text_rect != NULL) {        // does not seem to be quite right; see seg009:0948 (?)
		read_dialog_peel(dialog_info);
	}
	return dialog_info;
}

// seg009:0BE7
void calc_dialog_peel_rect(dialog_type* dialog) {
	dialog_settings_type* settings;
	settings = dialog->settings;
	dialog->peel_rect.left = dialog->text_rect.left - settings->left_border;
	dialog->peel_rect.top = dialog->text_rect.top - settings->top_border;
	dialog->peel_rect.right = dialog->text_rect.right + settings->right_border + settings->shadow_right;
	dialog->peel_rect.bottom = dialog->text_rect.bottom + settings->bottom_border + settings->shadow_bottom;
}

// seg009:0BB0
void read_dialog_peel(dialog_type* dialog) {
	if (dialog->has_peel) {
		if (dialog->peel == NULL) {
			dialog->peel = read_peel_from_screen(&dialog->peel_rect);
		}
		dialog->has_peel = 1;
		draw_dialog_frame(dialog);
	}
}

// seg009:09DE
void draw_dialog_frame(dialog_type* dialog) {
	dialog->settings->method_2_frame(dialog);
}

// A pointer to this function is the first field of dialog_settings (data:2944)
// Perhaps used when replacing a dialog's text with another text (?)
// seg009:096F
void add_dialog_rect(dialog_type* dialog) {
	draw_rect(&dialog->text_rect, color_0_black);
}

// seg009:09F0
void dialog_method_2_frame(dialog_type* dialog) {
	rect_type rect;
	short shadow_right = dialog->settings->shadow_right;
	short shadow_bottom = dialog->settings->shadow_bottom;
	short bottom_border = dialog->settings->bottom_border;
	short outer_border = dialog->settings->outer_border;
	short peel_top = dialog->peel_rect.top;
	short peel_left = dialog->peel_rect.left;
	short peel_bottom = dialog->peel_rect.bottom;
	short peel_right = dialog->peel_rect.right;
	short text_top = dialog->text_rect.top;
	short text_left = dialog->text_rect.left;
	short text_bottom = dialog->text_rect.bottom;
	short text_right = dialog->text_rect.right;
	// Draw outer border
	rect = (rect_type) { peel_top, peel_left, peel_bottom - shadow_bottom, peel_right - shadow_right };
	draw_rect(&rect, color_0_black);
	// Draw shadow (right)
	rect = (rect_type) { text_top, peel_right - shadow_right, peel_bottom, peel_right };
	draw_rect(&rect, get_text_color(0, color_8_darkgray /*dialog's shadow*/, 0));
	// Draw shadow (bottom)
	rect = (rect_type) { peel_bottom - shadow_bottom, text_left, peel_bottom, peel_right };
	draw_rect(&rect, get_text_color(0, color_8_darkgray /*dialog's shadow*/, 0));
	// Draw inner border (left)
	rect = (rect_type) { peel_top + outer_border, peel_left + outer_border, text_bottom, text_left };
	draw_rect(&rect, color_15_brightwhite);
	// Draw inner border (top)
	rect = (rect_type) { peel_top + outer_border, text_left, text_top, text_right + dialog->settings->right_border - outer_border };
	draw_rect(&rect, color_15_brightwhite);
	// Draw inner border (right)
	rect.top = text_top;
	rect.left =  text_right;
	rect.bottom = text_bottom + bottom_border - outer_border;           // (rect.right stays the same)
	draw_rect(&rect, color_15_brightwhite);
	// Draw inner border (bottom)
	rect = (rect_type) { text_bottom, peel_left + outer_border, text_bottom + bottom_border - outer_border, text_right };
	draw_rect(&rect, color_15_brightwhite);
}

// seg009:0C44
void show_dialog(const char* text) {
	char string[256];
	snprintf(string, sizeof(string), "%s\n\nPress any key to continue.", text);
	showmessage(string, 1, &key_test_quit);
}

// seg009:0791
int get_text_center_y(const rect_type* rect) {
	const font_type* font;
	short empty_height; // height of empty space above+below the line of text
	font = &hc_font;//current_target_surface->ptr_font;
	empty_height = rect->bottom - font->height_above_baseline - font->height_below_baseline - rect->top;
	return ((empty_height - empty_height % 2) >> 1) + font->height_above_baseline + empty_height % 2 + rect->top;
}

// seg009:3E77
int get_cstring_width(const char* text) {
	int width = 0;
	const char* text_pos = text;
	char curr_char;
	while ('\0' != (curr_char = *text_pos)) {
		text_pos++;
		width += get_char_width(curr_char);
	}
	return width;
}

// seg009:0767
void draw_text_cursor(int xpos,int ypos,int color) {
	set_curr_pos(xpos, ypos);
	/*current_target_surface->*/textstate.textcolor = color;
	draw_text_character('_');
	//restore_curr_color();
	textstate.textcolor = 15;
}

// seg009:053C
int input_str(const rect_type* rect,char* buffer,int max_length,const char *initial,int has_initial,int arg_4,int color,int bgcolor) {
	// Display the screen keyboard if supported.
	//SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");
	SDL_Rect sdlrect;
	rect_to_sdlrect(rect, &sdlrect);
	SDL_SetTextInputRect(&sdlrect);
	SDL_StartTextInput();

	word key;
	short current_xpos;
	short length = 0;
	short cursor_visible = 0;
	draw_rect(rect, bgcolor);
	short init_length = strlen(initial);
	if (has_initial) {
		strcpy(buffer, initial);
		length = init_length;
	}
	current_xpos = rect->left + arg_4;
	short ypos = get_text_center_y(rect);
	set_curr_pos(current_xpos, ypos);
	/*current_target_surface->*/textstate.textcolor = color;
	draw_cstring(initial);
	//restore_curr_pos?();
	current_xpos += get_cstring_width(initial) + (init_length != 0) * arg_4;
	do {
		key = 0;
		do {
			if (cursor_visible) {
				draw_text_cursor(current_xpos, ypos, color);
			} else {
				draw_text_cursor(current_xpos, ypos, bgcolor);
			}
			cursor_visible = !cursor_visible;
			start_timer(timer_0, 6);
			if (key) {
				if (cursor_visible) {
					draw_text_cursor(current_xpos, ypos, color);
					cursor_visible = !cursor_visible;
				}
				if (key == SDL_SCANCODE_RETURN) { // Enter
					buffer[length] = 0;
					SDL_StopTextInput();
					return length;
				} else break;
			}
			while (!has_timer_stopped(timer_0) && (key = key_test_quit()) == 0) idle();
		} while (1);
		// Only use the printable ASCII chars (UTF-8 encoding)
		char entered_char = last_text_input <= 0x7E ? last_text_input : 0;
		clear_kbd_buf();

		if (key == SDL_SCANCODE_ESCAPE) { // Esc
			draw_rect(rect, bgcolor);
			buffer[0] = 0;
			SDL_StopTextInput();
			return -1;
		}
		if (length != 0 && (key == SDL_SCANCODE_BACKSPACE ||
				key == SDL_SCANCODE_DELETE)) { // Backspace, Delete
			--length;
			draw_text_cursor(current_xpos, ypos, bgcolor);
			current_xpos -= get_char_width(buffer[length]);
			set_curr_pos(current_xpos, ypos);
			/*current_target_surface->*/textstate.textcolor = bgcolor;
			draw_text_character(buffer[length]);
			//restore_curr_pos?();
			draw_text_cursor(current_xpos, ypos, color);
		}
		else if (entered_char >= 0x20 && entered_char <= 0x7E && length < max_length) {
			// Would the new character make the cursor go past the right side of the rect?
			if (get_char_width('_') + get_char_width(entered_char) + current_xpos < rect->right) {
				draw_text_cursor(current_xpos, ypos, bgcolor);
				set_curr_pos(current_xpos, ypos);
				/*current_target_surface->*/textstate.textcolor = color;
				buffer[length] = entered_char;
				length++;
				current_xpos += draw_text_character(entered_char);
			}
		}
	} while(1);
}

#else // USE_TEXT

// seg009:3706
int draw_text_character(byte character) {
	// stub
	printf("draw_text_character: %c\n",character);
	return 0;
}

// seg009:3E4F
void show_text(const rect_type* rect_ptr,int x_align,int y_align,const char *text) {
	// stub
	printf("show_text: %s\n",text);
}

// seg009:04FF
void show_text_with_color(const rect_type* rect_ptr,int x_align,int y_align,const char* text,int color) {
	//short saved_textcolor;
	//saved_textcolor = textstate.textcolor;
	//textstate.textcolor = color;
	show_text(rect_ptr, x_align, y_align, text);
	//textstate.textcolor = saved_textcolor;
}

// seg009:3A91
void set_curr_pos(int xpos,int ypos) {
	// stub
}

// seg009:0C44
void show_dialog(const char *text) {
	// stub
	puts(text);
}

// seg009:053C
int input_str(const rect_type* rect,char* buffer,int max_length,const char* initial,int has_initial,int arg_4,int color,int bgcolor) {
	// stub
	strncpy(buffer, "dummy input text", max_length);
	return strlen(buffer);
}

int showmessage(char* text,int arg_4,void *arg_0) {
	// stub
	puts(text);
	return 0;
}

void init_copyprot_dialog() {
	// stub
}

void draw_dialog_frame(dialog_type *dialog) {
	// stub
}

void add_dialog_rect(dialog_type *dialog) {
	// stub
}

void dialog_method_2_frame(dialog_type *dialog) {
	// stub
}

#endif // USE_TEXT

// seg009:37E8
void draw_rect(const rect_type* rect,int color) {
	method_5_rect(rect, blitters_0_no_transp, color);
}

// seg009:3985
surface_type *rect_sthg(surface_type* surface,const rect_type* rect) {
	// stub
	return surface;
}

// seg009:39CE
rect_type *shrink2_rect(rect_type* target_rect,const rect_type* source_rect,int delta_x,int delta_y) {
	target_rect->top    = source_rect->top    + delta_y;
	target_rect->left   = source_rect->left   + delta_x;
	target_rect->bottom = source_rect->bottom - delta_y;
	target_rect->right  = source_rect->right  - delta_x;
	return target_rect;
}

// seg009:3BBA
void restore_peel(peel_type* peel_ptr) {
	//printf("restoring peel at (x=%d, y=%d)\n", peel_ptr.rect.left, peel_ptr.rect.top); // debug
#ifdef POP_RP2350
	// Direct memcpy for 8bpp indexed → 8bpp indexed (same format)
	SDL_Surface* peel = peel_ptr->peel;
	if (!peel || !peel->pixels) {
		free_peel(peel_ptr);
		return;
	}
	
	// Use peel surface dimensions directly (rect was already clipped in read_peel_from_screen)
	int peel_w = peel->w;
	int peel_h = peel->h;
	int dst_x = peel_ptr->rect.left;
	int dst_y = peel_ptr->rect.top;
	
	// Safety check: ensure we don't write past screen bounds
	int screen_w = current_target_surface->w;
	int screen_h = current_target_surface->h;
	
	if (dst_x < 0 || dst_y < 0 || dst_x + peel_w > screen_w || dst_y + peel_h > screen_h) {
		// Rect was not properly clipped - skip to avoid corruption
		free_peel(peel_ptr);
		return;
	}
	
	if (peel_w <= 0 || peel_h <= 0) {
		free_peel(peel_ptr);
		return;
	}
	
	uint8_t* src_pixels = (uint8_t*)peel->pixels;
	uint8_t* dst_pixels = (uint8_t*)current_target_surface->pixels;
	int src_pitch = peel->pitch;
	int dst_pitch = current_target_surface->pitch;
	
	for (int y = 0; y < peel_h; ++y) {
		memcpy(dst_pixels + (dst_y + y) * dst_pitch + dst_x,
		       src_pixels + y * src_pitch,
		       peel_w);
	}
#else
	method_6_blit_img_to_scr(peel_ptr->peel, peel_ptr->rect.left, peel_ptr->rect.top, /*0x10*/0);
#endif
	free_peel(peel_ptr);
	//SDL_FreeSurface(peel_ptr.peel);
}

// seg009:3BE9
peel_type* read_peel_from_screen(const rect_type* rect) {
#ifdef POP_RP2350
	// Use SRAM mode so peels can be properly freed later
	psram_set_sram_mode(1);
#endif
	// stub
	peel_type* result = calloc(1, sizeof(peel_type));
	//memset(&result, 0, sizeof(result));
#ifdef POP_RP2350
	// On RP2350, screen is 8bpp indexed. Create peel in same format for direct copy.
	// IMPORTANT: Clip rectangle to screen bounds BEFORE creating the surface
	int screen_w = current_target_surface->w;
	int screen_h = current_target_surface->h;
	
	int clip_left = rect->left;
	int clip_top = rect->top;
	int clip_right = rect->right;
	int clip_bottom = rect->bottom;
	
	// Clip to screen bounds
	if (clip_left < 0) { clip_left = 0; }
	if (clip_top < 0) { clip_top = 0; }
	if (clip_right > screen_w) { clip_right = screen_w; }
	if (clip_bottom > screen_h) { clip_bottom = screen_h; }
	
	// Store the CLIPPED rectangle so restore_peel uses correct coordinates
	result->rect.left = clip_left;
	result->rect.top = clip_top;
	result->rect.right = clip_right;
	result->rect.bottom = clip_bottom;
	
	int peel_w = clip_right - clip_left;
	int peel_h = clip_bottom - clip_top;
	
	if (peel_w <= 0 || peel_h <= 0) {
		// Completely off screen - return dummy peel (no palette needed)
		SDL_Surface* peel_surface = SDL_CreateRGBSurface(SDL_NO_PALETTE, 1, 1, 8, 0, 0, 0, 0);
		psram_set_sram_mode(0);
		if (peel_surface) {
			SDL_SurfaceAdoptPalette(peel_surface, current_target_surface->format->palette);
		}
		result->peel = peel_surface;
		return result;
	}
	
	// Use SDL_NO_PALETTE to avoid allocating a 1KB palette - we'll adopt screen's palette
	SDL_Surface* peel_surface = SDL_CreateRGBSurface(SDL_NO_PALETTE, peel_w, peel_h, 8, 0, 0, 0, 0);
	psram_set_sram_mode(0);
	if (peel_surface == NULL) {
		sdlperror("read_peel_from_screen: SDL_CreateRGBSurface");
		// Don't quit - return NULL peel and hope for the best
		result->peel = NULL;
		return result;
	}
	// Share palette reference instead of copying (saves memory)
	SDL_SurfaceAdoptPalette(peel_surface, current_target_surface->format->palette);
	
	// Direct memcpy for 8bpp indexed → 8bpp indexed (same format)
	uint8_t* src_pixels = (uint8_t*)current_target_surface->pixels;
	uint8_t* dst_pixels = (uint8_t*)peel_surface->pixels;
	int src_pitch = current_target_surface->pitch;
	int dst_pitch = peel_surface->pitch;
	
	for (int y = 0; y < peel_h; ++y) {
		memcpy(dst_pixels + y * dst_pitch,
		       src_pixels + (clip_top + y) * src_pitch + clip_left,
		       peel_w);
	}
	
	result->peel = peel_surface;
	return result;
#else
	result->rect = *rect;
#ifndef USE_ALPHA
	SDL_Surface* peel_surface = SDL_CreateRGBSurface(0, rect->right - rect->left, rect->bottom - rect->top,
	                                                 24, Rmsk, Gmsk, Bmsk, 0);
#else
	SDL_Surface* peel_surface = SDL_CreateRGBSurface(0, rect->right - rect->left, rect->bottom - rect->top, 32, Rmsk, Gmsk, Bmsk, Amsk);
#endif
	if (peel_surface == NULL) {
		sdlperror("read_peel_from_screen: SDL_CreateRGBSurface");
		quit(1);
	}
	result->peel = peel_surface;
	rect_type target_rect = {0, 0, rect->right - rect->left, rect->bottom - rect->top};
	method_1_blit_rect(result->peel, current_target_surface, &target_rect, rect, 0);
	return result;
#endif
}

// seg009:3D95
int intersect_rect(rect_type* output,const rect_type* input1,const rect_type* input2) {
	short left = MAX(input1->left, input2->left);
	short right = MIN(input1->right, input2->right);
	if (left < right) {
		output->left = left;
		output->right = right;
		short top = MAX(input1->top, input2->top);
		short bottom = MIN(input1->bottom, input2->bottom);
		if (top < bottom) {
			output->top = top;
			output->bottom = bottom;
			return 1;
		}
	}
	memset(output, 0, sizeof(rect_type));
	return 0;
}

// seg009:4063
rect_type* union_rect(rect_type* output,const rect_type* input1,const rect_type* input2) {
	short top = MIN(input1->top, input2->top);
	short left = MIN(input1->left, input2->left);
	short bottom = MAX(input1->bottom, input2->bottom);
	short right = MAX(input1->right, input2->right);
	output->top = top;
	output->left = left;
	output->bottom = bottom;
	output->right = right;
	return output;
}

enum userevents {
	userevent_SOUND,
	userevent_TIMER,
};

short speaker_playing = 0;
short digi_playing = 0;
short midi_playing = 0;
short ogg_playing = 0;

// The currently playing sound buffer for the PC speaker.
speaker_type* current_speaker_sound;
// Index for which note is currently playing.
int speaker_note_index;
// Tracks how long the last (partially played) speaker note has been playing (for the audio callback).
int current_speaker_note_samples_already_emitted;

void speaker_sound_stop(void) {
	if (!speaker_playing) return;
	SDL_LockAudio();
	speaker_playing = 0;
	current_speaker_sound = NULL;
	speaker_note_index = 0;
	current_speaker_note_samples_already_emitted = 0;
	SDL_UnlockAudio();
}

// The current buffer, holds the resampled sound data.
byte* digi_buffer = NULL;
// The current position in digi_buffer.
byte* digi_remaining_pos = NULL;
// The remaining length.
int digi_remaining_length = 0;

#ifdef POP_RP2350
// Streaming resampler state for RP2350 (to avoid huge pre-conversion memory usage)
static byte* digi_src_samples = NULL;      // Original 8-bit samples
static int digi_src_sample_count = 0;      // Total source samples
static int digi_src_sample_rate = 0;       // Source sample rate (e.g., 11000)
static float digi_src_position = 0.0f;     // Current fractional position in source
static float digi_src_ratio = 0.0f;        // Source samples per output sample
#endif

// The properties of the audio device.
SDL_AudioSpec* digi_audiospec = NULL;
// The desired samplerate. Everything will be resampled to this.
const int digi_samplerate = 44100;

void stop_digi(void) {
//	SDL_PauseAudio(1);
	if (!digi_playing) return;
	SDL_LockAudio();
	digi_playing = 0;
	/*
//	if (SDL_GetAudioStatus() == SDL_AUDIO_PLAYING) {
		SDL_PauseAudio(1);
		SDL_CloseAudio();
//	}
	if (digi_audiospec != NULL) {
		free(digi_audiospec);
		digi_audiospec = NULL;
	}
	*/
	digi_buffer = NULL;
	digi_remaining_length = 0;
	digi_remaining_pos = NULL;
#ifdef POP_RP2350
	digi_src_samples = NULL;
	digi_src_sample_count = 0;
	digi_src_position = 0.0f;
#endif
	SDL_UnlockAudio();
}

// Decoder for the currently playing OGG sound. (This also holds the playback position.)
stb_vorbis* ogg_decoder;

void stop_ogg(void) {
	SDL_PauseAudio(1);
	if (!ogg_playing) return;
	ogg_playing = 0;
	SDL_LockAudio();
	ogg_decoder = NULL;
	SDL_UnlockAudio();
}

// seg009:7214
void stop_sounds() {
	// stub
	stop_digi();
	stop_midi();
	speaker_sound_stop();
	stop_ogg();
}

short square_wave_state = 4000; // If the amplitude is too high, the speaker sounds will be really loud!
float square_wave_samples_since_last_flip;

void generate_square_wave(byte* stream, float note_freq, int samples) {
	int channels = digi_audiospec->channels;
	float half_period_in_samples = (digi_audiospec->freq / note_freq) * 0.5f;

	int samples_left = samples;
	while (samples_left > 0) {
		if (square_wave_samples_since_last_flip > half_period_in_samples) {
			// Produce a square wave by flipping the signal.
			square_wave_state = ~square_wave_state;
			// Note(Falcury): not completely sure that this is the right way to prevent glitches in the sound...
			// Because I can still hear some hiccups, e.g. in the music. Especially when switching between notes.
			square_wave_samples_since_last_flip -= half_period_in_samples;
		} else {
			int samples_until_next_flip = (int)(half_period_in_samples - square_wave_samples_since_last_flip);
			++samples_until_next_flip; // round up.

			int samples_to_emit = MIN(samples_until_next_flip, samples_left);
			for (int i = 0; i < samples_to_emit * channels; ++i) {
				*(short*)stream = square_wave_state;
				stream += sizeof(short);
			}
			samples_left -= samples_to_emit;
			square_wave_samples_since_last_flip += samples_to_emit;
		}
	}
}

void speaker_callback(void *userdata, Uint8 *stream, int len) {
	int output_channels = digi_audiospec->channels;
	int bytes_per_sample = sizeof(short) * output_channels;
	int samples_requested = len / bytes_per_sample;

	if (current_speaker_sound == NULL) return;
	word tempo = SDL_SwapLE16(current_speaker_sound->tempo);

	int total_samples_left = samples_requested;
	while (total_samples_left > 0) {
		note_type* note = current_speaker_sound->notes + speaker_note_index;
		if (SDL_SwapLE16(note->frequency) == 0x12 /*end*/) {
			speaker_playing = 0;
			current_speaker_sound = NULL;
			speaker_note_index = 0;
			SDL_Event event;
			memset(&event, 0, sizeof(event));
			event.type = SDL_USEREVENT;
			event.user.code = userevent_SOUND;
			SDL_PushEvent(&event);
			return;
		}

		int note_length_in_samples = (note->length * digi_audiospec->freq) / tempo;
		int note_samples_to_emit = MIN(note_length_in_samples - current_speaker_note_samples_already_emitted, total_samples_left);
		total_samples_left -= note_samples_to_emit;
		size_t copy_len = (size_t)note_samples_to_emit * bytes_per_sample;
		if (SDL_SwapLE16(note->frequency) <= 0x01 /*rest*/) {
			memset(stream, digi_audiospec->silence, copy_len);
		} else {
			generate_square_wave(stream, (float)SDL_SwapLE16(note->frequency), note_samples_to_emit);
		}
		stream += copy_len;

		int note_samples_emitted = current_speaker_note_samples_already_emitted + note_samples_to_emit;
		if (note_samples_emitted < note_length_in_samples) {
			current_speaker_note_samples_already_emitted += note_samples_to_emit;
		} else {
			++speaker_note_index;
			current_speaker_note_samples_already_emitted = 0;
		}
	}
}

// seg009:7640
void play_speaker_sound(sound_buffer_type* buffer) {
	speaker_sound_stop();
	stop_sounds();
	current_speaker_sound = &buffer->speaker;
	speaker_note_index = 0;
	speaker_playing = 1;
	SDL_PauseAudio(0);
}

void digi_callback(void *userdata, Uint8 *stream, int len) {
#ifdef POP_RP2350
	// Streaming resampler: convert 8-bit mono source to 16-bit stereo on-the-fly
	if (digi_src_samples == NULL) {
		memset(stream, digi_audiospec->silence, len);
		return;
	}
	
	int output_channels = digi_audiospec->channels;
	int bytes_per_frame = sizeof(short) * output_channels;
	int frames_requested = len / bytes_per_frame;
	short* dest = (short*)stream;
	int frames_filled = 0;
	
	if (is_sound_on) {
		for (int i = 0; i < frames_requested; ++i) {
			if (digi_src_position >= digi_src_sample_count - 1) {
				break; // End of source
			}

			int src_frame_0 = (int)digi_src_position;
			float alpha = digi_src_position - src_frame_0;

			// Get 8-bit unsigned samples, convert to 16-bit signed
			int sample_0 = (digi_src_samples[src_frame_0] | (digi_src_samples[src_frame_0] << 8)) - 32768;
			int sample_1 = sample_0;
			if (src_frame_0 + 1 < digi_src_sample_count) {
				sample_1 = (digi_src_samples[src_frame_0 + 1] | (digi_src_samples[src_frame_0 + 1] << 8)) - 32768;
			}

			// Linear interpolation, attenuate to 50% to leave headroom for mixing
			short interpolated = (short)(((1.0f - alpha) * sample_0 + alpha * sample_1) * 0.5f);

			// Additive mix into stereo output
			for (int ch = 0; ch < output_channels; ++ch) {
				int32_t mixed = (int32_t)*dest + interpolated;
				if (mixed > 32767) mixed = 32767;
				if (mixed < -32768) mixed = -32768;
				*dest++ = (short)mixed;
			}

			digi_src_position += digi_src_ratio;
			frames_filled++;
		}
	}
	
	// Fill remaining with silence
	int bytes_filled = frames_filled * bytes_per_frame;
	if (bytes_filled < len) {
		memset((byte*)stream + bytes_filled, digi_audiospec->silence, len - bytes_filled);
	}
	
	// If the sound ended, push an event
	if (digi_playing && digi_src_position >= digi_src_sample_count - 1) {
		SDL_Event event;
		memset(&event, 0, sizeof(event));
		event.type = SDL_USEREVENT;
		event.user.code = userevent_SOUND;
		digi_playing = 0;
		SDL_PushEvent(&event);
	}
#else
	// Original pre-converted path for desktop
	// Don't go over the end of either the input or the output buffer.
	size_t copy_len = MIN(len, digi_remaining_length);
	//printf("digi_callback(): copy_len = %d\n", copy_len);
	//printf("digi_callback(): len = %d\n", len);
	if (is_sound_on) {
		// Copy the next part of the input of the output.
		memcpy(stream, digi_remaining_pos, copy_len);
		// In case the sound does not fill the buffer: fill the rest of the buffer with silence.
		memset(stream + copy_len, digi_audiospec->silence, len - copy_len);
	} else {
		// If sound is off: Mute the sound but keep track of where we are.
		memset(stream, digi_audiospec->silence, len);
	}
	// If the sound ended, push an event.
	if (digi_playing && digi_remaining_length == 0) {
		//printf("digi_callback(): sound ended\n");
		SDL_Event event;
		memset(&event, 0, sizeof(event));
		event.type = SDL_USEREVENT;
		event.user.code = userevent_SOUND;
		digi_playing = 0;
		SDL_PushEvent(&event);
	}
	// Advance the pointer.
	digi_remaining_length -= copy_len;
	digi_remaining_pos += copy_len;
#endif
}

void ogg_callback(void *userdata, Uint8 *stream, int len) {
	int output_channels = digi_audiospec->channels;
	int bytes_per_sample = sizeof(short) * output_channels;
	int samples_requested = len / bytes_per_sample;

	int samples_filled;
	if (is_sound_on) {
		samples_filled = stb_vorbis_get_samples_short_interleaved(ogg_decoder, output_channels,
                                                                      (short*) stream, len / sizeof(short));
		if (samples_filled < samples_requested) {
			// In case the sound does not fill the buffer: fill the rest of the buffer with silence.
			int bytes_filled = samples_filled * bytes_per_sample;
			int remaining_bytes = (samples_requested - samples_filled) * bytes_per_sample;
			memset(stream + bytes_filled, digi_audiospec->silence, remaining_bytes);
		}
	} else {
		// If sound is off: Mute the sound, but keep track of where we are.
		memset(stream, digi_audiospec->silence, len);
		// Let the decoder run normally (to advance the position), but discard the result.
		byte* discarded_samples = alloca(len);
		samples_filled = stb_vorbis_get_samples_short_interleaved(ogg_decoder, output_channels,
																  (short*) discarded_samples, len / sizeof(short));
	}
	// Push an event if the sound has ended.
	if (samples_filled == 0) {
		//printf("ogg_callback(): sound ended\n");
		SDL_Event event;
		memset(&event, 0, sizeof(event));
		event.type = SDL_USEREVENT;
		event.user.code = userevent_SOUND;
		ogg_playing = 0;
		SDL_PushEvent(&event);
	}
}

#ifdef USE_FAST_FORWARD
int audio_speed = 1; // =1 normally, >1 during fast forwarding
#endif

void audio_callback(void* userdata, Uint8* stream_orig, int len_orig) {

	Uint8* stream;
	int len;
#ifdef USE_FAST_FORWARD
	if (audio_speed > 1) {
		len = len_orig * audio_speed;
		stream = malloc(len);
	} else
#endif
	{
		len = len_orig;
		stream = stream_orig;
	}

	memset(stream, digi_audiospec->silence, len);
	if (digi_playing) {
		digi_callback(userdata, stream, len);
	} else if (speaker_playing) {
		speaker_callback(userdata, stream, len);
	}
	// Note: music sounds and digi sounds are allowed to play simultaneously (will be blended together)
	// I.e., digi sounds and music will not cut each other short.
#ifdef POP_RP2350
	extern int midi_cache_playing;
	extern void midi_cached_callback(void*, Uint8*, int);
	if (midi_cache_playing) {
		midi_cached_callback(userdata, stream, len);
	} else
#endif
	if (midi_playing) {
		midi_callback(userdata, stream, len);
	} else if (ogg_playing) {
		ogg_callback(userdata, stream, len);
	}

#ifdef USE_FAST_FORWARD
	if (audio_speed > 1) {

#ifdef FAST_FORWARD_MUTE
		memset(stream_orig, digi_audiospec->silence, len_orig);
#else
#ifdef FAST_FORWARD_RESAMPLE_SOUND
		static SDL_AudioCVT cvt;
		static bool cvt_initialized = false;
		if (!cvt_initialized) {
			SDL_BuildAudioCVT(&cvt,
				digi_audiospec->format, digi_audiospec->channels, digi_audiospec->freq * audio_speed,
				digi_audiospec->format, digi_audiospec->channels, digi_audiospec->freq);
			cvt_initialized = true;
		}
		//realloc(stream, len * cvt.len_mult);
		//cvt.buf = stream;
		cvt.len = len;
		cvt.buf = malloc(cvt.len * cvt.len_mult);
		memcpy(cvt.buf, stream, cvt.len);
		//printf("cvt.needed = %d\n", cvt.needed);
		//printf("cvt.len_mult = %d\n", cvt.len_mult);
		//printf("cvt.len_ratio = %lf\n", cvt.len_ratio);
		SDL_ConvertAudio(&cvt);

		memcpy(stream_orig, cvt.buf, len_orig);
		free(cvt.buf);
		cvt.buf = NULL;
#else
		// Hack: use the beginning of the buffer instead of resampling.
		memcpy(stream_orig, stream, len_orig);
#endif
#endif

		free(stream);
	}
#endif

}

int digi_unavailable = 0;
void init_digi() {
	if (digi_unavailable) return;
	if (digi_audiospec != NULL) return;
	// Open the audio device. Called once.
	//printf("init_digi(): called\n");

	SDL_AudioFormat desired_audioformat;
	SDL_version version;
	SDL_GetVersion(&version);
	//printf("SDL Version = %d.%d.%d\n", version.major, version.minor, version.patch);
	if (version.major <= 2 && version.minor <= 0 && version.patch <= 3) {
		// In versions before 2.0.4, 16-bit audio samples don't work properly (the sound becomes garbled).
		// See: https://bugzilla.libsdl.org/show_bug.cgi?id=2389
		// Workaround: set the audio format to 8-bit, if we are linking against an older SDL2 version.
		desired_audioformat = AUDIO_U8;
		printf("Your SDL.dll is older than 2.0.4. Using 8-bit audio format to work around resampling bug.");
	} else {
		desired_audioformat = AUDIO_S16SYS;
	}

	SDL_AudioSpec *desired;
	desired = (SDL_AudioSpec *)malloc(sizeof(SDL_AudioSpec));
	memset(desired, 0, sizeof(SDL_AudioSpec));
	desired->freq = digi_samplerate; //buffer->digi.sample_rate;
	desired->format = desired_audioformat;
	desired->channels = 2;
	desired->samples = 1024;
	desired->callback = audio_callback;
	desired->userdata = NULL;
	if (SDL_OpenAudio(desired, NULL) != 0) {
		sdlperror("init_digi: SDL_OpenAudio");
		//quit(1);
		digi_unavailable = 1;
		return;
	}
	DBG_PRINTF("[init_digi] SDL_OpenAudio succeeded, setting digi_audiospec\n");
	//SDL_PauseAudio(0);
	digi_audiospec = desired;
	DBG_PRINTF("[init_digi] done\n");
}

const int sound_channel = 0;
const int max_sound_id = 58;

void load_sound_names() {
	DBG_PRINTF("[load_sound_names] entering\n");
	const char* names_path = locate_file("prince/music/names.txt");
	DBG_PRINTF("[load_sound_names] names_path=%s\n", names_path ? names_path : "(null)");
	if (sound_names != NULL) return;
	FILE* fp = fopen(names_path,"rt");
	DBG_PRINTF("[load_sound_names] fopen result=%p\n", (void*)fp);
	if (fp==NULL) return;
	sound_names = (char**) calloc(sizeof(char*) * max_sound_id, 1);
	while (!feof(fp)) {
		int index;
		char name[POP_MAX_PATH];
		if (fscanf(fp, "%d=%255s\n", &index, /*sizeof(name)-1,*/ name) != 2) {
			perror(names_path);
			continue;
		}
		//if (feof(fp)) break;
		//printf("sound_names[%d] = %s\n",index,name);
		if (index >= 0 && index < max_sound_id) {
			sound_names[index] = strdup(name);
		}
	}
	fclose(fp);
}

char* sound_name(int index) {
	if (sound_names != NULL && index >= 0 && index < max_sound_id) {
		return sound_names[index];
	} else {
		return NULL;
	}
}

sound_buffer_type* convert_digi_sound(sound_buffer_type* digi_buffer);

sound_buffer_type* load_sound(int index) {
	sound_buffer_type* result = NULL;
	DBG_PRINTF("[load_sound] index=%d\\n", index);
	init_digi();
	DBG_PRINTF("[load_sound] after init_digi, enable_music=%d digi_unavailable=%d\\n", enable_music, digi_unavailable);
	if (enable_music && !digi_unavailable && result == NULL && index >= 0 && index < max_sound_id) {
		//printf("Trying to load from music folder\n");

		//load_sound_names();  // Moved to load_sounds()
		if (sound_names != NULL && sound_name(index) != NULL) {
			//printf("Loading from music folder\n");
			do {
				FILE* fp = NULL;
				char filename[POP_MAX_PATH];
				if (!skip_mod_data_files) {
					// before checking the root directory, first try mods/MODNAME/
					snprintf_check(filename, sizeof(filename), "%s/music/%s.ogg", mod_data_path, sound_name(index));
					fp = fopen(filename, "rb");
				}
				if (fp == NULL && !skip_normal_data_files) {
					snprintf_check(filename, sizeof(filename), "prince/music/%s.ogg", sound_name(index));
					fp = fopen(locate_file(filename), "rb");
				}
				if (fp == NULL) {
					break;
				}
				// Read the entire file (undecoded) into memory.
				struct stat info;
				if (fstat(fileno(fp), &info))
					break;
				size_t file_size = (size_t) MAX(0, info.st_size);
				byte* file_contents = malloc(file_size);
				if (fread(file_contents, 1, file_size, fp) != file_size) {
					free(file_contents);
					fclose(fp);
					break;
				}
				fclose(fp);

				// Decoding the entire file immediately would make the loading time much longer.
				// However, we can also create the decoder now, and only use it when we are actually playing the file.
				// (In the audio callback, we'll decode chunks of samples to the output stream, as needed).
				int error = 0;
				stb_vorbis* decoder = stb_vorbis_open_memory(file_contents, (int)file_size, &error, NULL);
				if (decoder == NULL) {
					printf("Error %d when creating decoder from file \"%s\"!\n", error, filename);
					free(file_contents);
					break;
				}
				result = malloc(sizeof(sound_buffer_type));
				result->type = sound_ogg;
				result->ogg.total_length = stb_vorbis_stream_length_in_samples(decoder) * sizeof(short);
				result->ogg.file_contents = file_contents; // Remember in case we want to free the sound later.
				result->ogg.decoder = decoder;
			} while(0); // do once (breakable block)
		} else {
			//printf("sound_names = %p\n", sound_names);
			//printf("sound_names[%d] = %p\n", index, sound_name(index));
		}
	}
	if (result == NULL) {
		DBG_PRINTF("[load_sound] loading from DAT, index+10000=%d\\n", index + 10000);
		result = (sound_buffer_type*) load_from_opendats_alloc(index + 10000, "bin", NULL, NULL);
		DBG_PRINTF("[load_sound] load_from_opendats_alloc returned %p\\n", (void*)result);
	}
#ifdef POP_RP2350
	// On RP2350: keep raw digi sounds, resample on-the-fly during playback to save PSRAM
	// Just log the sound info without converting
	if (result != NULL && (result->type & 7) == sound_digi) {
		DBG_PRINTF("[load_sound] keeping raw digi sound (streaming mode)\\n");
	}
#else
	if (result != NULL && (result->type & 7) == sound_digi) {
		DBG_PRINTF("[load_sound] converting digi sound\\n");
		sound_buffer_type* converted = convert_digi_sound(result);
		DBG_PRINTF("[load_sound] freeing original buffer\\n");
		free(result);
		result = converted;
		DBG_PRINTF("[load_sound] conversion done, result=%p\\n", (void*)result);
	}
#endif
	if (result == NULL && !skip_normal_data_files) {
		fprintf(stderr, "Failed to load sound %d '%s'\n", index, sound_name(index));
	}
	return result;
}

void play_ogg_sound(sound_buffer_type *buffer) {
	init_digi();
	if (digi_unavailable) return;
	stop_sounds();

	// Need to rewind the music, or else the decoder might continue where it left off, the last time this sound played.
	stb_vorbis_seek_start(buffer->ogg.decoder);

	SDL_LockAudio();
	ogg_decoder = buffer->ogg.decoder;
	SDL_UnlockAudio();
	SDL_PauseAudio(0);

	ogg_playing = 1;
}

int wave_version = -1;

typedef struct waveinfo_type {
	int sample_rate, sample_size, sample_count;
	byte* samples;
} waveinfo_type;

bool determine_wave_version(sound_buffer_type *buffer, waveinfo_type* waveinfo) {
	int version = wave_version;
	if (version == -1) {
		// Determine the version of the wave data.
		version = 0;
		if (buffer->digi.sample_size == 8) version += 1;
		if (buffer->digi_new.sample_size == 8) version += 2;
		if (version == 1 || version == 2) wave_version = version;
	}

	switch (version) {
		case 1: // 1.0 and 1.1
			waveinfo->sample_rate = SDL_SwapLE16(buffer->digi.sample_rate);
			waveinfo->sample_size = buffer->digi.sample_size;
			waveinfo->sample_count = SDL_SwapLE16(buffer->digi.sample_count);
			waveinfo->samples = buffer->digi.samples;
			return true;
		case 2: // 1.3 and 1.4 (and PoP2)
			waveinfo->sample_rate = SDL_SwapLE16(buffer->digi_new.sample_rate);
			waveinfo->sample_size = buffer->digi_new.sample_size;
			waveinfo->sample_count = SDL_SwapLE16(buffer->digi_new.sample_count);
			waveinfo->samples = buffer->digi_new.samples;
			return true;
		case 3: // ambiguous
			printf("Warning: Ambiguous wave version.\n");
			return false;
		default: // case 0, unknown
			printf("Warning: Can't determine wave version.\n");
			return false;
	}
}

sound_buffer_type* convert_digi_sound(sound_buffer_type* digi_buffer) {
	DBG_PRINTF("[convert_digi_sound] entering\\n");
	init_digi();
	if (digi_unavailable) return NULL;
	waveinfo_type waveinfo;
	if (false == determine_wave_version(digi_buffer, &waveinfo)) return NULL;
	DBG_PRINTF("[convert_digi_sound] wave: rate=%d size=%d count=%d\\n", waveinfo.sample_rate, waveinfo.sample_size, waveinfo.sample_count);

	float freq_ratio = (float)waveinfo.sample_rate /  (float)digi_audiospec->freq;

	int source_length = waveinfo.sample_count;
	int expanded_frames = source_length * digi_audiospec->freq / waveinfo.sample_rate;
	int expanded_length = expanded_frames * 2 * sizeof(short);
	DBG_PRINTF("[convert_digi_sound] expanded_frames=%d expanded_length=%d\\n", expanded_frames, expanded_length);
#ifdef POP_RP2350
	sound_buffer_type* converted_buffer = pop_heap_alloc(sizeof(sound_buffer_type) + expanded_length);
#else
	sound_buffer_type* converted_buffer = malloc(sizeof(sound_buffer_type) + expanded_length);
#endif
	DBG_PRINTF("[convert_digi_sound] converted_buffer=%p\\n", (void*)converted_buffer);

	converted_buffer->type = sound_digi_converted;
	converted_buffer->converted.length = expanded_length;

	byte* source = waveinfo.samples;
	//short* dest = converted_buffer->converted.samples;
#ifdef POP_RP2350
	short* dest = pop_heap_alloc(sizeof(short) * converted_buffer->converted.length);
#else
	short* dest = malloc(sizeof(short) * converted_buffer->converted.length);
#endif
	DBG_PRINTF("[convert_digi_sound] dest=%p, starting loop\\n", (void*)dest);
        converted_buffer->converted.samples = dest;

	for (int i = 0; i < expanded_frames; ++i) {
		float src_frame_float = i * freq_ratio;
		int src_frame_0 = (int) src_frame_float; // truncation

		int sample_0 = (source[src_frame_0] | (source[src_frame_0] << 8)) - 32768;
		short interpolated_sample;
		if (src_frame_0 >= waveinfo.sample_count-1) {
			interpolated_sample = (short)sample_0;
		} else {
			int src_frame_1 = src_frame_0 + 1;
			float alpha = src_frame_float - src_frame_0;
			int sample_1 = (source[src_frame_1] | (source[src_frame_1] << 8)) - 32768;
			interpolated_sample = (short)((1.0f - alpha) * sample_0 + alpha * sample_1);
		}
		for (int channel = 0; channel < digi_audiospec->channels; ++channel) {
			*dest++ = interpolated_sample;
		}
	}
	DBG_PRINTF("[convert_digi_sound] loop done\\n");

	return converted_buffer;
}

// seg009:74F0
void play_digi_sound(sound_buffer_type* buffer) {
	//if (!is_sound_on) return;
	init_digi();
	if (digi_unavailable) return;
	stop_digi();
//	stop_sounds();
	//printf("play_digi_sound(): called\n");
#ifdef POP_RP2350
	// Streaming mode: accept raw digi sound and set up streaming resampler
	if ((buffer->type & 7) != sound_digi) {
		printf("play_digi_sound: expected raw digi sound, got type %d\n", buffer->type & 7);
		return;
	}
	waveinfo_type waveinfo;
	if (!determine_wave_version(buffer, &waveinfo)) {
		printf("play_digi_sound: failed to determine wave version\n");
		return;
	}
	SDL_LockAudio();
	digi_src_samples = waveinfo.samples;
	digi_src_sample_count = waveinfo.sample_count;
	digi_src_sample_rate = waveinfo.sample_rate;
	digi_src_position = 0.0f;
	digi_src_ratio = (float)waveinfo.sample_rate / (float)digi_audiospec->freq;
	digi_playing = 1;
	SDL_UnlockAudio();
#else
	if ((buffer->type & 7) != sound_digi_converted) {
		printf("Tried to play unconverted digi sound.\n");
		return;
	}
	SDL_LockAudio();
	digi_buffer = (byte*) buffer->converted.samples;
	digi_playing = 1;
	digi_remaining_length = buffer->converted.length;
	digi_remaining_pos = digi_buffer;
	SDL_UnlockAudio();
#endif
	SDL_PauseAudio(0);
}

void free_sound(sound_buffer_type* buffer) {
	if (buffer == NULL) return;
	if (buffer->type == sound_ogg) {
		stb_vorbis_close(buffer->ogg.decoder);
		free(buffer->ogg.file_contents);
	}
	free(buffer);
}

// seg009:7220
void play_sound_from_buffer(sound_buffer_type* buffer) {

#ifdef USE_REPLAY
	if (replaying && skipping_replay) return;
#endif

	// stub
	if (buffer == NULL) {
		printf("Tried to play NULL sound.\n");
		//quit(1);
		return;
	}
	switch (buffer->type & 7) {
		case sound_speaker:
			play_speaker_sound(buffer);
		break;
		case sound_digi_converted:
		case sound_digi:
			play_digi_sound(buffer);
		break;
		case sound_midi:
			play_midi_sound(buffer);
		break;
		case sound_ogg:
			play_ogg_sound(buffer);
		break;
		default:
			printf("Tried to play unimplemented sound type %d.\n", buffer->type);
			quit(1);
		break;
	}
}

void turn_music_on_off(byte new_state) {
	enable_music = new_state;
	turn_sound_on_off(is_sound_on);
}

// seg009:7273
void turn_sound_on_off(byte new_state) {
	// stub
	is_sound_on = new_state;
	//if (!is_sound_on) stop_sounds();
}

// seg009:7299
int check_sound_playing() {
	#ifdef POP_RP2350
	// RP2350: Check if MIDI cache playback or digi sounds are active
	extern int midi_cache_playing;
	return digi_playing || midi_playing || midi_cache_playing;
	#else
	return speaker_playing || digi_playing || midi_playing || ogg_playing;
	#endif
}

void apply_aspect_ratio() {
	// Allow us to use a consistent set of screen co-ordinates, even if the screen size changes
	if (use_correct_aspect_ratio) {
		SDL_RenderSetLogicalSize(renderer_, 320 * 5, 200 * 6); // 4:3
	} else {
		SDL_RenderSetLogicalSize(renderer_, 320, 200); // 16:10
	}
	window_resized();
}

void window_resized() {
#if SDL_VERSION_ATLEAST(2,0,5) // SDL_RenderSetIntegerScale
	if (use_integer_scaling) {
		int window_width, window_height;
		// On high-DPI screens, this is what we need instead of SDL_GetWindowSize().
		//SDL_GL_GetDrawableSize(window_, &window_width, &window_height);
		SDL_GetRendererOutputSize(renderer_, &window_width, &window_height);
		int render_width, render_height;
		SDL_RenderGetLogicalSize(renderer_, &render_width, &render_height);
		// Disable integer scaling if it would result in downscaling.
		// Because then the only suitable integer scaling factor is zero, i.e. the picture disappears.
		SDL_bool makes_sense = (window_width >= render_width && window_height >= render_height);
		SDL_RenderSetIntegerScale(renderer_, makes_sense);
	}
#endif
}

void init_overlay(void) {
	static bool initialized = false;
	if (!initialized) {
		overlay_surface = SDL_CreateRGBSurface(0, 320, 200, 32, Rmsk, Gmsk, Bmsk, Amsk);
		merged_surface = SDL_CreateRGBSurface(0, 320, 200, 24, Rmsk, Gmsk, Bmsk, 0);
		initialized = true;
	}
}

SDL_Surface* onscreen_surface_2x;

void init_scaling(void) {
	// Don't crash in validate mode.
	if (renderer_ == NULL) return;

	if (texture_sharp == NULL) {
		texture_sharp = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, 320, 200);
	}
	if (scaling_type == 1) {
		if (!is_renderer_targettexture_supported && onscreen_surface_2x == NULL) {
#ifdef __amigaos4__
			overlay_surface = SDL_CreateRGBSurface(0, 320*2, 200*2, 24, Rmsk, Gmsk, Bmsk, 0);
#else
			onscreen_surface_2x = SDL_CreateRGBSurface(0, 320*2, 200*2, 24, Rmsk, Gmsk, Bmsk, 0);
#endif
		}
		if (texture_fuzzy == NULL) {
			SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
			int access = is_renderer_targettexture_supported ? SDL_TEXTUREACCESS_TARGET : SDL_TEXTUREACCESS_STREAMING;
			texture_fuzzy = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_RGB24, access, 320*2, 200*2);
			SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0");
		}
		target_texture = texture_fuzzy;
	} else if (scaling_type == 2) {
		if (texture_blurry == NULL) {
			SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
			texture_blurry = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, 320, 200);
			SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0");
		}
		target_texture = texture_blurry;
	} else {
		target_texture = texture_sharp;
	}
	if (target_texture == NULL) {
		sdlperror("init_scaling: SDL_CreateTexture");
		quit(1);
	}
}

// seg009:38ED
void set_gr_mode(byte grmode) {
#ifdef SDL_HINT_WINDOWS_DISABLE_THREAD_NAMING
	SDL_SetHint(SDL_HINT_WINDOWS_DISABLE_THREAD_NAMING, "1");
#endif
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_NOPARACHUTE | SDL_INIT_GAMECONTROLLER) != 0) {
		sdlperror("set_gr_mode: SDL_Init");
		quit(1);
	}
	if (enable_controller_rumble) {
		if (SDL_InitSubSystem(SDL_INIT_HAPTIC) != 0) {
			printf("Warning: Haptic subsystem unavailable, ignoring enable_controller_rumble = true\n");
		}
	}

	//SDL_EnableUNICODE(1); //deprecated
	Uint32 flags = 0;
	if (!start_fullscreen) start_fullscreen = check_param("full") != NULL;
	if (start_fullscreen) flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
	flags |= SDL_WINDOW_RESIZABLE;
	flags |= SDL_WINDOW_ALLOW_HIGHDPI; // for Retina displays

	// Should use different default window dimensions when using 4:3 aspect ratio
	if (use_correct_aspect_ratio && pop_window_width == 640 && pop_window_height == 400) {
		pop_window_height = 480;
	}

#if _WIN32
	// Tell Windows that the application is DPI aware, to prevent unwanted bitmap stretching.
	// SetProcessDPIAware() is only available on Windows Vista and later, so we need to load it dynamically.
	typedef BOOL (WINAPI *dpiaware)(void);
	HMODULE user32dll = LoadLibraryA("User32.dll");
	if (user32dll) {
		dpiaware SetProcessDPIAware = (dpiaware)GetProcAddress(user32dll, "SetProcessDPIAware");
		if (SetProcessDPIAware) {
			SetProcessDPIAware();
		}
		FreeLibrary(user32dll);
	}
#endif

#ifdef USE_REPLAY
	if (!is_validate_mode) // run without a window if validating a replay
#endif
	window_ = SDL_CreateWindow(WINDOW_TITLE,
	                           SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
	                           pop_window_width, pop_window_height, flags);
	// Make absolutely sure that VSync will be off, to prevent timer issues.
	SDL_SetHint(SDL_HINT_RENDER_VSYNC, "0");
	flags = 0;
	switch (use_hardware_acceleration) {
		case 0:  flags |= SDL_RENDERER_SOFTWARE;    break;
		case 1:  flags |= SDL_RENDERER_ACCELERATED; break;
		case 2:  // let SDL decide
		         // fallthrough!
		default: break;
	}
	renderer_ = SDL_CreateRenderer(window_, -1 , flags | SDL_RENDERER_TARGETTEXTURE);
	SDL_RendererInfo renderer_info;
	if (SDL_GetRendererInfo(renderer_, &renderer_info) == 0) {
		if (renderer_info.flags & SDL_RENDERER_TARGETTEXTURE) {
			is_renderer_targettexture_supported = true;
		}
	}
	if (use_integer_scaling) {
#if SDL_VERSION_ATLEAST(2,0,5) // SDL_RenderSetIntegerScale
		SDL_RenderSetIntegerScale(renderer_, SDL_TRUE);
#else
		printf("Warning: You need to compile with SDL 2.0.5 or newer for the use_integer_scaling option.\n");
#endif
	}

	SDL_Surface* icon = IMG_Load(locate_file("prince/icon.png"));
	if (icon == NULL) {
		sdlperror("set_gr_mode: Could not load icon");
	} else {
		#ifdef POP_RP2350
		// RP2350: no real window system; also avoid leaking the surface.
		// (Keeping this simple helps avoid subtle lifetime issues during bring-up.)
		DBG_PRINTF("[set_gr_mode] icon loaded, freeing\n");
		SDL_FreeSurface(icon);
		DBG_PRINTF("[set_gr_mode] icon freed\n");
		#else
		SDL_SetWindowIcon(window_, icon);
		#endif
	}

	#ifdef POP_RP2350
	// RP2350: skip aspect-ratio/resize logic (depends on SDL renderer/window sizing APIs).
	DBG_PRINTF("[set_gr_mode] after icon, before onscreen surface\n");
	#else
	apply_aspect_ratio();
	window_resized();
	#endif

	/* Migration to SDL2: everything is still blitted to onscreen_surface_, however:
	 * SDL2 renders textures to the screen instead of surfaces; so, every screen
	 * update causes the onscreen_surface_ to be copied into the target_texture, which is
	 * subsequently displayed.
	 * The function handling the screen updates is update_screen()
	 * */
	#ifdef POP_RP2350
	DBG_PRINTF("[set_gr_mode] SDL_CreateRGBSurface 320x200x8\n");
	onscreen_surface_ = SDL_CreateRGBSurface(SDL_FORCE_FULL_PALETTE, 320, 200, 8, 0, 0, 0, 0);
	DBG_PRINTF("[set_gr_mode] onscreen_surface_=%p\n", onscreen_surface_);
	#else
	onscreen_surface_ = SDL_CreateRGBSurface(0, 320, 200, 24, Rmsk, Gmsk, Bmsk, 0);
	#endif
	if (onscreen_surface_ == NULL) {
		sdlperror("set_gr_mode: SDL_CreateRGBSurface");
		quit(1);
	}
	#ifdef POP_RP2350
	// Initialize palette to something non-black until SDLPoP sets the real HC palette.
	if (onscreen_surface_->format && onscreen_surface_->format->palette) {
		DBG_PRINTF("[set_gr_mode] init temp palette\n");
		SDL_Color colors[256];
		for (int i = 0; i < 240; ++i) {
			colors[i].r = (Uint8)i;
			colors[i].g = (Uint8)i;
			colors[i].b = (Uint8)i;
			colors[i].a = 255;
		}
		for (int i = 240; i < 256; ++i) {
			colors[i].r = colors[i].g = colors[i].b = 0;
			colors[i].a = 255;
		}
		SDL_SetPaletteColors(onscreen_surface_->format->palette, colors, 0, 256);
		DBG_PRINTF("[set_gr_mode] temp palette set\n");
	}
	// RP2350 scanout uses our SDL_UpdateTexture shim; no need for overlay/scaling textures.
	DBG_PRINTF("[set_gr_mode] done (RP2350)\n");
	#else
	init_overlay();
	init_scaling();
	#endif
	if (start_fullscreen) {
		SDL_ShowCursor(SDL_DISABLE);
	}


	//SDL_WM_SetCaption(WINDOW_TITLE, NULL);
//	if (SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY, SDL_DEFAULT_REPEAT_INTERVAL) != 0) {  //deprecated
//		sdlperror("set_gr_mode: SDL_EnableKeyRepeat");
//		quit(1);
//	}
	graphics_mode = gmMcgaVga;
#ifdef USE_TEXT
	DBG_PRINTF("[set_gr_mode] before load_font\n");
	load_font();
	DBG_PRINTF("[set_gr_mode] after load_font\n");
#endif
	DBG_PRINTF("[set_gr_mode] return\n");
}

SDL_Surface* get_final_surface() {
	if (!is_overlay_displayed) {
		return onscreen_surface_;
	} else {
		return merged_surface;
	}
}

void draw_overlay(void) {
	int overlay = 0;
	is_overlay_displayed = false;
#ifdef USE_DEBUG_CHEATS
	if (is_timer_displayed && start_level > 0) overlay = 1; // Timer overlay
	else if (fixes->fix_quicksave_during_feather &&
				is_feather_timer_displayed &&
				start_level > 0 &&
				is_feather_fall > 0) {
		overlay = 3; // Feather timer overlay
	}
#endif
#ifdef USE_MENU
	// Menu overlay - not drawn here directly, only copied from the overlay surface.
	if (is_paused && is_menu_shown) overlay = 2;
#endif
	if (overlay != 0) {
		is_overlay_displayed = true;
		surface_type* saved_target_surface = current_target_surface;
		current_target_surface = overlay_surface;
		rect_type drawn_rect;
		if (overlay == 1) {
#ifdef USE_DEBUG_CHEATS
			char timer_text[32];
			if (rem_min < 0) {
				snprintf(timer_text, sizeof(timer_text), "%02d:%02d:%02d",
				         -(rem_min + 1), (719 - rem_tick) / 12, (719 - rem_tick) % 12);
			} else {
				snprintf(timer_text, sizeof(timer_text), "%02d:%02d:%02d",
				         rem_min - 1, rem_tick / 12, rem_tick % 12);
			}
			int expected_numeric_chars = 6;
			int extra_numeric_chars = MAX(0, (int)strnlen(timer_text, sizeof(timer_text)) - 8);
			int line_width = 5 + (expected_numeric_chars + extra_numeric_chars) * 9;

			rect_type timer_box_rect = {0, 0, 11, 2 + line_width};
			rect_type timer_text_rect = {2, 2, 10, 100};
			draw_rect_with_alpha(&timer_box_rect, color_0_black, 128);
			show_text(&timer_text_rect, halign_left, valign_top, timer_text);

#ifdef USE_REPLAY
			// During playback, display the number of ticks since start, if the timer is shown (debug cheats: T).
			if (replaying) {
				char ticks_text[12];
				snprintf(ticks_text, sizeof(ticks_text), "T: %d", curr_tick);
				rect_type ticks_box_rect = timer_box_rect;
				ticks_box_rect.top += 12;
				ticks_box_rect.bottom += 12;
				rect_type ticks_text_rect = timer_text_rect;
				ticks_text_rect.top += 12;
				ticks_text_rect.bottom += 12;

				draw_rect_with_alpha(&ticks_box_rect, color_0_black, 128);
				show_text(&ticks_text_rect, halign_left, valign_top, ticks_text);

				timer_box_rect.bottom += 12;
			}
#endif

			drawn_rect = timer_box_rect; // Only need to blit this bit to the merged_surface.
#endif
		} else if (overlay == 3) { // Feather timer
#ifdef USE_DEBUG_CHEATS
			char timer_text[32];
			int ticks_per_sec = get_ticks_per_sec(timer_1);
			snprintf(timer_text, sizeof(timer_text), "%02d:%02d", is_feather_fall / ticks_per_sec, is_feather_fall % ticks_per_sec);
			int expected_numeric_chars = 6;
			int extra_numeric_chars = MAX(0, (int)strnlen(timer_text, sizeof(timer_text)) - 8);
			int line_width = 5 + (expected_numeric_chars + extra_numeric_chars) * 9;

			rect_type timer_box_rect = {0, 0, 11, 2 + line_width};
			rect_type timer_text_rect = {2, 2, 10, 100};
			draw_rect_with_alpha(&timer_box_rect, color_0_black, 128);
			show_text_with_color(&timer_text_rect, halign_left, valign_top, timer_text, color_10_brightgreen);

			drawn_rect = timer_box_rect; // Only need to blit this bit to the merged_surface.
#endif
		} else {
			drawn_rect = screen_rect; // We'll blit the whole contents of overlay_surface to the merged_surface.
		}
		SDL_Rect sdl_rect;
		rect_to_sdlrect(&drawn_rect, &sdl_rect);
		SDL_BlitSurface(onscreen_surface_, NULL, merged_surface, NULL);
		SDL_BlitSurface(overlay_surface, &sdl_rect, merged_surface, &sdl_rect);
		current_target_surface = saved_target_surface;
	}
}

static int update_screen_debug = 0;
void update_screen() {
	#ifdef POP_RP2350
	// Don't update screen while fade is at full black - scene is already in HDMI from load_intro
	if (graphics_get_fade_level() >= 48) {
		return;
	}
	// RP2350 uses a fixed 320x200 8bpp onscreen surface and a custom scanout.
	// Bypass SDL2 scaling/texture logic (which assumes 24bpp/renderer features).
	SDL_Surface* screen = onscreen_surface_;
	if (screen && screen->pixels) {
		SDL_UpdateTexture(NULL, NULL, screen->pixels, screen->pitch);
	}
	SDL_RenderPresent(renderer_);
	return;
	#endif
	draw_overlay();
	SDL_Surface* surface = get_final_surface();
	init_scaling();
	if (scaling_type == 1) {
		// Make "fuzzy pixels" like DOSBox does:
		// First scale to double size with nearest-neighbor scaling, then scale to full screen with smooth scaling.
		// The result is not as blurry as if we did only a smooth scaling, but not as sharp as if we did only nearest-neighbor scaling.
		if (is_renderer_targettexture_supported) {
			SDL_UpdateTexture(texture_sharp, NULL, surface->pixels, surface->pitch);
			SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
			SDL_SetRenderTarget(renderer_, target_texture);
			SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0");
			SDL_RenderClear(renderer_);
			SDL_RenderCopy(renderer_, texture_sharp, NULL, NULL);
			SDL_SetRenderTarget(renderer_, NULL);
		} else {
			SDL_BlitScaled(surface, NULL, onscreen_surface_2x, NULL);
			surface = onscreen_surface_2x;
			SDL_UpdateTexture(target_texture, NULL, surface->pixels, surface->pitch);
		}
	} else {
		SDL_UpdateTexture(target_texture, NULL, surface->pixels, surface->pitch);
	}
	SDL_RenderClear(renderer_);
	SDL_RenderCopy(renderer_, target_texture, NULL, NULL);
	SDL_RenderPresent(renderer_);
}

#ifdef POP_RP2350
static inline void rp2350_apply_palette_256(void);
#endif

// seg009:9289
void set_pal_arr(int start,int count,const rgb_type* array) {
	// stub
	for (int i = 0; i < count; ++i) {
		if (array) {
			set_pal(start + i, array[i].r, array[i].g, array[i].b);
		} else {
			set_pal(start + i, 0, 0, 0);
		}
	}
	#ifdef POP_RP2350
	// set_pal() pushes per-entry; bulk push keeps things consistent for large updates.
	rp2350_apply_palette_256();
	#endif
}

rgb_type palette[256];

#ifdef POP_RP2350
static inline void rp2350_apply_palette_entry(int index, int red, int green, int blue) {
	if (index < 0 || index > 255) return;
	if (onscreen_surface_ == NULL || onscreen_surface_->format == NULL || onscreen_surface_->format->palette == NULL) return;
	SDL_Color c;
	c.r = (Uint8)(red << 2);
	c.g = (Uint8)(green << 2);
	c.b = (Uint8)(blue << 2);
	c.a = SDL_ALPHA_OPAQUE;
	SDL_SetPaletteColors(onscreen_surface_->format->palette, &c, index, 1);
}

static inline void rp2350_apply_palette_256(void) {
	if (onscreen_surface_ == NULL || onscreen_surface_->format == NULL || onscreen_surface_->format->palette == NULL) return;
	SDL_Color scolors[256];
	for (int i = 0; i < 256; ++i) {
		scolors[i].r = (Uint8)(palette[i].r << 2);
		scolors[i].g = (Uint8)(palette[i].g << 2);
		scolors[i].b = (Uint8)(palette[i].b << 2);
		scolors[i].a = SDL_ALPHA_OPAQUE;
	}
	SDL_SetPaletteColors(onscreen_surface_->format->palette, scolors, 0, 256);
}
#endif

// seg009:92DF
void set_pal(int index,int red,int green,int blue) {
	// stub
	//palette[index] = ((red&0x3F)<<2)|((green&0x3F)<<2<<8)|((blue&0x3F)<<2<<16);
	palette[index].r = red;
	palette[index].g = green;
	palette[index].b = blue;
	#ifdef POP_RP2350
	rp2350_apply_palette_entry(index, red, green, blue);
	#endif
}

// seg009:969C
int add_palette_bits(byte n_colors) {
	// stub
	return 0;
}

// seg009:9C36
int find_first_pal_row(int which_rows_mask) {
	word which_row = 0;
	word row_mask = 1;
	do {
		if (row_mask & which_rows_mask) {
			return which_row;
		}
		++which_row;
		row_mask <<= 1;
	} while (which_row < 16);
	return 0;
}

// seg009:9C6C
int get_text_color(int cga_color,int low_half,int high_half_mask) {
	if (graphics_mode == gmCga || graphics_mode == gmHgaHerc) {
		return cga_color;
	} else if (graphics_mode == gmMcgaVga && high_half_mask != 0) {
		return (find_first_pal_row(high_half_mask) << 4) + low_half;
	} else {
		return low_half;
	}
}

void load_from_opendats_metadata(int resource_id, const char* extension, pop_file_t** out_fp, data_location* result, byte* checksum, int* size, dat_type** out_pointer) {
	char image_filename[POP_MAX_PATH];
	pop_file_t* fp = NULL;
	*result = data_none;
	// Go through all open DAT files.
	for (dat_type* pointer = dat_chain_ptr; fp == NULL && pointer != NULL; pointer = pointer->next_dat) {
		*out_pointer = pointer;
		if (pointer->handle != NULL) {
			// If it's an actual DAT file:
			fp = (pop_file_t*)pointer->handle;
			dat_table_type* dat_table = pointer->dat_table;
			int i;
			for (i = 0; i < SDL_SwapLE16(dat_table->res_count); ++i) {
				if (SDL_SwapLE16(dat_table->entries[i].id) == resource_id) {
					break;
				}
			}
			if (i < SDL_SwapLE16(dat_table->res_count)) {
				// found
				*result = data_DAT;
				*size = SDL_SwapLE16(dat_table->entries[i].size);
				if (strcmp(extension,"png") == 0 && *size <= 2) {
					// Skip empty images in DATs, so we can fall back to directories.
					// This is useful for teleport graphics for example.
					fp = NULL;
					*result = data_none;
					*size = 0;
				} else
				if (pop_seek(fp, (long)SDL_SwapLE32(dat_table->entries[i].offset), SEEK_SET) ||
				    pop_read(checksum, 1, 1, fp) != 1
				) {
					printf("Cannot seek or cannot read checksum: ");
					#ifdef POP_RP2350
					printf("%s\n", pointer->filename);
					#else
					perror(pointer->filename);
					#endif
					fp = NULL;
				}
			} else {
				// not found
				fp = NULL;
			}
		}
		// If the image is not in the DAT then try the directory as well.
		if (*result == data_none) {
			// If it's a directory:
			char filename_no_ext[POP_MAX_PATH];
			// strip the .DAT file extension from the filename (use folders simply named TITLE, KID, VPALACE, etc.)
			strncpy(filename_no_ext, pointer->filename, sizeof(filename_no_ext));
			size_t len = strlen(filename_no_ext);
			if (len >= 5 && filename_no_ext[len-4] == '.') {
				filename_no_ext[len-4] = '\0'; // terminate, so ".DAT" is deleted from the filename
			}
			snprintf_check(image_filename,sizeof(image_filename),"prince/%s/res%d.%s",filename_no_ext, resource_id, extension);
			if (!use_custom_levelset) {
				//printf("loading (binary) %s",image_filename);
				fp = pop_open(locate_file(image_filename), "rb");
			}
			else {
				if (!skip_mod_data_files) {
					char image_filename_mod[POP_MAX_PATH];
					// before checking prince/, first try mods/MODNAME/prince/
					snprintf_check(image_filename_mod, sizeof(image_filename_mod), "%s/%s", mod_data_path, image_filename);
					//printf("loading (binary) %s",image_filename_mod);
					fp = pop_open(locate_file(image_filename_mod), "rb");
				}
				if (fp == NULL && !skip_normal_data_files) {
					fp = pop_open(locate_file(image_filename), "rb");
				}
			}

			if (fp != NULL) {
				long sz = pop_size(fp);
				if (sz >= 0) {
					*result = data_directory;
					*size = (int)sz;
				} else {
					printf("Cannot stat: %s\n", image_filename);
					pop_close(fp);
					fp = NULL;
				}
			}
		}
	}
	*out_fp = fp;
	if (fp == NULL) {
		*result = data_none;
//		printf(" FAILED\n");
		//return NULL;
	}
	//...
}

// seg009:9F34
void close_dat(dat_type* pointer) {
	dat_type** prev = &dat_chain_ptr;
	dat_type* curr = dat_chain_ptr;
	while (curr != NULL) {
		if (curr == pointer) {
			*prev = curr->next_dat;
			#ifdef POP_RP2350
			if (curr->handle) pop_close((pop_file_t*)curr->handle);
			#else
			if (curr->handle) fclose((FILE*)curr->handle);
			#endif
			if (curr->dat_table) free(curr->dat_table);
			free(curr);
			return;
		}
		curr = curr->next_dat;
		prev = &((*prev)->next_dat);
	}
	// stub
}

// seg009:9F80
void *load_from_opendats_alloc(int resource, const char* extension, data_location* out_result, int* out_size) {
	// stub
	//printf("id = %d\n",resource);
	dat_type* pointer;
	data_location result;
	byte checksum;
	int size;
	pop_file_t* fp = NULL;
	load_from_opendats_metadata(resource, extension, &fp, &result, &checksum, &size, &pointer);
	if (out_result != NULL) *out_result = result;
	if (out_size != NULL) *out_size = size;
	if (result == data_none) return NULL;
	void* area = NULL;
	#ifdef POP_RP2350
	// IMPORTANT: psram_malloc is a bump allocator (psram_free is a no-op), so using it for
	// per-image file contents will leak PSRAM quickly. For PNG resources (which are consumed
	// immediately by IMG_Load_RW/decode_image) we can use the dedicated reusable PSRAM file buffer.
	if (extension && strcmp(extension, "png") == 0) {
		area = psram_get_file_buffer((size_t)size);
		if (area == NULL) {
			// Fallback: try general heap (PSRAM bump, then SRAM malloc). This may still OOM,
			// but keeps behavior consistent for oversized resources.
			area = pop_heap_alloc((size_t)size);
		}
	} else {
		area = pop_heap_alloc((size_t)size);
	}
	#else
	area = malloc(size);
	#endif
	if (area == NULL) {
		fprintf(stderr, "%s: out of memory for resource %d size %d from %s\n",
			__func__, resource, size, pointer ? pointer->filename : "(unknown)");
		if (result == data_directory) pop_close(fp);
		return NULL;
	}
	//read(fd, area, size);
	if (pop_read(area, size, 1, fp) != 1) {
		fprintf(stderr, "%s: %s, resource %d, size %d, failed: %s\n",
			__func__, pointer->filename, resource,
			size, strerror(errno));
		#ifdef POP_RP2350
		pop_heap_free(area);
		#else
		free(area);
		#endif
		area = NULL;
	}
	if (result == data_directory) pop_close(fp);
	/* XXX: check checksum */
	return area;
}

// seg009:A172
int load_from_opendats_to_area(int resource,void* area,int length, const char* extension) {
	// stub
	//return 0;
	dat_type* pointer;
	data_location result;
	byte checksum;
	int size;
	pop_file_t* fp = NULL;
	load_from_opendats_metadata(resource, extension, &fp, &result, &checksum, &size, &pointer);
	if (result == data_none) return 0;
	if (pop_read(area, MIN(size, length), 1, fp) != 1) {
		fprintf(stderr, "%s: %s, resource %d, size %d, failed: %s\n",
			__func__, pointer->filename, resource,
			size, strerror(errno));
		memset(area, 0, MIN(size, length));
	}
	if (result == data_directory) pop_close(fp);
	/* XXX: check checksum */
	return 0;
}

// SDL-specific implementations

void rect_to_sdlrect(const rect_type* rect, SDL_Rect* sdlrect) {
	sdlrect->x = rect->left;
	sdlrect->y = rect->top;
	sdlrect->w = rect->right - rect->left;
	sdlrect->h = rect->bottom - rect->top;
}

void method_1_blit_rect(surface_type* target_surface,surface_type* source_surface,const rect_type* target_rect, const rect_type* source_rect,int blit) {
	SDL_Rect src_rect;
	rect_to_sdlrect(source_rect, &src_rect);
	SDL_Rect dest_rect;
	rect_to_sdlrect(target_rect, &dest_rect);

	if (blit == blitters_0_no_transp) {
		// Disable transparency.
		if (SDL_SetColorKey(source_surface, 0, 0) != 0) {
			sdlperror("method_1_blit_rect: SDL_SetColorKey");
			quit(1);
		}
		// For RGBA surfaces, also disable alpha blending
		if (!SDL_ISPIXELFORMAT_INDEXED(source_surface->format->format)) {
			SDL_SetSurfaceBlendMode(source_surface, SDL_BLENDMODE_NONE);
		}
	} else {
		// Enable transparency.
		if (SDL_SetColorKey(source_surface, SDL_TRUE, 0) != 0) {
			sdlperror("method_1_blit_rect: SDL_SetColorKey");
			quit(1);
		}
	}
	if (SDL_BlitSurface(source_surface, &src_rect, target_surface, &dest_rect) != 0) {
		sdlperror("method_1_blit_rect: SDL_BlitSurface");
		quit(1);
	}
}

image_type* method_3_blit_mono(image_type* image,int xpos,int ypos,int blitter,byte color) {
	int w = image->w;
	int h = image->h;
	#ifdef POP_RP2350
	static int rp2350_m3_calls = 0;
	if (rp2350_m3_calls < 16) {
		DBG_PRINTF("[method_3_blit_mono] #%d img=%p %dx%d bpp=%d dst=(%d,%d) color=%u\n",
			rp2350_m3_calls, image, w, h, image->format->BytesPerPixel, xpos, ypos, (unsigned)color);
	}
	++rp2350_m3_calls;
	#endif
	
	if (SDL_SetColorKey(image, SDL_TRUE, 0) != 0) {
		sdlperror("method_3_blit_mono: SDL_SetColorKey");
		quit(1);
	}

#ifdef POP_RP2350
	// RP2350 optimization: For 8bpp mono images (fonts converted during load_image),
	// do a direct manual blit writing the destination palette index directly.
	// This avoids allocating a 32bpp temporary surface AND avoids palette mapping issues.
	// Font pixels: 0 = transparent (skip), 1 = foreground (write 'color' index).
	if (image->format->BytesPerPixel == 1 && current_target_surface->format->BytesPerPixel == 1) {
		// Direct 8bpp→8bpp mono blit: write destination index 'color' for foreground pixels
		uint8_t* src_pixels = (uint8_t*)image->pixels;
		uint8_t* dst_pixels = (uint8_t*)current_target_surface->pixels;
		int src_pitch = image->pitch;
		int dst_pitch = current_target_surface->pitch;
		// Use clip_rect for clipping instead of surface bounds
		SDL_Rect clip = current_target_surface->clip_rect;
		int clip_left = clip.x;
		int clip_top = clip.y;
		int clip_right = clip.x + clip.w;
		int clip_bottom = clip.y + clip.h;
		
		// Debug: check source pixel values and palette state
		if (rp2350_m3_calls < 3) {
			// Print first row of raw pixel values
			DBG_PRINTF("[method_3_blit_mono] src row0: ");
			for (int x = 0; x < w && x < 10; ++x) {
				DBG_PRINTF("%02X ", src_pixels[x]);
			}
			DBG_PRINTF("\n");
			// Count total fg/bg pixels in entire image
			int fg_total = 0, bg_total = 0;
			for (int y = 0; y < h; ++y) {
				for (int x = 0; x < w; ++x) {
					uint8_t pix = src_pixels[y * src_pitch + x];
					if (pix == 0) bg_total++;
					else fg_total++;
				}
			}
			DBG_PRINTF("[method_3_blit_mono] src total: fg=%d bg=%d (of %d)\n", fg_total, bg_total, w*h);
		}
		
		for (int y = 0; y < h; ++y) {
			int dy = ypos + y;
			if (dy < clip_top || dy >= clip_bottom) continue;
			for (int x = 0; x < w; ++x) {
				int dx = xpos + x;
				if (dx < clip_left || dx >= clip_right) continue;
				uint8_t src_pix = src_pixels[y * src_pitch + x];
				if (src_pix == 0) continue; // Transparent (colorkey)
				// Foreground pixel: write the destination palette index 'color'
				dst_pixels[dy * dst_pitch + dx] = color;
			}
		}
		if (rp2350_m3_calls < 16) {
			DBG_PRINTF("[method_3_blit_mono] 8bpp direct blit ok (color_idx=%u)\n", (unsigned)color);
		}
		return image;
	}
	// Fall through for 32bpp images or mismatched bpp
	if (rp2350_m3_calls < 16) {
		DBG_PRINTF("[method_3_blit_mono] WARNING: fallback path (src_bpp=%d dst_bpp=%d)\n", 
			image->format->BytesPerPixel, current_target_surface->format->BytesPerPixel);
	}
#endif

	SDL_Surface* colored_image = SDL_ConvertSurfaceFormat(image, SDL_PIXELFORMAT_ARGB8888, 0);
	#ifdef POP_RP2350
	if (rp2350_m3_calls < 16) {
		DBG_PRINTF("[method_3_blit_mono] converted=%p\n", colored_image);
	}
	#endif

	SDL_SetSurfaceBlendMode(colored_image, SDL_BLENDMODE_NONE);
	/* Causes problems with SDL 2.0.5 (see #105)
	if (SDL_SetColorKey(colored_image, SDL_TRUE, 0) != 0) {
		sdlperror("method_3_blit_mono: SDL_SetColorKey");
		quit(1);
	}
	*/

	if (SDL_LockSurface(colored_image) != 0) {
		sdlperror("method_3_blit_mono: SDL_LockSurface");
		quit(1);
	}

	rgb_type palette_color = palette[color];
	uint32_t rgb_color = SDL_MapRGB(colored_image->format, palette_color.r<<2, palette_color.g<<2, palette_color.b<<2) & 0xFFFFFF;
	int stride = colored_image->pitch;
	for (int y = 0; y < h; ++y) {
		uint32_t* pixel_ptr = (uint32_t*) ((byte*)colored_image->pixels + stride * y);
		for (int x = 0; x < w; ++x) {
			// set RGB but leave alpha
			*pixel_ptr = (*pixel_ptr & 0xFF000000) | rgb_color;
			//printf("pixel x=%d, y=%d, color = 0x%8x\n", x, y, *pixel_ptr);
			++pixel_ptr;
		}
	}
	SDL_UnlockSurface(colored_image);

	SDL_Rect src_rect = {0, 0, image->w, image->h};
	SDL_Rect dest_rect = {xpos, ypos, image->w, image->h};

	SDL_SetSurfaceBlendMode(colored_image, SDL_BLENDMODE_BLEND);
	SDL_SetSurfaceBlendMode(current_target_surface, SDL_BLENDMODE_BLEND);
	SDL_SetSurfaceAlphaMod(colored_image, 255);
	if (SDL_BlitSurface(colored_image, &src_rect, current_target_surface, &dest_rect) != 0) {
		sdlperror("method_3_blit_mono: SDL_BlitSurface");
		quit(1);
	}
	#ifdef POP_RP2350
	if (rp2350_m3_calls < 16) {
		DBG_PRINTF("[method_3_blit_mono] blit ok\n");
	}
	#endif
	SDL_FreeSurface(colored_image);

	return image;
}

// Workaround for a bug in SDL2 (before v2.0.4):
// https://bugzilla.libsdl.org/show_bug.cgi?id=2986
// SDL_FillRect onto a 24-bit surface swaps Red and Blue component

bool RGB24_bug_checked = false;
bool RGB24_bug_affected;

bool RGB24_bug_check(void) {
	if (!RGB24_bug_checked) {
		// Check if the bug occurs in this version of SDL.
		SDL_Surface* test_surface = SDL_CreateRGBSurface(0, 1, 1, 24, 0, 0, 0, 0);
		if (NULL == test_surface) sdlperror("SDL_CreateSurface in RGB24_bug_check");
		// Fill with red.
		SDL_FillRect(test_surface, NULL, SDL_MapRGB(test_surface->format, 0xFF, 0, 0));
		if (0 != SDL_LockSurface(test_surface)) sdlperror("SDL_LockSurface in RGB24_bug_check");
		// Read red component of pixel.
		RGB24_bug_affected = (*(Uint32*)test_surface->pixels & test_surface->format->Rmask) == 0;
		SDL_UnlockSurface(test_surface);
		SDL_FreeSurface(test_surface);
		RGB24_bug_checked = true;
	}
	return RGB24_bug_affected;
}

int safe_SDL_FillRect(SDL_Surface* dst, const SDL_Rect* rect, Uint32 color) {
	if (dst->format->BitsPerPixel == 24 && RGB24_bug_check()) {
		// In the buggy version, SDL_FillRect swaps R and B, so we swap it once more.
		color = ((color & 0xFF) << 16) | (color & 0xFF00) | ((color & 0xFF0000) >> 16);
	}
	return SDL_FillRect(dst, rect, color);
}
// End of workaround.

const rect_type* method_5_rect(const rect_type* rect,int blit,byte color) {
	SDL_Rect dest_rect;
	rect_to_sdlrect(rect, &dest_rect);
	rgb_type palette_color = palette[color];
#ifndef USE_ALPHA
	uint32_t rgb_color = SDL_MapRGBA(current_target_surface->format, palette_color.r<<2, palette_color.g<<2, palette_color.b<<2, 0xFF);
#else
	uint32_t rgb_color = SDL_MapRGBA(current_target_surface->format, palette_color.r<<2, palette_color.g<<2, palette_color.b<<2, color == 0 ? SDL_ALPHA_TRANSPARENT : SDL_ALPHA_OPAQUE);
#endif
	if (safe_SDL_FillRect(current_target_surface, &dest_rect, rgb_color) != 0) {
		sdlperror("method_5_rect: SDL_FillRect");
		quit(1);
	}
	return rect;
}

void draw_rect_with_alpha(const rect_type* rect, byte color, byte alpha) {
	SDL_Rect dest_rect;
	rect_to_sdlrect(rect, &dest_rect);
	rgb_type palette_color = palette[color];
	uint32_t rgb_color = SDL_MapRGBA(overlay_surface->format, palette_color.r<<2, palette_color.g<<2, palette_color.b<<2, alpha);
	if (safe_SDL_FillRect(current_target_surface, &dest_rect, rgb_color) != 0) {
		sdlperror("draw_rect_with_alpha: SDL_FillRect");
		quit(1);
	}
}

void draw_rect_contours(const rect_type* rect, byte color) {
	// TODO: handle 24 bit surfaces? (currently, 32 bit surface is assumed)
	if (current_target_surface->format->BitsPerPixel != 32) {
		printf("draw_rect_contours: not implemented for %d bit surfaces\n", current_target_surface->format->BitsPerPixel);
		return;
	}
	SDL_Rect dest_rect;
	rect_to_sdlrect(rect, &dest_rect);
	rgb_type palette_color = palette[color];
	uint32_t rgb_color = SDL_MapRGBA(overlay_surface->format, palette_color.r<<2, palette_color.g<<2, palette_color.b<<2, 0xFF);
	if (SDL_LockSurface(current_target_surface) != 0) {
		sdlperror("draw_rect_contours: SDL_LockSurface");
		quit(1);
	}
	int bytes_per_pixel = current_target_surface->format->BytesPerPixel;
	int pitch = current_target_surface->pitch;
	byte* pixels = current_target_surface->pixels;
	int xmin = MIN(dest_rect.x,               current_target_surface->w);
	int xmax = MIN(dest_rect.x + dest_rect.w, current_target_surface->w);
	int ymin = MIN(dest_rect.y,               current_target_surface->h);
	int ymax = MIN(dest_rect.y + dest_rect.h, current_target_surface->h);
	byte* row = pixels + ymin*pitch;
	uint32_t* pixel =  (uint32_t*) (row + xmin*bytes_per_pixel);
	for (int x = xmin; x < xmax; ++x) {
		*pixel++ = rgb_color;
	}
	for (int y = ymin+1; y < ymax-1; ++y) {
		row += pitch;
		*(uint32_t*)(row + xmin*bytes_per_pixel) = rgb_color;
		*(uint32_t*)(row + (xmax-1)*bytes_per_pixel) = rgb_color;
	}
	pixel = (uint32_t*) (pixels + (ymax-1)*pitch + xmin*bytes_per_pixel);
	for (int x = xmin; x < xmax; ++x) {
		*pixel++ = rgb_color;
	}

	SDL_UnlockSurface(current_target_surface);
}

#ifdef POP_RP2350
// RP2350-specific XOR blit for 8-bit indexed surfaces
// For the shadow prince, we want a dark semi-transparent effect.
// XOR on palette indices doesn't look good, so instead we draw the shadow
// as a darkened/inverted version by setting non-transparent pixels to a dark color.
void blit_xor(SDL_Surface* target_surface, SDL_Rect* dest_rect, SDL_Surface* image, SDL_Rect* src_rect) {
	if (dest_rect->w != src_rect->w || dest_rect->h != src_rect->h) {
		printf("blit_xor: dest_rect and src_rect have different sizes\n");
		return;
	}
	
	// Lock surfaces for direct pixel access
	if (SDL_LockSurface(target_surface) != 0) {
		return;
	}
	if (SDL_LockSurface(image) != 0) {
		SDL_UnlockSurface(target_surface);
		return;
	}
	
	// Both surfaces should be 8-bit indexed on RP2350
	int src_bpp = image->format->BytesPerPixel;
	int dst_bpp = target_surface->format->BytesPerPixel;
	
	if (src_bpp == 1 && dst_bpp == 1) {
		// 8-bit indexed: XOR the palette indices directly
		for (int y = 0; y < dest_rect->h; y++) {
			int src_y = src_rect->y + y;
			int dst_y = dest_rect->y + y;
			if (dst_y < 0 || dst_y >= target_surface->h) continue;
			
			byte* src_row = (byte*)image->pixels + src_y * image->pitch + src_rect->x;
			byte* dst_row = (byte*)target_surface->pixels + dst_y * target_surface->pitch + dest_rect->x;
			
			for (int x = 0; x < dest_rect->w; x++) {
				int dst_x = dest_rect->x + x;
				if (dst_x < 0 || dst_x >= target_surface->w) continue;
				
				byte src_pixel = src_row[x];
				// Skip transparent pixels (color 0)
				if (src_pixel != 0) {
					// XOR the palette index for the shadow effect
					dst_row[x] ^= src_pixel;
				}
			}
		}
	}
	
	SDL_UnlockSurface(image);
	SDL_UnlockSurface(target_surface);
}
#else
void blit_xor(SDL_Surface* target_surface, SDL_Rect* dest_rect, SDL_Surface* image, SDL_Rect* src_rect) {
	if (dest_rect->w != src_rect->w || dest_rect->h != src_rect->h) {
		printf("blit_xor: dest_rect and src_rect have different sizes\n");
		quit(1);
	}
	SDL_Surface* helper_surface = SDL_CreateRGBSurface(0, dest_rect->w, dest_rect->h, 24, Rmsk, Gmsk, Bmsk, 0);
	if (helper_surface == NULL) {
		sdlperror("blit_xor: SDL_CreateRGBSurface");
		quit(1);
	}
	SDL_Surface* image_24 = SDL_ConvertSurface(image, helper_surface->format, 0);
	//SDL_CreateRGBSurface(0, src_rect->w, src_rect->h, 24, Rmsk, Gmsk, Bmsk, 0);
	if (image_24 == NULL) {
		sdlperror("blit_xor: SDL_CreateRGBSurface");
		quit(1);
	}
	SDL_Rect dest_rect2 = *src_rect;
	// Read what is currently where we want to draw the new image.
	if (SDL_BlitSurface(target_surface, dest_rect, helper_surface, &dest_rect2) != 0) {
		sdlperror("blit_xor: SDL_BlitSurface");
		quit(1);
	}
	if (SDL_LockSurface(image_24) != 0) {
		sdlperror("blit_xor: SDL_LockSurface");
		quit(1);
	}
	if (SDL_LockSurface(helper_surface) != 0) {
		sdlperror("blit_xor: SDL_LockSurface");
		quit(1);
	}
	int size = helper_surface->h * helper_surface->pitch;
	byte *p_src = (byte*) image_24->pixels;
	byte *p_dest = (byte*) helper_surface->pixels;

	// Xor the old area with the image.
	for (int i = 0; i < size; ++i) {
		*p_dest ^= *p_src;
		++p_src; ++p_dest;
	}
	SDL_UnlockSurface(image_24);
	SDL_UnlockSurface(helper_surface);
	// Put the new area in place of the old one.
	if (SDL_BlitSurface(helper_surface, src_rect, target_surface, dest_rect) != 0) {
		sdlperror("blit_xor: SDL_BlitSurface 2065");
		quit(1);
	}
	SDL_FreeSurface(image_24);
	SDL_FreeSurface(helper_surface);
}
#endif

#ifdef USE_COLORED_TORCHES
void draw_colored_torch(int color, SDL_Surface* image, int xpos, int ypos) {
	if (SDL_SetColorKey(image, SDL_TRUE, 0) != 0) {
		sdlperror("draw_colored_torch: SDL_SetColorKey");
		quit(1);
	}

	SDL_Surface* colored_image = SDL_ConvertSurfaceFormat(image, SDL_PIXELFORMAT_ARGB8888, 0);
	SDL_SetSurfaceBlendMode(colored_image, SDL_BLENDMODE_NONE);

	if (SDL_LockSurface(colored_image) != 0) {
		sdlperror("draw_colored_torch: SDL_LockSurface");
		quit(1);
	}

	int w = colored_image->w;
	int h = colored_image->h;
	int iRed = ((color >> 4) & 3) * 85;
	int iGreen = ((color >> 2) & 3) * 85;
	int iBlue = ((color >> 0) & 3) * 85;
	uint32_t old_color = SDL_MapRGB(colored_image->format, 0xFC, 0x84, 0x00) & 0xFFFFFF; // the orange in the flame
	uint32_t new_color = SDL_MapRGB(colored_image->format, iRed, iGreen, iBlue) & 0xFFFFFF;
	int stride = colored_image->pitch;
	for (int y = 0; y < h; ++y) {
		uint32_t* pixel_ptr = (uint32_t*) ((byte*)colored_image->pixels + stride * y);
		for (int x = 0; x < w; ++x) {
			if ((*pixel_ptr & 0xFFFFFF) == old_color) {
				// set RGB but leave alpha
				*pixel_ptr = (*pixel_ptr & 0xFF000000) | new_color;
			}
			++pixel_ptr;
		}
	}
	SDL_UnlockSurface(colored_image);

	method_6_blit_img_to_scr(colored_image, xpos, ypos, blitters_0_no_transp);
	SDL_FreeSurface(colored_image);
}
#endif

image_type* method_6_blit_img_to_scr(image_type* image,int xpos,int ypos,int blit) {
	if (image == NULL) {
		DBG_PRINTF("method_6_blit_img_to_scr: image == NULL\n");
		//quit(1);
		return NULL;
	}

#ifdef POP_RP2350
	// Debug: trace when sprites start at y >= 192 (bypass hard clip)
	extern SDL_Surface* onscreen_surface_;
	if (current_target_surface == onscreen_surface_ && ypos >= 192 && ypos < 200) {
		static int bypass_clip_debug = 0;
		if (bypass_clip_debug < 50) {
			DBG_PRINTF("[BYPASS_CLIP] blit=0x%02X x=%d y=%d %dx%d bpp=%d (HP at y=194 expected)\n",
				blit, xpos, ypos, image->w, image->h, image->format->BytesPerPixel);
			bypass_clip_debug++;
		}
	}
	// Debug: trace sprites that would be clipped (y < 192 but extends past 192)
	if (current_target_surface == onscreen_surface_ && ypos < 192 && ypos + image->h > 192) {
		static int clipped_debug = 0;
		if (clipped_debug < 20) {
			DBG_PRINTF("[WILL_CLIP] blit=0x%02X x=%d y=%d h=%d -> would reach y=%d, clipping at 192\n",
				blit, xpos, ypos, image->h, ypos + image->h);
			clipped_debug++;
		}
	}
#endif

	if (blit == blitters_9_black) {
		method_3_blit_mono(image, xpos, ypos, blitters_9_black, 0);
		return image;
	}

	SDL_Rect src_rect = {0, 0, image->w, image->h};
	SDL_Rect dest_rect = {xpos, ypos, image->w, image->h};

	if (blit == blitters_3_xor) {
		blit_xor(current_target_surface, &dest_rect, image, &src_rect);
		return image;
	}

#ifdef USE_COLORED_TORCHES
	if (blit >= blitters_colored_flame && blit <= blitters_colored_flame_last) {
		draw_colored_torch(blit - blitters_colored_flame, image, xpos, ypos);
		return image;
	}
#endif

	SDL_SetSurfaceBlendMode(image, SDL_BLENDMODE_NONE);
	SDL_SetColorKey(image, SDL_FALSE, 0);
	SDL_SetSurfaceAlphaMod(image, 255);

	#ifdef POP_RP2350
	static int rp2350_m6_calls = 0;
	int is_indexed = SDL_ISPIXELFORMAT_INDEXED(image->format->format);
	int show_log = (rp2350_m6_calls < 100) && (image->w < 100 || image->h < 100);  // Show smaller sprites
	if (show_log) {
		DBG_PRINTF("[method_6] #%d blit=0x%02X idx=%d %dx%d dst=(%d,%d) src=%p\n",
			rp2350_m6_calls, blit, is_indexed,
			image->w, image->h, xpos, ypos, (void*)image);
	}
	++rp2350_m6_calls;
	#endif

	//printf("format = %s\n", SDL_GetPixelFormatName(image->format->format));
	// For transparency:
	// - Indexed surfaces: use colorkey (black=0 is transparent)
	// - RGBA surfaces blitting to indexed target: SDL can't do alpha blending,
	//   so we must manually copy only opaque pixels
	
	int is_rgba_to_indexed = (!SDL_ISPIXELFORMAT_INDEXED(image->format->format) && 
	                          image->format->BytesPerPixel == 4 &&
	                          SDL_ISPIXELFORMAT_INDEXED(current_target_surface->format->format));

	#ifdef POP_RP2350
	static int rp2350_rgba_debug = 0;
	// Print for EVERY RGBA sprite with transparency until we see the issue
	if (rp2350_rgba_debug < 20 && image->format->BytesPerPixel == 4 && blit != blitters_0_no_transp) {
		DBG_PRINTF("[RGBA_TRANSP] blit=0x%02X rgba_to_idx=%d target_fmt=0x%08lx (INDEX8=0x%08lx) target_bpp=%d\n",
			blit, is_rgba_to_indexed,
			(unsigned long)current_target_surface->format->format,
			(unsigned long)SDL_PIXELFORMAT_INDEX8,
			current_target_surface->format->BytesPerPixel);
		rp2350_rgba_debug++;
	}
	#endif
	
	// ALWAYS use manual path for RGBA→indexed, regardless of blit mode
	// The difference is: blit=0x00 copies all pixels, blit!=0x00 respects alpha
	if (is_rgba_to_indexed) {
		// Manual RGBA→indexed blit
		const int respect_alpha = (blit != blitters_0_no_transp);
		uint8_t* src_pixels = (uint8_t*)image->pixels;
		uint8_t* dst_pixels = (uint8_t*)current_target_surface->pixels;
		int src_pitch = image->pitch;
		int dst_pitch = current_target_surface->pitch;
		
		SDL_Palette* pal = current_target_surface->format->palette;
		
		// Use clip_rect for clipping
		SDL_Rect clip = current_target_surface->clip_rect;
		int clip_left = clip.x;
		int clip_top = clip.y;
		int clip_right = clip.x + clip.w;
		int clip_bottom = clip.y + clip.h;
		
#ifdef POP_RP2350
		// HARD CLIP: game sprites (ypos < 192) must not draw into health bar area
		// Only during gameplay (start_level >= 0)
		extern SDL_Surface* onscreen_surface_;
		extern short start_level;
		if (start_level >= 0 && current_target_surface == onscreen_surface_ && ypos < 192 && clip_bottom > 192) {
			clip_bottom = 192;
		}
#endif
		
		// Calculate clipped region
		int src_x = 0, src_y = 0;
		int dst_x = xpos, dst_y = ypos;
		int copy_w = image->w, copy_h = image->h;
		
		// Clip left
		if (dst_x < clip_left) { src_x = clip_left - dst_x; copy_w -= (clip_left - dst_x); dst_x = clip_left; }
		// Clip top
		if (dst_y < clip_top) { src_y = clip_top - dst_y; copy_h -= (clip_top - dst_y); dst_y = clip_top; }
		// Clip right
		if (dst_x + copy_w > clip_right) { copy_w = clip_right - dst_x; }
		// Clip bottom
		if (dst_y + copy_h > clip_bottom) { copy_h = clip_bottom - dst_y; }
		
		#ifdef POP_RP2350
		static int manual_blit_debug = 0;
		if (manual_blit_debug < 5 && respect_alpha) {
			DBG_PRINTF("[MANUAL_BLIT] %dx%d respect_alpha=%d amask=0x%08lx pal=%p ncolors=%d\n",
				copy_w, copy_h, respect_alpha, 
				(unsigned long)(image->format ? image->format->Amask : 0),
				(void*)pal, pal ? pal->ncolors : 0);
			// Print first few pixels
			if (copy_w > 0 && copy_h > 0) {
				uint32_t* first_row = (uint32_t*)(src_pixels + src_y * src_pitch) + src_x;
				DBG_PRINTF("[MANUAL_BLIT] first 4 pixels: 0x%08lx 0x%08lx 0x%08lx 0x%08lx\n",
					(unsigned long)first_row[0], 
					copy_w > 1 ? (unsigned long)first_row[1] : 0UL,
					copy_w > 2 ? (unsigned long)first_row[2] : 0UL,
					copy_w > 3 ? (unsigned long)first_row[3] : 0UL);
			}
			manual_blit_debug++;
		}
		#endif
		
		if (copy_w > 0 && copy_h > 0 && pal && pal->ncolors > 0) {
			// Get the pixel format masks
			uint32_t rmask = image->format->Rmask;
			uint32_t gmask = image->format->Gmask;
			uint32_t bmask = image->format->Bmask;
			uint32_t amask = image->format->Amask;
			
			// Calculate shifts from masks (find first set bit)
			int rshift = 0, gshift = 0, bshift = 0, ashift = 0;
			if (rmask) { uint32_t m = rmask; while (!(m & 1)) { m >>= 1; rshift++; } }
			if (gmask) { uint32_t m = gmask; while (!(m & 1)) { m >>= 1; gshift++; } }
			if (bmask) { uint32_t m = bmask; while (!(m & 1)) { m >>= 1; bshift++; } }
			if (amask) { uint32_t m = amask; while (!(m & 1)) { m >>= 1; ashift++; } }
			
			#ifdef POP_RP2350
			int pixels_written = 0, pixels_skipped = 0;
			#endif
			
			for (int y = 0; y < copy_h; ++y) {
				uint32_t* src_row = (uint32_t*)(src_pixels + (src_y + y) * src_pitch) + src_x;
				uint8_t* dst_row = dst_pixels + (dst_y + y) * dst_pitch + dst_x;
				
				for (int x = 0; x < copy_w; ++x) {
					uint32_t pixel = src_row[x];
					uint8_t alpha = (pixel & amask) >> ashift;
					
					// If respecting alpha: only copy opaque pixels (alpha > 0)
					// If not respecting alpha (blit=0x00): copy all pixels
					if (!respect_alpha || alpha > 0) {
						uint8_t r = (pixel & rmask) >> rshift;
						uint8_t g = (pixel & gmask) >> gshift;
						uint8_t b = (pixel & bmask) >> bshift;
						
						// Find nearest palette color (simple brute force)
						int best_dist = 0x7FFFFFFF;
						int best_idx = 0;
						for (int i = 0; i < pal->ncolors; ++i) {
							int dr = (int)r - pal->colors[i].r;
							int dg = (int)g - pal->colors[i].g;
							int db = (int)b - pal->colors[i].b;
							int dist = dr*dr + dg*dg + db*db;
							if (dist < best_dist) {
								best_dist = dist;
								best_idx = i;
							}
						}
						dst_row[x] = (uint8_t)best_idx;
						#ifdef POP_RP2350
						pixels_written++;
						#endif
					} else {
						#ifdef POP_RP2350
						pixels_skipped++;
						#endif
					}
					// If alpha == 0 and respect_alpha, don't write (preserve destination = transparency)
				}
			}
			
			#ifdef POP_RP2350
			static int blit_stats_count = 0;
			if (blit_stats_count < 10 && respect_alpha) {
				DBG_PRINTF("[BLIT_STATS] written=%d skipped=%d total=%d\n", 
					pixels_written, pixels_skipped, pixels_written + pixels_skipped);
				blit_stats_count++;
			}
			#endif
		}
		
		return image;
	}
	
	// Standard SDL blit path for indexed surfaces
	if (blit == blitters_0_no_transp) {
		if (SDL_ISPIXELFORMAT_INDEXED(image->format->format)) {
			SDL_SetColorKey(image, SDL_FALSE, 0);
		} else {
			SDL_SetSurfaceBlendMode(image, SDL_BLENDMODE_NONE);
		}
	}
	else {
		// Indexed surface with transparency: use colorkey
		SDL_SetColorKey(image, SDL_TRUE, 0);
		SDL_SetSurfaceBlendMode(image, SDL_BLENDMODE_NONE);
	}
	if (SDL_BlitSurface(image, &src_rect, current_target_surface, &dest_rect) != 0) {
		sdlperror("method_6_blit_img_to_scr: SDL_BlitSurface 2247");
		//quit(1);
	}
/*
	if (SDL_SetSurfaceAlphaMod(image, 0) != 0) {
		sdlperror("method_6_blit_img_to_scr: SDL_SetAlpha");
		quit(1);
	}
*/
	return image;
}

#ifndef USE_COMPAT_TIMER
int fps = BASE_FPS;
float milliseconds_per_tick = (1000.0f / (BASE_FPS*1.0f));
Uint64 timer_last_counter[NUM_TIMERS];
#endif
int wait_time[NUM_TIMERS];


#ifdef USE_COMPAT_TIMER
Uint32 timer_callback(Uint32 interval, void *param) {
	SDL_Event event;
	memset(&event, 0, sizeof(event));
	event.type = SDL_USEREVENT;
	event.user.code = userevent_TIMER;
	event.user.data1 = param;
	SDL_PushEvent(&event);
	return interval;
}
#endif

void reset_timer(int timer_index) {
#ifndef USE_COMPAT_TIMER
	timer_last_counter[timer_index] = SDL_GetPerformanceCounter();
#endif
}

double get_ticks_per_sec(int timer_index) {
	return (double) fps / wait_time[timer_index];
}

void recalculate_feather_fall_timer(double previous_ticks_per_second, double ticks_per_second) {
	if (is_feather_fall <= MAX(previous_ticks_per_second, ticks_per_second) ||
			previous_ticks_per_second == ticks_per_second) {
		return;
	}
	// there are more ticks per second in base mode vs fight mode so
	// feather fall length needs to be recalculated
	is_feather_fall = is_feather_fall / previous_ticks_per_second * ticks_per_second;
}

void set_timer_length(int timer_index, int length) {
	if (!fixes->fix_quicksave_during_feather) {
		wait_time[timer_index] = length;
		return;
	}
	if (is_feather_fall == 0 ||
			wait_time[timer_index] < custom->base_speed ||
			wait_time[timer_index] > custom->fight_speed) {
		wait_time[timer_index] = length;
		return;
	}
	double previous_ticks_per_second, ticks_per_second;
	previous_ticks_per_second = get_ticks_per_sec(timer_index);
	wait_time[timer_index] = length;
	ticks_per_second = get_ticks_per_sec(timer_index);
	recalculate_feather_fall_timer(previous_ticks_per_second, ticks_per_second);
}

void start_timer(int timer_index, int length) {
#ifdef USE_REPLAY
	if (replaying && skipping_replay) return;
#endif
#ifndef USE_COMPAT_TIMER
	timer_last_counter[timer_index] = SDL_GetPerformanceCounter();
#endif
	wait_time[timer_index] = length;
}

void toggle_fullscreen(void) {
	uint32_t flags = SDL_GetWindowFlags(window_);
	if (flags & SDL_WINDOW_FULLSCREEN_DESKTOP) {
		SDL_SetWindowFullscreen(window_, 0);
		SDL_ShowCursor(SDL_ENABLE);
	}
	else {
		SDL_SetWindowFullscreen(window_, SDL_WINDOW_FULLSCREEN_DESKTOP);
		SDL_ShowCursor(SDL_DISABLE);
	}
}

bool ignore_tab = false;

void process_events() {
	// Process all events in the queue.
	// Previously, this procedure would wait for *one* event and process it, then return.
	// Much like the x86 HLT instruction.
	// (We still want to process all events in the queue. For instance, there might be
	// simultaneous SDL2 KEYDOWN and TEXTINPUT events.)
	SDL_Event event;
	while (SDL_PollEvent(&event) == 1) { // while there are still events to be processed
		switch (event.type) {
			case SDL_KEYDOWN:
			{
				int modifier = event.key.keysym.mod;
				int scancode = event.key.keysym.scancode;

				// Handle these separately, so they won't interrupt things that are usually interrupted by a keypress. (pause, cutscene)
#ifdef USE_FAST_FORWARD
				if (scancode == SDL_SCANCODE_GRAVE) {
					init_timer(BASE_FPS * FAST_FORWARD_RATIO); // fast-forward on
					audio_speed = FAST_FORWARD_RATIO;
					break;
				}
#endif
#ifdef USE_SCREENSHOT
				if (scancode == SDL_SCANCODE_F12) {
					if (modifier & KMOD_SHIFT) {
						save_level_screenshot((modifier & KMOD_CTRL) != 0);
					} else {
						save_screenshot();
					}
				} else
#endif
#ifdef USE_MENU
				if (escape_key_suppressed &&
						(scancode == SDL_SCANCODE_BACKSPACE || (enable_pause_menu && scancode == SDL_SCANCODE_ESCAPE))
				) {
					break; // Prevent repeated keystrokes opening/closing the menu as long as the key is held down.
				} else
#endif
				if ((modifier & KMOD_ALT) &&
				    scancode == SDL_SCANCODE_RETURN)
				{
					// Only if the Enter key was pressed down right now.
					if ((key_states[scancode] & KEYSTATE_HELD) == 0) {
						// Alt+Enter: toggle fullscreen mode
						toggle_fullscreen();
						key_states[scancode] |= KEYSTATE_HELD | KEYSTATE_HELD_NEW;
					}
				} else {
					last_any_key_scancode = scancode; // for showmessage_any_key
					key_states[scancode] |= KEYSTATE_HELD | KEYSTATE_HELD_NEW;
					switch (scancode) {
						// Keys that are ignored by themselves:
						case SDL_SCANCODE_LCTRL:
						case SDL_SCANCODE_LSHIFT:
						case SDL_SCANCODE_LALT:
						case SDL_SCANCODE_LGUI:
						case SDL_SCANCODE_RCTRL:
						case SDL_SCANCODE_RSHIFT:
						case SDL_SCANCODE_RALT:
						case SDL_SCANCODE_RGUI:
						case SDL_SCANCODE_CAPSLOCK:
						case SDL_SCANCODE_SCROLLLOCK:
						case SDL_SCANCODE_NUMLOCKCLEAR:
						case SDL_SCANCODE_APPLICATION:
						case SDL_SCANCODE_PRINTSCREEN:
						case SDL_SCANCODE_VOLUMEUP:
						case SDL_SCANCODE_VOLUMEDOWN:
						// Why are there two mute key codes?
						case SDL_SCANCODE_MUTE:
						case SDL_SCANCODE_AUDIOMUTE:
						case SDL_SCANCODE_PAUSE:
							break;

						default:
							// If Alt is held down from Alt+Tab: ignore it until it's released.
							if (scancode == SDL_SCANCODE_TAB && ignore_tab) break;

							last_key_scancode = scancode;
							if (modifier & KMOD_SHIFT) last_key_scancode |= WITH_SHIFT;
							if (modifier & KMOD_CTRL ) last_key_scancode |= WITH_CTRL ;
							if (modifier & KMOD_ALT  ) last_key_scancode |= WITH_ALT  ;
							
							// Save Alt+Fn keys for title screen level select
							// (before do_paused/process_key clears last_key_scancode)
							if (last_key_scancode & WITH_ALT) {
								extern int title_skip_key;
								title_skip_key = last_key_scancode;
							}
					}

#ifdef USE_AUTO_INPUT_MODE
					switch (scancode) {
						// Keys that are used for keyboard control:
						case SDL_SCANCODE_LSHIFT:
						case SDL_SCANCODE_RSHIFT:
						case SDL_SCANCODE_LEFT:
						case SDL_SCANCODE_RIGHT:
						case SDL_SCANCODE_UP:
						case SDL_SCANCODE_DOWN:
						case SDL_SCANCODE_CLEAR:
						case SDL_SCANCODE_HOME:
						case SDL_SCANCODE_PAGEUP:
						case SDL_SCANCODE_KP_2:
						case SDL_SCANCODE_KP_4:
						case SDL_SCANCODE_KP_5:
						case SDL_SCANCODE_KP_6:
						case SDL_SCANCODE_KP_7:
						case SDL_SCANCODE_KP_8:
						case SDL_SCANCODE_KP_9:
							if (!is_keyboard_mode) {
								is_keyboard_mode = 1;
								is_joyst_mode = 0;
							}
					}
#endif
				}
				break;
			}
			case SDL_KEYUP:
				// If Alt was held down from Alt+Tab but now it's released: stop ignoring Tab.
				if (event.key.keysym.scancode == SDL_SCANCODE_TAB && ignore_tab) ignore_tab = false;

#ifdef USE_FAST_FORWARD
				if (event.key.keysym.scancode == SDL_SCANCODE_GRAVE) {
					init_timer(BASE_FPS); // fast-forward off
					audio_speed = 1;
					break;
				}
#endif

				key_states[event.key.keysym.scancode] &= ~KEYSTATE_HELD;
#ifdef USE_MENU
				// Prevent repeated keystrokes opening/closing the menu as long as the key is held down.
				if (event.key.keysym.scancode == SDL_SCANCODE_BACKSPACE || event.key.keysym.scancode == SDL_SCANCODE_ESCAPE) {
					escape_key_suppressed = false;
				}
#endif
				break;
			case SDL_CONTROLLERAXISMOTION:
				if (event.caxis.axis < 6) {
					joy_axis[event.caxis.axis] = event.caxis.value;
					if (abs(event.caxis.value) > abs(joy_axis_max[event.caxis.axis]))
						joy_axis_max[event.caxis.axis] = event.caxis.value;

#ifdef USE_AUTO_INPUT_MODE
					if (!is_joyst_mode && (event.caxis.value >= joystick_threshold || event.caxis.value <= -joystick_threshold)) {
						is_joyst_mode = 1;
						is_keyboard_mode = 0;
					}
#endif
				}
				break;
			case SDL_CONTROLLERDEVICEADDED:
				SDL_GameControllerOpen(event.cdevice.which);
				if (gamecontrollerdb_file[0] != '\0') {
					SDL_GameControllerAddMappingsFromFile(gamecontrollerdb_file);
				}
				is_joyst_mode = 1;
				using_sdl_joystick_interface = 0;
				break;
			case SDL_CONTROLLERDEVICEREMOVED:
				if (sdl_controller_ == SDL_GameControllerFromInstanceID(event.cdevice.which)) {
					sdl_controller_ = NULL;
					is_joyst_mode = 0;
					is_keyboard_mode = 1;
				}
				SDL_GameControllerClose(SDL_GameControllerFromInstanceID(event.cdevice.which));
				break;
			case SDL_CONTROLLERBUTTONDOWN:
				//Make sure sdl_controller_ always points to the active controller
				sdl_controller_ = SDL_GameControllerFromInstanceID(event.cdevice.which);
#ifdef USE_AUTO_INPUT_MODE
				if (!is_joyst_mode) {
					is_joyst_mode = 1;
					is_keyboard_mode = 0;
				}
#endif
				switch (event.cbutton.button)
				{
					case SDL_CONTROLLER_BUTTON_DPAD_LEFT:  joy_button_states[JOYINPUT_DPAD_LEFT] |= KEYSTATE_HELD | KEYSTATE_HELD_NEW; break; // left
					case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: joy_button_states[JOYINPUT_DPAD_RIGHT] |= KEYSTATE_HELD | KEYSTATE_HELD_NEW; break; // right
					case SDL_CONTROLLER_BUTTON_DPAD_UP:    joy_button_states[JOYINPUT_DPAD_UP] |= KEYSTATE_HELD | KEYSTATE_HELD_NEW; break; // up
					case SDL_CONTROLLER_BUTTON_DPAD_DOWN:  joy_button_states[JOYINPUT_DPAD_DOWN] |= KEYSTATE_HELD | KEYSTATE_HELD_NEW; break; // down

					case SDL_CONTROLLER_BUTTON_A:          joy_button_states[JOYINPUT_A] |= KEYSTATE_HELD | KEYSTATE_HELD_NEW; break; /*** A (down) ***/
					case SDL_CONTROLLER_BUTTON_Y:          joy_button_states[JOYINPUT_Y] |= KEYSTATE_HELD | KEYSTATE_HELD_NEW; break; /*** Y (up) ***/
					case SDL_CONTROLLER_BUTTON_X:          joy_button_states[JOYINPUT_X] |= KEYSTATE_HELD | KEYSTATE_HELD_NEW; break; /*** X (Shift) ***/
					case SDL_CONTROLLER_BUTTON_B:          joy_button_states[JOYINPUT_B] |= KEYSTATE_HELD | KEYSTATE_HELD_NEW; break; /*** B (unused) ***/

					case SDL_CONTROLLER_BUTTON_START:
					case SDL_CONTROLLER_BUTTON_BACK:
						if(event.cbutton.button == SDL_CONTROLLER_BUTTON_START)
							joy_button_states[JOYINPUT_START] |= KEYSTATE_HELD | KEYSTATE_HELD_NEW;
						else if(event.cbutton.button == SDL_CONTROLLER_BUTTON_BACK)
							joy_button_states[JOYINPUT_BACK] |= KEYSTATE_HELD | KEYSTATE_HELD_NEW;
#ifdef USE_MENU
						last_key_scancode = SDL_SCANCODE_BACKSPACE;  /*** bring up pause menu ***/
#else
						last_key_scancode = SDL_SCANCODE_ESCAPE;  /*** back (pause game) ***/
#endif
						break;

					default: break;
				}
				break;
			case SDL_CONTROLLERBUTTONUP:
				switch (event.cbutton.button)
				{
					case SDL_CONTROLLER_BUTTON_DPAD_LEFT:  joy_button_states[JOYINPUT_DPAD_LEFT] &= ~KEYSTATE_HELD; break; // left
					case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: joy_button_states[JOYINPUT_DPAD_RIGHT] &= ~KEYSTATE_HELD; break; // right
					case SDL_CONTROLLER_BUTTON_DPAD_UP:    joy_button_states[JOYINPUT_DPAD_UP] &= ~KEYSTATE_HELD; break; // up
					case SDL_CONTROLLER_BUTTON_DPAD_DOWN:  joy_button_states[JOYINPUT_DPAD_DOWN] &= ~KEYSTATE_HELD; break; // down

					case SDL_CONTROLLER_BUTTON_A:          joy_button_states[JOYINPUT_A] &= ~KEYSTATE_HELD; break; /*** A (down) ***/
					case SDL_CONTROLLER_BUTTON_Y:          joy_button_states[JOYINPUT_Y] &= ~KEYSTATE_HELD; break; /*** Y (up) ***/
					case SDL_CONTROLLER_BUTTON_X:          joy_button_states[JOYINPUT_X] &= ~KEYSTATE_HELD; break; /*** X (Shift) ***/
					case SDL_CONTROLLER_BUTTON_B:          joy_button_states[JOYINPUT_B] &= ~KEYSTATE_HELD; break; /*** B (unused) ***/

					case SDL_CONTROLLER_BUTTON_START:      joy_button_states[JOYINPUT_START] &= ~KEYSTATE_HELD; break;
					case SDL_CONTROLLER_BUTTON_BACK:       joy_button_states[JOYINPUT_BACK] &= ~KEYSTATE_HELD; break;

					default: break;
				}
				break;
			case SDL_JOYBUTTONDOWN:
			case SDL_JOYBUTTONUP:
			case SDL_JOYAXISMOTION:
				// Only handle the event if the joystick is incompatible with the SDL_GameController interface.
				// (Otherwise it will interfere with the normal action of the SDL_GameController API.)
				if (!using_sdl_joystick_interface) {
					break;
				}
				if (event.type == SDL_JOYAXISMOTION) {
					int axis = -1;
					if (event.jaxis.axis == SDL_JOYSTICK_X_AXIS) {
						axis = SDL_CONTROLLER_AXIS_LEFTX;
					}
					else if (event.jaxis.axis == SDL_JOYSTICK_Y_AXIS) {
						axis = SDL_CONTROLLER_AXIS_LEFTY;
					}
					if (axis == -1)
						break;
					joy_axis[axis] = event.jaxis.value;
					if (abs(event.jaxis.value) > abs(joy_axis_max[axis]))
						joy_axis_max[axis] = event.jaxis.value;

					// Disregard SDL_JOYAXISMOTION events within joystick 'dead zone'
					int joy_x = joy_axis[SDL_CONTROLLER_AXIS_LEFTX];
					int joy_y = joy_axis[SDL_CONTROLLER_AXIS_LEFTY];
					if ((dword)(joy_x*joy_x) + (dword)(joy_y*joy_y) < (dword)(joystick_threshold*joystick_threshold)) {
						break;
					}
				}
#ifdef USE_AUTO_INPUT_MODE
				if (!is_joyst_mode) {
					is_joyst_mode = 1;
					is_keyboard_mode = 0;
				}
#endif
				if (event.type == SDL_JOYBUTTONDOWN) {
					if      (event.jbutton.button == SDL_JOYSTICK_BUTTON_Y)   joy_button_states[JOYINPUT_Y] |= KEYSTATE_HELD | KEYSTATE_HELD_NEW; // Y (up)
					else if (event.jbutton.button == SDL_JOYSTICK_BUTTON_X)   joy_button_states[JOYINPUT_X] |= KEYSTATE_HELD | KEYSTATE_HELD_NEW;    // X (Shift)
				}
				else if (event.type == SDL_JOYBUTTONUP) {
					if      (event.jbutton.button == SDL_JOYSTICK_BUTTON_Y)   joy_button_states[JOYINPUT_Y] &= ~KEYSTATE_HELD;  // Y (up)
					else if (event.jbutton.button == SDL_JOYSTICK_BUTTON_X)   joy_button_states[JOYINPUT_X] &= ~KEYSTATE_HELD;    // X (Shift)
				}
				break;

			case SDL_TEXTINPUT:
				last_text_input = event.text.text[0]; // UTF-8 formatted char text input

				// Make the +/- keys work on the main keyboard, on any keyboard layout.
				// We check SDL_TEXTINPUT instead of SDL_KEYDOWN.
				// If '+' is on Shift+something then we can't detect it in SDL_KEYDOWN,
				// because event.key.keysym.sym only tells us what character would the key type without shift.
				switch (last_text_input) {
					case '-': last_key_scancode = SDL_SCANCODE_KP_MINUS; break;
					case '+': last_key_scancode = SDL_SCANCODE_KP_PLUS;  break;
				}

				break;
			case SDL_WINDOWEVENT:
				// In case the user switches away while holding a key: do as if all keys were released.
				// (DOSBox does the same.)

/* // not implemented in SDL2 for now
 *
			if ((event.active.state & SDL_APPINPUTFOCUS) && event.active.gain == 0) {
				memset(key_states, 0, sizeof(key_states));
			}
			// Note: event.active.state can contain multiple flags or'ed.
			// If the game is in full screen, and I switch away (Alt+Tab) and back, most of the screen will be black, until it is redrawn.
			if ((event.active.state & SDL_APPACTIVE) && event.active.gain == 1) {
				update_screen();
			}
*/
				switch (event.window.event) {
#ifdef __amigaos4__
					case SDL_WINDOWEVENT_MINIMIZED: /* pause game */
						if (!is_menu_shown) {
							last_key_scancode = SDL_SCANCODE_BACKSPACE;
						}
						break;
					case SDL_WINDOWEVENT_RESTORED: /* show "game paused/menu" */
						update_screen();
						break;
#endif
					case SDL_WINDOWEVENT_SIZE_CHANGED:
						window_resized();
						// fallthrough!
					//case SDL_WINDOWEVENT_MOVED:
					//case SDL_WINDOWEVENT_RESTORED:
					case SDL_WINDOWEVENT_EXPOSED:
						update_screen();
						break;

					case SDL_WINDOWEVENT_FOCUS_GAINED:
					// Fix for this bug: When playing back a recording, Alt+Tabbing back to SDLPoP stops the replay if Alt is released before Tab.
					{ // If Alt is held down from Alt+Tab: ignore it until it's released.
						const Uint8 *state = SDL_GetKeyboardState(NULL);
						if (state[SDL_SCANCODE_TAB]) ignore_tab = true;
					}
					break;
				}
				break;
			case SDL_USEREVENT:
				if (event.user.code == userevent_TIMER /*&& event.user.data1 == (void*)timer_index*/) {
#ifdef USE_COMPAT_TIMER
					for (int index = 0; index < NUM_TIMERS; ++index) {
						if (wait_time[index] > 0) --wait_time[index];
					}
#endif
				} else if (event.user.code == userevent_SOUND) {
					//sound_timer = 0;
					//stop_sounds();
				}
				break;
#ifdef USE_MENU
			case SDL_MOUSEBUTTONDOWN:
				switch(event.button.button) {
					case SDL_BUTTON_LEFT:
						if (!is_menu_shown) {
							last_key_scancode = SDL_SCANCODE_BACKSPACE;
						} else {
							mouse_clicked = true;
						}
						break;
					case SDL_BUTTON_RIGHT:
					case SDL_BUTTON_X1: // 'Back' button (on mice that have these extra buttons).
						mouse_button_clicked_right = true;
						break;
					default: break;
				}

				break;
			case SDL_MOUSEWHEEL:
				if (is_menu_shown) {
					menu_control_scroll_y = -event.wheel.y;
				}
				break;
#endif
			case SDL_QUIT:
#ifdef USE_MENU
				if (is_menu_shown) {
					menu_was_closed();
				}
#endif
				quit(0);
				break;
		}
	}
}

void idle() {
	process_events();
	update_screen();
#ifdef POP_RP2350
	SDL_AudioPump();  // Pump audio buffers (polled I2S on RP2350)
#endif
}

void do_simple_wait(int timer_index) {
#ifdef USE_REPLAY
	if ((replaying && skipping_replay) || is_validate_mode) return;
#endif
	update_screen();
	while (! has_timer_stopped(timer_index)) {
		SDL_Delay(1);
		process_events();
#ifdef POP_RP2350
		SDL_AudioPump();
#endif
	}
}

word word_1D63A = 1;
int do_wait(int timer_index) {
#ifdef USE_REPLAY
	if ((replaying && skipping_replay) || is_validate_mode) return 0;
#endif
	update_screen();
	while (! has_timer_stopped(timer_index)) {
		SDL_Delay(1);
		process_events();
#ifdef POP_RP2350
		SDL_AudioPump();
#endif
		int key = do_paused();
		if (key != 0 && (word_1D63A != 0 || key == 0x1B)) return 1;
	}
	return 0;
}

#ifdef USE_COMPAT_TIMER
SDL_TimerID global_timer = 0;
#endif
// seg009:78E9
void init_timer(int frequency) {
	perf_frequency = SDL_GetPerformanceFrequency();
#ifndef USE_COMPAT_TIMER
	fps = frequency;
	milliseconds_per_tick = 1000.0f / (float)fps;
	perf_counters_per_tick = perf_frequency / fps;
	milliseconds_per_counter = 1000.0f / perf_frequency;
#else
	global_timer = SDL_AddTimer(1000/frequency, timer_callback, NULL);
	if (global_timer != 0) {
		if (!SDL_RemoveTimer(global_timer)) {
			sdlperror("init_timer: SDL_RemoveTimer");
		}
	}
	if (global_timer == 0) {
		sdlperror("init_timer: SDL_AddTimer");
		quit(1);
	}
#endif
}

// seg009:35F6
void set_clip_rect(const rect_type* rect) {
	SDL_Rect clip_rect;
	rect_to_sdlrect(rect, &clip_rect);
#ifdef POP_RP2350
	static int set_clip_debug = 0;
	if (set_clip_debug < 10 && clip_rect.h < 200) {
		DBG_PRINTF("[SET_CLIP] target=%p clip=(%d,%d,%d,%d)\n", 
			(void*)current_target_surface, clip_rect.x, clip_rect.y, clip_rect.w, clip_rect.h);
		set_clip_debug++;
	}
#endif
	SDL_SetClipRect(current_target_surface, &clip_rect);
}

// seg009:365C
void reset_clip_rect() {
	SDL_SetClipRect(current_target_surface, NULL);
}

// seg009:1983
void set_bg_attr(int vga_pal_index,int hc_pal_index) {
	// stub
#ifdef USE_FLASH
	//palette[vga_pal_index] = vga_palette[hc_pal_index];
	if (!enable_flash) return;
	if (vga_pal_index == 0) {
		/*
		if (SDL_SetAlpha(offscreen_surface, SDL_SRCALPHA, 0) != 0) {
			sdlperror("set_bg_attr: SDL_SetAlpha");
			quit(1);
		}
		*/
		// Make the black pixels transparent.
		if (SDL_SetColorKey(offscreen_surface, SDL_TRUE, 0) != 0) {	// SDL_SRCCOLORKEY old
			sdlperror("set_bg_attr: SDL_SetColorKey");
			quit(1);
		}
		SDL_Rect rect = {0,0,0,0};
		rect.w = offscreen_surface->w;
		rect.h = offscreen_surface->h;
		rgb_type palette_color = palette[hc_pal_index];
		uint32_t rgb_color = SDL_MapRGB(onscreen_surface_->format, palette_color.r<<2, palette_color.g<<2, palette_color.b<<2) /*& 0xFFFFFF*/;
		//SDL_UpdateRect(onscreen_surface_, 0, 0, 0, 0);
		// First clear the screen with the color of the flash.
		if (safe_SDL_FillRect(onscreen_surface_, &rect, rgb_color) != 0) {
			sdlperror("set_bg_attr: SDL_FillRect");
			quit(1);
		}
		//SDL_UpdateRect(onscreen_surface_, 0, 0, 0, 0);
		if (upside_down) {
			flip_screen(offscreen_surface);
		}
		// Then draw the offscreen image onto it.
		if (SDL_BlitSurface(offscreen_surface, &rect, onscreen_surface_, &rect) != 0) {
			sdlperror("set_bg_attr: SDL_BlitSurface");
			quit(1);
		}
#ifdef USE_LIGHTING
		if (hc_pal_index == 0) update_lighting(&rect_top);
#endif
		if (upside_down) {
			flip_screen(offscreen_surface);
		}
		// And show it!
//		update_screen();
		// Give some time to show the flash.
		//SDL_Flip(onscreen_surface_);
//		if (hc_pal_index != 0) SDL_Delay(2*(1000/60));
		//SDL_Flip(onscreen_surface_);
		/*
		if (SDL_SetAlpha(offscreen_surface, 0, 0) != 0) {
			sdlperror("set_bg_attr: SDL_SetAlpha");
			quit(1);
		}
		*/
		if (SDL_SetColorKey(offscreen_surface, 0, 0) != 0) {
			sdlperror("set_bg_attr: SDL_SetColorKey");
			quit(1);
		}
	}
#endif // USE_FLASH
}

// seg009:07EB
rect_type* offset4_rect_add(rect_type* dest,const rect_type* source,int d_left,int d_top,int d_right,int d_bottom) {
	*dest = *source;
	dest->left += d_left;
	dest->top += d_top;
	dest->right += d_right;
	dest->bottom += d_bottom;
	return dest;
}

// seg009:3AA5
rect_type* offset2_rect(rect_type* dest,const rect_type *source,int delta_x,int delta_y) {
	dest->top    = source->top    + delta_y;
	dest->left   = source->left   + delta_x;
	dest->bottom = source->bottom + delta_y;
	dest->right  = source->right  + delta_x;
	return dest;
}

#ifdef USE_FADE
// seg009:19EF
void fade_in_2(surface_type* source_surface,int which_rows) {
	palette_fade_type* palette_buffer;
	if (graphics_mode == gmMcgaVga) {
		palette_buffer = make_pal_buffer_fadein(source_surface, which_rows, 2);
		while (fade_in_frame(palette_buffer) == 0) {
			process_events(); // modified
			do_paused();
#ifdef POP_RP2350
			SDL_AudioPump();  // Keep audio streaming during fade
#endif
		}
		pal_restore_free_fadein(palette_buffer);
	} else {
		// ...
	}
}

// seg009:1A51
palette_fade_type* make_pal_buffer_fadein(surface_type* source_surface,int which_rows,int wait_time) {
	palette_fade_type* palette_buffer;
	palette_buffer = (palette_fade_type*) malloc(sizeof(palette_fade_type));
	palette_buffer->which_rows = which_rows;
	palette_buffer->wait_time = wait_time;
	palette_buffer->fade_pos = 0x40;
	palette_buffer->proc_restore_free = &pal_restore_free_fadein;
	palette_buffer->proc_fade_frame = &fade_in_frame;
	read_palette_256(palette_buffer->original_pal);
	memcpy(palette_buffer->faded_pal, palette_buffer->original_pal, sizeof(palette_buffer->faded_pal));
#ifdef POP_RP2350
	// On RP2350, fade ALL colors uniformly using HDMI output-level fade.
	// Don't use the VGA palette row trick which causes some colors to appear before others.
	graphics_set_fade_level(0x40, 0);
#else
	for (word curr_row = 0, curr_row_mask = 1; curr_row < 0x10; ++curr_row, curr_row_mask<<=1) {
		if (which_rows & curr_row_mask) {
			memset(palette_buffer->faded_pal + (curr_row<<4), 0, sizeof(rgb_type[0x10]));
			set_pal_arr(curr_row<<4, 0x10, NULL);
		}
	}
#endif
	//method_1_blit_rect(onscreen_surface_, source_surface, &screen_rect, &screen_rect, 0);
	// for RGB
	//method_5_rect(&screen_rect, 0, color_0_black);
	return palette_buffer;
}

// seg009:1B64
void pal_restore_free_fadein(palette_fade_type* palette_buffer) {
#ifdef POP_RP2350
	// Reset HDMI fade level to full brightness
	graphics_set_fade_level(0, 0);
#endif
	set_pal_256(palette_buffer->original_pal);
	free(palette_buffer);
	// for RGB
	method_1_blit_rect(onscreen_surface_, offscreen_surface, &screen_rect, &screen_rect, 0);
}

// seg009:1B88
int fade_in_frame(palette_fade_type* palette_buffer) {
//	void* var_12;
	/**/start_timer(timer_1, palette_buffer->wait_time); // too slow?

	//printf("start ticks = %u\n",SDL_GetTicks());

#ifdef POP_RP2350
	// On RP2350, fade works by applying a brightness multiplier at the HDMI output level.
	// On first frame, offscreen already has background from load_intro and
	// draw_proom_drects has drawn characters on top. Blit everything to HDMI.
	graphics_set_fade_level(palette_buffer->fade_pos, 0);
	method_1_blit_rect(onscreen_surface_, offscreen_surface, &screen_rect, &screen_rect, 0);
	SDL_UpdateTexture(NULL, NULL, onscreen_surface_->pixels, onscreen_surface_->pitch);
	--palette_buffer->fade_pos;
#else
	for (word start=0,current_row_mask=1; start<0x100; start+=0x10, current_row_mask<<=1) {
		if (palette_buffer->which_rows & current_row_mask) {
			//var_12 = palette_buffer->
			rgb_type* original_pal_ptr = palette_buffer->original_pal + start;
			rgb_type* faded_pal_ptr = palette_buffer->faded_pal + start;
			for (word column = 0; column<0x10; ++column) {
				if (original_pal_ptr[column].r > palette_buffer->fade_pos) {
					++faded_pal_ptr[column].r;
				}
				if (original_pal_ptr[column].g > palette_buffer->fade_pos) {
					++faded_pal_ptr[column].g;
				}
				if (original_pal_ptr[column].b > palette_buffer->fade_pos) {
					++faded_pal_ptr[column].b;
				}
			}
		}
	}
	for (word start = 0, current_row_mask = 1; start<0x100; start+=0x10, current_row_mask<<=1) {
		if (palette_buffer->which_rows & current_row_mask) {
			set_pal_arr(start, 0x10, palette_buffer->faded_pal + start);
		}
	}

	int h = offscreen_surface->h;
	if (SDL_LockSurface(onscreen_surface_) != 0) {
		sdlperror("fade_in_frame: SDL_LockSurface");
		quit(1);
	}
	if (SDL_LockSurface(offscreen_surface) != 0) {
		sdlperror("fade_in_frame: SDL_LockSurface");
		quit(1);
	}
	int on_stride = onscreen_surface_->pitch;
	int off_stride = offscreen_surface->pitch;
	int fade_pos = palette_buffer->fade_pos;
	for (int y = 0; y < h; ++y) {
		byte* on_pixel_ptr = (byte*)onscreen_surface_->pixels + on_stride * y;
		byte* off_pixel_ptr = (byte*)offscreen_surface->pixels + off_stride * y;
		for (int x = 0; x < on_stride; ++x) {
			//if (*off_pixel_ptr > palette_buffer->fade_pos) *pixel_ptr += 4;
			int v = *off_pixel_ptr - fade_pos*4;
			if (v<0) v=0;
			*on_pixel_ptr = v;
			++on_pixel_ptr; ++off_pixel_ptr;
		}
	}
	SDL_UnlockSurface(onscreen_surface_);
	SDL_UnlockSurface(offscreen_surface);
#endif

	//SDL_UpdateRect(onscreen_surface_, 0, 0, 0, 0); // debug

	/**/do_simple_wait(1); // can interrupt fading of cutscene
	//do_wait(timer_1); // can interrupt fading of main title
	//printf("end ticks = %u\n",SDL_GetTicks());
	return palette_buffer->fade_pos == 0;
}

// seg009:1CC9
void fade_out_2(int rows) {
	palette_fade_type* palette_buffer;
	if (graphics_mode == gmMcgaVga) {
		palette_buffer = make_pal_buffer_fadeout(rows, 2);
		while (fade_out_frame(palette_buffer) == 0) {
			process_events(); // modified
			do_paused();
#ifdef POP_RP2350
			SDL_AudioPump();  // Keep audio streaming during fade
#endif
		}
		pal_restore_free_fadeout(palette_buffer);
	} else {
		// ...
	}
}

// seg009:1D28
palette_fade_type* make_pal_buffer_fadeout(int which_rows,int wait_time) {
	palette_fade_type* palette_buffer;
	palette_buffer = (palette_fade_type*) malloc(sizeof(palette_fade_type));
	palette_buffer->which_rows = which_rows;
	palette_buffer->wait_time = wait_time;
	palette_buffer->fade_pos = 00; // modified
	palette_buffer->proc_restore_free = &pal_restore_free_fadeout;
	palette_buffer->proc_fade_frame = &fade_out_frame;
	read_palette_256(palette_buffer->original_pal);
	memcpy(palette_buffer->faded_pal, palette_buffer->original_pal, sizeof(palette_buffer->faded_pal));
	// for RGB
	method_1_blit_rect(onscreen_surface_, offscreen_surface, &screen_rect, &screen_rect, 0);
	return palette_buffer;
}

// seg009:1DAF
void pal_restore_free_fadeout(palette_fade_type* palette_buffer) {
	surface_type* surface;
	surface = current_target_surface;
	current_target_surface = onscreen_surface_;
	draw_rect(&screen_rect, color_0_black);
	current_target_surface = surface;
#ifdef POP_RP2350
	// Keep the HDMI fade at full black (64) - don't reset it!
	// The screen content is black via draw_rect, but we leave the fade active
	// so that any residual colors stay black until the next scene starts.
	// The next fade_in will reset it properly.
#else
	set_pal_256(palette_buffer->original_pal);
#endif
	free(palette_buffer);
	// for RGB
	method_5_rect(&screen_rect, 0, color_0_black);
}

// seg009:1DF7
int fade_out_frame(palette_fade_type* palette_buffer) {
	word finished_fading = 1;
	++palette_buffer->fade_pos; // modified
	/**/start_timer(timer_1, palette_buffer->wait_time); // too slow?

#ifdef POP_RP2350
	// Heartbeat log every 16 frames to track fade progress
	if ((palette_buffer->fade_pos % 16) == 0) {
		printf("[FADE @%ums] fade_out_frame pos=%d/64\n", time_us_32() / 1000, palette_buffer->fade_pos);
	}
	
	// On RP2350, fade works by applying a brightness multiplier at the HDMI output level.
	// This avoids corrupting the game's palette, which would break SDL_BlitSurface color matching.
	// fade_pos: 64 = full black, 0 = full brightness
	// Use which_rows=0 to fade ALL palette rows uniformly
	graphics_set_fade_level(palette_buffer->fade_pos, 0);
	method_1_blit_rect(onscreen_surface_, offscreen_surface, &screen_rect, &screen_rect, 0);
	SDL_UpdateTexture(NULL, NULL, onscreen_surface_->pixels, onscreen_surface_->pitch);
	finished_fading = (palette_buffer->fade_pos >= 64);
#else
	for (word start=0,current_row_mask=1; start<0x100; start+=0x10, current_row_mask<<=1) {
		if (palette_buffer->which_rows & current_row_mask) {
			//var_12 = palette_buffer->
			//original_pal_ptr = palette_buffer->original_pal + start;
			rgb_type* faded_pal_ptr = palette_buffer->faded_pal + start;
			for (word column = 0; column<0x10; ++column) {
				byte* curr_color_ptr = &faded_pal_ptr[column].r;
				if (*curr_color_ptr != 0) {
					--*curr_color_ptr;
					finished_fading = 0;
				}
				curr_color_ptr = &faded_pal_ptr[column].g;
				if (*curr_color_ptr != 0) {
					--*curr_color_ptr;
					finished_fading = 0;
				}
				curr_color_ptr = &faded_pal_ptr[column].b;
				if (*curr_color_ptr != 0) {
					--*curr_color_ptr;
					finished_fading = 0;
				}
			}
		}
	}
	for (word start = 0, current_row_mask = 1; start<0x100; start+=0x10, current_row_mask<<=1) {
		if (palette_buffer->which_rows & current_row_mask) {
			set_pal_arr(start, 0x10, palette_buffer->faded_pal + start);
		}
	}

	int h = offscreen_surface->h;
	if (SDL_LockSurface(onscreen_surface_) != 0) {
		sdlperror("fade_out_frame: SDL_LockSurface");
		quit(1);
	}
	if (SDL_LockSurface(offscreen_surface) != 0) {
		sdlperror("fade_out_frame: SDL_LockSurface");
		quit(1);
	}
	int on_stride = onscreen_surface_->pitch;
	int off_stride = offscreen_surface->pitch;
	int fade_pos = palette_buffer->fade_pos;
	for (int y = 0; y < h; ++y) {
		byte* on_pixel_ptr = (byte*)onscreen_surface_->pixels + on_stride * y;
		byte* off_pixel_ptr = (byte*)offscreen_surface->pixels + off_stride * y;
		for (int x = 0; x < on_stride; ++x) {
			//if (*pixel_ptr >= 4) *pixel_ptr -= 4;
			int v = *off_pixel_ptr - fade_pos*4;
			if (v<0) v=0;
			*on_pixel_ptr = v;
			++on_pixel_ptr; ++off_pixel_ptr;
		}
	}
	SDL_UnlockSurface(onscreen_surface_);
	SDL_UnlockSurface(offscreen_surface);
#endif

	do_simple_wait(timer_1); // can interrupt fading of cutscene
	return finished_fading;
}

// seg009:1F28
void read_palette_256(rgb_type* target) {
	for (int i = 0; i < 256; ++i) {
		target[i] = palette[i];
	}
}

// seg009:1F5E
void set_pal_256(rgb_type* source) {
	for (int i = 0; i < 256; ++i) {
		palette[i] = source[i];
	}
	#ifdef POP_RP2350
	rp2350_apply_palette_256();
	#endif
}
#endif // USE_FADE

void set_chtab_palette(chtab_type* chtab, byte* colors, int n_colors) {
	if (chtab != NULL) {
		SDL_Color* scolors = (SDL_Color*) malloc(n_colors*sizeof(SDL_Color));
		//printf("scolors\n",i);
		for (int i = 0; i < n_colors; ++i) {
			//printf("i=%d\n",i);
			scolors[i].r = *colors << 2; ++colors;
			scolors[i].g = *colors << 2; ++colors;
			scolors[i].b = *colors << 2; ++colors;
			scolors[i].a = SDL_ALPHA_OPAQUE; // the SDL2 SDL_Color struct has an alpha component
		}

		// Color 0 of the palette data is not used, it is replaced by the background color.
		// Needed for correct alternate colors (v1.3) of level 8.
		scolors[0].r = scolors[0].g = scolors[0].b = 0;
		scolors[0].a = SDL_ALPHA_TRANSPARENT;

		//printf("setcolors\n",i);
		for (int i = 0; i < chtab->n_images; ++i) {
			//printf("i=%d\n",i);
			image_type* current_image = chtab->images[i];
			if (current_image != NULL) {

				int n_colors_to_be_set = n_colors;
				SDL_Palette* current_palette = current_image->format->palette;

				// Fix crashing with the guard graphics of Christmas of Persia.
				if (current_palette != NULL) {
					// one of the guard images (i=25) is only a single transparent pixel
					// this caused SDL_SetPaletteColors to fail, I think because that palette contains only 2 colors
					if (current_palette->ncolors < n_colors_to_be_set) {
						n_colors_to_be_set = current_palette->ncolors;
					}
					if (SDL_SetPaletteColors(current_palette, scolors, 0, n_colors_to_be_set) != 0) {
						sdlperror("set_chtab_palette: SDL_SetPaletteColors");
						quit(1);
					}
				}
			}
		}
		free(scolors);
	}
}

int has_timer_stopped(int timer_index) {
#ifdef USE_COMPAT_TIMER
	return wait_time[timer_index] == 0;
#else
#ifdef USE_REPLAY
	if ((replaying && skipping_replay) || is_validate_mode) return true;
#endif
	//PSP: overshoot always too big, 333mhz mandatory to read input!
	Uint64 current_counter = SDL_GetPerformanceCounter();
	int ticks_elapsed = (int)((current_counter / perf_counters_per_tick) - (timer_last_counter[timer_index] / perf_counters_per_tick));
	int overshoot = ticks_elapsed - wait_time[timer_index];
	if (overshoot >= 0) {
//		float milliseconds_elapsed = (current_counter - timer_last_counter[timer_index]) * milliseconds_per_counter;
//		printf("timer %d:   frametime (ms) = %5.1f    fps = %.1f    timer ticks elapsed = %d\n", timer_index, milliseconds_elapsed, 1000.0f / milliseconds_elapsed, ticks_elapsed);
		if (overshoot > 0 && overshoot <= 3) {
			current_counter -= overshoot * perf_counters_per_tick;
		}
		timer_last_counter[timer_index] = current_counter;
		return true;
	} else {
		return false;
	}
#endif
}
