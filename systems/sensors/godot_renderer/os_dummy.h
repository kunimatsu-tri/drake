#ifndef OS_DUMMY_H
#define OS_DUMMY_H

#include <drivers/unix/os_unix.h>
#include "io/ip.h"
#include "os/rw_lock.h"

class OS_Dummy : public OS {
  int window_width = 1024, window_height = 600;

public:
  OS_Dummy(int window_width, int window_height)
    : OS(), window_width(window_width), window_height(window_height) {}

  virtual int get_video_driver_count() const { return 0; }
  virtual const char *get_video_driver_name(int p_driver) const { return ""; }
  virtual VideoMode get_default_video_mode() const {
    return VideoMode(window_width, window_height, false);
  }

  virtual Error initialize(const VideoMode &p_desired, int p_video_driver, int p_audio_driver) { return OK; }
  virtual void set_main_loop(MainLoop *p_main_loop) {}
  virtual void finalize() {}

public:
  virtual String get_name() { return String("drake_dummy"); }
  virtual void delete_main_loop() {}
  virtual bool _check_internal_feature_support(const String & p_feature) {
    // These are the features explicitly listed by the osx and x11
    // implementations of OS::_check_internal_feature_support.
    return p_feature == "pc" || p_feature == "s3tc";
  }

  virtual Point2 get_mouse_position() const { return Point2(); }
  virtual int get_mouse_button_state() const { return 0; }
  virtual void set_window_title(const String &p_title) {}
  virtual void set_video_mode(const VideoMode &p_video_mode, int) {}
  virtual VideoMode get_video_mode(int p_screen = 0) const { return get_default_video_mode(); }
  virtual void get_fullscreen_mode_list(List<VideoMode> *p_list, int) const {}
  virtual Size2 get_window_size() const { return Size2(window_width, window_height); }
  virtual MainLoop *get_main_loop() const { return NULL; }
  virtual bool can_draw() const { return true; }
  virtual void set_custom_mouse_cursor(const RES &p_cursor, CursorShape p_shape, const Vector2 &p_hotspot) {}
  //====== from OS
  virtual void set_cursor_shape(CursorShape p_shape) {}
  virtual int get_audio_driver_count() const { return 0; }
  virtual const char *get_audio_driver_name(int p_driver) const { return ""; }
  virtual void initialize_core() {}
  virtual void finalize_core() {}
  virtual void alert(const String &p_alert, const String &p_t) {}
  virtual String get_stdin_string(bool p_block = true) { return String(); }
  virtual Error execute(const String &p_path, const List<String> &p_arguments, bool p_blocking, ProcessID *r_child_id = NULL, String *r_pipe = NULL, int *r_exitcode = NULL, bool read_stderr = false) { return OK; }
  virtual Error kill(const ProcessID &p_pid) { return OK; }
  virtual bool has_environment(const String &p_var) const { return false; }
  virtual String get_environment(const String &p_var) const { return String(); }
  virtual Date get_date(bool local = false) const { return Date(); }
  virtual Time get_time(bool local = false) const { return Time(); }
  virtual TimeZoneInfo get_time_zone_info() const { return TimeZoneInfo(); }
  virtual void delay_usec(uint32_t p_usec) const {}
  virtual uint64_t get_ticks_usec() const { return 0; }
  OS_Dummy() {}
};

class RWLockDummy : public RWLock {
  static RWLock *create(); ///< Create a rwlock

public:
  static void make_default();
  virtual void read_lock() {}
  virtual void read_unlock(){}; ///< Unlock the rwlock, let other threads continue
  virtual Error read_try_lock() { return OK; }; ///< Attempt to lock the rwlock, OK on success, ERROR means it can't lock.

  virtual void write_lock(){}; ///< Lock the rwlock, block if locked by someone else
  virtual void write_unlock(){}; ///< Unlock the rwlock, let other thwrites continue
  virtual Error write_try_lock() { return OK; }

  virtual ~RWLockDummy() {}
};

class IPDummy : public IP {
  //GDCLASS(IPDummy, IP);
  static IP *create();
  virtual IP_Address _resolve_hostname(const String &p_hostname, IP::Type p_type) {
    return IP_Address();
  }

public:
  virtual void get_local_addresses(List<IP_Address> *r_addresses) const {}
  static void make_default();
  IPDummy() {}
  virtual ~IPDummy() {}
};

#endif
