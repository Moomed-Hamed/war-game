///@file error message utility functions
#ifndef IDUTILS_HPP_
#define IDUTILS_HPP_
#include <cstring>
/// name of file being compiled, without leading path components
#define __INVDYN_FILE_WO_DIR__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#include "Bullet3Common/b3Logging.h"
#define bt_id_error_message(...) b3Error(__VA_ARGS__)
#define bt_id_warning_message(...) b3Warning(__VA_ARGS__)
#define id_printf(...) b3Printf(__VA_ARGS__)
#endif
