#pragma once
/*
 * Forwarder: PX4 open_drone_id_translations.hpp includes <mavlink/common/mavlink.h> (mavlink/ prefix),
 * but our c_library_v2 layout provides <common/mavlink.h> (-isystem mavlink/c_library_v2). This bridges
 * the two include styles.
 * ---
 * Yonlendirici: PX4 open_drone_id_translations.hpp <mavlink/common/mavlink.h> include eder (mavlink/ oneki),
 * ama bizim c_library_v2 yerlesimimiz <common/mavlink.h> saglar (-isystem mavlink/c_library_v2). Bu, iki
 * include stilini koprüler.
 */
#include <common/mavlink.h>
