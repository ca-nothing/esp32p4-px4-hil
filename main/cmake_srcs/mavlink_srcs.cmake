# GENERATED explicit (no-glob kurali; find ile, build-config wildcard'siz). Yeni .cpp -> bu dosyayi guncelle.
set(FC_MAVLINK_SRCS
    "PX4/src/modules/mavlink/MavlinkStatustextHandler.cpp"
    "PX4/src/modules/mavlink/mavlink_command_sender.cpp"
    "PX4/src/modules/mavlink/mavlink_events.cpp"
    "PX4/src/modules/mavlink/mavlink_ftp.cpp"
    "PX4/src/modules/mavlink/mavlink_log_handler.cpp"
    "PX4/src/modules/mavlink/mavlink_main.cpp"
    "PX4/src/modules/mavlink/mavlink_messages.cpp"
    "PX4/src/modules/mavlink/mavlink_mission.cpp"
    "PX4/src/modules/mavlink/mavlink_parameters.cpp"
    "PX4/src/modules/mavlink/mavlink_rate_limiter.cpp"
    "PX4/src/modules/mavlink/mavlink_receiver.cpp"
    # SIM-KOPRU (task #35-B): gercek PX4 HIL/Gazebo modulu. HIL_SENSOR->sensor uORB, actuator_outputs->
    # HIL_ACTUATOR_CONTROLS. Ayni platform-shim'leri (netif/termios/PX4Accel-Gyro-Mag) kullanir -> mavlink set'inde.
    "PX4/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp"
    # mavlink_shell.cpp HARIC: posix px4_daemon/pxh.h -> platforms/posix/apps.h (NuttX/POSIX interaktif
    # konsol) ESP-IDF'te yok + GCS-link icin gereksiz debug-ozelligi. MavlinkShell ref'leri link'te cikarsa stub.
    "PX4/src/modules/mavlink/mavlink_sign_control.cpp"
    "PX4/src/modules/mavlink/mavlink_simple_analyzer.cpp"
    "PX4/src/modules/mavlink/mavlink_stream.cpp"
    "PX4/src/modules/mavlink/mavlink_timesync.cpp"
    "PX4/src/modules/mavlink/mavlink_ulog.cpp"
    "PX4/src/modules/mavlink/open_drone_id_translations.cpp"
    "PX4/src/modules/mavlink/tune_publisher.cpp"
)
