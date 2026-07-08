idf_component_register(
    # fc_px4_params.c / fc_param_protocol.c excluded from build; real PX4 parameters.cpp
    # backend used instead (FC_PARAM_SRCS, end of file). fc_px4_params.c stays on disk
    # as the px4_parameters.hpp generation source (tools/gen_px4_params.py); not compiled.
    # ---
    # fc_px4_params.c / fc_param_protocol.c derlemeden CIKARILDI; yerine gercek PX4
    # parameters.cpp arka ucu kullaniliyor (FC_PARAM_SRCS, dosya sonu). fc_px4_params.c diskte
    # px4_parameters.hpp uretim kaynagi olarak duruyor (tools/gen_px4_params.py); derlenmez.
    SRCS "fc_core.c" "logger/fc_logger.c" "main.c" "fc_internal_new.cpp"
         "PX4/src/lib/geo/geo.cpp"
         "PX4/src/lib/world_magnetic_model/geo_mag_declination.cpp"
         "PX4/src/lib/atmosphere/atmosphere.cpp"
         "PX4/src/lib/lat_lon_alt/lat_lon_alt.cpp"
         # EKF comes from modules/ekf2/EKF (verbatim), not lib/ecl (ODR conflict fix).
         # PX4 pure-compute control classes (no uORB), verbatim like EKF.
         # ---
         # EKF, lib/ecl'den DEGIL modules/ekf2/EKF'ten gelir (verbatim; ODR-cakisma fix'i).
         # PX4 saf-hesap kontrol siniflari (uORB yok), EKF gibi verbatim.
         "PX4/src/lib/mc_pos_control/PositionControl/PositionControl.cpp"
         "PX4/src/lib/mc_pos_control/PositionControl/ControlMath.cpp"
         "PX4/src/lib/mc_att_control/AttitudeControl/AttitudeControl.cpp"
         "PX4/src/lib/rate_control/rate_control.cpp"
         "PX4/src/lib/control_allocation/control_allocation/ControlAllocation.cpp"
         "PX4/src/lib/control_allocation/control_allocation/ControlAllocationPseudoInverse.cpp"
         "PX4/src/lib/control_allocation/control_allocation/ControlAllocationSequentialDesaturation.cpp"
         # PX4 mc_pos_control/Takeoff + hysteresis (verbatim). hysteresis used only by
         # the Takeoff member; C callers keep using fc_hysteresis.c.
         # ---
         # PX4 mc_pos_control/Takeoff + hysteresis (verbatim). hysteresis yalnizca Takeoff
         # uyesi tarafindan kullanilir; C cagiranlar fc_hysteresis.c'yi kullanmaya devam eder.
         "PX4/src/lib/hysteresis/hysteresis.cpp"
         "PX4/src/lib/mc_pos_control/Takeoff/Takeoff.cpp"
         "PX4/src/lib/motion_planning/VelocitySmoothing.cpp"
         "PX4/src/lib/motion_planning/PositionSmoothing.cpp"
         # HeadingSmoothing (verbatim): FlightTaskOrbit yaw-smoothing dependency.
         # SlewRate.hpp is header-only; bound via fc_flight_task_orbit_px4 adapter.
         # ---
         # HeadingSmoothing (verbatim): FlightTaskOrbit yaw-yumusatma bagimliligi.
         # SlewRate.hpp yalnizca-baslik; fc_flight_task_orbit_px4 adaptoru uzerinden baglanir.
         "PX4/src/lib/motion_planning/HeadingSmoothing.cpp"

         # uORB-shim base: CDev + cdev_platform (verbatim). cdev_platform.cpp is the
         # full PX4 src/lib/cdev/posix/ backend (register_driver + px4_open/VFile/devmap/filemap).
         # ---
         # uORB-shim tabani: CDev + cdev_platform (verbatim). cdev_platform.cpp tam PX4
         # src/lib/cdev/posix/ arka ucudur (register_driver + px4_open/VFile/devmap/filemap).
         "PX4/src/lib/cdev/CDev.cpp"
         "PX4/src/lib/cdev/posix/cdev_platform.cpp"

         # uORB bus core (verbatim, 8 .cpp). uORBManagerUsr (PROTECTED-build user-side
         # proxy) is unused in the FLAT build, so not added.
         # ---
         # uORB veriyolu cekirdegi (verbatim, 8 .cpp). uORBManagerUsr (PROTECTED-derleme
         # kullanici-taraf proxy'si) FLAT derlemede kullanilmaz, o yuzden eklenmedi.
         "PX4/src/lib/uORB/uORB.cpp"
         "PX4/src/lib/uORB/uORBManager.cpp"
         "PX4/src/lib/uORB/uORBDeviceMaster.cpp"
         "PX4/src/lib/uORB/uORBDeviceNode.cpp"
         "PX4/src/lib/uORB/uORBUtils.cpp"
         # uORBMessageFields: topic field-name introspection. Generated header+source
         # (px_generate_uorb_compressed_fields.py) + real heatshrink (PX4 submodule);
         # keeps uORB complete (introspection included).
         # ---
         # uORBMessageFields: topic alan-adi introspeksiyonu. Uretilmis baslik+kaynak
         # (px_generate_uorb_compressed_fields.py) + gercek heatshrink (PX4 submodulu);
         # uORB'u eksiksiz tutar (introspeksiyon dahil).
         "PX4/src/lib/uORB/uORBMessageFields.cpp"
         "PX4/src/lib/uORB/topics/uORBMessageFieldsGenerated.cpp"
         "PX4/src/lib/heatshrink/heatshrink/heatshrink_decoder.c"
         "PX4/src/lib/uORB/Subscription.cpp"
         "PX4/src/lib/uORB/SubscriptionInterval.cpp"

         # module-shim: PX4 ModuleBase (verbatim). module.cpp = PRINT_MODULE_* help,
         # module_base.cpp = ModuleBase::main/start/stop/run_trampoline_impl.
         # fc_px4_tasks.cpp = P4 task backend (FreeRTOS; px4_tasks equivalent).
         # ---
         # module-shim: PX4 ModuleBase (verbatim). module.cpp = PRINT_MODULE_* yardim,
         # module_base.cpp = ModuleBase::main/start/stop/run_trampoline_impl.
         # fc_px4_tasks.cpp = P4 gorev arka ucu (FreeRTOS; px4_tasks esdegeri).
         "PX4/src/lib/matrix/px4_platform_common/module.cpp"
         "PX4/src/lib/matrix/px4_platform_common/module_base.cpp"
         "fc_px4_tasks.cpp"
         # module starter (uses uORB/PX4 headers -> scoped __PX4_POSIX, FC_PX4_POSIX_SRCS).
         # ---
         # modul baslatici (uORB/PX4 basliklarini kullanir -> kapsamli __PX4_POSIX, FC_PX4_POSIX_SRCS).
         "fc_px4_modules.cpp"
         # "fc_px4_adapter.cpp"

         "eth.c"
         "http_server.c"
         "littlefs_fs.c"
         "c6_controller.c"
    INCLUDE_DIRS "." "PX4/src" "PX4/src/include" "PX4/src/lib"
    REQUIRES
        nvs_flash
        esp_event
        esp_wifi
        esp_wifi_remote
        esp_hosted
        littlefs
        esp_http_server
        esp_netif
        esp_eth
        esp_timer
        pthread          # PX4 module tasks via esp_pthread (pthread_self/cond/mutex) | esp_pthread ile PX4 modul gorevleri (pthread_self/cond/mutex)
        lwip
        cjson
        esp_driver_rmt
        esp_driver_tsens
        esp_driver_gpio
        esp_driver_spi
        esp_driver_i2c
        driver
        fatfs
        sdmmc
)

# SYSTEM includes suppress upstream-header warnings that clash with ESP-IDF's global
# -Werror (mavlink -Werror=address-of-packed-member is a known upstream case).
# lib/matrix, lib/mathlib: PX4 ECL matrix/math libs, header-only.
# px4_boardconfig_stub: empty px4_boardconfig.h so px4_platform_common/defines.h
# resolves its #include on P4/ESP-IDF (content unused; __PX4_NUTTX/__PX4_POSIX undefined).
# param-shim: real PX4 C++ param framework (matrix/px4_platform_common/
# {param,param_macros,module_params}.h) resolved via "PX4/src/lib/matrix";
# its <containers/List.hpp> + <parameters/...> resolved via the "lib" include root.
# ---
# SYSTEM include'lari, ESP-IDF'in global -Werror'uyla cakisan upstream-baslik uyarilarini
# bastirir (mavlink -Werror=address-of-packed-member bilinen bir upstream durumu).
# lib/matrix, lib/mathlib: PX4 ECL matrix/math kutuphaneleri, yalnizca-baslik.
# px4_boardconfig_stub: bos px4_boardconfig.h; boylece px4_platform_common/defines.h
# kendi #include'unu P4/ESP-IDF'te cozer (icerik kullanilmaz; __PX4_NUTTX/__PX4_POSIX tanimsiz).
# param-shim: gercek PX4 C++ param cercevesi (matrix/px4_platform_common/
# {param,param_macros,module_params}.h) "PX4/src/lib/matrix" uzerinden cozulur;
# onun <containers/List.hpp> + <parameters/...>'i "lib" include kokunden cozulur.
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE
    "mavlink/c_library_v2"
    "mavlink/c_library_v2/common"
    "PX4/src/lib"
    "PX4/src/lib/matrix"
    "PX4/src/lib/matrix/matrix"
    "PX4/src/lib/px4_boardconfig_stub"
    # EKF include comes from modules/ekf2/EKF (scoped, in ekf2 block), not lib/ecl
    # ---
    # EKF include'u lib/ecl'den DEGIL modules/ekf2/EKF'ten gelir (kapsamli, ekf2 blogunda)
    "PX4/src/lib/mc_pos_control"                  # <uORB/topics/...setpoint.h> plain structs | <uORB/topics/...setpoint.h> duz struct'lar
    "PX4/src/lib/mc_pos_control/PositionControl"  # ControlMath.hpp / PositionControl.hpp
    "PX4/src/lib/mc_pos_control/Takeoff"          # Takeoff.hpp (fc_takeoff_px4.cpp adapter) | Takeoff.hpp (fc_takeoff_px4.cpp adaptoru)
    "PX4/src/lib/motion_planning"                 # VelocitySmoothing.hpp (fc_velocity_smoothing.cpp adapter) | VelocitySmoothing.hpp (fc_velocity_smoothing.cpp adaptoru)
    "PX4/src/lib/mc_att_control/AttitudeControl"  # AttitudeControl.hpp
    "PX4/src/lib/rate_control"                    # rate_control.hpp + rate_ctrl_status.h
    "PX4/src/lib/control_allocation"              # <uORB/topics/control_allocator_status.h> + actuator_effectiveness
    "PX4/src/lib/control_allocation/control_allocation"  # ControlAllocation*.hpp
)

# Force-include PX4 visibility.h (__EXPORT/__BEGIN_DECLS/__END_DECLS) into all sources
# (PX4's own build does this too), so uORB/cdev headers stay verbatim without editing.
# ---
# PX4 visibility.h'i (__EXPORT/__BEGIN_DECLS/__END_DECLS) tum kaynaklara zorla-dahil et
# (PX4'un kendi derlemesi de boyle yapar); boylece uORB/cdev basliklari duzenlenmeden verbatim kalir.
target_compile_options(${COMPONENT_LIB} PRIVATE "-include" "${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/visibility.h")

# HIL SIM-TIME CLOCK.
# px4_clock_settime is a no-op (no lockstep) -> HIL IMU real-time stamped -> dt jitter -> EKF accel-bias.
# Fix: linker --wrap intercepts hrt_absolute_time + clock_settime (impl: fc_px4_tasks.cpp __wrap_*).
# Core1 uses esp_timer -> 8kHz unaffected.
# ---
# HIL SIM-ZAMAN SAATI.
# px4_clock_settime bir no-op'tur (lockstep yok) -> HIL IMU gercek-zaman damgali -> dt jitter -> EKF ivme-onyargi.
# Fix: linker --wrap, hrt_absolute_time + clock_settime'i yakalar (uygulama: fc_px4_tasks.cpp __wrap_*).
# Core1 esp_timer kullanir -> 8kHz etkilenmez.
set(FC_HIL_SIMTIME ON)
if(FC_HIL_SIMTIME)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FC_HIL_SIMTIME=1)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=hrt_absolute_time" "-Wl,--wrap=clock_settime")
endif()

# FC_CORE1_MOTOR_OUT: representative motor-output load on Core1 (stress test).
# Core1 8kHz loop reads actuator_motors (via seqlock) and drives 4x DShot RMT
# (GPIO 21/20/26/27, no real ESC) to exercise real CPU+RMT output cost. Gazebo motors
# take a separate path (Core0 pwm_out_sim->simulator_mavlink), so sim flight is unaffected.
# OFF => Core1 single DShot channel (no load).
# ---
# FC_CORE1_MOTOR_OUT: Core1'de temsili motor-cikis yuku (stres testi).
# Core1 8kHz dongusu actuator_motors'u (seqlock ile) okur ve gercek CPU+RMT cikis maliyetini
# olcmek icin 4x DShot RMT'yi surer (GPIO 21/20/26/27, gercek ESC yok). Gazebo motorlari
# ayri bir yol izler (Core0 pwm_out_sim->simulator_mavlink), yani sim ucusu etkilenmez.
# OFF => Core1 tek DShot kanali (yuk yok).
set(FC_CORE1_MOTOR_OUT ON)
if(FC_CORE1_MOTOR_OUT)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FC_CORE1_MOTOR_OUT=1)
endif()

# FC_CORE1_MOTOR_SAFEGATE: MixingOutput-faithful armed gate for the motor bridge.
# On real HW motor output goes through MixingOutput, forced to _disarmed_value(0) on
# disarm/lockdown/kill/termination (mixer_module.cpp:433,491-505). Raw actuator_motors
# reads bypass that gate: when control_allocator stops publishing on disarm
# (ControlAllocator.cpp:709), Core1 would latch the last armed throttle = post-kill flyaway.
# Gate: motor=0 unless armed && !lockdown && !kill && !termination. OFF => raw read.
# ---
# FC_CORE1_MOTOR_SAFEGATE: motor koprusu icin MixingOutput-sadik armed kapisi.
# Gercek HW'de motor cikisi MixingOutput'tan gecer ve disarm/lockdown/kill/termination'da
# _disarmed_value(0)'a zorlanir (mixer_module.cpp:433,491-505). Ham actuator_motors
# okumalari bu kapiyi atlar: control_allocator disarm'da yayini durdurdugunda
# (ControlAllocator.cpp:709), Core1 son armed gazi mandallar = kill-sonrasi kacis.
# Kapi: armed && !lockdown && !kill && !termination degilse motor=0. OFF => ham okuma.
set(FC_CORE1_MOTOR_SAFEGATE ON)
if(FC_CORE1_MOTOR_SAFEGATE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FC_CORE1_MOTOR_SAFEGATE=1)
endif()

# FC_CORE1_STALE_GUARD: Core1 failsafe if Core0 (fc_ekf_task) dies/stalls.
# (1) seqlock-read retry cap (fc_core.h): if Core0 dies mid-write, Core1's 8kHz loop must not spin forever.
# (2) motor-cmd freshness (fc_core.c): if mc.time_us > 100ms stale, cut motors instead of latching last throttle (flyaway).
# Zero effect in normal flight (mc fresh every ~4ms; 1-3 read retries). Our Core1 code only; no PX4 files touched.
# OFF => unbounded read + valid-only.
# ---
# FC_CORE1_STALE_GUARD: Core0 (fc_ekf_task) olur/takilirsa Core1 failsafe.
# (1) seqlock-okuma yeniden-deneme siniri (fc_core.h): Core0 yazma-ortasinda olurse, Core1'in 8kHz dongusu sonsuza dek donmemeli.
# (2) motor-komut tazeligi (fc_core.c): mc.time_us > 100ms bayat ise, son gazi mandallamak yerine motorlari kes (kacis).
# Normal ucusta sifir etki (mc her ~4ms tazelenir; 1-3 okuma yeniden-denemesi). Yalnizca kendi Core1 kodumuz; hicbir PX4 dosyasina dokunulmadi.
# OFF => sinirsiz okuma + yalnizca-gecerli.
set(FC_CORE1_STALE_GUARD ON)
if(FC_CORE1_STALE_GUARD)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FC_CORE1_STALE_GUARD=1)
endif()

# FC_DSHOT_STATIC_PAYLOAD: DShot RMT sym-payload in a persistent per-channel buffer
# (fc_dshot_motor_t.dshot_syms) instead of the stack. IDF rmt_transmit reads the payload
# ASYNC (in ISR after return if the channel is busy) and requires esp_ptr_internal, so a
# stack array only worked by timing luck; DShot150/Core1-stall gave a dead stack -> garbage
# frame. Persistent internal .bss buffer satisfies the contract. Behavior identical otherwise.
# OFF => stack array.
# ---
# FC_DSHOT_STATIC_PAYLOAD: DShot RMT sym-yuku, yigin yerine kalici kanal-basi tamponda
# (fc_dshot_motor_t.dshot_syms) tutulur. IDF rmt_transmit yuku ASENKRON okur (kanal mesgulse
# donusten sonra ISR'de) ve esp_ptr_internal gerektirir, bu yuzden yigin dizisi yalnizca zamanlama
# sansiyla calisiyordu; DShot150/Core1-takilmasi olu bir yigin verdi -> cop cerceve. Kalici dahili
# .bss tamponu sozlesmeyi karsilar. Aksi halde davranis ozdes.
# OFF => yigin dizisi.
set(FC_DSHOT_STATIC_PAYLOAD ON)
if(FC_DSHOT_STATIC_PAYLOAD)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FC_DSHOT_STATIC_PAYLOAD=1)
endif()


# WQ-RUNNER PRIORITY: let INS0/ekf2 run without preemption.
# WorkQueueManager pthread_attr_setschedparam fails on esp_pthread -> all WQ at prio-2 -> EKF time-sliced.
# Fix: --wrap maps PX4 priority to P4 priority (impl: fc_px4_tasks.cpp __wrap_pthread_attr_setschedparam).
# Core0 only -> 8kHz Core1 unaffected.
# ---
# WQ-RUNNER ONCELIGI: INS0/ekf2'nin preemption'siz calismasini sagla.
# WorkQueueManager pthread_attr_setschedparam esp_pthread'de basarisiz olur -> tum WQ prio-2'de -> EKF zaman-dilimlenir.
# Fix: --wrap, PX4 onceligini P4 onceligine esler (uygulama: fc_px4_tasks.cpp __wrap_pthread_attr_setschedparam).
# Yalnizca Core0 -> 8kHz Core1 etkilenmez.
set(FC_WQ_PRIO ON)
if(FC_WQ_PRIO)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FC_WQ_PRIO=1)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=pthread_attr_setschedparam")
endif()

# PSRAM-FIX: internal-SRAM-first global operator new (fc_internal_new.cpp).
# With CONFIG_SPIRAM_USE_MALLOC + ALWAYSINTERNAL=2048, >2KB allocs (EKF2 object is tens of
# KB incl. 24x24 covariance+buffers) land in slow PSRAM -> wq:INS0 ~1240us vs algo ~174us (~7x).
# Fix: global operator new "internal-SRAM-first + PSRAM-fallback" moves EKF2/SimulatorMavlink
# objects to fast internal RAM. Platform override; no PX4 code touched.
# Default OFF (faithful): measured EKF2 still ~31% even when internal (compute-bound, not
# memory-bound); OFF returns large objects to PSRAM, freeing internal SRAM for the L2 cache.
# ---
# PSRAM-FIX: dahili-SRAM-oncelikli global operator new (fc_internal_new.cpp).
# CONFIG_SPIRAM_USE_MALLOC + ALWAYSINTERNAL=2048 ile, >2KB tahsisler (EKF2 nesnesi 24x24
# kovaryans+tamponlar dahil onlarca KB) yavas PSRAM'e duser -> wq:INS0 ~1240us, algo ~174us (~7x).
# Fix: global operator new "dahili-SRAM-oncelikli + PSRAM-yedek", EKF2/SimulatorMavlink
# nesnelerini hizli dahili RAM'e tasir. Platform gecersiz-kilma; hicbir PX4 koduna dokunulmadi.
# Varsayilan OFF (sadik): olculen EKF2 dahili olsa bile hala ~%31 (hesap-bagli, bellek-bagli
# degil); OFF buyuk nesneleri PSRAM'e geri verir ve dahili SRAM'i L2 cache icin serbest birakir.
set(FC_INTERNAL_NEW OFF)
if(FC_INTERNAL_NEW)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FC_INTERNAL_NEW=1)
endif()

# FC_POLL_CLOCK_FIX: root of QGC flapping / dataman timeout / navigator spin / GCS loss.
# cdev_platform.cpp:397 px4_poll deadline is CLOCK_MONOTONIC, but ESP-IDF sem_timedwait
# (pthread_semaphore.c:88) + pthread_cond_timedwait use CLOCK_REALTIME (condattr_setclock no-op).
# When QGC SYSTEM_TIME jumps CLOCK_REALTIME to 2026, the MONOTONIC deadline is in the past ->
# instant ETIMEDOUT -> px4_poll spin. Fix: --wrap wraps sem_timedwait, converting the MONOTONIC
# deadline to REALTIME (impl: fc_px4_tasks.cpp __wrap_sem_timedwait). No PX4 file touched.
# ---
# FC_POLL_CLOCK_FIX: QGC titremesi / dataman timeout / navigator donmesi / GCS kaybi kokenidir.
# cdev_platform.cpp:397 px4_poll son-tarihi CLOCK_MONOTONIC'tir, ama ESP-IDF sem_timedwait
# (pthread_semaphore.c:88) + pthread_cond_timedwait CLOCK_REALTIME kullanir (condattr_setclock no-op).
# QGC SYSTEM_TIME, CLOCK_REALTIME'i 2026'ya atladiginda, MONOTONIC son-tarih gecmiste kalir ->
# aninda ETIMEDOUT -> px4_poll donmesi. Fix: --wrap, sem_timedwait'i sarar ve MONOTONIC son-tarihi
# REALTIME'a cevirir (uygulama: fc_px4_tasks.cpp __wrap_sem_timedwait). Hicbir PX4 dosyasina dokunulmadi.
set(FC_POLL_CLOCK_FIX ON)
if(FC_POLL_CLOCK_FIX)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FC_POLL_CLOCK_FIX=1)
    target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=sem_timedwait")
endif()

# CONFIG_EKF2_*: PX4 ECL modular feature flags, matching PX4 Gazebo-SITL Kconfig defaults
# (ekf2/Kconfig: 17 features default=y). A hand-picked 3-flag subset (baro/mag/gnss) was a
# deviation; notably GRAVITY_FUSION is active IMU-only, and leaving it off diverged EKF
# behavior from PX4. Excludes VERBOSE_STATUS + MULTI_INSTANCE (default=n in PX4 too).
# Unfed sensors (flow/range/vision) stay passive; code path matches PX4-Gazebo.
# ---
# CONFIG_EKF2_*: PX4 ECL moduler ozellik bayraklari; PX4 Gazebo-SITL Kconfig varsayilanlariyla
# eslesir (ekf2/Kconfig: 17 ozellik varsayilan=y). Elle-secilmis 3-bayrakli alt-kume (baro/mag/gnss)
# bir sapmaydi; ozellikle GRAVITY_FUSION yalnizca-IMU'da aktiftir ve onu kapali birakmak EKF
# davranisini PX4'ten sapdirdi. VERBOSE_STATUS + MULTI_INSTANCE haric (PX4'te de varsayilan=n).
# Beslenmeyen sensorler (flow/range/vision) pasif kalir; kod yolu PX4-Gazebo ile eslesir.
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    CONFIG_EKF2_AIRSPEED
    CONFIG_EKF2_AUX_GLOBAL_POSITION
    CONFIG_EKF2_AUXVEL
    CONFIG_EKF2_BAROMETER
    CONFIG_EKF2_BARO_COMPENSATION
    CONFIG_EKF2_DRAG_FUSION
    CONFIG_EKF2_EXTERNAL_VISION
    CONFIG_EKF2_GNSS
    CONFIG_EKF2_GNSS_YAW
    CONFIG_EKF2_GRAVITY_FUSION
    CONFIG_EKF2_MAGNETOMETER
    CONFIG_EKF2_OPTICAL_FLOW
    CONFIG_EKF2_RANGE_FINDER
    CONFIG_EKF2_RANGING_BEACON
    CONFIG_EKF2_SIDESLIP
    CONFIG_EKF2_TERRAIN
    CONFIG_EKF2_WIND
    # PX4 work-queue config (WorkQueueManager.hpp wq_config_t constants). Values are
    # verbatim from PX4 platforms/common/px4_work_queue/Kconfig defaults.
    # ---
    # PX4 is-kuyrugu yapilandirmasi (WorkQueueManager.hpp wq_config_t sabitleri). Degerler
    # PX4 platforms/common/px4_work_queue/Kconfig varsayilanlarindan verbatim'dir.
    CONFIG_WQ_RATE_CTRL_STACKSIZE=3150 CONFIG_WQ_RATE_CTRL_PRIORITY=0
    CONFIG_WQ_SPI_STACKSIZE=2392
    CONFIG_WQ_SPI0_PRIORITY=-1 CONFIG_WQ_SPI1_PRIORITY=-2 CONFIG_WQ_SPI2_PRIORITY=-3
    CONFIG_WQ_SPI3_PRIORITY=-4 CONFIG_WQ_SPI4_PRIORITY=-5 CONFIG_WQ_SPI5_PRIORITY=-6
    CONFIG_WQ_SPI6_PRIORITY=-7
    CONFIG_WQ_I2C_STACKSIZE=2336
    CONFIG_WQ_I2C0_PRIORITY=-8 CONFIG_WQ_I2C1_PRIORITY=-9 CONFIG_WQ_I2C2_PRIORITY=-10
    CONFIG_WQ_I2C3_PRIORITY=-11 CONFIG_WQ_I2C4_PRIORITY=-12
    CONFIG_WQ_NAV_AND_CONTROLLERS_STACKSIZE=2240 CONFIG_WQ_NAV_AND_CONTROLLERS_PRIORITY=-13
    CONFIG_WQ_VTE_STACKSIZE=6000 CONFIG_WQ_VTE_PRIORITY=-14
    CONFIG_WQ_INS_STACKSIZE=6000
    CONFIG_WQ_INS0_PRIORITY=-14 CONFIG_WQ_INS1_PRIORITY=-15 CONFIG_WQ_INS2_PRIORITY=-16
    CONFIG_WQ_INS3_PRIORITY=-17
    CONFIG_WQ_HP_DEFAULT_STACKSIZE=2800 CONFIG_WQ_HP_DEFAULT_PRIORITY=-18
    CONFIG_WQ_UAVCAN_STACKSIZE=3754 CONFIG_WQ_UAVCAN_PRIORITY=-19
    CONFIG_WQ_TTY_STACKSIZE=1728
    CONFIG_WQ_TTY_S0_PRIORITY=-21 CONFIG_WQ_TTY_S1_PRIORITY=-22 CONFIG_WQ_TTY_S2_PRIORITY=-23
    CONFIG_WQ_TTY_S3_PRIORITY=-24 CONFIG_WQ_TTY_S4_PRIORITY=-25 CONFIG_WQ_TTY_S5_PRIORITY=-26
    CONFIG_WQ_TTY_S6_PRIORITY=-27 CONFIG_WQ_TTY_S7_PRIORITY=-28 CONFIG_WQ_TTY_S8_PRIORITY=-29
    CONFIG_WQ_TTY_S9_PRIORITY=-30 CONFIG_WQ_TTY_ACM0_PRIORITY=-31 CONFIG_WQ_TTY_UNKNOWN_PRIORITY=-32
    CONFIG_WQ_LP_DEFAULT_STACKSIZE=3500 CONFIG_WQ_LP_DEFAULT_PRIORITY=-50
)

# uORB-shim: PX4 OS layer (posix.h/tasks.h) asks "which OS?"; ESP-IDF matches no existing
# PX4 target (NUTTX/LINUX/DARWIN/QURT). __PX4_POSIX enables PX4's own POSIX branch
# (defines px4_pollfd_struct_t / px4_pollevent_t / POLLIN without editing files). __PX4_ESPIDF
# selects tasks.h's ESP-IDF branch (static priority, like QURT). Scoped to these TUs only.
# ---
# uORB-shim: PX4 OS katmani (posix.h/tasks.h) "hangi OS?" diye sorar; ESP-IDF mevcut hicbir
# PX4 hedefiyle eslesmez (NUTTX/LINUX/DARWIN/QURT). __PX4_POSIX, PX4'un kendi POSIX dalini etkinlestirir
# (dosyalari duzenlemeden px4_pollfd_struct_t / px4_pollevent_t / POLLIN tanimlar). __PX4_ESPIDF,
# tasks.h'nin ESP-IDF dalini secer (statik oncelik, QURT gibi). Yalnizca bu TU'lara kapsamli.
set(FC_PX4_POSIX_SRCS
    "PX4/src/lib/cdev/CDev.cpp"
    "PX4/src/lib/cdev/posix/cdev_platform.cpp"
    "PX4/src/lib/uORB/uORB.cpp"
    "PX4/src/lib/uORB/uORBManager.cpp"
    "PX4/src/lib/uORB/uORBDeviceMaster.cpp"
    "PX4/src/lib/uORB/uORBDeviceNode.cpp"
    "PX4/src/lib/uORB/uORBUtils.cpp"
    "PX4/src/lib/uORB/uORBMessageFields.cpp"
    "PX4/src/lib/uORB/topics/uORBMessageFieldsGenerated.cpp"
    "PX4/src/lib/uORB/Subscription.cpp"
    "PX4/src/lib/uORB/SubscriptionInterval.cpp"
    "PX4/src/lib/matrix/px4_platform_common/module.cpp"
    "PX4/src/lib/matrix/px4_platform_common/module_base.cpp"
    "fc_px4_tasks.cpp"
    # commander link-deps:
    # ---
    # commander baglanti-bagimliliklari:
    "PX4/src/lib/px4_sem.cpp"                  # px4_sem_* (posix pthread impl; matches sem.h struct) | px4_sem_* (posix pthread uygulamasi; sem.h struct'iyla eslesir)
    "PX4/src/lib/systemlib/mavlink_log.cpp"    # mavlink_vasprintf + mavlink_log_* (GCS message queue) | mavlink_vasprintf + mavlink_log_* (GCS mesaj kuyrugu)
    "fc_px4_modules.cpp"               # module starter: uORB::Manager::initialize() + commander spawn | modul baslatici: uORB::Manager::initialize() + commander spawn
    # "fc_px4_adapter.cpp"             # input-adapter: core seqlock -> uORB topic | girdi-adaptoru: cekirdek seqlock -> uORB topic
)
set_source_files_properties(${FC_PX4_POSIX_SRCS}
    PROPERTIES
    # MODULE_NAME: PX4 compiles every module/lib with -DMODULE_NAME (px4_add_module/library);
    # uORB/cdev PX4_ERR/INFO calls require it once real px4_log/log.h is present.
    # ---
    # MODULE_NAME: PX4 her modul/kutuphaneyi -DMODULE_NAME ile derler (px4_add_module/library);
    # gercek px4_log/log.h mevcut oldugunda uORB/cdev PX4_ERR/INFO cagrilari bunu gerektirir.
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"uORB\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    # errno.h/stdarg.h: PX4 assumes these arrive transitively (true on Linux/glibc) but
    # ESP-IDF/newlib does not provide them -> force-include, keeping PX4 files verbatim.
    # ---
    # errno.h/stdarg.h: PX4 bunlarin gecisli geldigini varsayar (Linux/glibc'de dogru) ama
    # ESP-IDF/newlib bunlari saglamaz -> zorla-dahil et, PX4 dosyalarini verbatim tut.
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h"
)
# px4_sem.cpp + mavlink_log.cpp are not in idf_component_register SRCS -> add via target_sources
# (FC_PX4_POSIX_SRCS is only a property list; scoped props applied above).
# ---
# px4_sem.cpp + mavlink_log.cpp idf_component_register SRCS'de degil -> target_sources ile eklenir
# (FC_PX4_POSIX_SRCS yalnizca bir ozellik listesidir; kapsamli ozellikler yukarida uygulandi).
target_sources(${COMPONENT_LIB} PRIVATE "PX4/src/lib/px4_sem.cpp" "PX4/src/lib/systemlib/mavlink_log.cpp")

# ============================================================================
# uORB TOPIC REGISTRY (get_orb_meta + each __orb_<topic>).
# uORBTopics.cpp::get_orb_meta + uorb_topics_list[] reference every topic's __orb_X
# metadata -> all topic .cpp must compile. Generated verbatim (ORB_DEFINE).
# uORBMessageFieldsGenerated.cpp already in FC_PX4_POSIX_SRCS -> excluded (no double-build).
# Scoped __PX4_POSIX, same as uORB core.
# ---
# uORB TOPIC KAYIT DEFTERI (get_orb_meta + her __orb_<topic>).
# uORBTopics.cpp::get_orb_meta + uorb_topics_list[] her topic'in __orb_X metadata'sina referans
# verir -> tum topic .cpp'leri derlenmeli. Verbatim uretilmis (ORB_DEFINE).
# uORBMessageFieldsGenerated.cpp zaten FC_PX4_POSIX_SRCS'de -> haric (cift-derleme yok).
# Kapsamli __PX4_POSIX, uORB cekirdegiyle ayni.
# ============================================================================
include("${CMAKE_CURRENT_LIST_DIR}/cmake_srcs/uorb_topics_srcs.cmake")  # explicit, no-glob (258 topics; FC_UORB_TOPIC_SRCS) | acik, glob-yok (258 topic; FC_UORB_TOPIC_SRCS)
target_sources(${COMPONENT_LIB} PRIVATE ${FC_UORB_TOPIC_SRCS})
set_source_files_properties(${FC_UORB_TOPIC_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"uORB\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h"
)

# ============================================================================
# commander: PX4 src/modules/commander (verbatim). 64 .cpp compiled; tests + failsafe/
# emscripten.cpp (PX4 web/wasm demo, non-firmware) excluded. Extra deps: lib/events/
# events.cpp (+ generated events_generated.h), shutdown.cpp, external_reset_lockout.cpp.
# modes/mavlink_log/drv_tone_alarm are header-only.
# Spawned as a Core0 WQ-task = active flight path (fc_px4_modules.cpp). Scoped __PX4_POSIX
# + force-include (same pattern as uORB); the 8kHz flight core runs on the other Core.
# ---
# commander: PX4 src/modules/commander (verbatim). 64 .cpp derlenir; testler + failsafe/
# emscripten.cpp (PX4 web/wasm demosu, firmware-disi) haric. Ek bagimliliklar: lib/events/
# events.cpp (+ uretilmis events_generated.h), shutdown.cpp, external_reset_lockout.cpp.
# modes/mavlink_log/drv_tone_alarm yalnizca-basliktir.
# Core0 WQ-gorevi olarak spawn edilir = aktif ucus yolu (fc_px4_modules.cpp). Kapsamli __PX4_POSIX
# + zorla-dahil (uORB ile ayni desen); 8kHz ucus cekirdegi diger Core'da calisir.
# ============================================================================
include("${CMAKE_CURRENT_LIST_DIR}/cmake_srcs/commander_srcs.cmake")  # explicit, no-glob (64 files; FC_COMMANDER_SRCS) | acik, glob-yok (64 dosya; FC_COMMANDER_SRCS)
set(FC_COMMANDER_ALL_SRCS
    ${FC_COMMANDER_SRCS}
    "PX4/src/lib/events/events.cpp"
    "PX4/src/lib/matrix/px4_platform_common/shutdown.cpp"
    "PX4/src/lib/matrix/px4_platform_common/external_reset_lockout.cpp"
    "PX4/src/lib/matrix/px4_platform_common/px4_log.cpp"  # PX4 logging backend (verbatim; daemon get_stdout->server_io.h shim) | PX4 loglama arka ucu (verbatim; daemon get_stdout->server_io.h shim)
    # commander-dependent PX4 libs (verbatim): sensor_calibration + circuit_breaker + conversion
    # ---
    # commander'a-bagimli PX4 kutuphaneleri (verbatim): sensor_calibration + circuit_breaker + conversion
    "PX4/src/lib/sensor_calibration/Accelerometer.cpp"
    "PX4/src/lib/sensor_calibration/Barometer.cpp"
    "PX4/src/lib/sensor_calibration/Gyroscope.cpp"
    "PX4/src/lib/sensor_calibration/Magnetometer.cpp"
    "PX4/src/lib/sensor_calibration/Utilities.cpp"
    "PX4/src/lib/circuit_breaker/circuit_breaker.cpp"
    "PX4/src/lib/conversion/rotation.cpp"
    "PX4/src/lib/perf/perf_counter.cpp"
    "PX4/src/lib/tunes/tunes.cpp"
    "PX4/src/lib/tunes/default_tunes.cpp"
)
target_sources(${COMPONENT_LIB} PRIVATE ${FC_COMMANDER_ALL_SRCS})
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE
    "PX4/src/modules/commander"                   # commander internal headers (commander_helper.h, HealthAndArmingChecks/..., ModeUtil/...) | commander dahili basliklari (commander_helper.h, HealthAndArmingChecks/..., ModeUtil/...)
)
set_source_files_properties(${FC_COMMANDER_ALL_SRCS} PROPERTIES
    # CONFIG_BOARD_NO_SDCARD: no FMU-SD on ESP32-P4 -> sdcardCheck.cpp:49 #ifndef drops the SD
    # block -> removes commander's hard "Missing FMU SD Card" arming failure (the real ARM blocker, not EKF).
    # CONSOLE_PRINT_ARMING_CHECK_EVENT (Common.hpp:48, normally commented): prints the ARM-deny
    # reason to serial via Report::report() -> Common.cpp:269/305 PX4_INFO_RAW -> ESP console.
    # Only fires when the report changes (rate-limited, Core0) -> no flood; surfaces the real
    # (events-only) arming check that is normally invisible on serial.
    # ---
    # CONFIG_BOARD_NO_SDCARD: ESP32-P4'te FMU-SD yok -> sdcardCheck.cpp:49 #ifndef, SD blogunu
    # dusurur -> commander'in kati "Missing FMU SD Card" arming hatasini kaldirir (gercek ARM engeli, EKF degil).
    # CONSOLE_PRINT_ARMING_CHECK_EVENT (Common.hpp:48, normalde yorumlu): ARM-red nedenini
    # Report::report() ile seri'ye yazdirir -> Common.cpp:269/305 PX4_INFO_RAW -> ESP konsolu.
    # Yalnizca rapor degistiginde tetiklenir (hiz-sinirli, Core0) -> tasma yok; seri'de normalde
    # gorunmeyen gercek (yalnizca-olay) arming kontrolunu gun yuzune cikarir.
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"commander\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\";CONFIG_BOARD_NO_SDCARD;CONSOLE_PRINT_ARMING_CHECK_EVENT"
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h"
)

# queue.c: PX4 platforms/common/work_queue/queue.c (verbatim) = NuttX-compatible sq_*/dq_*
# linked-queue impl (for perf_counter.cpp sq_addfirst/sq_rem). Plain C, self-contained;
# no scoped __PX4_POSIX needed.
# ---
# queue.c: PX4 platforms/common/work_queue/queue.c (verbatim) = NuttX-uyumlu sq_*/dq_*
# bagli-kuyruk uygulamasi (perf_counter.cpp sq_addfirst/sq_rem icin). Duz C, kendine-yeter;
# kapsamli __PX4_POSIX gerekmez.
target_sources(${COMPONENT_LIB} PRIVATE "PX4/src/lib/queue.c")  # provides real drv_hrt.cpp latency_buckets (replaces fc_px4_hrt_latency.c) | gercek drv_hrt.cpp latency_buckets saglar (fc_px4_hrt_latency.c'nin yerine)

# perf_counter.cpp expects queue.h (sq_queue_t/sq_entry_t) transitively (system headers on
# NuttX); not available on P4 -> force-include queue.h for this file only. A dedicated
# set overrides options, so it repeats all commander defines/options + queue.h.
# ---
# perf_counter.cpp, queue.h'i (sq_queue_t/sq_entry_t) gecisli bekler (NuttX'te sistem
# basliklari); P4'te mevcut degil -> yalnizca bu dosya icin queue.h'i zorla-dahil et. Ozel bir
# set secenekleri gecersiz kilar, bu yuzden tum commander define'larini/secenegini + queue.h'i tekrarlar.
set_source_files_properties("PX4/src/lib/perf/perf_counter.cpp" PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"commander\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/queue.h"
)

# ============================================================================
# PX4 navigator (verbatim) — same pattern as commander.
# 18 .cpp; excludes tests + vtol_takeoff.cpp (CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF-gated,
# not built for MC). Shared infra (events/px4_log/perf/queue/shutdown/...) already compiled
# in the commander block -> not re-added (no ODR/link clash). New nav deps: dataman_client
# (DatamanClient) + adsb (AdsbConflict) + rtl (rtl_time_estimator). Spawned as Core0 WQ-task.
# ---
# PX4 navigator (verbatim) — commander ile ayni desen.
# 18 .cpp; testler + vtol_takeoff.cpp haric (CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF-kapili,
# MC icin derlenmez). Paylasilan altyapi (events/px4_log/perf/queue/shutdown/...) zaten commander
# blogunda derlendi -> yeniden-eklenmez (ODR/baglanti cakismasi yok). Yeni nav bagimliliklari: dataman_client
# (DatamanClient) + adsb (AdsbConflict) + rtl (rtl_time_estimator). Core0 WQ-gorevi olarak spawn edilir.
# ============================================================================
include("${CMAKE_CURRENT_LIST_DIR}/cmake_srcs/navigator_srcs.cmake")  # explicit, no-glob (18 files; FC_NAVIGATOR_SRCS) | acik, glob-yok (18 dosya; FC_NAVIGATOR_SRCS)
set(FC_NAVIGATOR_ALL_SRCS
    ${FC_NAVIGATOR_SRCS}
    "PX4/src/lib/dataman_client/DatamanClient.cpp"   # nav DEPENDS dataman_client (mission storage client) | nav dataman_client'a BAGLIDIR (gorev-depolama istemcisi)
    "PX4/src/lib/adsb/AdsbConflict.cpp"              # nav DEPENDS adsb (traffic conflict) | nav adsb'ye BAGLIDIR (trafik cakismasi)
    "PX4/src/lib/rtl/rtl_time_estimator.cpp"         # nav DEPENDS rtl_time_estimator | nav rtl_time_estimator'a BAGLIDIR
)
target_sources(${COMPONENT_LIB} PRIVATE ${FC_NAVIGATOR_ALL_SRCS})
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE
    "PX4/src/modules"                                # resolves <navigator/navigation.h>, <dataman/dataman.h> | <navigator/navigation.h>, <dataman/dataman.h> cozer
    "PX4/src/modules/navigator"                      # navigator local headers | navigator yerel basliklari
)
# CONFIG_NUM_MISSION_ITMES_SUPPORTED (typo "ITMES" is PX4-original): sizes the
# DM_KEY_WAYPOINTS_OFFBOARD_*_MAX enum. PX4 generic-default=500; here set to 50 (see below).
# BOARD_HAS_NO_UUID: AdsbConflict.cpp skips board_get_px4_guid() (no PX4 board-GUID HW
# abstraction on P4; legit PX4 board option). -Wno-missing-field-initializers: PX4-standard
# flag (px4_add_common_flags.cmake:89) for partial-init code like rtl_direct.cpp (unchanged).
# ---
# CONFIG_NUM_MISSION_ITMES_SUPPORTED ("ITMES" yazim-hatasi PX4-orijinalidir): DM_KEY_WAYPOINTS_
# OFFBOARD_*_MAX enum'unu boyutlandirir. PX4 genel-varsayilan=500; burada 50'ye ayarli (asagiya bak).
# BOARD_HAS_NO_UUID: AdsbConflict.cpp board_get_px4_guid()'i atlar (P4'te PX4 board-GUID HW
# soyutlamasi yok; mesru PX4 board secenegi). -Wno-missing-field-initializers: PX4-standart
# bayrak (px4_add_common_flags.cmake:89), rtl_direct.cpp gibi kismi-init kod icin (degismedi).
set_source_files_properties(${FC_NAVIGATOR_ALL_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"navigator\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\";CONFIG_NUM_MISSION_ITMES_SUPPORTED=50;BOARD_HAS_NO_UUID"
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers"
)

# ============================================================================
# PX4 dataman (verbatim) — mission/geofence/safepoint storage SERVER.
# navigator's DatamanClient handshake (DM_GET_ID) connects here; without the server the nav
# constructor hits 4x 1000ms timeout -> "Failed to get client ID" -> nav 0-cycle.
# RAM backend (spawn '-r'): CONFIG_DATAMAN_PERSISTENT_STORAGE undefined -> all file/FS code
# (#ifdef-gated, 17 sites) is dropped; pure in-memory malloc (dataman.cpp:616 _ram_initialize).
# No writable FS on P4 -> RAM is verbatim-correct (PX4 NuttX no-FS board = 'dataman start -r').
# CONFIG_NUM_MISSION_ITMES_SUPPORTED=50: same as navigator block (shared dataman.h enum size;
# a mismatch corrupts struct offsets). Shared infra already compiled in commander block.
# ---
# PX4 dataman (verbatim) — gorev/geofence/safepoint depolama SUNUCUSU.
# navigator'in DatamanClient el-sikismasi (DM_GET_ID) buraya baglanir; sunucu olmadan nav
# kurucusu 4x 1000ms timeout'a carpar -> "Failed to get client ID" -> nav 0-dongu.
# RAM arka ucu (spawn '-r'): CONFIG_DATAMAN_PERSISTENT_STORAGE tanimsiz -> tum dosya/FS kodu
# (#ifdef-kapili, 17 yer) dusurulur; saf bellek-ici malloc (dataman.cpp:616 _ram_initialize).
# P4'te yazilabilir FS yok -> RAM verbatim-dogrudur (PX4 NuttX FS'siz board = 'dataman start -r').
# CONFIG_NUM_MISSION_ITMES_SUPPORTED=50: navigator blogu ile ayni (paylasilan dataman.h enum boyutu;
# uyusmazlik struct offset'lerini bozar). Paylasilan altyapi zaten commander blogunda derlendi.
# ============================================================================
target_sources(${COMPONENT_LIB} PRIVATE "PX4/src/modules/dataman/dataman.cpp")
set_source_files_properties("PX4/src/modules/dataman/dataman.cpp" PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"dataman\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\";CONFIG_NUM_MISSION_ITMES_SUPPORTED=50"
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h"
)

# lib_crc32.c: PX4 platforms/posix/src/px4/common/lib_crc32.c (verbatim; crc32/crc32part).
# For geofence.cpp <crc32.h> + crc32part() link. Plain C, self-contained.
# fc_mission.c's fc_crc32* functions are static (file-local) -> no clash.
# ---
# lib_crc32.c: PX4 platforms/posix/src/px4/common/lib_crc32.c (verbatim; crc32/crc32part).
# geofence.cpp <crc32.h> + crc32part() baglantisi icin. Duz C, kendine-yeter.
# fc_mission.c'nin fc_crc32* fonksiyonlari static'tir (dosya-yerel) -> cakisma yok.
target_sources(${COMPONENT_LIB} PRIVATE "PX4/src/lib/lib_crc32.c")

# ============================================================================
# PX4 flight_mode_manager + FlightTasks (verbatim).
# FlightTasks_generated.{hpp,cpp} generated via PX4 generate_flight_tasks.py (empy 3.3.4)
# with the 12 non-constrained-MC tasks in exact list order (not hand-written).
# New DEPENDS libs: collision_prevention/stick_yaw/sticks/weather_vane (slew_rate/
# motion_planning/mathlib already present). Shared infra is in the commander block.
# Spawned as a Core0 WQ-task; the 8kHz core (other Core) is untouched.
# ---
# PX4 flight_mode_manager + FlightTasks (verbatim).
# FlightTasks_generated.{hpp,cpp}, PX4 generate_flight_tasks.py (empy 3.3.4) ile uretilir;
# 12 kisitsiz-MC gorevi tam liste sirasinda (elle-yazilmamis).
# Yeni BAGIMLILIK kutuphaneleri: collision_prevention/stick_yaw/sticks/weather_vane (slew_rate/
# motion_planning/mathlib zaten mevcut). Paylasilan altyapi commander blogundadir.
# Core0 WQ-gorevi olarak spawn edilir; 8kHz cekirdek (diger Core) dokunulmamis kalir.
# ============================================================================
include("${CMAKE_CURRENT_LIST_DIR}/cmake_srcs/fmm_srcs.cmake")  # explicit, no-glob (18 files; FC_FMM_SRCS) | acik, glob-yok (18 dosya; FC_FMM_SRCS)
# NOTE: PX4 FlightTaskOrbit (tasks/Orbit/) is included (full 12-task set). The old hand-port
# fc_flight_task_orbit_px4 class was renamed to FcLegacyOrbit (ODR clash fixed; PX4 Orbit kept).
# ---
# NOT: PX4 FlightTaskOrbit (tasks/Orbit/) dahildir (tam 12-gorev kumesi). Eski elle-port
# fc_flight_task_orbit_px4 sinifi FcLegacyOrbit'e yeniden adlandirildi (ODR cakismasi fix'lendi; PX4 Orbit korundu).
set(FC_FMM_ALL_SRCS
    ${FC_FMM_SRCS}
    "PX4/src/lib/collision_prevention/CollisionPrevention.cpp"   # FlightTask collision-prevention DEPENDS | FlightTask carpisma-onleme BAGIMLILIGI
    "PX4/src/lib/collision_prevention/ObstacleMath.cpp"
    "PX4/src/lib/stick_yaw/StickYaw.cpp"                          # manual-task stick->yaw | manuel-gorev cubuk->yaw
    "PX4/src/modules/flight_mode_manager/tasks/AutoFollowTarget/follow_target_estimator/TargetEstimator.cpp"  # AutoFollowTarget task DEPENDS | AutoFollowTarget gorevi BAGIMLILIGI
    "PX4/src/lib/sticks/Sticks.cpp"                              # manual-task stick read | manuel-gorev cubuk okuma
    "PX4/src/lib/weather_vane/WeatherVane.cpp"                    # FMM DEPENDS WeatherVane | FMM WeatherVane'e BAGLIDIR
    "PX4/src/lib/motion_planning/ManualVelocitySmoothingXY.cpp"  # ManualAcceleration task smoother | ManualAcceleration gorevi yumusaticisi
    "PX4/src/lib/motion_planning/ManualVelocitySmoothingZ.cpp"   # ManualAltitudeSmoothVel task smoother | ManualAltitudeSmoothVel gorevi yumusaticisi
)
target_sources(${COMPONENT_LIB} PRIVATE ${FC_FMM_ALL_SRCS})
# Each FlightTask subdir goes on the include path: the generated hpp bare-includes
# "FlightTask<X>.hpp" (PX4 makes each task a separate lib + INTERFACE-include; the flat
# P4 build includes every task dir instead).
# ---
# Her FlightTask alt-dizini include yoluna eklenir: uretilmis hpp "FlightTask<X>.hpp"'i
# ciplak-dahil eder (PX4 her gorevi ayri kutuphane + INTERFACE-include yapar; duz
# P4 derlemesi bunun yerine her gorev dizinini dahil eder).
set(FC_FMM_TASKDIRS  # explicit, no-glob — 14 FMM task subdirs (include path) | acik, glob-yok — 14 FMM gorev alt-dizini (include yolu)
    "PX4/src/modules/flight_mode_manager/tasks/AltitudeCruise"
    "PX4/src/modules/flight_mode_manager/tasks/Auto"
    "PX4/src/modules/flight_mode_manager/tasks/AutoFollowTarget"
    "PX4/src/modules/flight_mode_manager/tasks/Descend"
    "PX4/src/modules/flight_mode_manager/tasks/Failsafe"
    "PX4/src/modules/flight_mode_manager/tasks/FlightTask"
    "PX4/src/modules/flight_mode_manager/tasks/ManualAcceleration"
    "PX4/src/modules/flight_mode_manager/tasks/ManualAccelerationSlow"
    "PX4/src/modules/flight_mode_manager/tasks/ManualAltitude"
    "PX4/src/modules/flight_mode_manager/tasks/ManualAltitudeSmoothVel"
    "PX4/src/modules/flight_mode_manager/tasks/ManualPosition"
    "PX4/src/modules/flight_mode_manager/tasks/Orbit"
    "PX4/src/modules/flight_mode_manager/tasks/Transition"
    "PX4/src/modules/flight_mode_manager/tasks/Utility")
foreach(d ${FC_FMM_TASKDIRS})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${d})
        target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE ${d})
    endif()
endforeach()
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE
    "PX4/src/modules/flight_mode_manager"
    "PX4/src/modules/flight_mode_manager/tasks/AutoFollowTarget/follow_target_estimator"
    # new nav/manual libs bare-include their own headers ("SlewRate.hpp" etc.)
    # ---
    # yeni nav/manuel kutuphaneler kendi basliklarini ciplak-dahil eder ("SlewRate.hpp" vb.)
    "PX4/src/lib/slew_rate" "PX4/src/lib/sticks" "PX4/src/lib/stick_yaw" "PX4/src/lib/collision_prevention"
    "PX4/src/lib/weather_vane" "PX4/src/lib/motion_planning"
)
# -Wno-cast-align: from PX4 FMM CMakeLists add_compile_options (verbatim).
# -Wno-missing-field-initializers: PX4-standard (partial-init FlightTask code unchanged).
# ---
# -Wno-cast-align: PX4 FMM CMakeLists add_compile_options'dan (verbatim).
# -Wno-missing-field-initializers: PX4-standart (kismi-init FlightTask kodu degismedi).
set_source_files_properties(${FC_FMM_ALL_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"flight_mode_manager\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align"
)

# MAVLINK: PX4 src/modules/mavlink (verbatim, 134 files). Real Mavlink class incl.
# mavlink_main.cpp — no hand-written shim; replaces fc_mavlink_streams/fc_px4_topics.
# Platform-boundary stubs: drivers/device/device.h, lib/version/version.h, uAvionix dialect
# path (mavlink/c_library_v2/uAvionix). The hand-written fc_mavlink_receiver/streams/
# px4_topics/param_protocol were the deviation (where mavlink bugs lived); to be retired once
# this is fully validated. (MAVLINK_USE_CONVENIENCE_FUNCTIONS dropped from CMakeLists ->
# mavlink_bridge_header.h already defines it; redefine-error fix.)
# ---
# MAVLINK: PX4 src/modules/mavlink (verbatim, 134 dosya). Gercek Mavlink sinifi,
# mavlink_main.cpp dahil — elle-yazilmis shim yok; fc_mavlink_streams/fc_px4_topics'in yerine.
# Platform-sinir stub'lari: drivers/device/device.h, lib/version/version.h, uAvionix lehce
# yolu (mavlink/c_library_v2/uAvionix). Elle-yazilmis fc_mavlink_receiver/streams/
# px4_topics/param_protocol sapmaydi (mavlink bug'larinin yasadigi yer); tamamen valide edildiginde
# emekliye ayrilacak. (MAVLINK_USE_CONVENIENCE_FUNCTIONS CMakeLists'ten dusuruldu ->
# mavlink_bridge_header.h onu zaten tanimlar; yeniden-tanim-hatasi fix'i.)
set(FC_CP_MAVLINK ON)  # real PX4 mavlink module (spawned, active GCS telemetry). lwIP<->PX4 transport-header | gercek PX4 mavlink modulu (spawn, aktif GCS telemetrisi). lwIP<->PX4 tasima-basligi
                       # reconciliation = POSIX-shim pattern (cli.h path + POLLIN/_IO guard). | uzlastirma = POSIX-shim deseni (cli.h yolu + POLLIN/_IO korumasi).
if(FC_CP_MAVLINK)
include("${CMAKE_CURRENT_LIST_DIR}/cmake_srcs/mavlink_srcs.cmake")  # explicit, no-glob (19 files; FC_MAVLINK_SRCS) | acik, glob-yok (19 dosya; FC_MAVLINK_SRCS)
list(APPEND FC_MAVLINK_SRCS "PX4/src/lib/timesync/Timesync.cpp" "PX4/src/lib/variable_length_ringbuffer/VariableLengthRingbuffer.cpp"
    "PX4/src/lib/ringbuffer/Ringbuffer.cpp"   # VariableLengthRingbuffer -> base Ringbuffer class | VariableLengthRingbuffer -> taban Ringbuffer sinifi
    # mavlink_receiver deps: HIL_SENSOR injection (PX4Accelerometer/Gyroscope) + GPS_RTCM_DATA (rtcm).
    # Not triggered on the :14555 path (no HIL_SENSOR/RTCM) but needed for LINK since receiver compiles.
    # ---
    # mavlink_receiver bagimliliklari: HIL_SENSOR enjeksiyonu (PX4Accelerometer/Gyroscope) + GPS_RTCM_DATA (rtcm).
    # :14555 yolunda tetiklenmez (HIL_SENSOR/RTCM yok) ama receiver derlendigi icin BAGLANTI icin gerekli.
    "PX4/src/lib/drivers/accelerometer/PX4Accelerometer.cpp"
    "PX4/src/lib/drivers/gyroscope/PX4Gyroscope.cpp"
    "PX4/src/lib/drivers/magnetometer/PX4Magnetometer.cpp"
    "PX4/src/lib/gnss/rtcm.cpp"
    "PX4/platforms/common/px4_cli.cpp"          # px4_get_parameter_value (mavlink_main start CLI) | px4_get_parameter_value (mavlink_main start CLI)
    # parameters_common.cpp (needs px4_parameters.hpp autogen; project uses fc_px4_params shim) and
    # version.c (#error on non-Linux/NuttX/QURT OS) both do not compile on ESP-IDF -> excluded.
    # param-introspection + version are platform-adapted in fc_px4_mavlink_platform.cpp.
    # ---
    # parameters_common.cpp (px4_parameters.hpp autogen gerektirir; proje fc_px4_params shim'i kullanir) ve
    # version.c (Linux/NuttX/QURT-disi OS'ta #error) ESP-IDF'te derlenmez -> haric.
    # param-introspeksiyon + version, fc_px4_mavlink_platform.cpp'de platform-uyarlanmistir.
    "PX4/src/modules/mavlink/mavlink.c"         # real mavlink_system global (verbatim mavlink.c:49, not a stub) | gercek mavlink_system global (verbatim mavlink.c:49, stub degil)
    "fc_px4_mavlink_shell_stub.cpp")            # MavlinkShell stub (no posix-shell; satisfies receiver ref) | MavlinkShell stub (posix-shell yok; receiver referansini karsilar)
target_sources(${COMPONENT_LIB} PRIVATE ${FC_MAVLINK_SRCS})
# P4 platform globals (PX4 version-stub + dup2 libc gap + mavlink_system c_library global).
# Compiled plainly (no mavlink force-includes). Also holds param-introspection platform adaptation.
# ---
# P4 platform global'leri (PX4 version-stub + dup2 libc boslugu + mavlink_system c_library global).
# Duz derlenir (mavlink zorla-dahil'leri yok). Ayrica param-introspeksiyon platform uyarlamasini tutar.
target_sources(${COMPONENT_LIB} PRIVATE "fc_px4_mavlink_platform.cpp")
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE "PX4/src/modules/mavlink" "PX4/src/modules/mavlink/streams"
    "mavlink/c_library_v2" "mavlink/c_library_v2/common" "mavlink/c_library_v2/uAvionix")
set_source_files_properties(${FC_MAVLINK_SRCS} PROPERTIES
    # CONFIG_NUM_MISSION_ITMES_SUPPORTED: fixed size in dataman.h (mavlink_mission.h->DatamanClient).
    # Unified to 50 to match the dataman/navigator blocks (cross-TU consistency); mavlink cleanly
    # rejects >50 with MAV_MISSION_NO_SPACE (matches DM_KEY_WAYPOINTS_OFFBOARD_0_MAX) and saves RAM.
    # ---
    # CONFIG_NUM_MISSION_ITMES_SUPPORTED: dataman.h'de sabit boyut (mavlink_mission.h->DatamanClient).
    # dataman/navigator bloklariyla eslesmek icin 50'ye birlestirildi (TU'lar-arasi tutarlilik); mavlink
    # >50'yi MAV_MISSION_NO_SPACE ile temiz reddeder (DM_KEY_WAYPOINTS_OFFBOARD_0_MAX ile eslesir) ve RAM tasarrufu saglar.
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"mavlink\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\";CONFIG_NUM_MISSION_ITMES_SUPPORTED=50"
    # lwIP<->PX4 transport-header shim (POSIX-shim pattern, mavlink-scoped):
    #  -isystem platforms/common/include -> cli.h (mavlink_main.h:65). CRITICAL: -isystem, NOT -I!
    #  -I is searched before -isystem, which would shadow the matrix-shim posix.h/px4_config.h with
    #  the originals (tasks.h "#error No target OS" + micro_hal cascade). -isystem lets the earlier
    #  matrix-shim win for posix.h/px4_config.h while cli.h still resolves.
    #  sys/poll.h + sys/socket.h force-included first -> POLLIN (posix.h:99) + _IO (px4_ioctl_compat:14)
    #  #ifndef guards skip in favor of system/lwIP (ends redefine-Werror). px4_ioctl_compat kept (system_* + _IOC).
    # ---
    # lwIP<->PX4 tasima-basligi shim'i (POSIX-shim deseni, mavlink-kapsamli):
    #  -isystem platforms/common/include -> cli.h (mavlink_main.h:65). KRITIK: -isystem, -I DEGIL!
    #  -I, -isystem'den once aranir ve matrix-shim posix.h/px4_config.h'i orijinallerle golgeler
    #  (tasks.h "#error No target OS" + micro_hal zinciri). -isystem, cli.h hala cozulurken posix.h/
    #  px4_config.h icin onceki matrix-shim'in kazanmasini saglar.
    #  sys/poll.h + sys/socket.h once zorla-dahil edilir -> POLLIN (posix.h:99) + _IO (px4_ioctl_compat:14)
    #  #ifndef korumalari sistem/lwIP lehine atlar (redefine-Werror'u bitirir). px4_ioctl_compat korundu (system_* + _IOC).
    COMPILE_OPTIONS "-isystem;${CMAKE_CURRENT_LIST_DIR}/PX4/platforms/common/include;-isystem;${CMAKE_CURRENT_LIST_DIR}/fc_px4_gen_stub;-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/fc_px4_netif_compat.h;-include;sys/poll.h;-include;sys/socket.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/fc_px4_board_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align;-Wno-error=address-of-packed-member;-fpermissive;-Wno-error"
)
endif()

# SIMULATOR_MAVLINK: PX4 HIL sensor/actuator bridge (verbatim, src/modules/simulation/simulator_mavlink).
# Gazebo --HIL_SENSOR/HIL_GPS--> PX4Accelerometer/Gyroscope/Magnetometer + sensor_baro/sensor_gps;
# actuator_outputs_sim --> HIL_ACTUATOR. Deps (conversion/geo/drivers_accel/gyro/mag) already compiled.
# ENABLE_LOCKSTEP_SCHEDULER undefined -> px4_lockstep_* no-op (drv_hrt.h:260) + px4_clock_settime no-op
# (time.h:24) -> real-time HIL (hrt=esp_timer/CLOCK_MONOTONIC). Transport: lwIP socket (mavlink-block
# pattern: __PX4_POSIX + isystem platforms/common/include + netif_compat + poll/socket force-include).
# Spawned as a Core0 WQ-task (HIL sensor source).
# ---
# SIMULATOR_MAVLINK: PX4 HIL sensor/actuator koprusu (verbatim, src/modules/simulation/simulator_mavlink).
# Gazebo --HIL_SENSOR/HIL_GPS--> PX4Accelerometer/Gyroscope/Magnetometer + sensor_baro/sensor_gps;
# actuator_outputs_sim --> HIL_ACTUATOR. Bagimliliklar (conversion/geo/drivers_accel/gyro/mag) zaten derlendi.
# ENABLE_LOCKSTEP_SCHEDULER tanimsiz -> px4_lockstep_* no-op (drv_hrt.h:260) + px4_clock_settime no-op
# (time.h:24) -> gercek-zaman HIL (hrt=esp_timer/CLOCK_MONOTONIC). Tasima: lwIP soketi (mavlink-blok
# deseni: __PX4_POSIX + isystem platforms/common/include + netif_compat + poll/socket zorla-dahil).
# Core0 WQ-gorevi olarak spawn edilir (HIL sensor kaynagi).
set(FC_SIMULATOR_SRCS "PX4/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp")
target_sources(${COMPONENT_LIB} PRIVATE ${FC_SIMULATOR_SRCS})
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE
    "PX4/src/modules/simulation/simulator_mavlink"
    "mavlink/c_library_v2" "mavlink/c_library_v2/common" "mavlink/c_library_v2/uAvionix")
set_source_files_properties(${FC_SIMULATOR_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"simulator_mavlink\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-isystem;${CMAKE_CURRENT_LIST_DIR}/PX4/platforms/common/include;-isystem;${CMAKE_CURRENT_LIST_DIR}/fc_px4_gen_stub;-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/fc_px4_netif_compat.h;-include;sys/poll.h;-include;sys/socket.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/fc_px4_board_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align;-Wno-double-promotion;-Wno-error=address-of-packed-member;-fpermissive;-Wno-error"
)

# sensors module (PX4 src/modules/sensors): fuses sensor_* (gyro/accel/mag/baro/gps) into
# vehicle_imu/magnetometer/air_data/gps_position (ekf2 subscribes). DEPENDS conversion+
# sensor_calibration+mathlib already compiled in commander block. Whole-module; runs in WQ threads.
# ---
# sensors modulu (PX4 src/modules/sensors): sensor_*'i (gyro/accel/mag/baro/gps)
# vehicle_imu/magnetometer/air_data/gps_position'a fuzyonlar (ekf2 abone olur). BAGIMLILIKLAR conversion+
# sensor_calibration+mathlib zaten commander blogunda derlendi. Tum-modul; WQ thread'lerinde calisir.
set(FC_SENSORS_SRCS
    "PX4/src/modules/sensors/sensors.cpp"
    "PX4/src/modules/sensors/voted_sensors_update.cpp"
    "PX4/src/modules/sensors/data_validator/DataValidator.cpp"
    "PX4/src/modules/sensors/data_validator/DataValidatorGroup.cpp"
    "PX4/src/modules/sensors/vehicle_acceleration/VehicleAcceleration.cpp"
    "PX4/src/modules/sensors/vehicle_air_data/VehicleAirData.cpp"
    "PX4/src/modules/sensors/vehicle_angular_velocity/VehicleAngularVelocity.cpp"
    "PX4/src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp"
    "PX4/src/modules/sensors/vehicle_gps_position/gps_blending.cpp"
    "PX4/src/modules/sensors/vehicle_gps_position/PpsTimeSync.cpp"
    "PX4/src/modules/sensors/vehicle_imu/VehicleIMU.cpp"
    "PX4/src/modules/sensors/vehicle_magnetometer/VehicleMagnetometer.cpp"
    "PX4/src/modules/sensors/vehicle_optical_flow/VehicleOpticalFlow.cpp")
target_sources(${COMPONENT_LIB} PRIVATE ${FC_SENSORS_SRCS})
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE "PX4/src/modules/sensors")
# PX4 sensors Kconfig is not processed in the P4 build, so CONFIG_SENSORS_VEHICLE_* stayed
# undefined -> sensors.cpp/hpp #if dropped VehicleGPSPosition/Magnetometer/AirData (VehicleIMU
# is unconditional, so attitude worked but GPS/mag/baro never reached ekf2 -> no origin, pos
# drift, ecl_healthy=0). Fix: define the PX4 sensors/Kconfig defaults (all 'y'). AIRSPEED excluded
# (vehicle_airspeed.cpp not in SRCS = FW sensor; defining it would instantiate VehicleAirspeed ->
# link error). Same pattern as CONFIG_EKF2_*.
# ---
# PX4 sensors Kconfig P4 derlemesinde islenmez, bu yuzden CONFIG_SENSORS_VEHICLE_* tanimsiz
# kaldi -> sensors.cpp/hpp #if, VehicleGPSPosition/Magnetometer/AirData'yi dusurdu (VehicleIMU
# kosulsuzdur, bu yuzden tutum calisti ama GPS/mag/baro ekf2'ye hic ulasmadi -> koken yok, pozisyon
# kaymasi, ecl_healthy=0). Fix: PX4 sensors/Kconfig varsayilanlarini tanimla (hepsi 'y'). AIRSPEED haric
# (vehicle_airspeed.cpp SRCS'de degil = FW sensoru; onu tanimlamak VehicleAirspeed'i ornekler ->
# baglanti hatasi). CONFIG_EKF2_* ile ayni desen.
set_source_files_properties(${FC_SENSORS_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"sensors\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\";CONFIG_SENSORS_VEHICLE_ACCELERATION;CONFIG_SENSORS_VEHICLE_AIR_DATA;CONFIG_SENSORS_VEHICLE_ANGULAR_VELOCITY;CONFIG_SENSORS_VEHICLE_GPS_POSITION;CONFIG_SENSORS_VEHICLE_MAGNETOMETER;CONFIG_SENSORS_VEHICLE_OPTICAL_FLOW"
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align")

# land_detector module (PX4 src/modules/land_detector):
# vehicle_local_position/acceleration -> vehicle_land_detected (landed/maybe_landed/ground_contact).
# commander/Core1 subscribe. DEPENDS hysteresis already compiled. Runs in a WQ thread.
# ---
# land_detector modulu (PX4 src/modules/land_detector):
# vehicle_local_position/acceleration -> vehicle_land_detected (landed/maybe_landed/ground_contact).
# commander/Core1 abone olur. BAGIMLILIK hysteresis zaten derlendi. Bir WQ thread'inde calisir.
set(FC_LAND_DETECTOR_SRCS
    "PX4/src/modules/land_detector/LandDetector.cpp"
    "PX4/src/modules/land_detector/MulticopterLandDetector.cpp"
    "PX4/src/modules/land_detector/FixedwingLandDetector.cpp"   # land_detector_main vehicle-type switch references all -> link | land_detector_main arac-tipi switch'i hepsine referans verir -> baglanti
    "PX4/src/modules/land_detector/RoverLandDetector.cpp"
    "PX4/src/modules/land_detector/AirshipLandDetector.cpp"
    "PX4/src/modules/land_detector/VtolLandDetector.cpp"
    "PX4/src/modules/land_detector/land_detector_main.cpp")
target_sources(${COMPONENT_LIB} PRIVATE ${FC_LAND_DETECTOR_SRCS})
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE "PX4/src/modules/land_detector")
set_source_files_properties(${FC_LAND_DETECTOR_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"land_detector\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align")

# ekf2 module (PX4 src/modules/ekf2, 47 files): EKF (verbatim) + EKF2.cpp + EKF2Selector +
# aid_sources. Chain: sensor_* -> sensors -> vehicle_imu/magnetometer -> ekf2 ->
# vehicle_local_position/vehicle_attitude. Replaces lib/ecl. DEPENDS geo/hysteresis/perf/
# world_magnetic_model already compiled. Runs in a WQ thread.
# CONFIG_EKF2_* (17 PX4 defaults) set in the global target_compile_definitions.
# ---
# ekf2 modulu (PX4 src/modules/ekf2, 47 dosya): EKF (verbatim) + EKF2.cpp + EKF2Selector +
# aid_sources. Zincir: sensor_* -> sensors -> vehicle_imu/magnetometer -> ekf2 ->
# vehicle_local_position/vehicle_attitude. lib/ecl'in yerine. BAGIMLILIKLAR geo/hysteresis/perf/
# world_magnetic_model zaten derlendi. Bir WQ thread'inde calisir.
# CONFIG_EKF2_* (17 PX4 varsayilani) global target_compile_definitions'da ayarlandi.
include("${CMAKE_CURRENT_LIST_DIR}/cmake_srcs/ekf2_srcs.cmake")  # explicit FC_EKF2_SRCS (no-glob) | acik FC_EKF2_SRCS (glob-yok)
target_sources(${COMPONENT_LIB} PRIVATE ${FC_EKF2_SRCS})
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE "PX4/src/modules/ekf2" "PX4/src/modules/ekf2/EKF" "PX4/src/modules/ekf2/EKF/python")
set_source_files_properties(${FC_EKF2_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"ekf2\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align")

# battery_simulator: real PX4 module (replaces the fc_battery hand-copy).
# Publishes battery_status uORB (Battery class). Runs in a WQ thread.
# ---
# battery_simulator: gercek PX4 modulu (fc_battery elle-kopyasinin yerine).
# battery_status uORB'unu yayinlar (Battery sinifi). Bir WQ thread'inde calisir.
set(FC_BATTERY_SIM_SRCS
    "PX4/src/modules/simulation/battery_simulator/BatterySimulator.cpp"
    "PX4/src/lib/battery/battery.cpp")
target_sources(${COMPONENT_LIB} PRIVATE ${FC_BATTERY_SIM_SRCS})
set_source_files_properties(${FC_BATTERY_SIM_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"battery_simulator\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers")

# mc_pos_control module (real PX4 MulticopterPositionControl): vehicle_local_position(ekf2) +
# trajectory_setpoint(FMM) + vehicle_control_mode(commander) -> vehicle_attitude_setpoint (q_d+thrust).
# Real module replaces hand-glue (fc_flight_control.c + fc_pos_control_px4.cpp), which rewrote
# Run() incompletely (missing adjustSetpointForEKFResets -> EKF-reset transients). PositionControl/
# ControlMath/Takeoff classes already compiled (byte-identical to the module copy -> no ODR).
# New here: module wrapper + GotoControl. Spawned as a Core0 WQ-task (active control chain).
# ---
# mc_pos_control modulu (gercek PX4 MulticopterPositionControl): vehicle_local_position(ekf2) +
# trajectory_setpoint(FMM) + vehicle_control_mode(commander) -> vehicle_attitude_setpoint (q_d+itki).
# Gercek modul elle-tutkalin (fc_flight_control.c + fc_pos_control_px4.cpp) yerine gecer; o tutkal
# Run()'i eksik yeniden-yazmisti (adjustSetpointForEKFResets eksik -> EKF-reset gecici durumlari). PositionControl/
# ControlMath/Takeoff siniflari zaten derlendi (modul kopyasiyla byte-ozdes -> ODR yok).
# Burada yeni: modul sarmalayici + GotoControl. Core0 WQ-gorevi olarak spawn edilir (aktif kontrol zinciri).
set(FC_MC_POS_CTRL_SRCS
    "PX4/src/modules/mc_pos_control/MulticopterPositionControl.cpp"
    "PX4/src/modules/mc_pos_control/GotoControl/GotoControl.cpp")
target_sources(${COMPONENT_LIB} PRIVATE ${FC_MC_POS_CTRL_SRCS})
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE
    "PX4/src/modules/mc_pos_control"
    "PX4/src/modules/mc_pos_control/GotoControl")
set_source_files_properties(${FC_MC_POS_CTRL_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"mc_pos_control\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align")

# mc_att_control (real PX4): vehicle_attitude(ekf2) + vehicle_attitude_setpoint(mc_pos_control) ->
# vehicle_rates_setpoint. AttitudeControl class already compiled (lib). Replaces fc_control.cpp
# att-chain glue. Spawned as a Core0 WQ-task (active).
# ---
# mc_att_control (gercek PX4): vehicle_attitude(ekf2) + vehicle_attitude_setpoint(mc_pos_control) ->
# vehicle_rates_setpoint. AttitudeControl sinifi zaten derlendi (lib). fc_control.cpp tutum-zinciri
# tutkalinin yerine. Core0 WQ-gorevi olarak spawn edilir (aktif).
set(FC_MC_ATT_SRCS "PX4/src/modules/mc_att_control/mc_att_control_main.cpp")
target_sources(${COMPONENT_LIB} PRIVATE ${FC_MC_ATT_SRCS})
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE
    "PX4/src/modules/mc_att_control" "PX4/src/modules/mc_att_control/AttitudeControl")
set_source_files_properties(${FC_MC_ATT_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"mc_att_control\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align")

# mc_rate_control (real PX4): vehicle_angular_velocity(sensors) + vehicle_rates_setpoint(mc_att_control)
# -> vehicle_torque_setpoint + vehicle_thrust_setpoint. rate_control class already compiled. Spawned (active).
# ---
# mc_rate_control (gercek PX4): vehicle_angular_velocity(sensors) + vehicle_rates_setpoint(mc_att_control)
# -> vehicle_torque_setpoint + vehicle_thrust_setpoint. rate_control sinifi zaten derlendi. Spawn edilir (aktif).
set(FC_MC_RATE_SRCS "PX4/src/modules/mc_rate_control/MulticopterRateControl.cpp")
target_sources(${COMPONENT_LIB} PRIVATE ${FC_MC_RATE_SRCS})
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE "PX4/src/modules/mc_rate_control")
set_source_files_properties(${FC_MC_RATE_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"mc_rate_control\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align")

# control_allocator (real PX4 ControlAllocator): vehicle_torque_setpoint + vehicle_thrust_setpoint
# (mc_rate_control) -> actuator_motors. Replaces fc_control_alloc_px4.cpp glue. ControlAllocation/
# PseudoInverse/SequentialDesaturation classes already compiled (top SRCS) -> not re-added here
# (would duplicate-symbol). New: module (ControlAllocator + ActuatorGroupPreflightCheck) +
# VehicleActuatorEffectiveness lib (15 effectiveness classes + RpmControl) + ActuatorEffectiveness base.
# CA_* params present in px4_parameters.hpp enum (compile-safe). Spawned as a Core0 WQ-task (active).
# ---
# control_allocator (gercek PX4 ControlAllocator): vehicle_torque_setpoint + vehicle_thrust_setpoint
# (mc_rate_control) -> actuator_motors. fc_control_alloc_px4.cpp tutkalinin yerine. ControlAllocation/
# PseudoInverse/SequentialDesaturation siniflari zaten derlendi (ust SRCS) -> burada yeniden-eklenmez
# (cift-sembol olurdu). Yeni: modul (ControlAllocator + ActuatorGroupPreflightCheck) +
# VehicleActuatorEffectiveness kutuphanesi (15 etkinlik sinifi + RpmControl) + ActuatorEffectiveness tabani.
# CA_* parametreleri px4_parameters.hpp enum'unda mevcut (derleme-guvenli). Core0 WQ-gorevi olarak spawn edilir (aktif).
set(FC_CA_SRCS
    "PX4/src/modules/control_allocator/ControlAllocator.cpp"
    "PX4/src/modules/control_allocator/ActuatorGroupPreflightCheck.cpp"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessUUV.cpp"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessControlSurfaces.cpp"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessCustom.cpp"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessFixedWing.cpp"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHelicopter.cpp"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessHelicopterCoaxial.cpp"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessMCTilt.cpp"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessMultirotor.cpp"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessTilts.cpp"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.cpp"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessSpacecraft.cpp"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessStandardVTOL.cpp"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessTiltrotorVTOL.cpp"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessTailsitterVTOL.cpp"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness/RpmControl.cpp"
    "PX4/src/lib/control_allocation/actuator_effectiveness/ActuatorEffectiveness.cpp")
target_sources(${COMPONENT_LIB} PRIVATE ${FC_CA_SRCS})
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE
    "PX4/src/modules/control_allocator"
    "PX4/src/modules/control_allocator/VehicleActuatorEffectiveness"
    "PX4/src/lib/control_allocation/actuator_effectiveness"
    "PX4/src/lib/slew_rate")
set_source_files_properties(${FC_CA_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"control_allocator\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align")

# rc_update (real PX4 RCUpdate): input_rc -> rc_channels + manual_control_setpoint/switches.
# SRCS=rc_update.cpp; DEPENDS hysteresis+mathlib+px4_work_queue already compiled. RC_* params
# already in px4_parameters.hpp autogen -> no separate *_params.c (mc_pos_control/ekf2 pattern).
# Entry: rc_update_main (ModuleBase). Spawned; same pattern as other MC modules.
# ---
# rc_update (gercek PX4 RCUpdate): input_rc -> rc_channels + manual_control_setpoint/switches.
# SRCS=rc_update.cpp; BAGIMLILIKLAR hysteresis+mathlib+px4_work_queue zaten derlendi. RC_* parametreleri
# zaten px4_parameters.hpp autogen'de -> ayri *_params.c yok (mc_pos_control/ekf2 deseni).
# Giris: rc_update_main (ModuleBase). Spawn edilir; diger MC modulleriyle ayni desen.
set(FC_RC_UPDATE_SRCS "PX4/src/modules/rc_update/rc_update.cpp")
target_sources(${COMPONENT_LIB} PRIVATE ${FC_RC_UPDATE_SRCS})
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE "PX4/src/modules/rc_update")
set_source_files_properties(${FC_RC_UPDATE_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"rc_update\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align")

# manual_control (real PX4 ManualControl): manual_control_setpoint(rc_update/mavlink) ->
# action_request + manual_control_setpoint(selected source) + landing_gear.
# SRCS=ManualControl.cpp+ManualControlSelector.cpp; DEPENDS hysteresis+px4_work_queue already
# compiled. COM_*/MAN_*/RC_MAP_PAY_SW params in px4_parameters.hpp autogen -> no *_params.c.
# Entry: manual_control_main. Spawned (active).
# ---
# manual_control (gercek PX4 ManualControl): manual_control_setpoint(rc_update/mavlink) ->
# action_request + manual_control_setpoint(secilen kaynak) + landing_gear.
# SRCS=ManualControl.cpp+ManualControlSelector.cpp; BAGIMLILIKLAR hysteresis+px4_work_queue zaten
# derlendi. COM_*/MAN_*/RC_MAP_PAY_SW parametreleri px4_parameters.hpp autogen'de -> *_params.c yok.
# Giris: manual_control_main. Spawn edilir (aktif).
set(FC_MANUAL_CONTROL_SRCS
    "PX4/src/modules/manual_control/ManualControl.cpp"
    "PX4/src/modules/manual_control/ManualControlSelector.cpp")
target_sources(${COMPONENT_LIB} PRIVATE ${FC_MANUAL_CONTROL_SRCS})
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE "PX4/src/modules/manual_control")
set_source_files_properties(${FC_MANUAL_CONTROL_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"manual_control\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align")

# mc_hover_thrust_estimator (real PX4 MulticopterHoverThrustEstimator): vehicle_local_position +
# vehicle_acceleration -> hover_thrust_estimate. DEPENDS hysteresis+mathlib+px4_work_queue already
# compiled + zero_order_hover_thrust_ekf (local lib, added in this block). HTE_*/MPC_THR_HOVER
# params in px4_parameters.hpp autogen -> no *_params.c. Entry: mc_hover_thrust_estimator_main. Spawned.
# ---
# mc_hover_thrust_estimator (gercek PX4 MulticopterHoverThrustEstimator): vehicle_local_position +
# vehicle_acceleration -> hover_thrust_estimate. BAGIMLILIKLAR hysteresis+mathlib+px4_work_queue zaten
# derlendi + zero_order_hover_thrust_ekf (yerel lib, bu blokta eklendi). HTE_*/MPC_THR_HOVER
# parametreleri px4_parameters.hpp autogen'de -> *_params.c yok. Giris: mc_hover_thrust_estimator_main. Spawn edilir.
set(FC_HTE_SRCS
    "PX4/src/modules/mc_hover_thrust_estimator/MulticopterHoverThrustEstimator.cpp"
    "PX4/src/modules/mc_hover_thrust_estimator/zero_order_hover_thrust_ekf.cpp")  # DEPENDS zero_order_hover_thrust_ekf (local lib) | zero_order_hover_thrust_ekf'e BAGLIDIR (yerel lib)
target_sources(${COMPONENT_LIB} PRIVATE ${FC_HTE_SRCS})
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE "PX4/src/modules/mc_hover_thrust_estimator")
set_source_files_properties(${FC_HTE_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"mc_hover_thrust_estimator\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align")

# mc_autotune_attitude_control (real PX4): vehicle_torque_setpoint + vehicle_angular_velocity ->
# autotune_attitude_control_status (PID auto-tuning). SRCS=mc_autotune_attitude_control.cpp;
# DEPENDS hysteresis+mathlib+px4_work_queue already compiled + SystemIdentification (added in this
# block; pid_design header-only via lib include root). MC_AT_*/MC_*RATE_*/IMU_GYRO_CUTOFF params
# in px4_parameters.hpp autogen -> no *_params.c. Entry: mc_autotune_attitude_control_main. Spawned.
# ---
# mc_autotune_attitude_control (gercek PX4): vehicle_torque_setpoint + vehicle_angular_velocity ->
# autotune_attitude_control_status (PID oto-ayarlama). SRCS=mc_autotune_attitude_control.cpp;
# BAGIMLILIKLAR hysteresis+mathlib+px4_work_queue zaten derlendi + SystemIdentification (bu blokta
# eklendi; pid_design lib include kokunden yalnizca-baslik). MC_AT_*/MC_*RATE_*/IMU_GYRO_CUTOFF parametreleri
# px4_parameters.hpp autogen'de -> *_params.c yok. Giris: mc_autotune_attitude_control_main. Spawn edilir.
set(FC_MC_AUTOTUNE_SRCS
    "PX4/src/modules/mc_autotune_attitude_control/mc_autotune_attitude_control.cpp"
    "PX4/src/lib/system_identification/system_identification.cpp")  # DEPENDS SystemIdentification (lib) | SystemIdentification'a BAGLIDIR (lib)
target_sources(${COMPONENT_LIB} PRIVATE ${FC_MC_AUTOTUNE_SRCS})
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE
    "PX4/src/modules/mc_autotune_attitude_control"
    "PX4/src/lib/system_identification"
    "PX4/src/lib/pid_design")
set_source_files_properties(${FC_MC_AUTOTUNE_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"mc_autotune_attitude_control\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align")

# pwm_out_sim (real PX4 PWMSim) — verbatim output driver: converts control_allocator's
# actuator_motors to actuator_outputs_sim via MixingOutput. simulator_mavlink::send()
# (SimulatorMavlink.cpp:1280 orb_subscribe_multi(actuator_outputs_sim)) reads it and sends
# HIL_ACTUATOR_CONTROLS (#93) to Gazebo -> last link of the output chain. New lib = mixer_module
# (MixingOutput + OutputModuleInterface + ActuatorTest + header-only Function* providers).
# CRITICAL: <mixer_module/output_functions.hpp> (OutputFunction enum) is GENERATED via PX4
# generate_function_header.py into main/fc_px4_gen_stub/mixer_module/ (PX4/ kept read-only) ->
# resolved by -isystem fc_px4_gen_stub. Spawned in fc_px4_modules.cpp (pwm_out_sim_main).
# ---
# pwm_out_sim (gercek PX4 PWMSim) — verbatim cikis surucusu: control_allocator'in
# actuator_motors'unu MixingOutput ile actuator_outputs_sim'e cevirir. simulator_mavlink::send()
# (SimulatorMavlink.cpp:1280 orb_subscribe_multi(actuator_outputs_sim)) onu okur ve Gazebo'ya
# HIL_ACTUATOR_CONTROLS (#93) gonderir -> cikis zincirinin son halkasi. Yeni lib = mixer_module
# (MixingOutput + OutputModuleInterface + ActuatorTest + yalnizca-baslik Function* saglayicilari).
# KRITIK: <mixer_module/output_functions.hpp> (OutputFunction enum) PX4 generate_function_header.py
# ile main/fc_px4_gen_stub/mixer_module/'a URETILIR (PX4/ salt-okunur tutuldu) ->
# -isystem fc_px4_gen_stub ile cozulur. fc_px4_modules.cpp'de spawn edilir (pwm_out_sim_main).
set(FC_PWM_SIM_SRCS
    "PX4/src/lib/mixer_module/mixer_module.cpp"
    "PX4/src/lib/mixer_module/actuator_test.cpp"
    "PX4/src/modules/simulation/pwm_out_sim/PWMSim.cpp")
target_sources(${COMPONENT_LIB} PRIVATE ${FC_PWM_SIM_SRCS})
target_include_directories(${COMPONENT_LIB} SYSTEM PRIVATE
    "PX4/src/lib/mixer_module"
    "PX4/src/lib/mixer_module/functions"
    "PX4/src/modules/simulation/pwm_out_sim")
set_source_files_properties(${FC_PWM_SIM_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"pwm_out_sim\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-isystem;${CMAKE_CURRENT_LIST_DIR}/fc_px4_gen_stub;-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align")

# PX4 work-queue scheduler (Core0). Real PX4 WQ -> modules run in PX4's own work-queue instead
# of manual ticks. CRITICAL: the sensors module's private VehicleIMU sub-WorkItems (sensors.cpp:447
# new VehicleIMU) only run under the WQ -> full sensor_*->vehicle_imu->ekf2 pipeline. ScheduleNow/
# ScheduleOnInterval -> WorkQueueManager -> WorkQueueRunner threads (POSIX pthread). Until the
# manager starts, _wq is null -> ScheduleNow no-op (additive; engages once fc_core boot starts it).
# ---
# PX4 is-kuyrugu zamanlayicisi (Core0). Gercek PX4 WQ -> moduller manuel tick yerine PX4'un kendi
# is-kuyrugunda calisir. KRITIK: sensors modulunun ozel VehicleIMU alt-WorkItem'lari (sensors.cpp:447
# new VehicleIMU) yalnizca WQ altinda calisir -> tam sensor_*->vehicle_imu->ekf2 boru hatti. ScheduleNow/
# ScheduleOnInterval -> WorkQueueManager -> WorkQueueRunner thread'leri (POSIX pthread). Yonetici
# baslayana kadar _wq null'dur -> ScheduleNow no-op (eklemeli; fc_core boot onu baslattiginda devreye girer).
set(FC_PX4_WQ_SRCS
    "PX4/platforms/common/px4_work_queue/WorkQueueManager.cpp"
    "PX4/platforms/common/px4_work_queue/WorkQueue.cpp"
    "PX4/platforms/common/px4_work_queue/WorkItem.cpp"
    "PX4/platforms/common/px4_work_queue/ScheduledWorkItem.cpp"
    "PX4/platforms/common/px4_work_queue/WorkItemSingleShot.cpp")
target_sources(${COMPONENT_LIB} PRIVATE ${FC_PX4_WQ_SRCS})
set_source_files_properties(${FC_PX4_WQ_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"wq\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-include;${CMAKE_CURRENT_LIST_DIR}/fc_wq_pthread_compat.h;-Wno-missing-field-initializers;-Wno-cast-align")

# PX4 hrt (high-res timer) — used by ScheduledWorkItem ScheduleDelayed/OnInterval and uORB
# SubscriptionInterval (hrt_absolute_time/hrt_call_after/every/cancel). POSIX hrt (timer-thread +
# callback-queue), first linked in this build. + px4_getopt (module argv -h/-u parsing).
# fc_px4_hrt_latency.c does NOT provide hrt (it only measures latency).
# ---
# PX4 hrt (yuksek-cozunurluklu zamanlayici) — ScheduledWorkItem ScheduleDelayed/OnInterval ve uORB
# SubscriptionInterval tarafindan kullanilir (hrt_absolute_time/hrt_call_after/every/cancel). POSIX hrt
# (zamanlayici-thread + geri-cagirma-kuyrugu), bu derlemede ilk kez baglanir. + px4_getopt (modul argv -h/-u ayristirma).
# fc_px4_hrt_latency.c hrt SAGLAMAZ (yalnizca gecikmeyi olcer).
set(FC_PX4_HRT_SRCS
    "PX4/platforms/posix/src/px4/common/drv_hrt.cpp"
    # Legacy hrt dispatcher (hrt_thread.c/hrt_queue.c/hrt_work_cancel.c) replaced by
    # fc_sched/fc_hrt_esp_timer.c (esp_timer one-shot, 1us hardware). Those 3 files also defined
    # hrt_work_queue_init/queue/cancel, so all 3 are dropped to avoid double-definition link errors.
    # drv_hrt.cpp (callout_queue sq_* + _hrt_lock + hrt_tim_isr) stays verbatim; only the hrt_work_*
    # backend moved from a POSIX-emulation thread to the native timer. g_hrt_work/_hrt_work_lock were
    # used only by the deleted files (hrt_work.h inlines are static -> no emit) -> no dangling ref.
    # ---
    # Eski hrt dagiticisi (hrt_thread.c/hrt_queue.c/hrt_work_cancel.c) fc_sched/fc_hrt_esp_timer.c
    # (esp_timer tek-atim, 1us donanim) ile degistirildi. Bu 3 dosya ayrica hrt_work_queue_init/queue/
    # cancel'i tanimliyordu, bu yuzden cift-tanim baglanti hatalarindan kacinmak icin 3'u de dusuruldu.
    # drv_hrt.cpp (callout_queue sq_* + _hrt_lock + hrt_tim_isr) verbatim kalir; yalnizca hrt_work_*
    # arka ucu POSIX-emulasyon thread'inden native zamanlayiciya tasindi. g_hrt_work/_hrt_work_lock
    # yalnizca silinen dosyalarca kullaniliyordu (hrt_work.h inline'lari static -> emit yok) -> sarkan referans yok.
    "fc_sched/fc_hrt_esp_timer.c"
    # drv_hrt callout_queue (sq_*) helpers: sq_addafter (drv_hrt:229) + sq_addlast (from sq_addafter.c:63).
    # Others orphaned and dropped (sq_remfirst, dq_addlast/dq_rem/dq_remfirst — only used by the
    # deleted dispatcher + never-compiled generic work_queue.c/work_thread.c; grep-verified).
    # PX4/ read-only: files stay on disk, just excluded from the build.
    # ---
    # drv_hrt callout_queue (sq_*) yardimcilari: sq_addafter (drv_hrt:229) + sq_addlast (sq_addafter.c:63'ten).
    # Digerleri oksuz kaldi ve dusuruldu (sq_remfirst, dq_addlast/dq_rem/dq_remfirst — yalnizca silinen
    # dagitici + hic-derlenmeyen genel work_queue.c/work_thread.c tarafindan kullanildi; grep-dogrulandi).
    # PX4/ salt-okunur: dosyalar diskte kalir, yalnizca derlemeden haric tutulur.
    "PX4/platforms/common/work_queue/sq_addafter.c"
    "PX4/platforms/common/work_queue/sq_addlast.c"
    "PX4/platforms/common/px4_getopt.c")
target_sources(${COMPONENT_LIB} PRIVATE ${FC_PX4_HRT_SRCS})
set_source_files_properties(${FC_PX4_HRT_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"hrt\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    COMPILE_OPTIONS "-I;${CMAKE_CURRENT_LIST_DIR}/PX4/platforms/posix/include;-include;errno.h;-include;stdarg.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align")

# ============================================================================
# Real PX4 param backend. Replaces fc_px4_params (table) + fc_param_protocol (custom store +
# param_* C-API) + fc_px4_mavlink_glue (introspection bridge). parameters.cpp #includes
# parameters_common.cpp (not added separately). Firmware defaults from px4_parameters.hpp
# autogen (tools/gen_px4_params.py --from-existing; param_info_s parameters[]).
# Store = ConstLayer(default) <- DynamicSparseLayer(runtime) <- DynamicSparseLayer(user).
# RAM-only persistence (param_get_default_file=NULL -> flash-stub no-op); autosave silenced in
# fc_px4_modules via param_control_autosave(false). Deps already compiled; only tinybson.cpp is new.
# ---
# Gercek PX4 param arka ucu. fc_px4_params (tablo) + fc_param_protocol (ozel depo +
# param_* C-API) + fc_px4_mavlink_glue (introspeksiyon koprusu) yerine gecer. parameters.cpp,
# parameters_common.cpp'yi #include eder (ayri eklenmez). Firmware varsayilanlari px4_parameters.hpp
# autogen'den (tools/gen_px4_params.py --from-existing; param_info_s parameters[]).
# Depo = ConstLayer(varsayilan) <- DynamicSparseLayer(calisma-zamani) <- DynamicSparseLayer(kullanici).
# Yalnizca-RAM kalicilik (param_get_default_file=NULL -> flash-stub no-op); autosave, fc_px4_modules'ta
# param_control_autosave(false) ile susturulur. Bagimliliklar zaten derlendi; yalnizca tinybson.cpp yeni.
# ============================================================================
set(FC_PARAM_SRCS
    "PX4/src/lib/parameters/parameters.cpp"          # #includes parameters_common.cpp | parameters_common.cpp'yi #include eder
    "PX4/src/lib/parameters/param_translation.cpp"   # param_modify_on_import (import compat) | param_modify_on_import (import uyumlulugu)
    "PX4/src/lib/parameters/atomic_transaction.cpp"  # AtomicTransaction (lock) | AtomicTransaction (kilit)
    "PX4/src/lib/parameters/autosave.cpp"            # ParamAutosave (ScheduledWorkItem; passive in RAM-only) | ParamAutosave (ScheduledWorkItem; yalnizca-RAM'de pasif)
    "PX4/src/lib/tinybson/tinybson.cpp")             # parameters.cpp BSON export/import dep | parameters.cpp BSON disa/ice-aktarim bagimliligi
target_sources(${COMPONENT_LIB} PRIVATE ${FC_PARAM_SRCS})
set_source_files_properties(${FC_PARAM_SRCS} PROPERTIES
    COMPILE_DEFINITIONS "__PX4_POSIX;__PX4_ESPIDF;MODULE_NAME=\"param\";CONFIG_BOARD_ROOT_PATH=\"/sdcard\""
    # -isystem fc_px4_gen_stub: for parameters.cpp <px4_platform_common/micro_hal.h> (board-HW stub, our dir).
    # -include sys/stat.h: param_save_default/export ::open() needs S_IRUSR/S_IWUSR/... POSIX mode constants.
    # ---
    # -isystem fc_px4_gen_stub: parameters.cpp <px4_platform_common/micro_hal.h> icin (board-HW stub, bizim dizin).
    # -include sys/stat.h: param_save_default/export ::open() S_IRUSR/S_IWUSR/... POSIX mod sabitlerine ihtiyac duyar.
    COMPILE_OPTIONS "-isystem;${CMAKE_CURRENT_LIST_DIR}/fc_px4_gen_stub;-include;errno.h;-include;stdarg.h;-include;sys/stat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/px4_compat/px4_ioctl_compat.h;-include;${CMAKE_CURRENT_LIST_DIR}/PX4/src/lib/matrix/px4_platform_common/time.h;-Wno-missing-field-initializers;-Wno-cast-align")

# LittleFS image | LittleFS imaji
littlefs_create_partition_image(littlefs ../littlefs_image FLASH_IN_PROJECT)
