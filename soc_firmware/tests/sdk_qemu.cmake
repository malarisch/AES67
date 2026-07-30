# Point Zephyr's emulator resolution at the SDK hosttools' QEMU, the
# same way the application's CMakeLists does — the SDK is unpacked into
# the west workspace (external/zephyr/toolchains) rather than onto PATH.
# A caller-provided QEMU_BIN_PATH always wins.
#
# Include this BEFORE find_package(Zephyr).

if(NOT DEFINED ENV{QEMU_BIN_PATH})
    file(GLOB _sdk_qemu_bins
         "${CMAKE_CURRENT_LIST_DIR}/../../external/zephyr/toolchains/zephyr-sdk-*/sysroots/*/usr/bin")
    foreach(_dir ${_sdk_qemu_bins})
        if(EXISTS "${_dir}/qemu-system-riscv64")
            set(ENV{QEMU_BIN_PATH} "${_dir}")
            break()
        endif()
    endforeach()
endif()
