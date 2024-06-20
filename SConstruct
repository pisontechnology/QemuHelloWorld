# SConstruct
import os
import platform
import scons_compiledb as sc_db
from SCons.Script import (
    DefaultEnvironment,
    AddOption,
    Exit,
    Command,
    SConscript,
    GetOption,
)

cwd = os.getcwd()


def find_arm_toolchain():
    base_path = "/Applications/ArmGNUToolchain"
    if os.path.exists(base_path):
        # List all version directories and pick the latest
        versions = sorted(
            [
                d
                for d in os.listdir(base_path)
                if os.path.isdir(os.path.join(base_path, d))
            ]
        )
        if versions:
            latest_version = versions[-1]
            toolchain_bin_path = os.path.join(
                base_path, latest_version, "arm-none-eabi", "bin"
            )
            if os.path.exists(toolchain_bin_path):
                return toolchain_bin_path
    return None


qcc_env = DefaultEnvironment()
sc_db.enable(qcc_env)

toolchain_prefix = "arm-none-eabi-"  # Prefix for the cross-compiler tools
toolchain_bin_path = find_arm_toolchain()
print(toolchain_bin_path)

CROSS_COMPILER = "arm-linux-gcc"

# Ensure the toolchain bin directory is in the PATH
qcc_env.AppendENVPath("PATH", toolchain_bin_path)

qcc_env["AR"] = toolchain_prefix + "ar"
qcc_env["AS"] = toolchain_prefix + "as"
qcc_env["CC"] = toolchain_prefix + "gcc"
qcc_env["CXX"] = toolchain_prefix + "g++"
qcc_env["LINK"] = qcc_env["CXX"]
qcc_env["RANLIB"] = toolchain_prefix + "ranlib"
qcc_env["OBJCOPY"] = toolchain_prefix + "objcopy"
qcc_env["PROGSUFFIX"] = ".elf"

# Ensure the toolchain bin directory is in the PATH
qcc_env.AppendENVPath("PATH", toolchain_bin_path)

if not qcc_env.WhereIs(qcc_env["CC"]):
    # print(qcc_env.Dump())
    print(f"Cross-compiler '{qcc_env['CC']}' not found.")
    print("Please make sure it is installed and available in your PATH.")
    print("You may need to install the appropriate cross-compilation toolchain.")
    print("Aborting.")
    Exit(1)
include_path = os.path.join(cwd, "src", "inc")

###########
# GENERIC #
###########

#################
# GCC TOOLCHAIN #
#################
qcc_env.Append(
    CPPPATH=[
        include_path,
        # TODO make these generic
        "/Applications/ArmGNUToolchain/13.2.Rel1/arm-none-eabi/arm-none-eabi/include/c++/13.2.1",
        "/Applications/ArmGNUToolchain/13.2.Rel1/arm-none-eabi/arm-none-eabi/include/c++/13.2.1/arm-none-eabi/thumb/v8-m.main+fp/hard/",
        "/Applications/ArmGNUToolchain/13.2.Rel1/arm-none-eabi/lib/gcc/arm-none-eabi/13.2.1/include",
        "/Applications/ArmGNUToolchain/13.2.Rel1/arm-none-eabi/arm-none-eabi/include",
    ]
)

##########################
# EXTERNAL FREERTOS PATH #
##########################
# NOTE: STM32L552-EV includes its own flavor of FreeRTOS, if there's include errors coming from FreeRTOS files it's probably due to a duplicate include there

externals_path = os.path.join(cwd, "external")
free_rtos_path = os.path.join(externals_path, "FreeRTOS")
free_rtos_kernel_path = os.path.join(externals_path, "FreeRTOS-Kernel")
free_rtos_kernel_include_path = os.path.join(free_rtos_kernel_path, "include")
# free_rtos_kernel_portable_m33_path = os.path.join(
#     free_rtos_kernel_path, "portable/ARMv8M/non_secure/context/portable/GCC/ARM_CM33/"
# )
# free_rtos_kernel_ns_m33_path = os.path.join(
#     free_rtos_kernel_path, "portable/GCC/ARM_CM33/non_secure"
# )
# free_rtos_kernel_s_m33_path = os.path.join(
#     free_rtos_kernel_path, "portable/GCC/ARM_CM33/secure"
# )
free_rtos_kernel_gcc_m33_path = os.path.join(free_rtos_kernel_path, "portable/GCC/ARM_CM33_NTZ/non_secure")

qcc_env.Append(
    CPPPATH=[
        free_rtos_kernel_include_path,
        # free_rtos_kernel_ns_m33_path,
        # free_rtos_kernel_s_m33_path,
        # free_rtos_kernel_portable_m33_path,
        free_rtos_kernel_gcc_m33_path,
    ]
)

########################
# QEMU M33 ENVIRONMENT #
########################
qcc_env.Append(
    CPPDEFINES=[
        "DUAL_CORE",
        "CORE1",
        # "USE_STDPERIPH_DRIVER",
        # "USE_FULL_ASSERT",
    ]
)

qcc_env.Append(CXXFLAGS=["-fno-exceptions", "-fno-rtti", "-std=gnu++17"])

qcc_env.Append(
    CCFLAGS=[
        "-mthumb",
        "-mcpu=cortex-m33",
        "-mfloat-abi=hard",
        "-mfpu=fpv5-sp-d16",
        # "-mfloat-abi=soft",
        "-O0",
        "-ffunction-sections",
        "-fdata-sections",
        "-g",
        "-Wall",
        "-specs=nano.specs",
        # "-fno-common",
    ]
)

# Linker flags
qcc_env.Append(
    LINKFLAGS=[
        "-mthumb",
        "-mcpu=cortex-m33",
        "-mfloat-abi=hard",
        "-mfpu=fpv5-sp-d16",
        # "-mfloat-abi=soft",
        # "-specs=nosys.specs",
        "-static",
        "-Wl,-cref,-u,Reset_Handler",
        "-Wl,-Map=main.map",
        "-Wl,--gc-sections",
        "-Wl,--defsym=malloc_getpagesize_P=0x80",
        "-Wl,--start-group",
        "-lc",
        "-lm",
        "-Wl,--end-group",
        "-specs=nano.specs",
        "-Wl,--undefined=_gettimeofday",
        # "-TProject/Linker/" + PROCESSOR_DERIVATIVE + "_LINKER.ld",
    ]
)

# GENERIC SOURCE #
##################

free_rtos_portable_path = os.path.join(free_rtos_kernel_path, "portable", "MemMang")
generic_source = (
    Glob(os.path.join(free_rtos_kernel_path, "*.c")),
    # Glob(os.path.join(free_rtos_kernel_ns_m33_path, "*.c")),
    Glob(os.path.join(free_rtos_portable_path, "heap_4.c")),
)

###############
# QEMU SOURCE #
###############
cmsis_path = os.path.join(externals_path, "CMSIS_5")
cmsis_arm_m33_path = os.path.join(cmsis_path, "Device", "ARM", "ARMCM33")
cmsis_arm_m33_source_path = os.path.join(cmsis_arm_m33_path, "Source")
cmsis_arm_m33_gcc_path = os.path.join(cmsis_arm_m33_source_path, "GCC")

cmsis_template_path = os.path.join(
    cmsis_path, "Device", "_Template_Vendor", "Vendor", "Device_A"
)
cmsis_template_include_path = os.path.join(cmsis_template_path, "Include")

cmsis_core_path = os.path.join(cmsis_path, "CMSIS", "Core")
cmsis_core_include_path = os.path.join(cmsis_core_path, "Include")


vendor_source_path = os.path.join(externals_path, "vendor")
vendor_device_source_path = os.path.join(vendor_source_path, "Device")
vendor_device_include_path = os.path.join(vendor_device_source_path, "Include")

src_path = os.path.join(cwd, "src")
mock_path = os.path.join(cwd, "mock")

source = (
    Glob(os.path.join(src_path, "*.c"))
    + Glob(os.path.join(cmsis_arm_m33_gcc_path, "*.S"))
    + Glob(os.path.join(mock_path, "*.c"))
    + Glob(os.path.join(vendor_source_path, "*.c"))
    # + Glob(os.path.join(free_rtos_kernel_path, "*.c"))
    + Glob(os.path.join(free_rtos_kernel_gcc_m33_path, "*.c"))
    # + Glob(os.path.join(free_rtos_kernel_ns_m33_path, "*.c"))
    # + Glob(os.path.join(free_rtos_kernel_s_m33_path, "*.c"))
    # + Glob(os.path.join(free_rtos_kernel_portable_m33_path, "*.c"))
)

qcc_env.Append(
    CPPPATH=[
        cmsis_core_include_path,
        cmsis_template_include_path,
        mock_path,
        vendor_source_path,
        vendor_device_source_path,
        vendor_device_include_path,
        free_rtos_kernel_path,
        free_rtos_portable_path,
    ]
)

###################
# QEMU M33 LINKER #
###################
linker_script = "gcc_arm.ld"
linker_script_path = os.path.join(cmsis_arm_m33_gcc_path, linker_script)
qcc_env.Append(
    LINKFLAGS=["-T", linker_script_path, "-Wl,-Map=main.map", "-Wl,--gc-sections"]
)

for s in generic_source:
    source += s

# Build embedded executable
prg = qcc_env.Program(
    target="main",
    source=source,
)
