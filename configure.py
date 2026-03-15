import os

# detect os ( linux or windows )
if os.name == "nt":
    os_name = "windows"
else:
    os_name = "linux"


if os_name == "windows":    
    # Auto-detect common MinGW / MSYS2 locations and the user's scoop install
    _candidates = [
        os.path.expandvars(r"%USERPROFILE%\scoop\apps\mingw\current\bin"),
        r"C:\msys64\mingw64\bin",
        r"C:\mingw64\bin",
        r"C:\MinGW\bin",
    ]

    for p in _candidates:
        gcc = os.path.join(p, "gcc.exe")
        if os.path.isfile(gcc):
            env = DefaultEnvironment()
            cur = env["ENV"].get("PATH", "")
            env["ENV"]["PATH"] = p + os.pathsep + cur
            break

if os_name == "linux":
    Import("env")

    env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)
    env.Replace(COMPILATIONDB_PATH=os.path.join("$BUILD_DIR", "compile_commands.json"))