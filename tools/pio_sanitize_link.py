# PlatformIO extra script for [env:native_sanitize]: link with the same
# sanitizer runtimes the sources were compiled with.
Import("env")  # noqa: F821  (provided by PlatformIO/SCons)

env.Append(LINKFLAGS=["-fsanitize=address,undefined", "-fno-sanitize-recover=all"])  # noqa: F821
