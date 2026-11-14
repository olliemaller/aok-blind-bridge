# PlatformIO pre-build script: gzips ui/index.html into include/web_ui_gz.h
import gzip, os
try:
    Import("env")  # noqa: F821  (only defined inside PlatformIO)
    root = env["PROJECT_DIR"]  # noqa: F821
except NameError:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

src = os.path.join(root, "ui", "index.html")
dst = os.path.join(root, "include", "web_ui_gz.h")
raw = open(src, "rb").read()
gz = gzip.compress(raw, compresslevel=9, mtime=0)
lines = []
for i in range(0, len(gz), 20):
    lines.append("  " + ", ".join("0x%02x" % b for b in gz[i:i + 20]) + ",")
out = (
    "// Generated from ui/index.html by tools/embed_ui.py. Do not edit.\n"
    "#pragma once\n#include <pgmspace.h>\n#include <stddef.h>\n"
    "static const unsigned char WEB_UI_GZ[] PROGMEM = {\n" + "\n".join(lines) + "\n};\n"
    "static const size_t WEB_UI_GZ_LEN = %d;\n" % len(gz)
)
if not os.path.exists(dst) or open(dst).read() != out:
    open(dst, "w").write(out)
print("web UI: %d bytes, %d gzipped" % (len(raw), len(gz)))
