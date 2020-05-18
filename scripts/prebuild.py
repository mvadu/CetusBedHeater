import json
import os
import re
import time
from pathlib import Path
from datetime import datetime

#######update the version info##########


def update_version(line):
    versioninfo = re.search(
        "\{\s*(?P<version>.*Minor:\s+(?P<minor>\d+).*BuildTime:\s+(?P<buildtime>\S+))",
        line,
    )
    version = versioninfo.group("version")

    line = line.replace(
        "{},".format(versioninfo.group("minor")),
        "{},".format(int(versioninfo.group("minor")) + 1),
    )
    line = line.replace(
        versioninfo.group("buildtime"), '"{}"'.format(datetime.utcnow().isoformat())
    )
    versioninfo = re.search(
        "\{\s*(?P<version>.*Minor:\s+(?P<minor>\d+).*BuildTime:\s+(?P<buildtime>\S+))",
        line,
    )
    print("Updated Version {} ==> {}".format(version, versioninfo.group("version")))
    return line


def update_versionInfo():
    filename = os.path.join(env["PROJECTSRC_DIR"], "versioninfo.h")
    with open(filename, "r") as f:
        lines = (line.rstrip() for line in f)
        altered_lines = [
            update_version(line) if re.search("^const VersionInfo", line) else line
            for line in lines
        ]
    with open(filename, "w") as f:
        f.write("\n".join(altered_lines) + "\n")


# with open(os.path.join(os.environ['TEMP'], 'env.json')) as json_file:
#    env = json.load(json_file)
Import("env")
print(os.getcwd())
target = os.path.join(env["PROJECT_BUILD_DIR"], env["PIOENV"], "firmware.elf")
print("Project Target: %s" % target)

if os.path.isfile(target):
    compressHtml = False
    updateVersion = False
    targetModified = os.path.getmtime(target)
    with os.scandir(os.path.join(env["PROJECT_DIR"], "static")) as files:
        for entry in files:
            if entry.name.endswith(".html") and entry.is_file():
                if os.path.getmtime(entry.path) > targetModified:
                    compressHtml = True
    if compressHtml:
        os.system(
            'powershell.exe -ExecutionPolicy Bypass -Command "& Get-childitem ./static -filter *.html | .\scripts\compress-file-str.ps1 -target .\src\web_pages.h"'
        )
    else:
        print(
            "static content not modified since {}".format(
                time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(targetModified))
            )
        )
    all_files = []
    for ext in env["CPPSUFFIXES"]:
        all_files.extend(Path(env["PROJECTSRC_DIR"]).glob("**/*{}".format(ext)))
        all_files.extend(Path(env["LIBSOURCE_DIRS"][0]).glob("**/*{}".format(ext)))
    for entry in all_files:
        if os.path.getmtime(entry) > targetModified:
            updateVersion = True
    if updateVersion:
        update_versionInfo()
    else:
        print(
            "source files not modified since {}".format(
                time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(targetModified))
            )
        )
else:
    os.system(
        'powershell.exe -ExecutionPolicy Bypass -Command "& Get-childitem ./static -filter *.html | .\scripts\compress-file-str.ps1 -target .\src\web_pages.h"'
    )
    update_versionInfo()
