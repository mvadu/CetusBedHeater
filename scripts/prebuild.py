import os
import re
import time

print("executing pre build powershell command")
print(os.getcwd())
#print("Current build targets", map(str, BUILD_TARGETS))
os.system('powershell.exe -ExecutionPolicy Bypass -Command "& Get-childitem ./static -filter *.html | .\scripts\compress-file-str.ps1 -target .\src\web_pages.h"')



#######update the version info##########
def update_version(line):
    versioninfo = re.search('(?P<version>\{.*Minor:\s+(?P<minor>\d+).*BuildTime:\s+(?P<buildtime>\d+)\})',line)
    line = line.replace(versioninfo.group('minor'),str(int(versioninfo.group('minor'))+1))
    line = line.replace(versioninfo.group('buildtime'),str(int(time.time() * 1000)))
    return line


filename = ".\\src\\versioninfo.h"
with open(filename, "r") as f:
    lines = (line.rstrip() for line in f)
    altered_lines = [update_version(line) if re.search('^const VersionInfo',line) else line for line in lines]
with open(filename, "w") as f:
    f.write('\n'.join(altered_lines) + '\n')

