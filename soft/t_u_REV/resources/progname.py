Import("env")

import subprocess

version = subprocess.check_output("tail -n1 'TU_version.h'|tr -d '\"'", shell=True).decode().strip()
git_rev = subprocess.check_output("git rev-parse --short HEAD", shell=True).decode().strip()

env.Replace(PROGNAME=f"temps_utile-{version}-{git_rev}")
