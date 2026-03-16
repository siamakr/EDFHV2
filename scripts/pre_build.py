"""
PlatformIO extra script — runs gen_config.py before every build.
Referenced in platformio.ini via:  extra_scripts = pre:pre_build.py
"""
Import("env")
import subprocess, sys

def generate_config(*args, **kwargs):
    script = env.subst("$PROJECT_DIR/gen_config.py")
    result = subprocess.run([sys.executable, script], capture_output=True, text=True)
    if result.stdout:
        print(result.stdout, end="")
    if result.returncode != 0:
        print(result.stderr)
        raise RuntimeError("gen_config.py failed — fix config.yaml before building")

env.AddPreAction("buildprog", generate_config)
