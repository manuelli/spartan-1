import subprocess
import os
import sys
import time
import spartan.utils.utils as spartanUtils


def start_external_force_monitor():
    global external_force_monitor
    monitor_script_name = os.path.join(spartanUtils.getSpartanSourceDir(), 'yunzhu', 'scripts', 'external_force_monitor.py')

    cmd = "python " + monitor_script_name
    external_force_monitor = subprocess.Popen(cmd, shell=True,
                                              stdout=subprocess.PIPE)

def stop_external_force_monitor(s='/external_force_monitor'):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)


start_external_force_monitor()
time.sleep(10)
stop_external_force_monitor()
