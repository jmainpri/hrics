#!/usr/bin/env python

# Copyright (c) 2015 Max Planck Institute
# All rights reserved.
#
# Permission to use, copy, modify, and distribute this software for any purpose
# with or without   fee is hereby granted, provided   that the above  copyright
# notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS  SOFTWARE INCLUDING ALL  IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS. IN NO EVENT SHALL THE AUTHOR  BE LIABLE FOR ANY SPECIAL, DIRECT,
# INDIRECT, OR CONSEQUENTIAL DAMAGES OR  ANY DAMAGES WHATSOEVER RESULTING  FROM
# LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
# OTHER TORTIOUS ACTION,   ARISING OUT OF OR IN    CONNECTION WITH THE USE   OR
# PERFORMANCE OF THIS SOFTWARE.
#
#                                           Jim Mainprice on Sunday May 17 2015

import sys
import subprocess
import os
import shlex
from os import path
from threading import Thread, Event


def kill_on_timeout(done, timeout, proc):
    if not done.wait(timeout):
        print "Stop PROCESS"
        os.system("pkill roslaunch")


def generate_one_split(block, run_id, bag):
    done = Event()

    command = str("roslaunch hrics_py human_robot_experiment_viz.launch")
    roslaunch = str(
        "{command} block:={block} run_id:={run_id} bagfile:={bag}").format(
        **locals())
    proc = subprocess.Popen(
        shlex.split(roslaunch),
        stdin=sys.stdin,
        stdout=sys.stdout)

    timeout = 400
    watcher = Thread(target=kill_on_timeout, args=(done, timeout, proc))
    watcher.daemon = True
    watcher.start()

    proc.wait()
    done.set()


def generate_all_splits():
    save_dir = '/usr/local/jim_local/Dropbox/move3d/catkin_ws_move3d/src/'
    save_dir += 'hrics-or-rafi/python_module/'
    save_dir += 'bioik/user_human_robot_experiment/tro_experiment'

    data_dir = '/usr/local/share/Experiments/'
    data_dir += 'human-robot-experiment-tro/20160114_tro_full.zip'

    blocks = [x for x in os.listdir(data_dir)
              if not path.isfile(data_dir + os.sep + x)]
    for b in blocks:
        block_dir = data_dir + os.sep + b
        run_id = sorted([x for x in os.listdir(block_dir) if
                         not path.isfile(block_dir + os.sep + x)])
        for id in run_id:
            id_dir = block_dir + os.sep + id
            times = [x for x in os.listdir(id_dir) if x == "Times.csv"]
            if len(times) == 1:
                bag = [x for x in os.listdir(id_dir)
                       if (id_dir + os.sep + x).endswith('.bag')]
                if len(bag) == 1:
                    print('blocks : {b}, run id: {id}, bag: {bag}'.format(
                        **locals()))
                    # Create directories
                    run_save_dir = save_dir + os.sep + b + os.sep + id
                    human_run_dir = run_save_dir + os.sep + "human"
                    robot_run_dir = run_save_dir + os.sep + "robot"
                    if not os.path.exists(human_run_dir):
                        os.makedirs(human_run_dir)
                    if not os.path.exists(robot_run_dir):
                        os.makedirs(robot_run_dir)

                    generate_one_split(b, id, bag[0])


# run the server
if __name__ == "__main__":
    generate_all_splits()
    print "generate_all_splits Done!!"
