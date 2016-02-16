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

import os
from os import path


def list_all_runs():
    save_dir = '/usr/local/jim_local/Dropbox/move3d/catkin_ws_move3d/src/'
    save_dir += 'hrics-or-rafi/python_module/'
    save_dir += 'bioik/user_human_robot_experiment/tro_experiment'

    blocks = [x for x in os.listdir(save_dir)
              if not path.isfile(save_dir + os.sep + x)]

    found = int(0)

    for b in blocks:

        block_dir = save_dir + os.sep + b
        run_ids = sorted([x for x in os.listdir(block_dir)
                         if not path.isfile(block_dir + os.sep + x)])
        for run in run_ids:

            id_dir = block_dir + os.sep + run

            human_run_dir = id_dir + os.sep + "human"
            robot_run_dir = id_dir + os.sep + "robot"
            if (os.path.exists(human_run_dir) and
                    os.path.exists(robot_run_dir)):

                human_trajs = [x for x in os.listdir(human_run_dir)
                               if (human_run_dir +
                                   os.sep + x).endswith('.traj')]

                robot_trajs = [x for x in os.listdir(robot_run_dir)
                               if (robot_run_dir +
                                   os.sep + x).endswith('.traj')]

                if len(human_trajs) != len(robot_trajs):
                    print "not the same number of trajs!!!"
                    print(' ** block : {b}'.format(**locals()))
                    print(' ** run : {run}'.format(**locals()))
                    print(' ** human_trajs nb : {}'.format(len(human_trajs)))
                    print(' ** robot_trajs nb : {}'.format(len(robot_trajs)))
                else:
                    if len(human_trajs) > 1:
                        print('blocks : {b}, run id: {run}'.format(**locals()))
                        found += 1
                        #for t in robot_trajs:
                        #    print t[0:11]
                        # if len(human_trajs) != 18:
                        #     print('-- human_trajs nb : {}'.format(
                        #         len(human_trajs)))
                        #     print('-- robot_trajs nb : {}'.format(
                        #         len(robot_trajs)))
    return found


# run the server
if __name__ == "__main__":
    nb_of_run_found = list_all_runs()
    print('nb_of_run_found : {nb_of_run_found}'.format(**locals()))
    print("generate_all_splits Done!!")
