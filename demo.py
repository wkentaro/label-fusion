#!/usr/bin/env python

import argparse
import distutils.spawn
import os.path as osp
import subprocess


here = osp.dirname(osp.abspath(__file__))


def demo(name):
    input_type = name.split('_')[0]

    cmd = '%s %s' % (osp.join(here, 'build/%s' % name),
                     osp.join(here, 'data/%ss' % input_type))
    subprocess.call(cmd, shell=True)

    out_file = osp.join(here, '%s.ply' % name)
    if distutils.spawn.find_executable('meshlab'):
        cmd = 'meshlab %s' % out_file
        subprocess.call(cmd, shell=True)
    else:
        print('Please install meshlab to view mesh file %s' % out_file)
        print('  sudo apt-get install meshlab')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('name', choices=['mask_view', 'mask_fusion',
                                         'label_view', 'label_fusion'])
    args = parser.parse_args()
    demo(name=args.name)


if __name__ == '__main__':
    main()
