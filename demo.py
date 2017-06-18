#!/usr/bin/env python

import argparse
import distutils.spawn
import os.path as osp
import subprocess


here = osp.dirname(osp.abspath(__file__))


def demo(input):
    cmd = '%s %s' % (osp.join(here, 'build/%s_fusion' % input),
                     osp.join(here, 'data/%ss' % input))
    subprocess.call(cmd, shell=True)

    out_file = osp.join(here, 'out_%s_fusion.ply' % input)
    if distutils.spawn.find_executable('meshlab'):
        cmd = 'meshlab %s' % out_file
        subprocess.call(cmd, shell=True)
    else:
        print('Please install meshlab to view mesh file %s' % out_file)
        print('  sudo apt-get install meshlab')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input', choices=['mask', 'label'])
    args = parser.parse_args()
    demo(input=args.input)


if __name__ == '__main__':
    main()
