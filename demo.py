#!/usr/bin/env python

import argparse
import distutils.spawn
import os.path as osp
import shutil
import subprocess


here = osp.dirname(osp.abspath(__file__))


def demo_mask_fusion():
    cmd = '%s %s' % (osp.join(here, 'build/mask_fusion'), osp.join(here, 'data/masks'))
    subprocess.call(cmd, shell=True)

    out_file = osp.join(here, 'out_mask_fusion.ply')
    if distutils.spawn.find_executable('meshlab'):
        cmd = 'meshlab %s' % out_file
        subprocess.call(cmd, shell=True)
    else:
        print('Please install meshlab to view mesh file %s' % out_file)
        print('  sudo apt-get install meshlab')


def demo_label_fusion():
    cmd = '%s %s' % (osp.join(here, 'build/label_fusion'), osp.join(here, 'data/labels'))
    subprocess.call(cmd, shell=True)

    out_file = osp.join(here, 'out_label_fusion.ply')
    if distutils.spawn.find_executable('meshlab'):
        cmd = 'meshlab %s' % out_file
        subprocess.call(cmd, shell=True)
    else:
        print('Please install meshlab to view mesh file %s' % out_file)
        print('  sudo apt-get install meshlab')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('demo', choices=['mask_fusion', 'label_fusion'])
    args = parser.parse_args()

    if args.demo == 'mask_fusion':
        demo_mask_fusion()
    elif args.demo == 'label_fusion':
        demo_label_fusion()
    else:
        raise ValueError('Unsupported demo: %s' % args.demo)


if __name__ == '__main__':
    main()
