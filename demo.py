#!/usr/bin/env python

import argparse
import distutils.spawn
import os.path as osp
import subprocess


here = osp.dirname(osp.abspath(__file__))


def open_mesh(mesh_file):
    if distutils.spawn.find_executable('meshlab'):
        cmd = 'meshlab %s' % mesh_file
        subprocess.call(cmd, shell=True)
    else:
        print('Please install meshlab to view mesh file %s' % mesh_file)
        print('  sudo apt-get install meshlab')


def demo_fusion(name, use_depth):
    input_type = name.split('_')[0]

    exe = osp.join(here, 'build/%s' % name)
    data_path = osp.join(here, 'data/%ss' % input_type)
    if use_depth:
        cmd = '%s --depth %s' % (exe, data_path)
    else:
        cmd = '%s %s' % (exe, data_path)
    subprocess.call(cmd, shell=True)

    out_file = osp.join(here, '%s.ply' % name)
    open_mesh(out_file)


def demo_view(name):
    input_type = name.split('_')[0]

    cmd = '%s %s' % (osp.join(here, 'build/%s' % name),
                     osp.join(here, 'data/%ss' % input_type))
    subprocess.call(cmd, shell=True)

    out_file = osp.join(here, '%s.ply' % name)
    open_mesh(out_file)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('name', choices=['mask_view', 'mask_fusion',
                                         'label_view', 'label_fusion'])
    parser.add_argument('-d', '--depth', action='store_true')
    args = parser.parse_args()
    if args.name.endswith('_fusion'):
        demo_fusion(name=args.name, use_depth=args.depth)
    elif args.name.endswith('_view'):
        if args.depth:
            print('--depth is not supported for demo: %s' % args.name)
            return
        demo_view(name=args.name)
    else:
        print('Unsupported demo: %s' % args.name)


if __name__ == '__main__':
    main()
