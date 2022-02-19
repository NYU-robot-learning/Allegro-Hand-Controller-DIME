#!/usr/bin/env python
import argparse
import trimesh
from tqdm import tqdm

from ll4ma_util import file_util, ui_util


if __name__ == '__main__':
    """
    Simple mesh conversion script using Trimesh https://trimsh.org/index.html.

    Right now this assumes you want to convert all files in a directory with a
    particular extension to a different extension, saved to that same directory.
    Feel free to add more processing options as you desire.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--input_dir', type=str, required=True,
                        help="Absolute path to directory containing input mesh files")
    parser.add_argument('-i', '--in_extension', type=str, required=True,
                        choices=trimesh.available_formats(),
                        help="File extension of files in input_dir to be processed")
    parser.add_argument('-o', '--out_extension', type=str, required=True,
                        choices=trimesh.available_formats(),
                        help="File extension to give generated output mesh files")
    parser.add_argument('--show', action='store_true', help="Visualize generated meshes if true")
    args = parser.parse_args()

    in_filenames = file_util.list_dir(args.input_dir, args.in_extension)

    print("\nConverting meshes...")
    for in_filename in tqdm(in_filenames):
        out_filename = file_util.change_extension(in_filename, args.out_extension)
        mesh = trimesh.load(in_filename)
        mesh.export(out_filename)
        if args.show:
            new_mesh = trimesh.load(out_filename)
            new_mesh.show()
    ui_util.print_happy("Mesh conversion complete.\n")
