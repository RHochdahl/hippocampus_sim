#!/usr/bin/env python
import sys
from xml.etree import ElementTree
from xml.dom import minidom
import shutil
import os
import re
import glob
import argparse
import errno
import rospy

TAG_REPO_URL = "https://github.com/HippoCampusRobotics/apriltags.git"
TAG_SIZE_X = 0.096
TAG_SIZE_Y = 0.096
TAG_SIZE_Z = 0.005

BASE_DAE = """
<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <asset>
    <contributor>
      <author>Blender User</author>
      <authoring_tool>Blender 2.90.1 commit date:2020-09-23, commit time:06:43, hash:3e85bb34d0d7</authoring_tool>
    </contributor>
    <created>2020-10-15T02:18:18</created>
    <modified>2020-10-15T02:18:18</modified>
    <unit name="meter" meter="1" />
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_geometries>
    <geometry id="Cube-mesh" name="Cube">
      <mesh>
        <source id="Cube-mesh-positions">
          <float_array id="Cube-mesh-positions-array" count="24">1 1 0 1 1 -2 1 -1 0 1 -1 -2 -1 1 0 -1 1 -2 -1 -1 0 -1 -1 -2</float_array>
          <technique_common>
            <accessor source="#Cube-mesh-positions-array" count="8" stride="3">
              <param name="X" type="float" />
              <param name="Y" type="float" />
              <param name="Z" type="float" />
            </accessor>
          </technique_common>
        </source>
        <source id="Cube-mesh-normals">
          <float_array id="Cube-mesh-normals-array" count="18">0 0 1 0 -1 0 -1 0 0 0 0 -1 1 0 0 0 1 0</float_array>
          <technique_common>
            <accessor source="#Cube-mesh-normals-array" count="6" stride="3">
              <param name="X" type="float" />
              <param name="Y" type="float" />
              <param name="Z" type="float" />
            </accessor>
          </technique_common>
        </source>
        <source id="Cube-mesh-map-0">
          <float_array id="Cube-mesh-map-0-array" count="72">1.00076e-4 0.9999001 0.9998998 9.9957e-5 0.9998999 0.9998998 -0.239227 0.7204706 -0.4892271 0.9704706 -0.4892271 0.7204706 -0.239227 -0.02952939 -0.4892271 0.2204706 -0.4892271 -0.02952939 -0.4892271 0.4704706 -0.7392271 0.7204706 -0.7392271 0.4704706 -0.239227 0.4704706 -0.4892271 0.7204706 -0.4892271 0.4704706 0.7372117 -0.4468938 0.4872117 -0.1968937 0.4872117 -0.4468938 1.00076e-4 0.9999001 9.9957e-5 1.00195e-4 0.9998998 9.9957e-5 -0.239227 0.7204706 -0.239227 0.9704706 -0.4892271 0.9704706 -0.239227 -0.02952939 -0.239227 0.2204706 -0.4892271 0.2204706 -0.4892271 0.4704706 -0.4892271 0.7204706 -0.7392271 0.7204706 -0.239227 0.4704706 -0.239227 0.7204706 -0.4892271 0.7204706 0.7372117 -0.4468938 0.7372117 -0.1968937 0.4872117 -0.1968937</float_array>
          <technique_common>
            <accessor source="#Cube-mesh-map-0-array" count="36" stride="2">
              <param name="S" type="float" />
              <param name="T" type="float" />
            </accessor>
          </technique_common>
        </source>
        <vertices id="Cube-mesh-vertices">
          <input semantic="POSITION" source="#Cube-mesh-positions" />
        </vertices>
        <triangles count="12">
          <input semantic="VERTEX" source="#Cube-mesh-vertices" offset="0" />
          <input semantic="NORMAL" source="#Cube-mesh-normals" offset="1" />
          <input semantic="TEXCOORD" source="#Cube-mesh-map-0" offset="2" set="0" />
          <p>4 0 0 2 0 1 0 0 2 2 1 3 7 1 4 3 1 5 6 2 6 5 2 7 7 2 8 1 3 9 7 3 10 5 3 11 0 4 12 3 4 13 1 4 14 4 5 15 1 5 16 5 5 17 4 0 18 6 0 19 2 0 20 2 1 21 6 1 22 7 1 23 6 2 24 4 2 25 5 2 26 1 3 27 3 3 28 7 3 29 0 4 30 2 4 31 3 4 32 4 5 33 0 5 34 1 5 35</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="Cube" name="Cube" type="NODE">
        <matrix sid="transform">0.5 0 0 0 0 0.5 0 0 0 0 0.5 0 0 0 0 1</matrix>
        <instance_geometry url="#Cube-mesh" name="Cube">
          <bind_material>
            <technique_common>
              <instance_material symbol="Material-material" target="#Material-material">
                <bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0" />
              </instance_material>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene>
    <instance_visual_scene url="#Scene" />
  </scene>
</COLLADA>
"""


def create_tag_dir(out_dir, tag_name, force):
    tag_dir = os.path.join(out_dir, tag_name)
    try:
        os.mkdir(tag_dir)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise e
        if force:
            shutil.rmtree(tag_dir)
            os.mkdir(tag_dir)
        else:
            return False
    create_model_config(tag_name, tag_dir)
    create_model_sdf(tag_name, tag_dir)
    create_material(tag_name, tag_dir)
    create_meshes(tag_dir)
    return True


def create_model_config(tag_name, tag_dir):
    config_filepath = os.path.join(tag_dir, "model.config")
    model = ElementTree.Element("model")
    name = ElementTree.SubElement(model, "name")
    name.text = tag_name
    version = ElementTree.SubElement(model, "version")
    version.text = "1.0"
    sdf = ElementTree.SubElement(model, "sdf", dict(version="1.5"))
    sdf.text = "model.sdf"
    author = ElementTree.SubElement(model, "author")
    name = ElementTree.SubElement(author, "name")
    name.text = "Thies Lennart Alff"
    email = ElementTree.SubElement(author, "email")
    email.text = "thies.lennart.alff@tuhh.de"

    desccr = ElementTree.SubElement(model, "description")
    descr.text = "This model was auto generated."
    string = ElementTree.tostring(model, "utf-8")
    string = minidom.parseString(string)
    string = string.toprettyxml(indent="  ")

    with open(config_filepath, "w") as f:
        f.write(string)


def create_model_sdf(tag_name, tag_dir):
    sdf = ElementTree.Element("sdf", dict(version="1.5"))
    model = ElementTree.SubElement(sdf, "model", dict(name=tag_name))
    static = ElementTree.SubElement(model, "static")
    static.text = "true"
    link = ElementTree.SubElement(model, "link", dict(name="base_link"))
    visual = ElementTree.SubElement(link, "visual", dict(name="visual"))
    material = ElementTree.SubElement(visual, "material")
    script = ElementTree.SubElement(material, "script")
    uri = ElementTree.SubElement(script, "uri")
    uri.text = "model://{}/materials/scripts".format(tag_name)
    uri = ElementTree.SubElement(script, "uri")
    uri.text = "model://{}/materials/textures".format(tag_name)
    name = ElementTree.SubElement(script, "name")
    name.text = "{}/Image".format(tag_name)
    geometry = ElementTree.SubElement(visual, "geometry")
    mesh = ElementTree.SubElement(geometry, "mesh")
    uri = ElementTree.SubElement(mesh, "uri")
    uri.text = "model://{}/meshes/cube.dae".format(tag_name)
    scale = ElementTree.SubElement(mesh, "scale")
    scale.text = "{} {} {}".format(TAG_SIZE_X, TAG_SIZE_Y, TAG_SIZE_Z)
    model_filepath = os.path.join(tag_dir, "model.sdf")
    string = ElementTree.tostring(sdf, "utf-8")
    string = minidom.parseString(string)
    string = string.toprettyxml(indent="  ")
    with open(model_filepath, "w") as f:
        f.write(string)


def create_material(tag_name, tag_dir):
    material_path = os.path.join(tag_dir, "materials")
    scripts_path = os.path.join(material_path, "scripts")
    textures_path = os.path.join(material_path, "textures")
    os.mkdir(material_path)
    os.mkdir(scripts_path)
    os.mkdir(textures_path)
    create_script(scripts_path, tag_name)
    copy_textures(textures_path, tag_name)


def create_script(script_path, tag_name):
    file_name = "{}.material".format(tag_name)
    file_path = os.path.join(script_path, file_name)
    string = """
material {}/Image
{{
    technique
    {{
        pass
        {{
            ambient 0.5 0.5 0.5 1.0
            diffuse 1.0 1.0 1.0 1.0
            specular 0.03 0.03 0.03 1.0

            texture_unit
            {{
                texture texture.png
                tex_address_mode clamp
                filtering none
            }}
        }}

    }}
}}
""".format(tag_name)
    with open(file_path, "w") as f:
        f.write(string)


def copy_textures(textures_path, tag_name):
    src_file_name = "{}.png".format(tag_name)
    dst_file_name = "texture.png"
    src_path = os.path.join("/tmp", src_file_name)
    dst_path = os.path.join(textures_path, dst_file_name)
    if os.path.isfile(src_path):
        shutil.move(src_path, dst_path)
    else:
        print("Could not find texture at '{}'.".format(src_path))


def create_meshes(tag_dir):
    meshes_dir = os.path.join(tag_dir, "meshes")
    os.mkdir(meshes_dir)
    file_path = os.path.join(meshes_dir, "cube.dae")
    with open(file_path, "w") as f:
        f.write(BASE_DAE)


def download_tags(url, force=False):
    tmp_dir = "/tmp/tags"
    print("Downloading tags into '{}'...".format(tmp_dir))
    if os.path.isdir(tmp_dir):
        if force:
            shutil.rmtree(tmp_dir)
    os.system("git clone {} {} > /dev/null 2>&1".format(url, tmp_dir))
    path = os.path.join(tmp_dir, "tag36h11")
    if os.path.isdir(path):
        return path
    return ""


def generate_tags(force, n_tags, src_dir, dst_dir):
    if not os.path.isdir(src_dir):
        print("Image directory '{}' does not exist.".format(src_dir))
        return False
    if not os.path.isdir(dst_dir):
        print("Output directory '{}' does not exist.".format(dst_dir))
        return False

    all_file_paths = sorted(glob.glob(os.path.join(src_dir, "*.png")))
    n_available = len(all_file_paths)
    if n_available < n_tags:
        print("Requested {} tags, but only {} images provided!", n_tags,
              n_available)

    n_not_replaced = 0
    n_generated = 0

    for i, file_path in enumerate(all_file_paths):
        if i >= n_tags:
            break
        file_name = os.path.basename(file_path)
        pattern = re.compile(r"(tag36_11_(\d+))\.png")
        result = pattern.search(file_name)
        if not result:
            continue

        tag_id = int(result.group(2))
        tag_name = result.group(1)
        file_name = result.group(0)
        file_path = os.path.join(src_dir, file_name)
        dst_path = os.path.join("/tmp", file_name)
        shutil.copyfile(file_path, dst_path)
        success = create_tag_dir(dst_dir, tag_name, force)
        if not success:
            n_not_replaced += 1
            print("Ignoring already existing tag: '{}'".format(tag_name))
        else:
            n_generated += 1

    print("Finished!")
    print("Created {} of {} requested models. Skipped {} models.".format(
        n_generated, n_tags, n_not_replaced))
    return True


def main(args):
    parser = argparse.ArgumentParser(args)
    parser.add_argument("--out-dir", action="store", default=os.getcwd())
    parser.add_argument("-n", action="store", default=63)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--force-download", action="store_true")
    args = parser.parse_args()

    img_dir = download_tags(TAG_REPO_URL, force=args.force_download)
    if not img_dir:
        print("Failed to download the tags.")
        print("Rerunning with '--force-download' might fix this.")
    ret = generate_tags(force=args.overwrite,
                        n_tags=args.n,
                        src_dir=img_dir,
                        dst_dir=args.out_dir)
    if ret:
        print("Generated tags successfully.")
    else:
        print("Failed to generate tags.")


if __name__ == "__main__":
    main(rospy.myargv(argv=sys.argv))
