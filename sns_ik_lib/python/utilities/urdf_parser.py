from __future__ import print_function
import xml.etree.ElementTree as ET
import argparse
import json

class Joint:
    def __init__(self, element=None):
        self.parent = None
        self.child = None
        self.origin = None
        self.axis = None
        self.limits = None
        self.dynamics = None

        if element is not None:
            self.parse_elem(element)

    def __str__(self):
        return "Joint: " + self.name

    def parse_elem(self, element):
        self.name = element.attrib['name']
        self.type = element.attrib['type']

        for c in element:
            if c.tag == 'parent':
                self.parent = c.attrib['link']
            elif c.tag == 'child':
                self.child = c.attrib['link']
            elif c.tag == 'origin':
                self.origin = c.attrib
            elif c.tag == 'axis':
                self.axis = c.attrib
            elif c.tag == 'limit':
                self.limits = c.attrib
            elif c.tag == 'dynamics':
                self.dynamics = c.attrib

    def print_formatted_chain(self):
        s = "Joint: " + self.name + "\n"
        s += "  Type: " + self.type + "\n"
        if self.origin is not None:
            s += "  Origin: " + json.dumps(self.origin) + "\n"
        if self.parent is not None:
            s += "  Parent: " + self.parent + "\n"
        if self.child is not None:
            s += "  Child: " + self.child + "\n"

        print(s)

    def print_formatted_limits(self):
        s = "Joint: " + self.name + "\n"
        if self.limits is not None:
            s += "  Limits:" + json.dumps(self.limits) + "\n"
        else:
            s += "  == No Limits =="

        print(s)

class Link:
    def __init__(self, element=None):
        if element is not None:
            self.parse_elem(element)

    def __str__(self):
        return "Link: " + self.name

    def parse_elem(self, element):
        self.name = element.attrib['name']

class URDF:
    def __init__(self, file=None):
        self.segments = []
        self.joints = []

        if file is not None:
            self.parse_urdf_file(file)

    def parse_urdf_file(self, file):
        self.parse_urdf_xml_tree(ET.parse(file))

    def parse_urdf_xml_tree(self, tree):
        for child in tree.getroot():
            if child.tag == 'link':
                self.segments.append(Link(child))
            elif child.tag == 'joint':
                self.joints.append(Joint(child))

def print_chain(urdf):
    for j in urdf.joints:
        j.print_formatted_chain()

def print_limits(urdf):
    for j in urdf.joints:
        j.print_formatted_limits()

def print_segments(urdf):
    for s in urdf.segments:
        print(s.name)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', '-f', default=None, required=True, help="path to URDF file to parse")
    parser.add_argument('--chain', '-c', action='store_true', help="Set to print KDL Chain information")
    parser.add_argument('--limits', '-l', action='store_true', help="Set to print KDL Joint Limit information")
    parser.add_argument('--segments', '-s', action='store_true', help="Set to print KDL Segment information")

    args = parser.parse_args()

    urdf = URDF(args.file);

    if args.chain:
        print_chain(urdf)
    if args.limits:
        print_limits(urdf)
    if args.segments:
        print_segments(urdf)


if __name__ == "__main__":
    main()
