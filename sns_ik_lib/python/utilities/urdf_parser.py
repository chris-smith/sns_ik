from __future__ import print_function
import xml.etree.ElementTree as ET
import argparse

class Joint:
    def __init__(self, element=None):
        if element is not None:
            self.parse_elem(element)

    def __str__(self):
        return "Joint: " + self.name

    def parse_elem(self, element):
        print("Found joint " + element.attrib['name'])
        self.name = element.attrib['name']
        self.type = element.attrib['type']

class Link:
    def __init__(self, element=None):
        if element is not None:
            self.parse_elem(element)

    def __str__(self):
        return "Link: " + self.name

    def parse_elem(self, element):
        print("Found link " + element.attrib['name'])
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

    def print(self):
        for s in self.segments:
            print(s)
        for j in self.joints:
            print(j)

def parse_urdf(file):
    yield

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', '-f', default=None, required=True, help="path to URDF file to parse")
    parser.add_argument('--chain', '-c', action='store_true', help="Set to print KDL Chain information")
    parser.add_argument('--limits', '-l', action='store_true', help="Set to print KDL Joint Limit information")

    args = parser.parse_args()

    urdf = URDF(args.file);
    urdf.print()

    if args.chain:
        print_chain(urdf)
    if args.limits:
        print_limits(urdf)


if __name__ == "__main__":
    main()
