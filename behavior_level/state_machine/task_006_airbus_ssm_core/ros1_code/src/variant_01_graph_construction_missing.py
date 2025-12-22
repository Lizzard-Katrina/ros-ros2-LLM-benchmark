import xml.etree.ElementTree as ET

class SCXMLLoaderVariant(object):
    def __init__(self, scxml_path):
        self.scxml_path = scxml_path
        self.nodes = {}

    def load(self):
        tree = ET.parse(self.scxml_path)
        root = tree.getroot()
        initial = root.attrib.get('initial', None)

        # TODO
        # Steps to restore:
        #   - iterate children of root; for each 'state' or 'final', get id and type
        #   - collect <transition> children: read 'event','target','cond'
        #   - add node
        # Expected outcome: after load(), return (initial, self.nodes) consistent with original SCXML structure
        # END

