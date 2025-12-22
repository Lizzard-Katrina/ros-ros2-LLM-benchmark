#!/usr/bin/env python
# ORIGINAL scxml_loader: minimal loader that parses SCXML and builds node dict
import xml.etree.ElementTree as ET

class SCXMLLoader(object):
    def __init__(self, scxml_path):
        self.scxml_path = scxml_path
        self.nodes = {}  # id -> node dict: {'id': id, 'type': 'state'/'final', 'transitions':[]}

    def load(self):
        tree = ET.parse(self.scxml_path)
        root = tree.getroot()
        ns = {'scxml': root.tag.split('}')[0].strip('{')} if '}' in root.tag else {}
        initial = root.attrib.get('initial', None)
        for child in root:
            tag = child.tag.split('}')[-1]
            if tag == 'state' or tag == 'final':
                nid = child.attrib.get('id')
                ntype = 'final' if tag == 'final' else 'state'
                node = {'id': nid, 'type': ntype, 'transitions': []}
                for sub in child:
                    stag = sub.tag.split('}')[-1]
                    if stag == 'transition':
                        ev = sub.attrib.get('event', None)
                        tgt = sub.attrib.get('target', None)
                        cond = sub.attrib.get('cond', None)
                        node['transitions'].append({'event':ev, 'target':tgt, 'cond':cond})
                self.nodes[nid] = node
        return initial, self.nodes
