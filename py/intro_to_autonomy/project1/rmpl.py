import re
import math

class RMPLPreProcessor:
    def __init__(self, input_file, env, path_planner):
        self.input_file = input_file
        self.env = env
        self.path_planner = path_planner

    def location_class(self):
        out = 'class Location {\n'
        for region in self.env.features:
            out += "    value %s;\n"%region.name
        out += "}\n"
        return out

    def primitive_method(self, from_region, to_region, motion_primitive, roadmap_name, condition=None):
        if condition:
            out = '    primitive method %s__%s__%s() %s == %s && %s => %s == %s;'\
                  % (motion_primitive, from_region.name, to_region.name, roadmap_name, from_region.name, condition, roadmap_name, to_region.name)
        else:
            out = '    primitive method %s__%s__%s() %s == %s => %s == %s;'\
                  % (motion_primitive, from_region.name, to_region.name, roadmap_name, from_region.name, roadmap_name, to_region.name)
        return out

    def primitive_methods(self, motion_primitive, roadmap_name, condition):
        primitives = ""
        paths=""
        for from_region in self.env.features:
            for to_region in self.env.features:
                if from_region==to_region:
                    continue
                path = self.path_planner(from_region, to_region)
                if path is None:
                    continue
                primitives += self.primitive_method(from_region,\
                                                    to_region,\
                                                    motion_primitive,\
                                                    roadmap_name,\
                                                    condition)
                paths += "%s:%s:%s\n"%(from_region.name,
                                       to_region.name,
                                       path.to_wkt())
                primitives += "\n"
        return primitives, paths
    
    def preprocess(self):
        rmpl_file = open(self.input_file)
        rmpl = rmpl_file.read()
        rmpl_file.close()
        #print rmpl
        # Find the LOCATION_CLASS statement and replace it.
        r = re.compile("^\\s*#LOCATION_CLASS\\s*$",re.MULTILINE)
        rmpl = re.sub(r, self.location_class(), rmpl)
        # Build up the motion primitives.
        r = re.compile("^\\s*#MOTION_PRIMITIVES\\s*\\(\\s*(\\w*)\\s*,\\s*(\\w*)\\s*(?:,\\s*(.*)\\s*)?\\)\\s*$",re.MULTILINE)
        
        matches = r.finditer(rmpl)
        for m in matches:
            if len(m.groups()) == 2:
                roadmap_name, motion_primitive = m.groups()
                condition = None
            else:
                roadmap_name, motion_primitive, condition = m.groups()
            primitives, paths = self.primitive_methods(motion_primitive, roadmap_name, condition)
            rmpl = rmpl.replace(m.group(), primitives, 1)

        return rmpl, paths
