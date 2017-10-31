import yaml
import math

import shapely.geometry as geom
from shapely import affinity
import itertools

class Environment:
    def __init__(self, yaml_file=None, bounds=None):
        self.yaml_file = yaml_file
        self.environment_loaded = False
        self.obstacles = []
        self.obstacles_map = {}
        self.features = []
        self.features = {}
        self.bounds = bounds
        if not yaml_file is None:
            if self.load_from_yaml_file(yaml_file):
                if bounds is None:
                    self.calculate_scene_dimensions()
                self.environment_loaded = True

    @property
    def bounds(self):
        return self.bounds

    def add_obstacles(self, obstacles):
        self.obstacles = self.obstacles + obstacles
        self.calculate_scene_dimensions()

    def calculate_scene_dimensions(self):
        """Compute scene bounds from obstacles and features."""
        points = []
        for elem in self.obstacles:
            points = points + list(elem.boundary.coords)
        for elem in self.features:
            points = points + list(elem.boundary.coords)

        mp = geom.MultiPoint(points)
        self.bounds = mp.bounds

    def load_from_yaml_file(self, yaml_file):
        f = open(yaml_file)
        self.data = yaml.safe_load(f)
        f.close()
        return self.parse_yaml_data(self.data)

    def parse_yaml_data(self, data):
        if 'environment' in data:
            env = data['environment']
            self.parse_yaml_obstacles(env['obstacles'])
            self.parse_yaml_features(env['features'])
            return True
        else:
            return False

    def parse_yaml_obstacles(self, obstacles):
        self.obstacles = []
        self.obstacles_map = {}
        for name, description in obstacles.iteritems():
            # Double underscore not allowed in region names.
            if name.find("__") != -1:
                raise Exception("Names cannot contain double underscores.")
            if description['shape'] == 'rectangle':
                parsed = self.parse_rectangle(name, description)
            elif description['shape'] == 'polygon':
                parsed = self.parse_polygon(name, description)
            else:
                raise Exception("not a rectangle")
            if not parsed.is_valid:
                raise Exception("%s is not valid!"%name)
            self.obstacles.append(parsed)
            self.obstacles_map[name] = parsed
        self.expanded_obstacles = [obs.buffer(0.75/2, resolution=2) for obs in self.obstacles]

    def parse_yaml_features(self, features):
        self.features = []
        self.features_map = {}
        for name, description in features.iteritems():
            if description['shape'] == 'rectangle':
                parsed = self.parse_rectangle(name, description)
            elif description['shape'] == 'polygon':
                parsed = self.parse_polygon(name, description)
            else:
                raise Exception("unknown feature shape")
            self.features.append(parsed)
            self.features_map[name] = parsed
            
    def parse_rectangle(self, name, description):
        center = description['center']
        center = geom.Point((center[0], center[1]))
        length = description['length']
        width = description['width']
        # convert rotation to radians
        rotation = description['rotation']# * math.pi/180
        # figure out the four corners.
        corners = [(center.x - length/2., center.y - width/2.),
                   (center.x + length/2., center.y - width/2.),
                   (center.x + length/2., center.y + width/2.),
                   (center.x - length/2., center.y + width/2.)]
        # print corners
        polygon = geom.Polygon(corners)
        out = affinity.rotate(polygon, rotation, origin=center)
        out.name = name
        out.cc_length = length
        out.cc_width = width
        out.cc_rotation = rotation
        return out

    def parse_polygon(self, name, description):
        _points = description['corners']
        for points in itertools.permutations(_points):
            polygon = geom.Polygon(points)
            polygon.name = name
            if polygon.is_valid:
                return polygon

    def save_to_yaml(self, yaml_file):
        yaml_dict = {}
        obstacles = {}
        for i, ob in enumerate(self.obstacles):
            ob_dict = {}
            ob_dict['shape'] = 'polygon'
            ob_dict['corners'] = [list(t) for t in list(ob.boundary.coords)]
            ob_name = "obstacle%.4d"%i
            obstacles[ob_name] = ob_dict
        yaml_dict['environment'] = {'obstacles' : obstacles}
        
        f = open(yaml_file, 'w')
        f.write(yaml.dump(yaml_dict, default_flow_style=None))
        f.close()
    
