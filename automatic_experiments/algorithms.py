import os
import xmltodict

base_dir = '/home/lior/workspace/robust-matching'
template_file_path = f'{base_dir}/automatic_experiments/templates/virtual_forces.argos'


class Algorithm:
    def __init__(self, controller_type: str, range: int, library, id = "") -> None:
        self.controller_type = controller_type
        if id == "":
            self.id = controller_type
        else:
            self.id = id
        self.range = range
        self.library = library
     
    def get_loop_functions_params(self):
        return {}
    
    def get_loop_functions(self):
        raise NotImplemented()

    def get_controller_params(self):
        return {}
    
    def to_dict(self):
        return {
            "@id": self.id,
            "@library": self.library,
            "actuators": {
                "differential_steering": {
                    "@implementation": "default"
                },
                "epuck2_leds": {
                    "@implementation": "default",
                    "@medium": "leds"
                },
                "range_and_bearing": {
                    "@implementation": "default"
                }
            },
            "sensors":{
                "epuck2_encoder": {
                    "@implementation": "default"
                },
                "epuck2_battery":{
                    "@implementation": "default"
                },
                "range_and_bearing":{
                    "@implementation": "medium",
                    "@medium": "rab",
                    "@show_rays": "true"
                }
            },
            "params": {
                "wheel_turning": {
                    "@hard_turn_angle_threshold":"90",
                    "@soft_turn_angle_threshold":"70",
                    "@no_turn_angle_threshold":"10",
                    "@max_speed":"1"
                }
            }
        }
        # return {
        #     "name": self.name,
        #     "controller_type": self.controller_type,
        #     "loop_function_params": self.get_loop_functions_params(),
        #     "controller_params": self.get_controller_params()
        # }

class NonFaultyAlgorithm(Algorithm):
    def __init__(self, controller_type: str, range: int, library, template_file_path: str,random_exploration:bool = False) -> None:
        super().__init__(controller_type, range=range,library=library)
        self.template_file_path = template_file_path
        self.robot_type = "e-puck2"
        self.arena_size = 4.0
        self.random_exploration=random_exploration

    def to_dict(self):
        d = super().to_dict()
        if self.random_exploration:
            d["params"]["exploration"] = {
                "@implementation": "random"
            }
        return d
        

class VirtualForces(NonFaultyAlgorithm):
    def __init__(self, range: int,controller_type="virtual_forces_bot_controller") -> None:
        super().__init__(controller_type=controller_type,
                         template_file_path=f'{base_dir}/automatic_experiments/templates/virtual_forces.argos',
                          range=range,
                          library="build/src/controllers/virtual_forces/libvirtual_forces.so",
                          )
    
    def get_loop_functions(self):
        return {
            "@library":  f'{base_dir}/build/src/loop_functions/print_experiment_loop_fuctions/libprint_experiment_loop_fuctions',
            "@label": "print_experiment_loop_fuctions",
            "params": self.get_loop_functions_params()
        }
class VirtualForcesRandom(VirtualForces):
    def __init__(self, range: int) -> None:
        super().__init__(range=range,controller_type = "virtual_forces_random_controller")

class AlgoMatching(NonFaultyAlgorithm):
    def __init__(self, is_commited: bool, range: int, id="algo_matching", repeate_interval: int = 100000000) -> None:
        # id = "algo_matching" if is_commited else f"repeated_{repeate_interval}"
        super().__init__(controller_type="robust_matching_controller",
                         template_file_path=f'{base_dir}/automatic_experiments/templates/matching.argos',
                         range=range,
                         library="build/src/controllers/robust_matching/librobust_matching.so")
        self.id = id
        self.is_commited = is_commited
        self.repeate_interval = repeate_interval
        if is_commited and self.repeate_interval < 100000000:
            raise "is_commited has to be with very large interval"
        
    def get_loop_functions_params(self):
        dict = {
            "@is_commited": "true" if self.is_commited else "false",
            "@repeat_interval": self.repeate_interval,
            "@range": self.range
        }
        return {**super().get_loop_functions_params(), **dict}
    
    def get_loop_functions(self):
        return {
            "@library": f'{base_dir}/build/src/loop_functions/matching_loop_functions/libmatching_loop_functions',
            "@label": "matching_loop_functions",
            "params": self.get_loop_functions_params()
        }

class MeetingPointsEpuck(NonFaultyAlgorithm):
    def __init__(self, range: int) -> None:
        super().__init__(template_file_path=f'{base_dir}/automatic_experiments/templates/virtual_forces.argos',
                          range=range,
                          controller_type="meeting_point_epuck_controller",
                          library="build/src/controllers/meeting_points/libmeeting_points.so")

    def get_loop_functions(self):
        return {
            "@library":  f'{base_dir}/build/src/loop_functions/iterated_meeting_points_loop_functions/libiterated_meeting_points_loop_functions',
            "@label": "iterated_meeting_points_epuck_loop_functions",
            "params": self.get_loop_functions_params()
        }
    
class GreedyMeetingPoints(NonFaultyAlgorithm):
    def __init__(self, range: int, random_exploration) -> None:
        super().__init__(controller_type="greedy_meeting_point_controller",
                         template_file_path=f'{base_dir}/automatic_experiments/templates/virtual_forces.argos',
                         range=range,
                         library="build/src/controllers/meeting_points/libmeeting_points.so",
                         )
        self.random_exploration = random_exploration
        if random_exploration:
            self.id = "greedy_meeting_point_controller_random"

    def get_loop_functions(self):
        return {
            "@library":  f'{base_dir}/build/src/loop_functions/greedy_meeting_points_loop_functions/libgreedy_meeting_points_loop_functions.so',
            "@label": "greedy_meeting_points_loop_functions",
            "params": self.get_loop_functions_params()
        }
    
class MeetingPointsDist(NonFaultyAlgorithm):
    def __init__(self, range: int) -> None:
        super().__init__(controller_type="neighborhood_graph_controller",
                         template_file_path=f'{base_dir}/automatic_experiments/templates/virtual_forces.argos',
                         range=range,
                         library="build/src/controllers/meeting_points/libmeeting_points.so",
                         )
        # self.random_exploration = random_exploration
        # if random_exploration:
        #     self.id = "greedy_meeting_point_controller_random"

    def get_loop_functions(self):
        return {
            "@library":  f'{base_dir}/build/src/loop_functions/greedy_meeting_points_loop_functions/libgreedy_meeting_points_loop_functions.so',
            "@label": "neiberg_loop_functions",
            "params": self.get_loop_functions_params()
        }
    def to_dict(self):
        d = super().to_dict()
        d["params"]["@max_distance"] = 5
        d["params"]["@range"] = self.range
        return d

class TripletVirtuualForces(NonFaultyAlgorithm):
    def __init__(self, range):
        super().__init__(controller_type="triplet_forces_controller",
                        range=range,
                        library="build/src/controllers/triplet/libtriplet.so", 
                        template_file_path=f'{base_dir}/automatic_experiments/templates/virtual_forces.argos')

    def get_loop_functions(self):
        return {
            "@library":  f'{base_dir}/build/src/loop_functions/print_experiment_loop_fuctions/libprint_experiment_loop_fuctions',
            "@label": "print_experiment_loop_fuctions",
            "params": self.get_loop_functions_params()
        }

class FaultyAlgorithm(Algorithm):
    def __init__(self, id: str, nonfaulty_algorithm: NonFaultyAlgorithm) -> None:
        super().__init__(id=id, range=nonfaulty_algorithm.range, controller_type=nonfaulty_algorithm.controller_type, library=nonfaulty_algorithm.library)
        self.nonfaulty_algorithm = nonfaulty_algorithm

    def to_dict(self):
        d = super().to_dict()
        d["params"]["fault"] = {
            "@type": self.id
        }
        return d
        
class Crash(FaultyAlgorithm):
    def __init__(self, nonfaulty_algorithm, start_crash_time=0, end_crash_time=1) -> None:
        super().__init__(id=f"crash_{start_crash_time}_{end_crash_time}",nonfaulty_algorithm=nonfaulty_algorithm)
        self.start_crash_time = start_crash_time
        self.end_crash_time = end_crash_time
    
    def to_dict(self):
        d = super().to_dict()
        d["params"]["fault"]["@m_crash_starttime"] = self.start_crash_time
        d["params"]["fault"]["@m_crash_endtime"] = self.end_crash_time
        d["params"]["@max_distance"] = 5
        d["params"]["@range"] = self.range
        return d

class VirtualForcesWalkAway(FaultyAlgorithm):
    def __init__(self, nonfaulty_algorithm) -> None:
        super().__init__(id="virtual_forces_walk_away", nonfaulty_algorithm=nonfaulty_algorithm),
class AlgoMatchingWalkAway(FaultyAlgorithm):
    def __init__(self, nonfaulty_algorithm) -> None:
        super().__init__(id="opposite", nonfaulty_algorithm=nonfaulty_algorithm)
class KeepDistance(FaultyAlgorithm):
    def __init__(self, nonfaulty_algorithm) -> None:
        super().__init__(id="keep_distance", nonfaulty_algorithm=nonfaulty_algorithm)
    def to_dict(self):
        d = super().to_dict()
        d["params"]["wheel_turning"]["@max_speed"] = "20"
        return d

def faultalgorithmFactory(name, nonfaultyalgorithm):
    if name == "crash":
        return Crash(nonfaultyalgorithm,0,1)
    elif name == "virtual_forces_walk_away":
        return VirtualForcesWalkAway(nonfaultyalgorithm)
    elif name == "algo_matching_walk_away":
        return AlgoMatchingWalkAway(nonfaultyalgorithm)
    elif name == "keep_distance":
        return KeepDistance(nonfaultyalgorithm)
    else:
        raise name + " is not found"
    # elif "virtual_forces_random_crash" in name:
    #     times = name.split("-")[1]
    #     start_time = times.split("_")[0]
    #     end_time = times.split("_")[1]
    #     return VirtualForcesRandomCrash(range=range, start_crash_time=start_time,end_crash_time=end_time)

def algorithmFactory(name, range, random_exploration=False) -> Algorithm:
    if name == "virtual_forces":
        return VirtualForces(range=range)
    elif name == "commited":
        return AlgoMatching(is_commited=True,range=range)
    elif name == "repeated":
        return AlgoMatching(is_commited=False, id="repeated", repeate_interval=10,range=range)
    elif name == "virtual_forces_random":
        return VirtualForcesRandom(range=range)
    elif name == "meeting_points_epuck":
        return MeetingPointsEpuck(range=range)
    elif name == "greedy_meeting_points":
        return GreedyMeetingPoints(range=range,random_exploration=random_exploration)
    elif name == "triplet_forces":
        return TripletVirtuualForces(range=range)
    elif name == "meeting_points_dist":
        return MeetingPointsDist(range=range)
    else:
        raise name + " is not found"



class Experiment:
    def __init__(self, non_faulty_count: int, faulty_count: int, non_faulty_algorithm: NonFaultyAlgorithm, faulty_algorithm: FaultyAlgorithm, random_seed: int, run_tag:str, length = 300,visualization = False, file_path:str = None) -> None:
        if file_path is None:
            file_path = f'{base_dir}/automatic_experiments/results/{run_tag}/{non_faulty_algorithm.id}_{faulty_algorithm.id}/faulty{faulty_count}/random_seed{random_seed}.argos'
        self.faulty_count = faulty_count
        self.non_faulty_count = non_faulty_count
        self.non_faulty_algorithm = non_faulty_algorithm
        self.faulty_algorithm = faulty_algorithm
        self.random_seed = random_seed
        self.length = length
        self.visualization = visualization
        self.argos_file_path = file_path
        self.log_file = f"{self.argos_file_path}.log"
    
    def get_loop_functions_params(self):
        dict = {
            '@log_file_path': self.log_file
        }
        return {**dict, **self.non_faulty_algorithm.get_loop_functions_params(), **self.faulty_algorithm.get_loop_functions_params()}
    
    def get_loop_functions(self):
        loop_functions = self.non_faulty_algorithm.get_loop_functions()
        loop_functions['params'] = self.get_loop_functions_params()
        return loop_functions

    def generate_argos_file(self):
        os.makedirs(os.path.dirname(self.argos_file_path), exist_ok=True)
        with open(template_file_path) as fd:
            doc = xmltodict.parse(fd.read())

        doc['argos-configuration']['framework']['experiment']['@random_seed'] = self.random_seed
        doc['argos-configuration']['framework']['experiment']['@length'] = self.length
        # doc['argos-configuration']['arena']['distribute'][0]['entity']['@quantity'] = self.non_faulty_count
        # doc['argos-configuration']['arena']['distribute'][1]['entity']['@quantity'] = self.faulty_count
        # doc['argos-configuration']['arena']['distribute'][0]['entity']['e-puck2']['controller']['@config'] = self.non_faulty_algorithm.controller_type
        # doc['argos-configuration']['arena']['distribute'][0]['entity']['e-puck2']['@rab_range'] = self.non_faulty_algorithm.range
        # doc['argos-configuration']['arena']['distribute'][1]['entity']['e-puck2']['controller']['@config'] = self.faulty_algorithm.controller_type
        # doc['argos-configuration']['arena']['distribute'][1]['entity']['e-puck2']['@rab_range'] = self.faulty_algorithm.range
        if self.visualization:
            doc['argos-configuration']['visualization']["qt-opengl"]['camera']['placements']['placement']["@position"] = "0.0,0," + str(self.non_faulty_algorithm.arena_size)
        else:
            doc['argos-configuration']['visualization'] = {}
        doc['argos-configuration']['loop_functions'] = self.get_loop_functions()
        doc['argos-configuration']['loop_functions']['distribute_max_range'] = distribute_max_range(experiment=self)


        doc['argos-configuration']['controllers'] = {self.non_faulty_algorithm.controller_type: []}
        doc['argos-configuration']['controllers'][self.non_faulty_algorithm.controller_type].append(self.non_faulty_algorithm.to_dict())
        doc['argos-configuration']['controllers'][self.non_faulty_algorithm.controller_type].append(self.faulty_algorithm.to_dict())

        # if self.faulty_algorithm.controller_type in ["algo_matching_crash", "virtual_forces_random_crash"]:
        #     doc['argos-configuration']['controllers']['virtual_forces_random_crash_controller']['params']['crash']['@start_time'] = self.faulty_algorithm.start_crash_time
        #     doc['argos-configuration']['controllers']['virtual_forces_random_crash_controller']['params']['crash']['@end_time'] = self.faulty_algorithm.end_crash_time
        #     doc['argos-configuration']['controllers']['robust_matching_crash_controller']['params']['crash']['@start_time'] = self.faulty_algorithm.start_crash_time
        #     doc['argos-configuration']['controllers']['robust_matching_crash_controller']['params']['crash']['@end_time'] = self.faulty_algorithm.end_crash_time
        to_save_string = xmltodict.unparse(doc)

        with open(self.argos_file_path, 'w') as f:
            f.write(to_save_string)

        # print(self.argos_file_path)
        return self.argos_file_path
    



def distribute_max_range(experiment: Experiment):
    return {
        '@range': experiment.non_faulty_algorithm.range,
        '@arena_size': 1.5,
        'robot': [{
            '@quantity': experiment.non_faulty_count, 
            experiment.non_faulty_algorithm.robot_type: {
                '@id': 'non_faulty', 
                '@rab_range': experiment.non_faulty_algorithm.range, 
                '@rab_data_size': '1000', 
                'controller': {'@config': experiment.non_faulty_algorithm.id}
            }
        },{
            '@quantity': experiment.faulty_count, 
            experiment.non_faulty_algorithm.robot_type: {
                '@id': 'faulty', 
                '@rab_range': experiment.faulty_algorithm.range, 
                '@rab_data_size': '1000', 
                'controller': {'@config': experiment.faulty_algorithm.id}
            }
        }]
    }