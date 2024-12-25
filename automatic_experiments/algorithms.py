import os
import xmltodict

base_dir = '/home/lior/workspace/robust-matching'
template_file_path = f'{base_dir}/automatic_experiments/templates/virtual_forces.argos'


class Algorithm:
    def __init__(self, name: str, range: int) -> None:
        self.name = name
        self.controller_type = name
        self.range = range
     
    def get_loop_functions_params(self):
        return {}
    
    def get_loop_functions(self):
        raise NotImplemented()

    def get_controller_params(self):
        return {}

    def to_dict(self):
        return {
            "name": self.name,
            "controller_type": self.controller_type,
            "loop_function_params": self.get_loop_functions_params(),
            "controller_params": self.get_controller_params()
        }

class NonFaultyAlgorithm(Algorithm):
    def __init__(self, name: str, range: int, template_file_path: str) -> None:
        super().__init__(name, range=range)
        self.template_file_path = template_file_path
        self.robot_type = "e-puck2"
        self.arena_size = 4.0

class VirtualForces(NonFaultyAlgorithm):
    def __init__(self, range: int) -> None:
        super().__init__(name="virtual_forces",template_file_path=f'{base_dir}/automatic_experiments/templates/virtual_forces.argos', range=range)
    
    def get_loop_functions(self):
        return {
            "@library":  f'{base_dir}/build/src/loop_functions/print_experiment_loop_fuctions/libprint_experiment_loop_fuctions',
            "@label": "print_experiment_loop_fuctions",
            "params": self.get_loop_functions_params()
        }
class VirtualForcesRandom(VirtualForces):
    def __init__(self, range: int) -> None:
        super().__init__(range)
        self.name = "virtual_forces_random"
        self.controller_type = "virtual_forces_random"

class AlgoMatching(NonFaultyAlgorithm):
    def __init__(self, is_commited: bool, range: int, name="algo_matching", repeate_interval: int = 100000000) -> None:
        # name = "algo_matching" if is_commited else f"repeated_{repeate_interval}"
        super().__init__(name=name, template_file_path=f'{base_dir}/automatic_experiments/templates/matching.argos', range=range)
        self.controller_type = "algo_matching"
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

class MeetingPoints(NonFaultyAlgorithm):
    def __init__(self, range: int) -> None:
        super().__init__(name="meeting_points",template_file_path=f'{base_dir}/automatic_experiments/templates/virtual_forces.argos', range=range)
        self.name = "meeting_point"
        self.controller_type = "meeting_point"
        self.robot_type = "eye-bot"
        self.arena_size = 16

    def get_loop_functions(self):
        return {
            "@library":  f'{base_dir}/build/src/loop_functions/iterated_meeting_points_loop_functions/libiterated_meeting_points_loop_functions',
            "@label": "iterated_meeting_points_loop_functions",
            "params": self.get_loop_functions_params()
        }

class MeetingPointEpuck(NonFaultyAlgorithm):
    def __init__(self, range: int) -> None:
        super().__init__(name="meeting_point_epuck",template_file_path=f'{base_dir}/automatic_experiments/templates/virtual_forces.argos', range=range)
        self.name = "meeting_point_epuck"
        self.controller_type = "meeting_point_epuck"

    def get_loop_functions(self):
        return {
            "@library":  f'{base_dir}/build/src/loop_functions/iterated_meeting_points_loop_functions/libiterated_meeting_points_loop_functions',
            "@label": "iterated_meeting_points_epuck_loop_functions",
            "params": self.get_loop_functions_params()
        }
    
class GreedyMeetingPoints(NonFaultyAlgorithm):
    def __init__(self, range: int) -> None:
        super().__init__(name="greedy_meeting_points",template_file_path=f'{base_dir}/automatic_experiments/templates/virtual_forces.argos', range=range)
        self.name = "greedy_meeting_points"
        self.controller_type = "greedy_meeting_points"

    def get_loop_functions(self):
        return {
            "@library":  f'{base_dir}/build/src/loop_functions/greedy_meeting_points_loop_functions/libgreedy_meeting_points_loop_functions.so',
            "@label": "greedy_meeting_points_loop_functions",
            "params": self.get_loop_functions_params()
        }

class FaultyAlgorithm(Algorithm):
    def __init__(self, name: str, range: int) -> None:
        super().__init__(name, range)
        
class Crash(FaultyAlgorithm):
    def __init__(self, range: int) -> None:
        super().__init__(name="crash",range=range)
class VirtualForcesWalkAway(FaultyAlgorithm):
    def __init__(self, range: int) -> None:
        super().__init__(name="virtual_forces_walk_away", range=range),
class AlgoMatchingWalkAway(FaultyAlgorithm):
    def __init__(self, range: int) -> None:
        super().__init__(name="algo_matching_walk_away", range=range)
class KeepDistance(FaultyAlgorithm):
    def __init__(self, range: int) -> None:
        super().__init__(name="keep_distance", range=range)
class VirtualForcesRandomCrash(FaultyAlgorithm):
    def __init__(self, range: int, start_crash_time: int, end_crash_time) -> None:
        super().__init__(name=f"crash_{start_crash_time}_{end_crash_time}", range=range)
        self.controller_type = "virtual_forces_random_crash"
        self.start_crash_time = start_crash_time
        self.end_crash_time = end_crash_time
class AlgoMatchingCrash(FaultyAlgorithm):
    def __init__(self, range: int, start_crash_time: int, end_crash_time) -> None:
        super().__init__(name=f"crash_{start_crash_time}_{end_crash_time}", range=range)
        self.controller_type = "algo_matching_crash"
        self.start_crash_time = start_crash_time
        self.end_crash_time = end_crash_time
class MeetingPointsCrash(FaultyAlgorithm):
    def __init__(self, range: int) -> None:
        super().__init__(name=f"meeting_point_crash", range=range)

class GreedyMeetingPointsCrash(FaultyAlgorithm):
    def __init__(self, range: int) -> None:
        super().__init__(name=f"greedy_meeting_points_crash", range=range)


def algorithmFactory(name, range) -> Algorithm:
    if name == "virtual_forces":
        return VirtualForces(range=range)
    elif name == "commited":
        return AlgoMatching(is_commited=True,range=range)
    elif name == "repeated":
        return AlgoMatching(is_commited=False, name="repeated", repeate_interval=10,range=range)
    elif name == "crash":
        return Crash(range=range)
    elif name == "virtual_forces_walk_away":
        return VirtualForcesWalkAway(range=range)
    elif name == "algo_matching_walk_away":
        return AlgoMatchingWalkAway(range=range)
    elif name == "keep_distance":
        return KeepDistance(range=range)
    elif name == "virtual_forces_random":
        return VirtualForcesRandom(range=range)
    elif name == "meeting_points":
        return MeetingPointEpuck(range=range)
    elif name == "meeting_points_crash":
        return MeetingPointsCrash(range=range)
    elif name == "greedy_meeting_points":
        return GreedyMeetingPoints(range=range)
    elif name == "greedy_meeting_points_crash":
        return GreedyMeetingPointsCrash(range=range)
    elif "virtual_forces_random_crash" in name:
        times = name.split("-")[1]
        start_time = times.split("_")[0]
        end_time = times.split("_")[1]
        return VirtualForcesRandomCrash(range=range, start_crash_time=start_time,end_crash_time=end_time)
    elif "algo_matching_crash" in name:
        times = name.split("-")[1]
        start_time = times.split("_")[0]
        end_time = times.split("_")[1]
        return AlgoMatchingCrash(range=range, start_crash_time=start_time,end_crash_time=end_time)


class Experiment:
    def __init__(self, non_faulty_count: int, faulty_count: int, non_faulty_algorithm: NonFaultyAlgorithm, faulty_algorithm: FaultyAlgorithm, random_seed: int, run_tag:str, length = 300,visualization = False, file_path:str = None) -> None:
        if file_path is None:
            file_path = f'{base_dir}/automatic_experiments/results/{run_tag}/{non_faulty_algorithm.name}_{faulty_algorithm.name}/faulty{faulty_count}/random_seed{random_seed}.argos'
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

        if self.faulty_algorithm.controller_type in ["algo_matching_crash", "virtual_forces_random_crash"]:
            doc['argos-configuration']['controllers']['virtual_forces_random_crash_controller']['params']['crash']['@start_time'] = self.faulty_algorithm.start_crash_time
            doc['argos-configuration']['controllers']['virtual_forces_random_crash_controller']['params']['crash']['@end_time'] = self.faulty_algorithm.end_crash_time
            doc['argos-configuration']['controllers']['robust_matching_crash_controller']['params']['crash']['@start_time'] = self.faulty_algorithm.start_crash_time
            doc['argos-configuration']['controllers']['robust_matching_crash_controller']['params']['crash']['@end_time'] = self.faulty_algorithm.end_crash_time
        to_save_string = xmltodict.unparse(doc)

        with open(self.argos_file_path, 'w') as f:
            f.write(to_save_string)

        # print(self.argos_file_path)
        return self.argos_file_path
    



def distribute_max_range(experiment: Experiment):
    return {
        '@range': experiment.non_faulty_algorithm.range,
        '@arena_size': experiment.non_faulty_algorithm.arena_size,
        'robot': [{
            '@quantity': experiment.non_faulty_count, 
            experiment.non_faulty_algorithm.robot_type: {
                '@id': 'non_faulty', 
                '@rab_range': experiment.non_faulty_algorithm.range, 
                '@rab_data_size': '4', 
                'controller': {'@config': experiment.non_faulty_algorithm.controller_type}
            }
        },{
            '@quantity': experiment.faulty_count, 
            experiment.non_faulty_algorithm.robot_type: {
                '@id': 'faulty', 
                '@rab_range': experiment.faulty_algorithm.range, 
                '@rab_data_size': '4', 
                'controller': {'@config': experiment.faulty_algorithm.controller_type}
            }
        }]
    }