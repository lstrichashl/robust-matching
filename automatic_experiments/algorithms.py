import os
import xmltodict

base_dir = '/Users/lior.strichash/private/robust-matching/automatic_experiments'
template_file_path = f'{base_dir}/templates/virtual_forces.argos'
class Algorithm:
    def __init__(self, name) -> None:
        self.name = name
     
    def get_loop_functions_params(self):
        return {}
    
    def get_loop_functions(self):
        raise NotImplemented()

class NonFaultyAlgorithm(Algorithm):
    def __init__(self, name, template_file_path) -> None:
        super().__init__(name)
        self.template_file_path = template_file_path

class VirtualForces(NonFaultyAlgorithm):
    def __init__(self) -> None:
        super().__init__(name="virtual_forces",template_file_path=f'{base_dir}/templates/virtual_forces.argos')
    
    def get_loop_functions(self):
        return {
            "@library":  "/Users/lior.strichash/private/robust-matching/build/src/loop_functions/print_experiment_loop_fuctions/libprint_experiment_loop_fuctions",
            "@label": "print_experiment_loop_fuctions",
            "params": self.get_loop_functions_params()
        }

class AlgoMatching(NonFaultyAlgorithm):
    def __init__(self, is_commited, repeate_interval = 100000000) -> None:
        super().__init__(name="algo_matching", template_file_path=f'{base_dir}/templates/matching.argos')
        self.is_commited = is_commited
        self.repeate_interval = repeate_interval
        if is_commited and self.repeate_interval < 100000000:
            raise "is_commited has to be with very large interval"
        
    def get_loop_functions_params(self):
        dict = {
            "@is_commited": "true" if self.is_commited else "false",
            "@repeat_interval": self.repeate_interval
        }
        return {**super().get_loop_functions_params(), **dict}
    
    def get_loop_functions(self):
        return {
            "@library": "/Users/lior.strichash/private/robust-matching/build/src/loop_functions/matching_loop_functions/libmatching_loop_functions",
            "@label": "matching_loop_functions",
            "params": self.get_loop_functions_params()
        }

class FaultyAlgorithm(Algorithm):
    pass
class Crash(FaultyAlgorithm):
    def __init__(self) -> None:
        super().__init__(name="crash")
class VirtualForcesWalkAway(FaultyAlgorithm):
    def __init__(self) -> None:
        super().__init__(name="virtual_forces_walk_away")
class AlgoMatchingWalkAway(FaultyAlgorithm):
    def __init__(self) -> None:
        super().__init__(name="algo_matching_walk_away")

def algorithmFactory(name) -> Algorithm:
    if name == "virtual_forces":
        return VirtualForces()
    elif name == "commited":
        return AlgoMatching(is_commited=True)
    elif name == "crash":
        return Crash()
    elif name == "virtual_forces_walk_away":
        return VirtualForcesWalkAway()
    elif name == "algo_matching_walk_away":
        return AlgoMatchingWalkAway()

class Experiment:
    def __init__(self, non_faulty_count: int, faulty_count: int, non_faulty_algorithm: NonFaultyAlgorithm, faulty_algorithm: FaultyAlgorithm, random_seed: int, length = 500,visualization = False) -> None:
        self.faulty_count = faulty_count
        self.non_faulty_count = non_faulty_count
        self.non_faulty_algorithm = non_faulty_algorithm
        self.faulty_algorithm = faulty_algorithm
        self.random_seed = random_seed
        self.length = length
        self.visualization = visualization

        self.argos_file_path = tmp_file_path = f'{base_dir}/results/{non_faulty_algorithm.__class__.__name__}/faulty{faulty_count}/random_seed{random_seed}.argos'
        self.log_file = f"{tmp_file_path}.log"
    
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
        doc['argos-configuration']['arena']['distribute'][0]['entity']['@quantity'] = self.non_faulty_count
        doc['argos-configuration']['arena']['distribute'][1]['entity']['@quantity'] = self.faulty_count
        doc['argos-configuration']['arena']['distribute'][0]['entity']['e-puck2']['controller']['@config'] = self.non_faulty_algorithm.name
        doc['argos-configuration']['arena']['distribute'][1]['entity']['e-puck2']['controller']['@config'] = self.faulty_algorithm.name
        doc['argos-configuration']['visualization'] = doc['argos-configuration']['visualization'] if self.visualization else {}
        doc['argos-configuration']['loop_functions'] = self.get_loop_functions()
        # doc['argos-configuration']['loop_functions']['params'] = self.get_loop_functions_params()
        to_save_string = xmltodict.unparse(doc)

        with open(self.argos_file_path, 'w') as f:
            f.write(to_save_string)

        return self.argos_file_path