import os
import xmltodict

base_dir = '/Users/lior.strichash/private/robust-matching/automatic_experiments'

class Algorithm:
    def __init__(self) -> None:
        pass
     
    def get_loop_functions_params(self):
        return {}

class NonFaultyAlgorithm(Algorithm):
    def __init__(self, template_file_path) -> None:
        super().__init__()
        self.template_file_path = template_file_path

class VirtualForces(NonFaultyAlgorithm):
    def __init__(self) -> None:
        super().__init__(template_file_path=f'{base_dir}/templates/virtual_forces.argos')

class AlgoMatching(NonFaultyAlgorithm):
    def __init__(self, is_commited, repeate_interval = 100000000) -> None:
        super().__init__(template_file_path=f'{base_dir}/templates/matching.argos')
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

class FaultyAlgorithm(Algorithm):
    pass

class Crash(FaultyAlgorithm):
    pass

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
    
    def generate_argos_file(self):
        os.makedirs(os.path.dirname(self.argos_file_path), exist_ok=True)
        with open(self.non_faulty_algorithm.template_file_path) as fd:
            doc = xmltodict.parse(fd.read())

        doc['argos-configuration']['framework']['experiment']['@random_seed'] = self.random_seed
        doc['argos-configuration']['framework']['experiment']['@length'] = self.length
        doc['argos-configuration']['arena']['distribute'][0]['entity']['@quantity'] = self.non_faulty_count
        doc['argos-configuration']['arena']['distribute'][1]['entity']['@quantity'] = self.faulty_count
        doc['argos-configuration']['visualization'] = doc['argos-configuration']['visualization'] if self.visualization else {}
        doc['argos-configuration']['loop_functions']['params'] = self.get_loop_functions_params()
        to_save_string = xmltodict.unparse(doc)

        with open(self.argos_file_path, 'w') as f:
            f.write(to_save_string)

        return self.argos_file_path