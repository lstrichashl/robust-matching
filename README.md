# Installation
1. Download and install ARGoS3 on Ubuntu 20.04 (tested with ARGoS3 3.0.0-beta59).
2. Install the e-puck2 plugin from [here](https://gitlab.com/uniluxembourg/snt/pcog/adars/e-puck2).
3. Run:  `pip install -r automatic_experiments/requirements.txt`


# Build
```
cd robust_matching directiory
mkdir build
cd build
cmake ..
make
cd ..
```

# Run examples
1. Run an experiment where the robots use the `virtual_forces` algorithm (-a), the fault model (-fa) is `keep_distance`, there are 20 non-faulty robots (-nf) and 1 faulty robot (-f). The random seed (-s) is set to 1.

```
python3 automatic_experiments/test_experiment.py -s 1 -nf 20 -f 1 -a virtual_forces -fa keep_distance
```