# Installation
1. Download and install argos3 on run ubuntu 20.04 (It was tested with argos3.0.0-beta59)
2. Install e-puck2 plugin from https://gitlab.com/uniluxembourg/snt/pcog/adars/e-puck2
3. `pip install -r automatic_experiments/requirements.txt`


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
1. Run expriment where the robots are (-a) 'virtual_forces', the fault model (-fa) is 'keep_distance', there are 20 (-nf) nonfaulty robots and 1 faulty robot(-f). The random seed (-s) is 1.

```
python3 automatic_experiments/test_experiment.py -s 1 -nf 20 -f 1 -a virtual_forces -fa keep_distance
```