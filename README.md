# drive-primer overview

# Quick Install (Docker)

You can choose to use docker compose to setup.

First, make sure your ssh-agent has a key loaded (`ssh-add -l`)
If not: 

```eval "$(ssh-agent -s)"```

```ssh-add ~/.ssh/id_ed25519 # or some other key``` 

Second, make sure you have `docker`, `docker compose`, and `nvidia-container-toolkit` installed. Check your CUDA architecture and make sure CUDA_CHRONO_ARCHITECTURE in docker-compose.yaml is set correctly.


In docker directory:

`docker compose build`

`docker compose up -d dev`

`docker compose exec -it dev`

Use the `drive-primer` command to see a list of options. See ~/drive-primer/docker/sample_workspace to make sure your installation works or to test out the commands without setting up your own downlink telemetry.

The use case designed is to mount the telemetry through docker-compose in a volume and run the commands on the mounted volume


# Manual install (no Docker)
## Chrono distribution

Use this mirror that includes necessary patches not available upstream: https://github.jpl.nasa.gov/rsvp/chrono-wisc-mirror

Run `contrib/buildURDF.sh`, `contrib/buildVSG.sh` to install VSG, Parsers dependencies. These scripts were built for Ubuntu and modified & tested on Rocky, expect minor hitches (but be pleasantly surprised if there are no issues)

Build with FSI, VSG (if visualizing), Parsers, and Vehicles

Requires CUDA 12.2+, GCC 11.4+, MPI

When building cmars note the build/cmake directory for the Chrono_DIR build flag, also note the libimage_data.a and include dir when building image-data. You will also be required to point to the various VSG and URDF dependencies

<b>Note for any future work (as of 8/26/25)</b>: PyChrono has been worked on significantly in the past 2 months, so much so that they are beginning to rollout CRM, Parsers, etc support. If switched to PyChrono, it would not require this kind of build process (at the down sides of reduced customizability of the core simulator, should we need it). I also included a docker/* folder and modified the docker-compose template used for Chrono projects. I have not tested if it works (my computer doesn't have enough /var/tmp space to build the images), but you are welcome to take a crack at it. I modified it to point to the mirror hosted on RSVP, make sure your ssh-agent keys are in order.

# Brief Overview
## cmars 
- Chrono::Mars simulator
- Collection of classes and demos to simulate Perseverance in the Project Chrono multi-physics simulator
- To run a single instance of the primary demo, use `demo_cmars "$(cat <simdef JSON>)"` (assuming binary is in your PATH). Note simdef JSON's can use both relative and absolute paths to point to vicar heightmaps, and if not accessible can cause errors
- For an example simdef, see [here](param-identification/downlink/benchmark/benchmark2_simdef.json)

## dp-cli
- Master CLI command `drive-primer`, essentially aliases
- Install as pip package
- Must have the other pip packages installed for it to work

## drive-primer config
- Generates BayOpt config files and kinematic trajectory CSVs (`sim_input_dir`)
- Requires Tkinter
- Install as pip package, run `drive-primer config` or `dp-config`
- By default the config uses an SQLite database to store experiment results (used by optuna-dashboard), for highly parallel situations there is a little bit of code you can change to switch to Journal Filesystem (JFS)
  
## param-identification
- Bayesian optimization tool that uses an Optuna backend
- Install as pip package, can be run either by `dp-opt <bayopt JSON>` or `drive-primer optimize <bayopt JSON>`

## slip-table
- Use CMars slipslope demo to generate synthetic slip curves given a simdef JSON (see [simdef_slip.json](slip-table/src/tests/simdef_slip.json))
- Install as pipe package, run with `dp-table` or `drive-primer table -f <JSON>` to run slip-slope simulations and generate slip_*.csv files, use `drive-primer table-gp` in the directory of the slip files to get the Gaussian process curves. Telemetry can be added by modifying the Python file (oof)
  
