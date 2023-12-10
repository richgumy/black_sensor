## Performance metrics for the performance of the black sensor

This directory contains code to help gather data for the SPATIAL, TEMPORAL and STRESS resolution/performance of this EIT based pressure sensor. There are also many functions and code blocks for various desirable mesh manipulation functionality.

**See the notebooks in this directory for more info.**

To run the DOCKER_fem_blob_separator.py in another python script use:
```python
import subprocess
docker_command = [
    "docker", "run",
    "-v", "/c/Users/rel80/OneDrive - University of Canterbury/Postgrad/6. Projects/2. Pressure sensor array/1.0 Code:/external",
    "pymesh/pymesh", "sh", "-c",
    f"cd ../external && python /external/eit_analysis/DOCKER_fem_blob_separator.py {eit_data_file} {raw_data_dir} {frame}"
]
subprocess.run(docker_command, shell=False)
```
Ensure docker is downloaded. See [Pymesh docs](https://pymesh.readthedocs.io/en/latest/installation.html) for more info.
