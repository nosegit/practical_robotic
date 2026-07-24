# Python Environment Setup for Raspberry Pi

This guide helps you create and manage a Python virtual environment with access to system packages.

## Creating Python Environment

``` bash
# 1. Create the new environment with system access
python -m venv --system-site-packages my_pi_env

# 2. Activate it
source my_pi_env/bin/activate

# 3. Install the other packages from your export
pip install -r requirements.txt

```

The `--system-site-packages` flag allows the venv to access system-installed Python packages, which is useful for hardware libraries on Raspberry Pi.

## Using Python Environment

``` bash
# enable environment
source my_pi_env/bin/activate

# disable environment
deactivate
```

Once activated, your prompt will show `(my_pi_env)` to indicate the environment is active.

## Requirements

Ensure `requirements.txt` is in the same directory before running the installation command. This file should list all Python packages needed for your project.

