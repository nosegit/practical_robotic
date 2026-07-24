# Creating Python Environment

``` bash
# 1. Create the new environment with system access
python -m venv --system-site-packages my_pi_env

# 2. Activate it
source my_pi_env/bin/activate

# 3. Install the other packages from your export
pip install -r requirements.txt

```

# Using Python Environment
``` bash
# enable environment
source my_pi_env/bin/activate

# disable environment
deactivate

