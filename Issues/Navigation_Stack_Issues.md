# Navigation Stack and Chrony NTP Setup for Locobot and Create® 3

## Navigation Stack Notes

### Trossen Navigation Stack
The Trossen Navigation Stack has several issues. However, resolving the **base clock issues** in the navigation stack should resolve most of the problems. Refer to Trossen's navigation stack [documentation](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros1_packages/navigation_stack_configuration.html) for details.

---

### Important Note
When running the navigation stack on the Locobot, replace `robot_model:=locobot_wx200` with `robot_model:=locobot_wx250s` as shown below:
```bash
roslaunch interbotix_xslocobot_nav xslocobot_nav.launch robot_model:=locobot_wx250s use_lidar:=true rtabmap_args:=-d
```

---

### Issue #1: TF Transform Error
You may encounter the following error:
```bash
[ WARN] [1731885700.701499174]: Could not get transform from locobot/odom to locobot/base_footprint after 0.200000 seconds (for stamp=1731885700.314371)! Error="Could not find a connection between 'locobot/odom' and 'locobot/base_footprint' because they are not part of the same tree.Tf has two or more unconnected trees.. canTransform returned after 0.202066 timeout was 0.2.".
```
---

## Solution 1: Setting Up Chrony NTP Server for Create® 3 Robot

### Step 1: Install `chrony` NTP Server on the Compute Board
Run the following command on your Compute Board:
```bash
sudo apt install chrony
```

---

### Step 2: Edit the Configuration File
Open the `chrony.conf` configuration file:
```bash
sudo vi /etc/chrony/chrony.conf
```

Add the following lines **after** the `pool #.ubuntu.pool.ntp.org iburst maxsources #` block:
```conf
# Enable serving time to NTP clients on 192.168.186.0 subnet.
allow 192.168.186.0/24
```

If the Single Board Computer (SBC) does **not** have a connection to a reference clock (e.g., the Internet), optionally add:
```conf
# Serve time even if not synchronized to a time source
local stratum 10
```

---

### Step 3: Restart `chrony`
Restart the `chrony` service to apply the changes:
```bash
sudo service chrony restart
```

---

### Step 4: Modify NTP Sources in Create® 3 Web Application
1. Log into the **Create® 3 web application**.
2. Navigate to **Beta Features** (as of July 2023).
3. Add the following NTP server if it does not already exist:
   ```
   server 192.168.186.3 iburst
   ```
4. Restart the **NTPD server** or reboot the robot through the web application if changes were made.

---

### Step 5: Verify Communication Between Compute NTP Server and Create® 3
Run the following command on the Compute Board to verify:
```bash
sudo chronyc clients
```

Ensure `192.168.186.2` has a non-zero number in the `NTP` column. Example output:
```
Hostname                      NTP   Drop Int IntL Last     Cmd   Drop Int  Last
===============================================================================
192.168.186.2                  51      0   5   -    32       0      0   -     -
```

---

### Notes:
- If there is a large time jump, the Create® 3 may reject the time update. Check the logs for an error message like:
  ```
  user.notice ntpd: ntpd: reply from 192.168.186.3: delay ### is too high, ignoring
  ```
- In such cases, **reboot the robot**, and the **application**! via the web server to resolve the issue.

---

## Possible Solution 2: Reboot Create® 3 Robot
If the above solution does not work, follow the Create® 3 setup [instructions](https://edu.irobot.com/create3-setup), specifically steps 2-3, to access the online application. Then:
1. Navigate to the **Application** tab.
2. Press **Reboot Robot**.

This process may take several minutes. Wait until you hear the chime and the blue light stops blinking. Once complete, you can reboot the robot.

---
## Possible Solution 3: Refer to Documentation
From my understanding, this issue is caused by the Create® 3 base clock not being synchronized. Refer to the Create® 3 [documentation](https://iroboteducation.github.io/create3_docs/setup/compute-ntp) or Locobot Issues [documentation](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/troubleshooting.html#less-common-issues).
