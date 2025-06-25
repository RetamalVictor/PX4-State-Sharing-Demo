Those two “\[Err] … Failed to load system plugin” messages are coming straight from Ignition Gazebo when it reads your `server.config` and tries to bring up every plugin listed there (including the optional Optical-Flow and GStreamer camera systems). If you don’t actually need those sensors in your sim, you can safely ignore the warnings, but if you want to get rid of them you have two simple paths:

---

## 1) Install the missing plugin packages

On ROS 2 Humble + Gazebo Fortress, all of the common Ignition-Gazebo sensor/system plugins are shipped in the **ros-gz-plugins** package. Just do:

```bash
sudo apt update
sudo apt install ros-humble-ros-gz-plugins
```

That will drop in `/usr/lib/.../libOpticalFlowSystem.so`, `libGstCameraSystem.so`, etc., and Gazebo will load them cleanly on startup.

---

## 2) Remove them from the PX4 server.config

If you really don’t ever need an optical-flow or GStreamer camera sensor in your PX4 SITL, you can comment out those plugin lines so Gazebo never even tries:

```xml
# edit Tools/simulation/gz/server.config and locate the <system> block …
<system>
  <!-- comment out these two lines: -->
  <!-- <plugin filename="libOpticalFlowSystem.so" name="ignition::gazebo::systems::OpticalFlowSystem"/> -->
  <!-- <plugin filename="libGstCameraSystem.so"  name="ignition::gazebo::systems::GstCameraSystem"/> -->
  <!-- leave the PX4 bridge plugin (libgazebo_px4.so) alone -->
</system>
```

After that, the warnings will disappear.

---

### And what about the `libEGL` messages?

Those are just Mesa falling back because it can’t find a GPU DRI driver. You can either:

* **Ignore** them—they’re harmless if you don’t need hardware‐accelerated rendering.
* Or install the EGL/GL drivers so GPU acceleration works:

  ```bash
  sudo apt install libegl1-mesa libgles2-mesa
  ```

With either the plugins installed or the config lines removed (plus EGL libraries if you care about rendering performance), your Gazebo server will come up cleanly, the PX4 bridge will load as expected, and your  PX4 ↔ Gazebo ↔ QGC chain will run without those load-fail errors.
